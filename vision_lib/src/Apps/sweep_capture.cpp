// sweep_capture: block 3 of the task. Takes the views of the sweep: from
// the pose of block 2 (approach_object: color camera above the object,
// looking down), the wrist turns about the Z axis of the EE and the robot
// stops at 0, 90 and 180 deg (Task/sweep_views.h). At each stop: filtered
// depth, color image and TOOL pose. The controller moves the robot; this
// program prints the pose of the next view and waits for Enter.
//
// Usage (run from kinova_grasp/build/), robot at the pose of block 2:
//   ./sweep_capture            robot at 192.168.1.10
//   ./sweep_capture <ip>       robot at another IP
//
// Saves sweep_ddmm_hhmmss/view_K.pcd, view_K_color.png and view_K_pose.txt
// (K = 0, 1, 2), the same files as grasps_from_camera: each view can be
// read alone by grasps_from_file. The fusion of the views is block 4.
//
// The program ONLY READS from the robot: it does not move it.
//
// Steps (sub-blocks of sweep_views.h):
//   3a. Pose of the next view.
//   3b. Capture with the robot still (depth filter, 1_PreprocessCloud/depth_filter).
//   3c. Check the turn of the view; capture again if it is off.

#include <cmath>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <string>

#include "0_Input/depth_camera.h"
#include "0_Input/frames.h"
#include "1_PreprocessCloud/depth_filter.h"
#include "Task/sweep_views.h"

// Name of the folder of the sweep, from the current date and time:
// sweep_ddmm_hhmmss.
std::string sweepName() {
  const std::time_t now = std::time(nullptr);
  char text[32];
  std::strftime(text, sizeof(text), "sweep_%d%m_%H%M%S", std::localtime(&now));
  return text;
}

// printPose: one EE pose, position in m, angles in degrees.
void printPose(const char *label, const ToolPose &p) {
  printf("  %s:  x = %.4f  y = %.4f  z = %.4f [m]   tx = %.1f  ty = %.1f  tz = %.1f [deg]\n",
         label, p.x, p.y, p.z, p.theta_x, p.theta_y, p.theta_z);
}

int main(int argc, char *argv[]) {
  if (argc != 1 && argc != 2) {
    std::cout << "Usage: sweep_capture          (robot at 192.168.1.10)\n"
              << "       sweep_capture <ip>     (robot at <ip>)\n"
              << "Start at the pose of block 2 (./approach_object COLOR).\n";
    return 1;
  }
  const std::string ip_address = (argc == 2) ? argv[1] : "192.168.1.10";
  const std::string folder = sweepName();
  std::filesystem::create_directory(folder);

  ToolPose start{};
  double turns[SWEEP_VIEWS];
  for (int k = 0; k < SWEEP_VIEWS; k++) {
    // ---------- 3a. Pose of the view ----------
    if (k > 0) {
      std::cout << "\n========== VIEW " << k << " of " << SWEEP_VIEWS - 1 << ": wrist +"
                << sweepViewAngle(k) << " deg from view 0 ==========\n";
      printPose("EE target", sweepViewPose(start, k));
      std::cout << "Send it to the controller (turn the wrist, do not go the other way\n"
                   "round), wait until the robot stops, then press Enter.\n";
    }

    while (true) {
      if (k > 0) {
        std::string line;
        std::getline(std::cin, line);
        if (line == "q") {
          std::cout << "Stopped. Views saved in " << folder << "/\n";
          return 1;
        }
      }

      // ---------- 3b. Capture, robot still ----------
      Capture capture;
      if (!captureFromRobot(ip_address, capture, DEPTH_FILTER_W)) {
        return 1;
      }
      if (k == 0) {
        start = capture.tool;
      }

      // ---------- 3c. Check the turn and the position ----------
      const double turn = wristTurn(start, capture.tool);
      const double turn_error = std::remainder(turn - sweepViewAngle(k), 360.0);
      const double moved = (Eigen::Vector3d(capture.tool.x, capture.tool.y, capture.tool.z) -
                            Eigen::Vector3d(start.x, start.y, start.z))
                               .norm();
      printPose("EE now   ", capture.tool);
      printf("  Wrist turn from view 0: %.1f deg (view %d: %.0f deg), EE moved %.1f mm\n", turn, k,
             sweepViewAngle(k), moved * 1000.0);
      if (std::abs(turn_error) > SWEEP_TOLERANCE_DEG) {
        printf("  The turn is %.1f deg off (more than %.0f deg): send the EE target again,\n"
               "  then press Enter (q = stop).\n",
               turn_error, SWEEP_TOLERANCE_DEG);
        continue;
      }
      turns[k] = turn;
      saveCapture(folder + "/view_" + std::to_string(k), capture);
      break;
    }
  }

  std::cout << "\n========== SWEEP DONE ==========\n"
            << "Folder: " << folder << "/\n";
  for (int k = 0; k < SWEEP_VIEWS; k++) {
    printf("  view_%d: wrist turn %.1f deg\n", k, turns[k]);
  }
  std::cout << "Each view alone: ./grasps_from_file ../cfg/gpd_plate_params.cfg " << folder
            << "/view_0.pcd " << folder << "/view_0_pose.txt --color COLOR\n"
            << "================================\n";
  return 0;
}
