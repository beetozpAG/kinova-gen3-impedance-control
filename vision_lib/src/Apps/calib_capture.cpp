// calib_capture: block C1 of the camera calibration (src/Calibration/).
// Photographs the ChArUco board from the poses of CALIB_POSES
// (Calibration/calib_poses.h): prints the EE pose of the next photo, waits
// for Enter, captures with the robot still and checks that the board is
// seen. The controller moves the robot.
//
// Usage (run from kinova_grasp/build/), board flat on the table, still:
//   ./calib_capture            robot at 192.168.1.10
//   ./calib_capture <ip>       robot at another IP
// Start with the color camera about 0.40 m above the board, looking down,
// with the whole board in the image (pose 0).
//
// Saves calib_ddmm_hhmmss/pose_K.pcd, pose_K_color.png and pose_K_pose.txt
// (the files of saveCapture, 0_Input/depth_camera.h), only for the photos
// where the board was found. The depth cloud is not used by the hand-eye
// methods; it is kept to check the calibration on the depth too.
//
// The program ONLY READS from the robot: it does not move it.
//
// Steps:
//   C1a. Pose 0: capture, find the board, its center in BASE.
//   C1b. Pose K: target EE pose (calibPoseTarget).
//   C1c. Capture with the robot still; find the board (C2, board_pose.h).

#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <string>

#include "Robot/Gen3_model.h"  // only for TOOL_OFFSET_Z_2F85 (0.120 m)
#include "0_Input/depth_camera.h"
#include "0_Input/frames.h"
#include "Calibration/board_pose.h"
#include "Calibration/calib_poses.h"

// Name of the folder, from the current date and time: calib_ddmm_hhmmss.
static std::string calibName() {
  const std::time_t now = std::time(nullptr);
  char text[32];
  std::strftime(text, sizeof(text), "calib_%d%m_%H%M%S", std::localtime(&now));
  return text;
}

// printPose: one EE pose, position in m, angles in degrees.
static void printPose(const char *label, const ToolPose &p) {
  printf("  %s:  x = %.4f  y = %.4f  z = %.4f [m]   tx = %.1f  ty = %.1f  tz = %.1f [deg]\n",
         label, p.x, p.y, p.z, p.theta_x, p.theta_y, p.theta_z);
}

// waitKey: reads one line; returns it ("" = Enter).
static std::string waitKey() {
  std::string line;
  std::getline(std::cin, line);
  return line;
}

int main(int argc, char *argv[]) {
  if (argc != 1 && argc != 2) {
    std::cout << "Usage: calib_capture          (robot at 192.168.1.10)\n"
              << "       calib_capture <ip>     (robot at <ip>)\n"
              << "Start with the color camera about 0.40 m above the board, looking down.\n";
    return 1;
  }
  const std::string ip_address = (argc == 2) ? argv[1] : "192.168.1.10";
  const double ee_offset_z = KinovaGen3::TOOL_OFFSET_Z_2F85;

  BoardSize size;
  if (!readBoardSize(BOARD_FILE, size)) {
    return 1;
  }
  const cv::aruco::CharucoBoard board = makeBoard(size);
  const Eigen::Vector3d center_board(BOARD_COLS * size.square / 2.0,
                                     BOARD_ROWS * size.square / 2.0, 0.0);

  const std::string folder = calibName();
  std::filesystem::create_directory(folder);

  ToolPose start{};
  Eigen::Vector3d center_base = Eigen::Vector3d::Zero();
  Intrinsics color_intr{};
  int saved = 0;
  const int poses = static_cast<int>(CALIB_POSES.size());
  for (int k = 0; k < poses; k++) {
    // ---------- C1b. Target pose ----------
    if (k > 0) {
      const CalibPose &c = CALIB_POSES[k];
      printf("\n========== POSE %d of %d: %.2f m, tilt %.0f deg toward %.0f deg, wrist %+.0f deg "
             "==========\n",
             k, poses - 1, c.distance, c.tilt, c.azimuth, c.turn);
      printPose("EE target", calibPoseTarget(start, center_base, color_intr, ee_offset_z, k));
      std::cout << "Send it to the controller, wait until the robot stops, then press Enter\n"
                   "(s = skip this pose, q = stop).\n";
    }

    while (true) {
      if (k > 0) {
        const std::string key = waitKey();
        if (key == "q") {
          std::cout << "Stopped. " << saved << " photos in " << folder << "/\n";
          return 0;
        }
        if (key == "s") {
          break;
        }
      }

      // ---------- C1c. Capture and find the board ----------
      Capture capture;
      if (!captureFromRobot(ip_address, capture)) {
        std::cout << "  Capture failed: press Enter to try again (s = skip, q = stop).\n";
        if (k == 0 && waitKey() == "q") {
          return 1;
        }
        continue;
      }
      BoardView view;
      if (!boardPose(capture.color_image, capture.color_intr, board, view)) {
        std::cout << "  The board is not seen: move the robot so the whole board is in the\n"
                     "  image, then press Enter (s = skip, q = stop).\n";
        if (k == 0 && waitKey() == "q") {
          return 1;
        }
        continue;
      }
      printPose("EE now   ", capture.tool);
      printf("  Board: %zu corners, reprojection %.2f px%s, %.0f mm from the camera\n",
             view.board_points.size(), view.reprojection_px,
             view.reprojection_px > BOARD_MAX_REPROJECTION_PX ? " (TOO LARGE: blurred?)" : "",
             view.T_color_board.block<3, 1>(0, 3).norm() * 1000.0);

      // ---------- C1a. Pose 0: board center in BASE ----------
      if (k == 0) {
        start = capture.tool;
        color_intr = capture.color_intr;
        const Eigen::Matrix4d T_base_color =
            intToBase(toolToBase(start), ee_offset_z) * colorToInt();
        center_base = (T_base_color * view.T_color_board * center_board.homogeneous()).head<3>();
        printf("  Board center in BASE: x = %.4f  y = %.4f  z = %.4f [m]\n", center_base.x(),
               center_base.y(), center_base.z());
      }
      saveCapture(folder + "/pose_" + std::to_string(saved), capture);
      saved++;
      break;
    }
  }

  std::cout << "\n========== CAPTURE DONE ==========\n"
            << "Folder: " << folder << "/ (" << saved << " photos with the board)\n"
            << "Next (block C3): ./hand_eye_tsai " << folder << "\n"
            << "                 ./hand_eye_park " << folder << "\n"
            << "                 ./hand_eye_daniilidis " << folder << "\n"
            << "Then (block C4):  ./hand_eye_compare " << folder << "\n"
            << "==================================\n";
  return 0;
}
