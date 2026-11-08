// grasps_from_camera: take a point cloud from the robot's wrist camera,
// detect grasps with GPD and print, for each grasp, the end effector (EE)
// pose in BASE.
// It is a capture + grasps_from_file (pose file mode) in one run, without
// writing and reading the cloud in between.
//
// Usage (run from kinova_grasp/build/):
//   ./grasps_from_camera ../cfg/gpd_params.cfg         robot at 192.168.1.10
//   ./grasps_from_camera ../cfg/gpd_params.cfg <ip>    robot at another IP
//   Add --color NAME to look for grasps only on the plate of that color
//   (cfg/plate_colors.cfg), e.g.
//   ./grasps_from_camera ../cfg/gpd_plate_params.cfg --color blue
//   Add --filter to filter the depth for about one second before the
//   cloud is made (less noise; 1_PreprocessCloud/depth_filter.h). Keep the
//   robot still meanwhile.
//
// The program ONLY READS from the robot: it does not move it and does not
// send the poses anywhere (sending them by UDP is a later step).
//
// Steps:
//   1. Read the GPD config (hand_depth, camera position).
//   2. Capture: cloud in the DEPTH frame + color image + TOOL pose measured
//      by the robot (depth_camera.cpp).
//   3. Save it as camera_ddmm_hhmmss.pcd / _color.png / _pose.txt, so the
//      same capture can be run again later with grasps_from_file.
//   4. INT and EE poses in BASE from the robot's measured pose (frames.h).
//   5. Drop the points below the table (crop_table.cpp, step 1.0). The saved
//      capture of step 3 keeps every point.
//   6. Detect grasps with GPD (grasp_pipeline.cpp).
//   7. Convert each grasp to the EE pose in BASE and print it (best first).

#include <ctime>
#include <iostream>
#include <string>

#include "Robot/Gen3_model.h"  // only for TOOL_OFFSET_Z_2F85 (0.120 m)
#include "0_Input/depth_camera.h"
#include "Pipeline/grasp_pipeline.h"
#include "0_Input/frames.h"
#include "1_PreprocessCloud/crop_table.h"
#include "1_PreprocessCloud/depth_filter.h"
#include "3_Sample/plate_samples.h"

// Name for the saved capture, from the current date and time:
// camera_ddmm_hhmmss.
std::string captureName() {
  const std::time_t now = std::time(nullptr);
  char text[32];
  std::strftime(text, sizeof(text), "camera_%d%m_%H%M%S", std::localtime(&now));
  return text;
}

// takeFilterOption: true if "--filter" is in the arguments; it is taken out
// of argv, so the other arguments keep their places.
bool takeFilterOption(int &argc, char *argv[]) {
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--filter") {
      for (int j = i; j + 1 < argc; j++) {
        argv[j] = argv[j + 1];
      }
      argc -= 1;
      return true;
    }
  }
  return false;
}

int main(int argc, char *argv[]) {
  // "--color NAME" and "--filter" can be anywhere; they are taken out first.
  const std::string color_name = takeColorOption(argc, argv);
  const bool use_filter = takeFilterOption(argc, argv);
  if (argc != 2 && argc != 3) {
    std::cout << "Usage: grasps_from_camera CONFIG_FILE          (robot at 192.168.1.10)\n"
              << "       grasps_from_camera CONFIG_FILE <ip>     (robot at <ip>)\n"
              << "Add --color NAME to grasp only the plate of that color\n"
              << "Add --filter to filter the depth (about 1 s, robot still)\n"
              << "Example (from build/): ./grasps_from_camera ../cfg/gpd_params.cfg\n";
    return 1;
  }
  const std::string config_filename = argv[1];
  const std::string ip_address = (argc == 3) ? argv[2] : "192.168.1.10";

  // ---------- 1. GPD config ----------
  // Read first, so a wrong path is found before connecting to the robot.
  GpdSetup setup;
  if (!readGpdSetup(config_filename, setup)) {
    return 1;
  }

  // ---------- 2. Capture ----------
  Capture capture;
  // With --filter: depth filtered for 3 / DEPTH_FILTER_W seconds.
  if (!captureFromRobot(ip_address, capture, use_filter ? DEPTH_FILTER_W : 0.0)) {
    return 1;
  }
  if (capture.cloud.empty()) {
    std::cout << "Empty point cloud: the camera measured no depth.\n";
    return 1;
  }

  // ---------- 3. Save the capture ----------
  const std::string name = captureName();
  saveCapture(name, capture);

  // ---------- 4. INT and EE poses in BASE ----------
  // From the TOOL pose measured by the robot (more exact than Gen3_model.h).
  // T_base_ee also chooses between the two symmetric EE orientations.
  const Eigen::Matrix4d T_base_ee = toolToBase(capture.tool);
  const Eigen::Matrix4d T_base_int = intToBase(T_base_ee, KinovaGen3::TOOL_OFFSET_Z_2F85);

  // ---------- 5. Drop the points below the table (step 1.0) ----------
  const Eigen::Matrix4d T_base_depth = T_base_int * depthToInt();
  const pcl::PointCloud<pcl::PointXYZ> cloud = cropBelowTable(capture.cloud, T_base_depth);

  // ---------- 6. Detect grasps (DEPTH frame) ----------
  // With --color: GPD looks for grasps only on the plate of that color.
  Eigen::Matrix3Xd samples;
  if (!color_name.empty() &&
      !plateSamples(cloud, capture.color_image, capture.color_intr, color_name, samples)) {
    return 1;
  }
  Grasps grasps;
  if (!detectGraspsInCloud(config_filename, cloud, setup, grasps, samples)) {
    return 1;
  }

  // With --color: plate edge, and the grasp nearest to its middle first.
  PlateEdge edge;
  const bool has_edge =
      !color_name.empty() && plateEdgeOfGrasps(cloud, capture.color_image, capture.color_intr,
                                               color_name, samples, setup, grasps, edge);

  // ---------- 7. Convert and print ----------
  const ToolPose &tool = capture.tool;
  std::cout << "\n========== EE POSES w.r.t. BASE ==========\n"
            << "Robot pose: measured by the robot (saved in " << name << "_pose.txt)\n"
            << "  EE now (m, deg): " << tool.x << " " << tool.y << " " << tool.z << "  "
            << tool.theta_x << " " << tool.theta_y << " " << tool.theta_z << "\n";
  const Eigen::Vector3d ee_angles_now(tool.theta_x, tool.theta_y, tool.theta_z);
  printEEPoses(grasps, setup, T_base_int, T_base_ee, ee_angles_now,
               KinovaGen3::TOOL_OFFSET_Z_2F85, samples, cloud);
  // With --color: length of the plate edge and where each grasp is on it.
  if (has_edge) {
    printPlateEdgeOfGrasps(edge, grasps, T_base_depth);
  }
  std::cout << "Same capture again: ./grasps_from_file " << config_filename << " " << name
            << ".pcd " << name << "_pose.txt\n";
  return 0;
}
