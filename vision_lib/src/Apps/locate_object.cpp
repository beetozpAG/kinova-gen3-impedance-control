// locate_object: find the object of one color with the robot's wrist camera
// and print where it is in the BASE frame (the centroid of its points).
// It is a rough position, to send the EE near the object; then
// grasps_from_camera, with the camera close to it, finds the grasp.
// No GPD here: only the color of the points (2_GetROI/object_position.h).
//
// Usage (run from kinova_grasp/build/):
//   ./locate_object COLOR                       robot at 192.168.1.10
//   ./locate_object COLOR <ip>                  robot at another IP
//   ./locate_object COLOR NAME.pcd NAME_pose.txt
//       a saved capture (grasps_from_camera, view_cloud or this program),
//       no robot needed
//   COLOR is a name of cfg/plate_colors.cfg, e.g. ./locate_object blue
//
// The program ONLY READS from the robot: it does not move it and does not
// send the position anywhere (sending it to the controller is a later step).
//
// Steps:
//   1. Capture: cloud in the DEPTH frame + color image + TOOL pose measured
//      by the robot, saved as locate_ddmm_hhmmss (or read from the files).
//   2. DEPTH frame in BASE, from the measured TOOL pose (frames.h).
//   3. Drop the points below the table (crop_table.cpp, step 1.0).
//   4. Centroid of the points of the color, in BASE, and print it.

#include <ctime>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>

#include "Robot/Gen3_model.h"  // only for TOOL_OFFSET_Z_2F85 (0.120 m)
#include "0_Input/capture_file.h"
#include "0_Input/depth_camera.h"
#include "0_Input/frames.h"
#include "1_PreprocessCloud/crop_table.h"
#include "2_GetROI/object_position.h"

// Name for the saved capture, from the current date and time:
// locate_ddmm_hhmmss.
std::string captureName() {
  const std::time_t now = std::time(nullptr);
  char text[32];
  std::strftime(text, sizeof(text), "locate_%d%m_%H%M%S", std::localtime(&now));
  return text;
}

// readSavedCapture: the cloud, TOOL pose and color image of a saved capture.
// Returns false (and prints why) if a file cannot be read.
bool readSavedCapture(const std::string &pcd_file, const std::string &pose_file,
                      Capture &capture) {
  if (pcl::io::loadPCDFile(pcd_file, capture.cloud) != 0) {
    std::cout << "Error: could not read the point cloud file: " << pcd_file << "\n";
    return false;
  }
  if (!readPoseFile(pose_file, capture.tool, capture.q_deg)) {
    std::cout << "Error: could not read a tool_pose line in " << pose_file << "\n";
    return false;
  }
  if (!readColorCapture(pose_file, capture.color_intr, capture.color_image)) {
    std::cout << "Error: no color image in " << pose_file
              << " (saved before the color camera was added)\n";
    return false;
  }
  return true;
}

int main(int argc, char *argv[]) {
  if (argc != 2 && argc != 3 && argc != 4) {
    std::cout << "Usage: locate_object COLOR                        (robot at 192.168.1.10)\n"
              << "       locate_object COLOR <ip>                   (robot at <ip>)\n"
              << "       locate_object COLOR NAME.pcd NAME_pose.txt (saved capture)\n"
              << "COLOR: a name of cfg/plate_colors.cfg. Example (from build/): "
                 "./locate_object blue\n";
    return 1;
  }
  const std::string color_name = argv[1];

  // ---------- 1. Capture ----------
  Capture capture;
  std::string source;
  if (argc == 4) {
    if (!readSavedCapture(argv[2], argv[3], capture)) {
      return 1;
    }
    source = argv[3];
  } else {
    const std::string ip_address = (argc == 3) ? argv[2] : "192.168.1.10";
    if (!captureFromRobot(ip_address, capture)) {
      return 1;
    }
    source = captureName();
    saveCapture(source, capture);
    source += "_pose.txt";
  }
  if (capture.cloud.empty()) {
    std::cout << "Empty point cloud: the camera measured no depth.\n";
    return 1;
  }

  // ---------- 2. DEPTH frame in BASE ----------
  const Eigen::Matrix4d T_base_ee = toolToBase(capture.tool);
  const Eigen::Matrix4d T_base_int = intToBase(T_base_ee, KinovaGen3::TOOL_OFFSET_Z_2F85);
  const Eigen::Matrix4d T_base_depth = T_base_int * depthToInt();

  // ---------- 3. Drop the points below the table (step 1.0) ----------
  const pcl::PointCloud<pcl::PointXYZ> cloud = cropBelowTable(capture.cloud, T_base_depth);

  // ---------- 4. Centroid of the object ----------
  ObjectPosition object;
  if (!objectPosition(cloud, capture.color_image, capture.color_intr, color_name, T_base_depth,
                      object)) {
    return 1;
  }
  const ToolPose &tool = capture.tool;
  std::cout << "\n========== OBJECT " << color_name << " w.r.t. BASE ==========\n"
            << "Robot pose: measured by the robot (" << source << ")\n"
            << "  EE now (m):      " << tool.x << " " << tool.y << " " << tool.z << "\n"
            << "  Points:          " << object.points << "\n"
            << "  Centroid (m):    " << object.centroid.transpose() << "\n"
            << "  Size (m, X Y Z): " << object.size.transpose() << "\n"
            << "  Distance from the EE now (m): "
            << (object.centroid - T_base_ee.block<3, 1>(0, 3)).norm() << "\n"
            << "===================================================\n";
  return 0;
}
