// approach_object: block 2 of the task. Finds the object of one color and
// prints the EE pose that puts the color camera above it, looking straight
// down, at APPROACH_HEIGHT, with the object at the center of the color image
// (Task/approach_pose.h). The controller moves the robot; then run it again
// until it prints "IN VIEW" (the whole region inside the color image).
//
// Usage (run from kinova_grasp/build/):
//   ./approach_object COLOR                       robot at 192.168.1.10
//   ./approach_object COLOR <ip>                  robot at another IP
//   ./approach_object COLOR NAME.pcd NAME_pose.txt
//       a saved capture (grasps_from_camera, view_cloud, locate_object or
//       this program), no robot needed
//   COLOR is a name of cfg/plate_colors.cfg, e.g. ./approach_object blue
//
// The program ONLY READS from the robot: it does not move it.
//
// Steps (sub-blocks of approach_pose.h):
//   1.  Capture: cloud + color image + TOOL pose, saved as
//       approach_ddmm_hhmmss (or read from the files).
//   2.  DEPTH frame in BASE, drop the points below the table.
//   2a. Centroid of the region of the color.
//   2b. Does the region touch the border of the color image?
//   2c. Target EE pose.
//   2d. Already there?

#include <cstdio>
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
#include "Task/approach_pose.h"

// Name for the saved capture, from the current date and time:
// approach_ddmm_hhmmss.
std::string captureName() {
  const std::time_t now = std::time(nullptr);
  char text[32];
  std::strftime(text, sizeof(text), "approach_%d%m_%H%M%S", std::localtime(&now));
  return text;
}

// readSavedCapture: the cloud, TOOL pose and color image of a saved capture
// (same as in locate_object.cpp). Returns false (and prints why) if a file
// cannot be read.
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

// printPose: one EE pose, position in m, angles in degrees.
void printPose(const char *label, const ToolPose &p) {
  printf("  %s:  x = %.4f  y = %.4f  z = %.4f [m]   tx = %.1f  ty = %.1f  tz = %.1f [deg]\n",
         label, p.x, p.y, p.z, p.theta_x, p.theta_y, p.theta_z);
}

int main(int argc, char *argv[]) {
  if (argc != 2 && argc != 3 && argc != 4) {
    std::cout << "Usage: approach_object COLOR                        (robot at 192.168.1.10)\n"
              << "       approach_object COLOR <ip>                   (robot at <ip>)\n"
              << "       approach_object COLOR NAME.pcd NAME_pose.txt (saved capture)\n"
              << "COLOR: a name of cfg/plate_colors.cfg. Example (from build/): "
                 "./approach_object blue\n";
    return 1;
  }
  const std::string color_name = argv[1];

  // ---------- 1. Capture ----------
  Capture capture;
  if (argc == 4) {
    if (!readSavedCapture(argv[2], argv[3], capture)) {
      return 1;
    }
  } else {
    const std::string ip_address = (argc == 3) ? argv[2] : "192.168.1.10";
    if (!captureFromRobot(ip_address, capture)) {
      return 1;
    }
    saveCapture(captureName(), capture);
  }
  if (capture.cloud.empty()) {
    std::cout << "Empty point cloud: the camera measured no depth.\n";
    return 1;
  }

  // ---------- 2. DEPTH frame in BASE, drop the points below the table ----------
  const Eigen::Matrix4d T_base_ee = toolToBase(capture.tool);
  const Eigen::Matrix4d T_base_int = intToBase(T_base_ee, KinovaGen3::TOOL_OFFSET_Z_2F85);
  const Eigen::Matrix4d T_base_depth = T_base_int * depthToInt();
  const pcl::PointCloud<pcl::PointXYZ> cloud = cropBelowTable(capture.cloud, T_base_depth);

  // ---------- 2a. Centroid of the region ----------
  ObjectPosition object;
  if (!objectPosition(cloud, capture.color_image, capture.color_intr, color_name, T_base_depth,
                      object)) {
    return 1;
  }

  // ---------- 2b. Region on the border of the color image ----------
  ImageBorders borders;
  if (!regionBorders(cloud, capture.color_image, capture.color_intr, color_name, borders)) {
    return 1;
  }

  // ---------- 2c and 2d. Target EE pose, already there? ----------
  const ToolPose target =
      approachPose(object.centroid, capture.tool, capture.color_intr, APPROACH_HEIGHT,
                   KinovaGen3::TOOL_OFFSET_Z_2F85);
  double distance = 0.0;
  const bool at_target = atApproachPose(capture.tool, target, distance);

  std::cout << "\n========== APPROACH " << color_name << " ==========\n";
  printf("  Centroid in BASE (m): %.4f %.4f %.4f  (%zu points)\n", object.centroid.x(),
         object.centroid.y(), object.centroid.z(), object.points);
  printf("  Height of the color camera over it: %.0f cm (APPROACH_HEIGHT)\n",
         APPROACH_HEIGHT * 100.0);
  printPose("EE now   ", capture.tool);
  printPose("EE target", target);
  printf("  Distance to the target: %.1f cm (tolerance %.0f cm)\n", distance * 100.0,
         POSITION_TOLERANCE * 100.0);
  if (borders.any()) {
    printf("  The region touches the border of the color image:%s%s%s%s\n",
           borders.left ? " left" : "", borders.right ? " right" : "",
           borders.top ? " top" : "", borders.bottom ? " bottom" : "");
    printf("  Part of the object may be out of the image.\n");
  }
  if (!borders.any()) {
    std::cout << "  IN VIEW: the whole region is inside the color image; go on to block 3\n"
                 "  (views). Centering it (EE target) is optional.\n";
  } else if (at_target) {
    std::cout << "  AT TARGET, but the region still touches the border: the object may be\n"
                 "  larger than the image. Raise APPROACH_HEIGHT (Task/approach_pose.h).\n";
  } else {
    std::cout << "  MOVE: send the EE target to the controller, then run again.\n";
  }
  std::cout << "==========================================\n";
  return 0;
}
