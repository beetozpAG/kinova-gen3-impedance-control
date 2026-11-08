// File:    approach_pose.h
// Module:  Task (block 2: approach)
// Purpose: The EE pose that puts the color camera above the object of one
//          color, looking straight down, at APPROACH_HEIGHT from it, with
//          the object at the center of the color image. Our code only prints
//          the pose; the controller moves the robot.
//
// Sub-blocks:
//   2a. Centroid of the region of the color (objectPosition, 2_GetROI).
//   2b. Does the region touch the border of the color image? (regionBorders)
//       Then part of the object may be out of the image.
//   2c. Target EE pose (approachPose).
//   2d. Done when the region touches no border (IN VIEW); centering is
//       optional. atApproachPose: is the EE at the target? Run again after
//       each move: the centroid moves to the center of the image and is
//       seen better.
//
// Limits: the centroid is the one of the seen part of the region, not of the
// whole object. Each new look from closer corrects it (2d).
#pragma once

#include <string>

#include <Eigen/Dense>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "0_Input/frames.h"
#include "0_Input/vision_module.h"

// Distance from the color camera to the centroid, along the camera axis
// (vertical) [m]. The depth camera does not measure closer than 0.18 m
// (user guide r12.0, p.23). At 0.30 m the color image covers about
// 30 x 17 cm (fx = 1298, 1280 x 720 pixels, camera_2310_182109).
const double APPROACH_HEIGHT = 0.30;

// The robot is at the target if its EE is closer than this to it [m].
const double POSITION_TOLERANCE = 0.01;

// A point of the region this close to the edge of the color image [pixels]
// touches the border.
const int BORDER_MARGIN = 5;

// Sides of the color image touched by the region (as the image is seen).
struct ImageBorders {
  bool left = false;
  bool right = false;
  bool top = false;
  bool bottom = false;
  bool any() const { return left || right || top || bottom; }
};

// regionBorders: which sides of the color image the region of color_name
// touches (the same points as objectPosition: color mask, then the largest
// group). cloud is in the DEPTH frame. Returns false (and prints why) if the
// color is not in PLATE_COLORS_FILE.
bool regionBorders(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &color_image,
                   const Intrinsics &color_intr, const std::string &color_name,
                   ImageBorders &borders);

// approachPose: the EE pose with the color camera at height above centroid
// (BASE, m), looking straight down, centroid at the center of the color
// image. theta_z is the one of tool_now (the wrist does not turn); theta_x
// and theta_y are set to straight down in the same form tool_now has
// (0, +-180 or +-180, 0). ee_offset_z: TOOL ahead of INT along Z (as in
// intToBase, frames.h).
ToolPose approachPose(const Eigen::Vector3d &centroid, const ToolPose &tool_now,
                      const Intrinsics &color_intr, double height, double ee_offset_z);

// atApproachPose: true if tool_now is closer than POSITION_TOLERANCE to
// target. distance is that distance [m].
bool atApproachPose(const ToolPose &tool_now, const ToolPose &target, double &distance);
