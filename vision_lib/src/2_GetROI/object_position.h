// File:    object_position.h
// Module:  2_GetROI
// Purpose: Where the object of one color is: the centroid of its points in
//          the BASE frame. It is a rough position (a few cm), to send the
//          EE near the object; then GPD, with the camera close to it, finds
//          the exact grasp.
//
// How: the points of the color (color_mask.h), only the largest group of
// them (largest_cluster.h), their mean in the DEPTH frame, then moved to
// BASE with T_base_depth (frames.h).
//
// Limits: the camera sees only the faces toward it, so the centroid of the
// points is on the seen side of the object, not at its real center (half
// the object size off, at most).
#pragma once

#include <string>

#include <Eigen/Dense>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "0_Input/vision_module.h"

// Fewer points of the color than this: no object, only loose points (blue
// in camera_2110_091753, where no blue object is seen: 5 points).
const size_t MIN_OBJECT_POINTS = 50;

// Object of one color, in the BASE frame [m].
struct ObjectPosition {
  Eigen::Vector3d centroid;  // mean of the points
  Eigen::Vector3d size;      // size of the box around the points, along X, Y, Z of BASE
  size_t points;             // number of points of the object
};

// objectPosition: the object of cloud (DEPTH frame) whose color, taken from
// color_image, is inside the range named color_name in PLATE_COLORS_FILE.
// T_base_depth is the DEPTH frame with respect to BASE when the cloud was
// taken. Returns false (and prints why) if the color is not in the file or
// fewer than MIN_OBJECT_POINTS points have that color.
bool objectPosition(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &color_image,
                    const Intrinsics &color_intr, const std::string &color_name,
                    const Eigen::Matrix4d &T_base_depth, ObjectPosition &object);
