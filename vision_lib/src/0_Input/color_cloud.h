// File:    color_cloud.h
// Module:  0_Input
// Purpose: Gives each point of the depth cloud the color of the scene, taken
//          from the color image. With it the plates can be told apart by
//          their color (2_GetROI), and the cloud can be seen in real colors.
//
// How (user guide r12.0, Table 92, p.225):
//   1. DEPTH -> COLOR: both frames have the same orientation as INT, so a
//      point only moves by a translation: p_color = p_depth + t_color_depth.
//      Table 92 gives the two sensors with respect to INT:
//        t_DEPTH = (27.50, 66.00, -3.05) mm,  t_COLOR = (0, 56.39, -3.05) mm
//      so t_color_depth = t_DEPTH - t_COLOR = (27.50, 9.61, 0) mm. This
//      value is not in the table: the code computes it from the two matrices
//      (colorToInt and depthToInt, frames.h).
//   2. COLOR point -> pixel, pinhole model of the color camera, with the same
//      axis signs as depthToCloud (FLIP_XY, depth_camera.h):
//        col = cx - fx * x / z,   row = cy - fy * y / z
//   3. The point takes the color of that pixel.
//
// Limits:
//   - The two cameras are 2.9 cm apart, so each one sees a little behind the
//     edges of the objects that the other does not. A point the color camera
//     cannot see takes the color of what hides it. Small, only at edges.
//   - The color and depth images are not taken at the same instant
//     (color_camera.h): the robot and the scene must be still.
#pragma once

#include <Eigen/Dense>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "0_Input/vision_module.h"

// Color of the points that fall outside the color image (R, G, B).
const uint8_t NO_COLOR_RGB[3] = {80, 80, 80};

// DepthToColor: steps 1 and 2 above, and back. The translation is computed
// once, so it can be used for many points. Also used by plate_edge_image.h.
class DepthToColor {
 public:
  explicit DepthToColor(const Intrinsics &color_intr);
  // pixel: where a point of the DEPTH frame is seen in the color image, and
  // its distance z along the color camera axis [m]. False if the point is
  // behind the camera. col and row may be outside the image.
  bool pixel(const Eigen::Vector3d &p_depth, int &col, int &row, double &z) const;
  // toDepth: the point seen at pixel (col, row) at distance z from the color
  // camera, in the DEPTH frame (inverse of pixel).
  Eigen::Vector3d toDepth(double col, double row, double z) const;

 private:
  Intrinsics intr_;
  Eigen::Vector3d t_color_depth_;  // DEPTH origin in COLOR [m]
  double sign_;                    // FLIP_XY
};

// colorCloud: same points as cloud (DEPTH frame), each with the color of its
// pixel in color_image (8 bits, BGR, see color_camera.h). Points that fall
// outside the color image get NO_COLOR_RGB. Same order and number of points
// as cloud, so point i of both clouds is the same point.
pcl::PointCloud<pcl::PointXYZRGB> colorCloud(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                             const cv::Mat &color_image,
                                             const Intrinsics &color_intr);
