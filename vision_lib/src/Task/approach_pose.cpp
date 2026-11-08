// File:    approach_pose.cpp
// Module:  Task (block 2: approach)
// Purpose: EE pose above the object of one color. See approach_pose.h.
#include "Task/approach_pose.h"

#include <cmath>

#include "0_Input/color_cloud.h"
#include "0_Input/depth_camera.h"
#include "2_GetROI/color_mask.h"
#include "2_GetROI/largest_cluster.h"

// regionBorders: points of the region (as in objectPosition), then each one
// to its pixel of the color image (as in colorCloud, color_cloud.cpp).
bool regionBorders(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &color_image,
                   const Intrinsics &color_intr, const std::string &color_name,
                   ImageBorders &borders) {
  ColorRange range;
  if (!readColorRange(PLATE_COLORS_FILE, color_name, range)) {
    return false;
  }
  const pcl::PointCloud<pcl::PointXYZRGB> colored = colorCloud(cloud, color_image, color_intr);
  const std::vector<int> indices = largestCluster(cloud, colorMask(colored, range));

  // T_color_depth: only a translation (frames.h).
  const Eigen::Vector3d t = (colorToInt().inverse() * depthToInt()).block<3, 1>(0, 3);
  const double sign = FLIP_XY ? -1.0 : 1.0;
  borders = ImageBorders();
  for (int i : indices) {
    const pcl::PointXYZ &p = cloud[i];
    const double x = p.x + t.x();
    const double y = p.y + t.y();
    const double z = p.z + t.z();
    const double col = color_intr.cx + sign * color_intr.fx * x / z;
    const double row = color_intr.cy + sign * color_intr.fy * y / z;
    borders.left |= col < BORDER_MARGIN;
    borders.right |= col > color_intr.width - 1 - BORDER_MARGIN;
    borders.top |= row < BORDER_MARGIN;
    borders.bottom |= row > color_intr.height - 1 - BORDER_MARGIN;
  }
  return true;
}

// approachPose: the centroid must be on the ray of the image center, at
// height along the camera axis:
//   d       = ray of the center pixel in COLOR, with d.z = 1 (inverse of
//             the pinhole of colorCloud)
//   p_color = centroid - R * (height * d)
//   p_tool  = p_color - R * t_tool_color
// R is the rotation of the target pose; COLOR has the same orientation as
// TOOL (frames.h), so R is also the rotation of COLOR.
ToolPose approachPose(const Eigen::Vector3d &centroid, const ToolPose &tool_now,
                      const Intrinsics &color_intr, double height, double ee_offset_z) {
  ToolPose target = tool_now;
  if (std::abs(tool_now.theta_y) >= std::abs(tool_now.theta_x)) {
    target.theta_x = 0.0;
    target.theta_y = std::copysign(180.0, tool_now.theta_y);
  } else {
    target.theta_x = std::copysign(180.0, tool_now.theta_x);
    target.theta_y = 0.0;
  }
  target.x = target.y = target.z = 0.0;
  const Eigen::Matrix3d R = toolToBase(target).block<3, 3>(0, 0);

  const double sign = FLIP_XY ? -1.0 : 1.0;
  const Eigen::Vector3d d(sign * (color_intr.width / 2.0 - color_intr.cx) / color_intr.fx,
                          sign * (color_intr.height / 2.0 - color_intr.cy) / color_intr.fy, 1.0);
  const Eigen::Matrix4d T_tool_color =
      intToBase(Eigen::Matrix4d::Identity(), ee_offset_z) * colorToInt();
  const Eigen::Vector3d p_color = centroid - R * (height * d);
  const Eigen::Vector3d p_tool = p_color - R * T_tool_color.block<3, 1>(0, 3);

  target.x = p_tool.x();
  target.y = p_tool.y();
  target.z = p_tool.z();
  return target;
}

// atApproachPose: distance between the two EE positions.
bool atApproachPose(const ToolPose &tool_now, const ToolPose &target, double &distance) {
  distance = (Eigen::Vector3d(tool_now.x, tool_now.y, tool_now.z) -
              Eigen::Vector3d(target.x, target.y, target.z))
                 .norm();
  return distance < POSITION_TOLERANCE;
}
