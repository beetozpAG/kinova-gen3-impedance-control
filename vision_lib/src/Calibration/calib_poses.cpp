// File:    calib_poses.cpp
// Module:  Calibration (block C1: poses of the robot)
// Purpose: EE poses that look at the board. See calib_poses.h.
#include "Calibration/calib_poses.h"

#include <algorithm>
#include <cmath>

#include "0_Input/depth_camera.h"

// Angle in degrees, wrapped to [-180, 180].
static double wrapAngle(double angle_deg) {
  return std::remainder(angle_deg, 360.0);
}

// anglesClosestTo: x-y-z extrinsic angles of R (R = Rz * Ry * Rx, user
// guide r12.0, p.116). R also has the angles (tx + 180, 180 - ty, tz + 180):
// the set closer to now is kept (same rule as sweepViewPose).
static void anglesClosestTo(const Eigen::Matrix3d &R, const ToolPose &now, ToolPose &pose) {
  const double rad2deg = 180.0 / M_PI;
  const double ty = std::asin(-std::max(-1.0, std::min(1.0, R(2, 0)))) * rad2deg;
  const double tx = std::atan2(R(2, 1), R(2, 2)) * rad2deg;
  const double tz = std::atan2(R(1, 0), R(0, 0)) * rad2deg;
  const double first[3] = {tx, ty, tz};
  const double other[3] = {wrapAngle(tx + 180.0), wrapAngle(180.0 - ty), wrapAngle(tz + 180.0)};
  const double start[3] = {now.theta_x, now.theta_y, now.theta_z};
  double d_first = 0.0, d_other = 0.0;
  for (int i = 0; i < 3; i++) {
    d_first += std::pow(wrapAngle(first[i] - start[i]), 2);
    d_other += std::pow(wrapAngle(other[i] - start[i]), 2);
  }
  const double *best = (d_other < d_first) ? other : first;
  pose.theta_x = start[0] + wrapAngle(best[0] - start[0]);
  pose.theta_y = start[1] + wrapAngle(best[1] - start[1]);
  pose.theta_z = start[2] + wrapAngle(best[2] - start[2]);
}

// calibPoseTarget:
//   R_k     = Rot(u, tilt) * R_color0 * Rz(turn),  u = (-sin az, cos az, 0)
//             (the tilt moves the camera toward the azimuth, looking back)
//   p_color = center - distance * R_k * d,  d = unit ray of the image center
//   T_base_tool = T_base_color * T_color_int * T_int_tool
ToolPose calibPoseTarget(const ToolPose &start, const Eigen::Vector3d &board_center,
                         const Intrinsics &color_intr, double ee_offset_z, int k) {
  if (k <= 0 || k >= static_cast<int>(CALIB_POSES.size())) {
    return start;
  }
  const CalibPose &c = CALIB_POSES[k];
  const double deg2rad = M_PI / 180.0;

  const Eigen::Matrix4d T_base_color0 = intToBase(toolToBase(start), ee_offset_z) * colorToInt();
  const Eigen::Vector3d u(-std::sin(c.azimuth * deg2rad), std::cos(c.azimuth * deg2rad), 0.0);
  const Eigen::Matrix3d R_k = Eigen::AngleAxisd(c.tilt * deg2rad, u).toRotationMatrix() *
                              T_base_color0.block<3, 3>(0, 0) *
                              Eigen::AngleAxisd(c.turn * deg2rad, Eigen::Vector3d::UnitZ())
                                  .toRotationMatrix();

  const double sign = FLIP_XY ? -1.0 : 1.0;
  const Eigen::Vector3d d =
      Eigen::Vector3d(sign * (color_intr.width / 2.0 - color_intr.cx) / color_intr.fx,
                      sign * (color_intr.height / 2.0 - color_intr.cy) / color_intr.fy, 1.0)
          .normalized();

  Eigen::Matrix4d T_base_color = Eigen::Matrix4d::Identity();
  T_base_color.block<3, 3>(0, 0) = R_k;
  T_base_color.block<3, 1>(0, 3) = board_center - c.distance * (R_k * d);
  Eigen::Matrix4d T_int_tool = Eigen::Matrix4d::Identity();
  T_int_tool(2, 3) = ee_offset_z;
  const Eigen::Matrix4d T_base_tool = T_base_color * colorToInt().inverse() * T_int_tool;

  ToolPose target;
  target.x = T_base_tool(0, 3);
  target.y = T_base_tool(1, 3);
  target.z = T_base_tool(2, 3);
  anglesClosestTo(T_base_tool.block<3, 3>(0, 0), start, target);
  return target;
}
