// File:    sweep_views.cpp
// Module:  Task (block 3: views)
// Purpose: EE poses of the sweep views. See sweep_views.h.
#include "Task/sweep_views.h"

#include <algorithm>
#include <cmath>

// Angle in degrees, wrapped to [-180, 180].
static double wrapAngle(double angle_deg) {
  return std::remainder(angle_deg, 360.0);
}

// sweepViewAngle: equal steps from 0 to SWEEP_TOTAL_DEG.
double sweepViewAngle(int k) {
  return (SWEEP_VIEWS > 1) ? k * SWEEP_TOTAL_DEG / (SWEEP_VIEWS - 1) : 0.0;
}

// sweepViewPose: R_k = R_start * Rz(angle), then its x-y-z extrinsic angles
// (R = Rz(theta_z) * Ry(theta_y) * Rx(theta_x), user guide r12.0, p.116).
// The same rotation also has the angles (theta_x + 180, 180 - theta_y,
// theta_z + 180): the one closer to the start angles is kept.
ToolPose sweepViewPose(const ToolPose &start, int k) {
  const double angle = sweepViewAngle(k) * M_PI / 180.0;
  const Eigen::Matrix3d R = toolToBase(start).block<3, 3>(0, 0) *
                            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const double rad2deg = 180.0 / M_PI;
  const double ty = std::asin(-std::max(-1.0, std::min(1.0, R(2, 0)))) * rad2deg;
  const double tx = std::atan2(R(2, 1), R(2, 2)) * rad2deg;
  const double tz = std::atan2(R(1, 0), R(0, 0)) * rad2deg;
  const double other[3] = {wrapAngle(tx + 180.0), wrapAngle(180.0 - ty), wrapAngle(tz + 180.0)};
  const double first[3] = {tx, ty, tz};
  const double now[3] = {start.theta_x, start.theta_y, start.theta_z};
  double d_first = 0.0, d_other = 0.0;
  for (int i = 0; i < 3; i++) {
    d_first += std::pow(wrapAngle(first[i] - now[i]), 2);
    d_other += std::pow(wrapAngle(other[i] - now[i]), 2);
  }
  const double *best = (d_other < d_first) ? other : first;

  ToolPose view = start;
  view.theta_x = now[0] + wrapAngle(best[0] - now[0]);
  view.theta_y = now[1] + wrapAngle(best[1] - now[1]);
  view.theta_z = now[2] + wrapAngle(best[2] - now[2]);
  return view;
}

// wristTurn: R_start^T * R_now is the turn in the EE frame; its angle
// about Z is atan2 of its (1, 0) and (0, 0) entries.
double wristTurn(const ToolPose &start, const ToolPose &now) {
  const Eigen::Matrix3d R = toolToBase(start).block<3, 3>(0, 0).transpose() *
                            toolToBase(now).block<3, 3>(0, 0);
  return std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
}
