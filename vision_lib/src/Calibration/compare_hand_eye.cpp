// File:    compare_hand_eye.cpp
// Module:  Calibration (block C4: compare the methods)
// Purpose: Agreement and validation of the hand-eye results. See
//          compare_hand_eye.h.
#include "Calibration/compare_hand_eye.h"

#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>

#include "0_Input/frames.h"

// meanRotation: the rotation closest to the mean of the matrices (SVD of
// their sum, projected back to a rotation).
static Eigen::Matrix3d meanRotation(const std::vector<Eigen::Matrix3d> &rotations) {
  Eigen::Matrix3d sum = Eigen::Matrix3d::Zero();
  for (const Eigen::Matrix3d &R : rotations) {
    sum += R;
  }
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(sum, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  if (R.determinant() < 0) {
    Eigen::Matrix3d U = svd.matrixU();
    U.col(2) *= -1.0;
    R = U * svd.matrixV().transpose();
  }
  return R;
}

// rotationAngle: angle of a rotation [deg], from its trace.
static double rotationAngle(const Eigen::Matrix3d &R) {
  const double c = std::max(-1.0, std::min(1.0, (R.trace() - 1.0) / 2.0));
  return std::acos(c) * 180.0 / M_PI;
}

// checkHandEye: board in BASE per photo, its mean, then spread and
// reprojection of the mean.
HandEyeCheck checkHandEye(const HandEyeData &data, const Eigen::Matrix4d &T_int_color) {
  HandEyeCheck check;
  const size_t n = data.names.size();
  if (n == 0) {
    return check;
  }
  std::vector<Eigen::Matrix4d> T_base_board(n);
  std::vector<Eigen::Matrix3d> rotations(n);
  std::vector<Eigen::Vector3d> centers(n);
  Eigen::Vector3d mean_origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_center = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < n; i++) {
    T_base_board[i] = data.T_base_int[i] * T_int_color * data.board[i].T_color_board;
    rotations[i] = T_base_board[i].block<3, 3>(0, 0);
    centers[i] = (T_base_board[i] * data.board_center.homogeneous()).head<3>();
    mean_origin += T_base_board[i].block<3, 1>(0, 3) / n;
    mean_center += centers[i] / n;
  }
  Eigen::Matrix4d T_mean = Eigen::Matrix4d::Identity();
  T_mean.block<3, 3>(0, 0) = meanRotation(rotations);
  T_mean.block<3, 1>(0, 3) = mean_origin;

  double sum_mm2 = 0.0, sum_deg2 = 0.0, sum_px2 = 0.0;
  size_t corners = 0;
  for (size_t i = 0; i < n; i++) {
    const double mm = (centers[i] - mean_center).norm() * 1000.0;
    sum_mm2 += mm * mm;
    if (mm > check.spread_max_mm) {
      check.spread_max_mm = mm;
      check.worst_view = data.names[i];
    }
    const double deg = rotationAngle(T_mean.block<3, 3>(0, 0).transpose() * rotations[i]);
    sum_deg2 += deg * deg;

    const Eigen::Matrix4d T_color_board =
        (data.T_base_int[i] * T_int_color).inverse() * T_mean;
    const double px = rmsPixels(projectBoard(data.board[i].board_points, T_color_board,
                                             data.color_intr[i]),
                                data.board[i].image_points);
    sum_px2 += px * px * data.board[i].board_points.size();
    corners += data.board[i].board_points.size();
  }
  check.spread_rms_mm = std::sqrt(sum_mm2 / n);
  check.rotation_rms_deg = std::sqrt(sum_deg2 / n);
  check.reprojection_px = corners > 0 ? std::sqrt(sum_px2 / corners) : 0.0;
  return check;
}

// motionDifference: |t_a - t_b| and the angle of R_a^T * R_b.
void motionDifference(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b, double &mm, double &deg) {
  mm = (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm() * 1000.0;
  deg = rotationAngle(a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0));
}

// writeSuggestedCfg: the same lines as cfg/camera_calibration.cfg, with
// depth_from_color_mm of the cfg in use (the factory distance is kept).
bool writeSuggestedCfg(const std::string &folder, const HandEyeResult &best,
                       const HandEyeCheck &check) {
  const std::string file_name = folder + "/camera_calibration_suggested.cfg";
  std::ofstream file(file_name);
  if (!file.good()) {
    printf("Could not write %s\n", file_name.c_str());
    return false;
  }
  const Eigen::Vector3d t_mm = best.T_int_color.block<3, 1>(0, 3) * 1000.0;
  const Eigen::Vector3d angles = anglesXYZ(best.T_int_color.block<3, 3>(0, 0));
  const Eigen::Vector3d depth_mm =
      (colorToInt().inverse() * depthToInt()).block<3, 1>(0, 3) * 1000.0;
  char date[32];
  const std::time_t now = std::time(nullptr);
  std::strftime(date, sizeof(date), "%Y-%m-%d", std::localtime(&now));

  char text[1024];
  snprintf(text, sizeof(text),
           "# Suggested by hand_eye_compare, %s: method %s, %d photos of %s/.\n"
           "# Board spread %.2f mm RMS (max %.2f mm), %.3f deg RMS, reprojection %.2f px.\n"
           "# Copy these lines into cfg/camera_calibration.cfg; there, comment the old\n"
           "# values out and write why they changed (see Calibration/README.md).\n"
           "color_position_mm = %.2f %.2f %.2f\n"
           "color_rotation_deg = %.3f %.3f %.3f\n"
           "depth_from_color_mm = %.2f %.2f %.2f\n",
           date, best.method.c_str(), best.views, folder.c_str(), check.spread_rms_mm,
           check.spread_max_mm, check.rotation_rms_deg, check.reprojection_px, t_mm.x(),
           t_mm.y(), t_mm.z(), angles.x(), angles.y(), angles.z(), depth_mm.x(), depth_mm.y(),
           depth_mm.z());
  file << text;
  printf("Saved: %s\n", file_name.c_str());
  return true;
}
