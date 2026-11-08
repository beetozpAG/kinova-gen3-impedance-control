// File:    hand_eye_data.cpp
// Module:  Calibration (input of blocks C3, and their result files)
// Purpose: Input, OpenCV call and result files of the hand-eye methods.
//          See hand_eye_data.h.
#include "Calibration/hand_eye_data.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <opencv2/core/eigen.hpp>

#include "0_Input/capture_file.h"
#include "0_Input/frames.h"

// readHandEyeData: pose_0 to pose_(HAND_EYE_MAX_VIEWS - 1); missing ones are skipped.
bool readHandEyeData(const std::string &folder, double ee_offset_z, HandEyeData &data) {
  BoardSize size;
  if (!readBoardSize(BOARD_FILE, size)) {
    return false;
  }
  const cv::aruco::CharucoBoard board = makeBoard(size);
  printf("Board: square %.2f mm, marker %.2f mm (%s)\n", size.square * 1000.0,
         size.marker * 1000.0, BOARD_FILE.c_str());

  data = HandEyeData();
  data.board_center = Eigen::Vector3d(BOARD_COLS * size.square / 2.0, BOARD_ROWS * size.square / 2.0, 0.0);
  for (int k = 0; k < HAND_EYE_MAX_VIEWS; k++) {
    const std::string name = "pose_" + std::to_string(k);
    const std::string pose_file = folder + "/" + name + "_pose.txt";
    if (!std::filesystem::exists(pose_file)) {
      continue;
    }
    ToolPose tool;
    double q_deg[7];
    Intrinsics intr;
    cv::Mat image;
    if (!readPoseFile(pose_file, tool, q_deg) || !readColorCapture(pose_file, intr, image)) {
      printf("  %s: no tool_pose or no color image, left out\n", name.c_str());
      continue;
    }
    BoardView view;
    if (!boardPose(image, intr, board, view)) {
      printf("  %s: left out\n", name.c_str());
      continue;
    }
    if (view.reprojection_px > BOARD_MAX_REPROJECTION_PX) {
      printf("  %s: reprojection %.2f px (more than %.1f), left out\n", name.c_str(),
             view.reprojection_px, BOARD_MAX_REPROJECTION_PX);
      continue;
    }
    printf("  %s: %zu corners, reprojection %.2f px, board at %.0f mm\n", name.c_str(),
           view.board_points.size(), view.reprojection_px,
           view.T_color_board.block<3, 1>(0, 3).norm() * 1000.0);
    data.names.push_back(name);
    data.T_base_int.push_back(intToBase(toolToBase(tool), ee_offset_z));
    data.board.push_back(view);
    data.color_intr.push_back(intr);
  }
  if (static_cast<int>(data.names.size()) < HAND_EYE_MIN_VIEWS) {
    printf("Only %zu photos with the board in %s/ (at least %d needed)\n", data.names.size(),
           folder.c_str(), HAND_EYE_MIN_VIEWS);
    return false;
  }
  return true;
}

// solveHandEye: OpenCV names: gripper = INT, cam = COLOR, target = BOARD.
Eigen::Matrix4d solveHandEye(const HandEyeData &data, cv::HandEyeCalibrationMethod method) {
  std::vector<cv::Mat> R_gripper2base, t_gripper2base, R_target2cam, t_target2cam;
  // One cv::Mat per value: a cv::Mat copy shares its data, so reusing one
  // would overwrite the values already in the lists.
  for (size_t i = 0; i < data.names.size(); i++) {
    cv::Mat R_wrist, t_wrist, R_board, t_board;
    cv::eigen2cv(Eigen::Matrix3d(data.T_base_int[i].block<3, 3>(0, 0)), R_wrist);
    cv::eigen2cv(Eigen::Vector3d(data.T_base_int[i].block<3, 1>(0, 3)), t_wrist);
    cv::eigen2cv(Eigen::Matrix3d(data.board[i].T_color_board.block<3, 3>(0, 0)), R_board);
    cv::eigen2cv(Eigen::Vector3d(data.board[i].T_color_board.block<3, 1>(0, 3)), t_board);
    R_gripper2base.push_back(R_wrist);
    t_gripper2base.push_back(t_wrist);
    R_target2cam.push_back(R_board);
    t_target2cam.push_back(t_board);
  }
  cv::Mat R_cam2gripper, t_cam2gripper;
  cv::calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, R_cam2gripper,
                       t_cam2gripper, method);

  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(R_cam2gripper, R);
  cv::cv2eigen(t_cam2gripper, t);
  Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
  X.block<3, 3>(0, 0) = R;
  X.block<3, 1>(0, 3) = t;
  return X;
}

std::string handEyeFile(const std::string &folder, const std::string &method) {
  return folder + "/hand_eye_" + method + ".txt";
}

// anglesXYZ: inverse of R = Rz(tz) * Ry(ty) * Rx(tx).
Eigen::Vector3d anglesXYZ(const Eigen::Matrix3d &R) {
  const double rad2deg = 180.0 / M_PI;
  return Eigen::Vector3d(std::atan2(R(2, 1), R(2, 2)),
                         std::asin(-std::max(-1.0, std::min(1.0, R(2, 0)))),
                         std::atan2(R(1, 0), R(0, 0))) *
         rad2deg;
}

// writeHandEyeResult: "key = values" lines, as the pose files.
bool writeHandEyeResult(const std::string &folder, const HandEyeResult &result) {
  const std::string file_name = handEyeFile(folder, result.method);
  std::ofstream file(file_name);
  if (!file.good()) {
    printf("Could not write %s\n", file_name.c_str());
    return false;
  }
  const Eigen::Vector3d t_mm = result.T_int_color.block<3, 1>(0, 3) * 1000.0;
  const Eigen::Vector3d angles = anglesXYZ(result.T_int_color.block<3, 3>(0, 0));
  const Eigen::Vector3d manual_mm = colorToIntManual().block<3, 1>(0, 3) * 1000.0;

  file << "# Hand-eye calibration, method " << result.method << ", " << result.views
       << " photos of " << folder << "/\n"
       << "# COLOR with respect to INT (T_int_color). Same keys as cfg/camera_calibration.cfg.\n"
       << "method = " << result.method << "\n"
       << "views = " << result.views << "\n";
  char line[160];
  snprintf(line, sizeof(line), "color_position_mm = %.2f %.2f %.2f\n", t_mm.x(), t_mm.y(),
           t_mm.z());
  file << line;
  snprintf(line, sizeof(line), "color_rotation_deg = %.3f %.3f %.3f\n", angles.x(), angles.y(),
           angles.z());
  file << line;
  file << "T_int_color =";
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      snprintf(line, sizeof(line), " %.9f", result.T_int_color(r, c));
      file << line;
    }
  }
  file << "\n";

  printf("\n========== %s (%d photos) ==========\n", result.method.c_str(), result.views);
  printf("  COLOR in INT:  x = %.2f  y = %.2f  z = %.2f [mm]   rx = %.3f  ry = %.3f  rz = %.3f "
         "[deg]\n",
         t_mm.x(), t_mm.y(), t_mm.z(), angles.x(), angles.y(), angles.z());
  printf("  Manual (Table 92, p.225): x = %.2f  y = %.2f  z = %.2f [mm], no rotation\n",
         manual_mm.x(), manual_mm.y(), manual_mm.z());
  printf("  Difference:    dx = %+.2f  dy = %+.2f  dz = %+.2f [mm]\n", t_mm.x() - manual_mm.x(),
         t_mm.y() - manual_mm.y(), t_mm.z() - manual_mm.z());
  printf("Saved: %s\n", file_name.c_str());
  return true;
}

// readHandEyeResult: keys method, views and T_int_color (16 numbers).
bool readHandEyeResult(const std::string &file_name, HandEyeResult &result) {
  std::ifstream file(file_name);
  if (!file.good()) {
    return false;
  }
  bool has_matrix = false;
  std::string line;
  while (std::getline(file, line)) {
    const size_t equal_sign = line.find('=');
    if (line.empty() || line[0] == '#' || equal_sign == std::string::npos) {
      continue;
    }
    std::istringstream key_stream(line.substr(0, equal_sign));
    std::string key;
    key_stream >> key;
    std::istringstream values(line.substr(equal_sign + 1));
    if (key == "method") {
      values >> result.method;
    } else if (key == "views") {
      values >> result.views;
    } else if (key == "T_int_color") {
      for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
          values >> result.T_int_color(r, c);
        }
      }
      has_matrix = !values.fail();
    }
  }
  return has_matrix;
}
