// File:    hand_eye_data.h
// Module:  Calibration (input of blocks C3, and their result files)
// Purpose: What the three hand-eye methods have in common:
//          - their input: for each photo of the board, where the wrist was
//            (T_base_int, from the robot) and where the board was seen
//            (T_color_board, block C2);
//          - the call to OpenCV (cv::calibrateHandEye) with the method;
//          - the result file each method writes and block C4 reads.
//
// The problem all methods solve ("AX = XB"). The board does not move, so for
// any two photos i, j:
//   T_base_int_i * X * T_color_board_i = T_base_int_j * X * T_color_board_j
// with X = T_int_color, the unknown mount of the camera. Moving the terms:
//   A * X = X * B,   A = T_base_int_j^-1 * T_base_int_i   (motion of the wrist)
//                    B = T_color_board_j * T_color_board_i^-1  (motion of the
//                                                               camera, seen
//                                                               on the board)
// The methods differ in HOW they find X from many pairs (A, B).
// Reference: OpenCV, cv::calibrateHandEye,
// https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html
#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>

#include "Calibration/board_pose.h"

// Fewest photos for a calibration. 3 is the mathematical minimum; with
// noise, 10 or more are needed for a result to the millimeter.
const int HAND_EYE_MIN_VIEWS = 10;

// Photos looked for: pose_0 to pose_99.
const int HAND_EYE_MAX_VIEWS = 100;

// Input of the methods.
struct HandEyeData {
  std::vector<std::string> names;              // pose_K
  std::vector<Eigen::Matrix4d> T_base_int;     // wrist, from the robot
  std::vector<BoardView> board;                // board, from the image
  std::vector<Intrinsics> color_intr;          // camera of each photo
  Eigen::Vector3d board_center;                // center of the board, BOARD frame [m]
};

// readHandEyeData: reads FOLDER/pose_K_pose.txt and FOLDER/pose_K_color.png
// (calib_capture), K = 0 to HAND_EYE_MAX_VIEWS - 1, and finds the board in
// each photo (boardPose). A missing K is skipped: to leave a bad photo out,
// move its files to another folder. Photos without the board, or with a
// reprojection error above BOARD_MAX_REPROJECTION_PX, are left out (and
// printed). ee_offset_z: TOOL ahead of INT along Z (intToBase, frames.h).
// Returns false (and prints why) if fewer than HAND_EYE_MIN_VIEWS remain.
bool readHandEyeData(const std::string &folder, double ee_offset_z, HandEyeData &data);

// solveHandEye: cv::calibrateHandEye with the method; X = T_int_color.
Eigen::Matrix4d solveHandEye(const HandEyeData &data, cv::HandEyeCalibrationMethod method);

// One result: method name and X.
struct HandEyeResult {
  std::string method;
  int views = 0;
  Eigen::Matrix4d T_int_color = Eigen::Matrix4d::Identity();
};

// handEyeFile: FOLDER/hand_eye_METHOD.txt
std::string handEyeFile(const std::string &folder, const std::string &method);

// writeHandEyeResult: writes the result file and prints it (position in mm,
// angles x-y-z in deg, and the difference with the manual, Table 92 p.225).
// Returns false (and prints why) if the file cannot be written.
bool writeHandEyeResult(const std::string &folder, const HandEyeResult &result);

// readHandEyeResult: reads a file of writeHandEyeResult. Returns false if
// it is missing or has no T_int_color line.
bool readHandEyeResult(const std::string &file_name, HandEyeResult &result);

// anglesXYZ: x-y-z extrinsic angles of R [deg] (R = Rz * Ry * Rx, the
// convention of tool_pose and cfg/camera_calibration.cfg).
Eigen::Vector3d anglesXYZ(const Eigen::Matrix3d &R);
