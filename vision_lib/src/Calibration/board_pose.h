// File:    board_pose.h
// Module:  Calibration (block C2: board in the image)
// Purpose: Finds the ChArUco board in one color image and gives its pose in
//          the COLOR frame (T_color_board): "where the board is, seen from the
//          camera". It is the input of the hand-eye methods (blocks C3).
//
// How:
//   C2a. Find the markers and, from them, the inner corners of the chessboard
//        with sub-pixel precision (cv::aruco::CharucoDetector).
//   C2b. Pose of the board from its corners: their position on the board is
//        known (square size), their pixel is measured, so the pose that
//        projects one onto the other is found (cv::solvePnP, perspective-n-
//        point; IPPE for a plane, then Levenberg-Marquardt refinement).
//   C2c. Reprojection error: the corners projected with that pose, against
//        the measured pixels. Large = blurred image, bent board or wrong
//        square size.
//   C2d. OpenCV frame -> COLOR frame. OpenCV: x to the right of the image,
//        y down. COLOR (frames.h, FLIP_XY in depth_camera.h): x and y the
//        other way. So T_color_cv = diag(-1, -1, 1).
//
// The camera parameters are the ones of the robot (factory): fx, fy, cx, cy,
// with no lens distortion (Intrinsics has none).
#pragma once

#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "0_Input/vision_module.h"
#include "Calibration/charuco_board.h"

// Fewest corners for a pose: 6 (of the 24 inner corners), so the board is
// not a thin line of points.
const int BOARD_MIN_CORNERS = 6;

// Largest reprojection error accepted for one image [px] (RMS).
const double BOARD_MAX_REPROJECTION_PX = 1.0;

// The board seen in one image.
struct BoardView {
  Eigen::Matrix4d T_color_board;           // board pose in COLOR [m]
  std::vector<cv::Point3f> board_points;   // corners in the BOARD frame [m]
  std::vector<cv::Point2f> image_points;   // the same corners in the image [px]
  double reprojection_px = 0.0;            // RMS error of C2c [px]
};

// boardPose: C2a to C2d. Returns false (and prints why) if fewer than
// BOARD_MIN_CORNERS corners are found. It does not check
// BOARD_MAX_REPROJECTION_PX: the caller decides.
bool boardPose(const cv::Mat &color_image, const Intrinsics &color_intr,
               const cv::aruco::CharucoBoard &board, BoardView &view);

// projectBoard: pixels of the board corners for a pose T_color_board
// (the pinhole model of C2b, in the COLOR frame).
std::vector<cv::Point2f> projectBoard(const std::vector<cv::Point3f> &board_points,
                                      const Eigen::Matrix4d &T_color_board,
                                      const Intrinsics &color_intr);

// rmsPixels: RMS distance between two lists of pixels of the same size [px].
double rmsPixels(const std::vector<cv::Point2f> &a, const std::vector<cv::Point2f> &b);
