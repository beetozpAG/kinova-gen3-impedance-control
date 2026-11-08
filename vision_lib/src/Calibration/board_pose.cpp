// File:    board_pose.cpp
// Module:  Calibration (block C2: board in the image)
// Purpose: Pose of the ChArUco board in one color image. See board_pose.h.
#include "Calibration/board_pose.h"

#include <cmath>
#include <cstdio>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "0_Input/depth_camera.h"

// T_color_cv: the OpenCV camera frame in the COLOR frame (C2d).
static Eigen::Matrix4d colorFromOpenCV() {
  const double sign = FLIP_XY ? -1.0 : 1.0;
  return Eigen::Vector4d(sign, sign, 1.0, 1.0).asDiagonal();
}

// cameraMatrix: K of the pinhole model, for OpenCV.
static cv::Mat cameraMatrix(const Intrinsics &intr) {
  return (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.cx, 0, intr.fy, intr.cy, 0, 0, 1);
}

// boardPose: detect, solvePnP (IPPE + LM), error, frame change.
bool boardPose(const cv::Mat &color_image, const Intrinsics &color_intr,
               const cv::aruco::CharucoBoard &board, BoardView &view) {
  // C2a. Markers, then the chessboard corners between them.
  cv::Mat gray;
  cv::cvtColor(color_image, gray, cv::COLOR_BGR2GRAY);
  const cv::aruco::CharucoDetector detector(board);
  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  detector.detectBoard(gray, corners, ids);
  if (static_cast<int>(ids.size()) < BOARD_MIN_CORNERS) {
    printf("  Board: %zu corners found (at least %d needed)\n", ids.size(), BOARD_MIN_CORNERS);
    return false;
  }
  board.matchImagePoints(corners, ids, view.board_points, view.image_points);

  // C2b. Pose of the board in the OpenCV camera frame.
  const cv::Mat K = cameraMatrix(color_intr);
  const cv::Mat no_distortion = cv::Mat::zeros(1, 5, CV_64F);
  cv::Mat rvec, tvec;
  cv::solvePnP(view.board_points, view.image_points, K, no_distortion, rvec, tvec, false,
               cv::SOLVEPNP_IPPE);
  cv::solvePnPRefineLM(view.board_points, view.image_points, K, no_distortion, rvec, tvec);

  cv::Mat R;
  cv::Rodrigues(rvec, R);
  Eigen::Matrix4d T_cv_board = Eigen::Matrix4d::Identity();
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) {
      T_cv_board(r, c) = R.at<double>(r, c);
    }
    T_cv_board(r, 3) = tvec.at<double>(r);
  }

  // C2d. In the COLOR frame.
  view.T_color_board = colorFromOpenCV() * T_cv_board;

  // C2c. Reprojection error.
  view.reprojection_px =
      rmsPixels(projectBoard(view.board_points, view.T_color_board, color_intr), view.image_points);
  return true;
}

// projectBoard: BOARD -> COLOR, then the pinhole with the signs of FLIP_XY
// (as in colorCloud, color_cloud.cpp).
std::vector<cv::Point2f> projectBoard(const std::vector<cv::Point3f> &board_points,
                                      const Eigen::Matrix4d &T_color_board,
                                      const Intrinsics &color_intr) {
  const double sign = FLIP_XY ? -1.0 : 1.0;
  std::vector<cv::Point2f> pixels;
  for (const cv::Point3f &b : board_points) {
    const Eigen::Vector3d p = (T_color_board * Eigen::Vector4d(b.x, b.y, b.z, 1.0)).head<3>();
    pixels.emplace_back(static_cast<float>(color_intr.cx + sign * color_intr.fx * p.x() / p.z()),
                        static_cast<float>(color_intr.cy + sign * color_intr.fy * p.y() / p.z()));
  }
  return pixels;
}

// rmsPixels: square root of the mean squared distance.
double rmsPixels(const std::vector<cv::Point2f> &a, const std::vector<cv::Point2f> &b) {
  if (a.empty() || a.size() != b.size()) {
    return 0.0;
  }
  double sum = 0.0;
  for (size_t i = 0; i < a.size(); i++) {
    const cv::Point2f d = a[i] - b[i];
    sum += d.x * d.x + d.y * d.y;
  }
  return std::sqrt(sum / a.size());
}
