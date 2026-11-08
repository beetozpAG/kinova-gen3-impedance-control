// File:    compare_hand_eye.h
// Module:  Calibration (block C4: compare the methods)
// Purpose: Takes the results of the three methods (blocks C3a, C3b, C3c)
//          and the photos, and says how good each one is and how much they
//          agree. Writes the best one as a suggested cfg (block C5).
//
// Two checks:
//   C4a. Agreement: the difference between each pair of methods, in
//        position [mm] and rotation [deg]. They solve the same equation in
//        different ways: if the photos are good, they give almost the same X.
//   C4b. Validation with the photos. The board did not move, so with the
//        right X the board must be in the same place of BASE in every photo:
//          T_base_board_i = T_base_int_i * X * T_color_board_i
//        - spread: distance of the board center of each photo to their mean
//          [mm], and of the board rotation [deg];
//        - reprojection: the mean board pose seen from each photo with X,
//          projected on the image, against the corners found [px].
//        The same is done with the mount of the manual (Table 92, p.225),
//        so the gain of the calibration is seen.
// The method with the smallest spread is suggested.
#pragma once

#include <string>
#include <vector>

#include "Calibration/hand_eye_data.h"

// The methods agree if their positions differ less than this [mm] and
// their rotations less than this [deg].
const double AGREE_MM = 1.0;
const double AGREE_DEG = 0.3;

// A calibration is good if the board center spread (RMS) is below this
// [mm]. It also holds the error of the robot pose, so it is not zero.
const double SPREAD_OK_MM = 1.5;

// Result of C4b for one X.
struct HandEyeCheck {
  double spread_rms_mm = 0.0;   // board center: RMS distance to the mean
  double spread_max_mm = 0.0;   // board center: largest distance to the mean
  double rotation_rms_deg = 0.0;  // board rotation: RMS angle to the mean
  double reprojection_px = 0.0;   // RMS over all corners of all photos
  std::string worst_view;         // photo farthest from the mean
};

// checkHandEye: C4b for one X = T_int_color.
HandEyeCheck checkHandEye(const HandEyeData &data, const Eigen::Matrix4d &T_int_color);

// motionDifference: between two transforms, the distance of the positions
// [mm] and the angle of the relative rotation [deg].
void motionDifference(const Eigen::Matrix4d &a, const Eigen::Matrix4d &b, double &mm, double &deg);

// writeSuggestedCfg: FOLDER/camera_calibration_suggested.cfg, with the keys
// of cfg/camera_calibration.cfg and the result of the method (to copy by
// hand). Returns false (and prints why) if it cannot be written.
bool writeSuggestedCfg(const std::string &folder, const HandEyeResult &best,
                       const HandEyeCheck &check);
