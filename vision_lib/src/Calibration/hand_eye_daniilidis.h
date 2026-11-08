// File:    hand_eye_daniilidis.h
// Module:  Calibration (block C3c: method of Daniilidis)
// Purpose: Finds the camera mount X = T_int_color with the method of
//          Daniilidis, from the photos of the board (hand_eye_data.h).
//
// Reference: K. Daniilidis, "Hand-eye calibration using dual quaternions",
// The International Journal of Robotics Research, 18(3):286-298, 1999.
// https://doi.org/10.1177/02783649922066213
//
// The idea, step by step. Same equation A * X = X * B, but rotation and
// position are found TOGETHER, in one step:
//
//   1. Each motion (rotation + position) is written as one "dual
//      quaternion": 8 numbers, a quaternion for the rotation (q) and a
//      second one for the position (q'). Like a quaternion for rotations,
//      but for full motions.
//   2. With dual quaternions, A X = X B becomes one LINEAR equation for the
//      8 numbers of X, per pair of photos. All pairs are stacked in one
//      matrix T (6 rows per pair).
//   3. The solution is in the null space of T: its singular value
//      decomposition (SVD) gives the 2 vectors with the smallest singular
//      values; X is the combination of both that is a valid motion (unit
//      quaternion, q . q' = 0), found with a quadratic equation.
//
// Difference with Tsai and Park: the rotation error does not pass to the
// position, because both are fitted at the same time. Same need for varied
// poses (if all rotations have the same axis, T has more than 2 small
// singular values and X is not unique).
#pragma once

#include "Calibration/hand_eye_data.h"

// handEyeDaniilidis: X = T_int_color (cv::CALIB_HAND_EYE_DANIILIDIS).
HandEyeResult handEyeDaniilidis(const HandEyeData &data);
