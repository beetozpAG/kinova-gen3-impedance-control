// File:    hand_eye_park.h
// Module:  Calibration (block C3b: method of Park and Martin)
// Purpose: Finds the camera mount X = T_int_color with the method of Park
//          and Martin, from the photos of the board (hand_eye_data.h).
//
// Reference: F. C. Park and B. J. Martin, "Robot sensor calibration:
// solving AX = XB on the Euclidean group", IEEE Transactions on Robotics
// and Automation, 10(5):717-721, 1994. https://doi.org/10.1109/70.326576
//
// The idea, step by step. Same equation A * X = X * B, and also in two
// parts (rotation, then position), but the rotation is found differently:
//
//   1. Rotation. Each rotation is written as its "logarithm": the vector
//      axis * angle (alpha for A, beta for B). Then A X = X B becomes
//        alpha = R_X * beta
//      for every pair: R_X is the rotation that best turns all the beta
//      vectors onto the alpha vectors. This has a closed solution (no
//      iterations):
//        M   = sum of  beta * alpha^T
//        R_X = (M^T M)^(-1/2) * M^T
//      This is the "best rotation between two sets of vectors" problem
//      (the same as aligning two point clouds by their directions).
//   2. Position. As in Tsai and Lenz: (R_A - I) * t_X = R_X * t_B - t_A,
//      least squares over all pairs.
//
// Difference with Tsai: the rotation is fitted with all the axes at once,
// with the geometry of rotations (no modified Rodrigues vector), so noise
// spreads more evenly. Same need for varied poses, and the same weak point:
// the rotation error passes to the position.
#pragma once

#include "Calibration/hand_eye_data.h"

// handEyePark: X = T_int_color (cv::CALIB_HAND_EYE_PARK).
HandEyeResult handEyePark(const HandEyeData &data);
