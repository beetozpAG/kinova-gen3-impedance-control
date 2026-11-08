// File:    hand_eye_tsai.h
// Module:  Calibration (block C3a: method of Tsai and Lenz)
// Purpose: Finds the camera mount X = T_int_color with the method of Tsai
//          and Lenz, from the photos of the board (hand_eye_data.h).
//
// Reference: R. Y. Tsai and R. K. Lenz, "A new technique for fully
// autonomous and efficient 3D robotics hand/eye calibration", IEEE
// Transactions on Robotics and Automation, 5(3):345-358, 1989.
// https://doi.org/10.1109/70.34770
//
// The idea, step by step. From each pair of photos (i, j):
//   A = motion of the wrist (robot),  B = motion of the camera (board),
//   and A * X = X * B (hand_eye_data.h).
// It is solved in two parts: first the rotation of X, then its position.
//
//   1. Rotation. Write each rotation as an axis and an angle. A and B are
//      the same motion seen from two places, so they turn the same ANGLE,
//      and X turns the axis of B into the axis of A:  axis_A = R_X * axis_B.
//      Tsai and Lenz write this with the "modified Rodrigues vector"
//      (axis * 2 sin(angle/2)), which makes it a linear equation:
//        skew(P_A + P_B) * P' = P_B - P_A
//      One pair gives 3 equations but only 2 are independent, so at least
//      2 pairs with DIFFERENT axes are needed. With many pairs: least
//      squares. From P' the rotation R_X is rebuilt.
//   2. Position. With R_X known, the position part of A X = X B is linear:
//        (R_A - I) * t_X = R_X * t_B - t_A
//      Again least squares over all pairs.
//
// Why the poses must be varied: in step 1 the axes of the motions must
// point in different directions; in step 2, R_A - I loses rank along the
// axis of R_A, so if all rotations are about the same axis, t_X along that
// axis is unknown (what happened with our sweeps: only wrist turns, height
// unknown).
// Weak point: the rotation is found first, and its error passes to the
// position.
#pragma once

#include "Calibration/hand_eye_data.h"

// handEyeTsai: X = T_int_color (cv::CALIB_HAND_EYE_TSAI).
HandEyeResult handEyeTsai(const HandEyeData &data);
