// File:    sweep_views.h
// Module:  Task (block 3: views)
// Purpose: The EE poses of the views of the sweep: from the pose of block 2
//          (color camera above the object, looking down), the wrist turns
//          about the Z axis of the EE in SWEEP_VIEWS stops from 0 to
//          SWEEP_TOTAL_DEG. The robot stops at each one; our code only
//          prints the poses, the controller moves the robot.
//
// Why: the depth camera is a stereo pair (two IR imagers about 5 cm apart).
// Next to an edge there is a band that only one imager sees: no depth there
// (stereo shadow), always on the same side. At 180 deg the imagers swap
// sides, so the band falls on the other side and the views fill each
// other (block 4). 90 deg adds the shadows along the other direction.
//
// The object stays in the image: the wrist turns about the EE axis, and the
// center of the color image is only about 2.8 cm from it at 0.30 m (the
// principal point is high in the image, cy = 238 of 720). The part always
// seen while turning is a circle of about 5.5 cm around the EE axis: a bolt
// head is always whole; the ends of a 16 cm plate leave the image at some
// angles (view 0, where the samples come from, sees it whole).
//
// Sub-blocks:
//   3a. Pose of view k (sweepViewPose).
//   3b. Capture at each stop (robot still, depth filter): sweep_capture.
//   3c. Check the turn of each view (wristTurn).
#pragma once

#include "0_Input/frames.h"

// Number of views, from 0 to SWEEP_TOTAL_DEG: 0, 90 and 180 deg.
const int SWEEP_VIEWS = 3;
const double SWEEP_TOTAL_DEG = 180.0;

// A view is accepted if the wrist turned this close to its angle [deg].
const double SWEEP_TOLERANCE_DEG = 3.0;

// sweepViewAngle: turn of view k from view 0 [deg].
double sweepViewAngle(int k);

// sweepViewPose: the EE pose of view k: the same position as start, turned
// sweepViewAngle(k) about the Z axis of the EE (R_k = R_start * Rz(angle)).
// The angles are written close to those of start (as the robot shows them).
ToolPose sweepViewPose(const ToolPose &start, int k);

// wristTurn: how much the EE turned about its own Z axis from start to now
// [deg, -180 to 180].
double wristTurn(const ToolPose &start, const ToolPose &now);
