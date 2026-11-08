// File:    calib_poses.h
// Module:  Calibration (block C1: poses of the robot)
// Purpose: The EE poses where the robot stops to photograph the board
//          (calib_capture). All of them look at the center of the board,
//          from different heights, tilts and wrist turns.
//
// Why so many different poses: the hand-eye methods find the camera mount
// from how the board seems to move when the camera moves. An error of the
// mount only shows up when the camera ROTATES between two poses, and only
// in the directions perpendicular to that rotation:
//   - turning only the wrist (rotations about Z) leaves the height of the
//     camera unknown (our sweeps, 2026-10-24);
//   - tilting about X and about Y gives the rest.
// Tsai and Lenz (1989) advise large angles between the rotation axes of the
// poses, and a camera close to the board.
//
// Each pose: the color camera at DISTANCE from the board center, along the
// ray of the image center (so the board is centered in the image even with
// the principal point high, cy = 238 of 720), tilted TILT from vertical
// toward the direction AZIMUTH (seen from above, in BASE), and the wrist
// turned TURN from the first pose.
#pragma once

#include <vector>

#include <Eigen/Dense>

#include "0_Input/frames.h"
#include "0_Input/vision_module.h"

// One pose of the list (angles in degrees, distance in meters).
struct CalibPose {
  double distance;  // color camera to the board center
  double tilt;      // from vertical
  double azimuth;   // direction of the tilt, about Z of BASE
  double turn;      // wrist turn from pose 0, about the camera axis
};

// The list: pose 0 is where the user leaves the robot (camera about 0.40 m
// above the board, looking down); the others are computed from it.
// 15 poses: 3 straight down, 4 at 20 deg, 4 at 30 deg, 4 at 25 deg farther.
const std::vector<CalibPose> CALIB_POSES = {
    {0.40, 0, 0, 0},     {0.35, 0, 0, 45},     {0.48, 0, 0, -45},
    {0.40, 20, 0, 0},    {0.40, 20, 90, 30},   {0.40, 20, 180, -30},  {0.40, 20, 270, 60},
    {0.38, 30, 45, -60}, {0.38, 30, 135, 0},   {0.38, 30, 225, 45},   {0.38, 30, 315, -20},
    {0.48, 25, 0, 90},   {0.48, 25, 90, -90},  {0.48, 25, 180, 20},   {0.48, 25, 270, -45},
};

// calibPoseTarget: the EE pose of CALIB_POSES[k], from pose 0 (start, the
// TOOL pose measured there) and the board center in BASE (seen at pose 0).
// ee_offset_z: TOOL ahead of INT along Z (as in intToBase, frames.h).
// The x-y-z angles closest to the ones of start are chosen (as
// sweepViewPose, Task/sweep_views.h), so the controller does not turn the
// wrist the long way round.
ToolPose calibPoseTarget(const ToolPose &start, const Eigen::Vector3d &board_center,
                         const Intrinsics &color_intr, double ee_offset_z, int k);
