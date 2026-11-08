// grasp_to_ee: convert a GPD grasp (DEPTH frame) to the pose of the
// end effector (EE) frame in the BASE frame.
//
// Frames (Kinova Gen3 user guide r12.0):
//   DEPTH: depth camera frame. The point cloud and the GPD grasps are in it.
//   INT:   interface module frame (end of the arm). DEPTH -> INT: Table 92, p.225.
//   EE:    Robotiq 2F-85 end effector frame ("TOOL" in the manual)
//          = INT + 0.120 m along Z_INT, no rotation (Table 28, p.92).
//   BASE:  robot base frame. INT -> BASE: forward kinematics (Table 90, p.221-222).
//
// The result is the EE pose that the robot must reach to make the grasp:
// the same values as the Kinova web app (meters, degrees x-y-z extrinsic).
#pragma once

#include <Eigen/Dense>

#include <gpd/candidate/hand.h>

// Distance from INT to the finger tips of the open 2F-85, along Z_INT [m].
// Source: 2F-85_2F-140_Instruction_Manual.pdf, Fig. 5-1 (opened), p.64: 149.3 mm.
// TODO: verify on the robot that 149.3 mm is measured from the INT face
// (and not from the gripper coupling).
const double FINGERTIP_FROM_INT = 0.1493;

// Maximum opening of the 2F-85, between the inner faces of the pads [m].
// Source: 2F-85_2F-140_Instruction_Manual.pdf, Fig. 5-1, p.64 (85 mm).
// The gripper position goes from 0 (open, 85 mm) to 255 (closed) with a
// "quasi-linear" relationship (same manual, section 4.3.4, p.34), so the
// opening in % is approximately width / GRIPPER_MAX_OPENING.
const double GRIPPER_MAX_OPENING = 0.085;

// How much the finger tips move forward (along the approach) when the 2F-85
// closes from fully open to a given opening [m].
// Why: the pads move on an arc (parallel links), so closing also pushes the
// tips forward. GPD checks the OPEN hand only, so without this the tips go
// below the planned depth when the gripper closes (they hit the support).
// Source: measured on our gripper, 2026-10-14, with a vernier caliper. Each
// step was measured from the previous one; the values below are the sums:
//   opening 85 -> 64.1 mm: +6.7   64.1 -> 42.3: +4.6   42.3 -> 22.07: +1.6
//   22.07 -> 0: +0.7   total 13.6 mm.
// Check: manual total = 162.8 (closed) - 149.3 (open) = 13.5 mm
// (2F-85_2F-140_Instruction_Manual.pdf, Fig. 5-1 p.64, Fig. 5-2 p.65).
// Between the measured points: linear interpolation (fingertipAdvance).
const int TIP_ADVANCE_POINTS = 5;
const double TIP_ADVANCE_OPENING[TIP_ADVANCE_POINTS] = {0.085, 0.0641, 0.0423, 0.02207, 0.0};
const double TIP_ADVANCE[TIP_ADVANCE_POINTS] = {0.0, 0.0067, 0.0113, 0.0129, 0.0136};

// Finger tip advance [m] when the gripper closes until its opening is
// "opening" [m] (the object width). Clamped to the measured range.
double fingertipAdvance(double opening);

// Pre-grasp distance [m]: the robot first goes to the grasp pose moved back
// this much along the approach (Z_EE), with the same orientation, and then
// moves straight to the grasp pose. So the fingers come to the object along
// the approach and do not sweep the object or the table on the way.
// (chosen) 10 cm: more than the pad length (37.5 mm, hand geometry .cfg), so
// the open fingers start fully above the object.
const double PREGRASP_DISTANCE = 0.10;

// EE pose in the BASE frame.
struct EEPose {
  Eigen::Vector3d position;  // EE origin [m]
  Eigen::Vector3d pregrasp_position;  // EE origin before the grasp [m] (same rotation)
  double tip_advance;  // finger tip advance when closing, compensated [m]
  Eigen::Matrix3d rotation;  // columns = X, Y, Z axes of the EE frame
  double theta_x, theta_y, theta_z;  // Euler angles x-y-z extrinsic [deg]
  double palm_depth_from_int;  // INT -> base of the GPD fingers, used here [m]
};

// Euler angles x-y-z extrinsic [deg] of a rotation, R = Rz * Ry * Rx
// (user guide r12.0, p.116). Returns (theta_x, theta_y, theta_z), with
// theta_y in [-90, 90].
Eigen::Vector3d eulerXYZFromRotation(const Eigen::Matrix3d &R);

// Convert one GPD grasp to the EE pose in BASE.
//   grasp:          GPD grasp in the DEPTH frame (its axes).
//   grasp_position: base of the fingers, DEPTH frame: grasp.getPosition(),
//                   or that position centered on the object (center_grasp.h).
//   hand_depth:     GPD finger length [m] (hand_depth in the hand geometry .cfg).
//   T_base_int:     INT pose in BASE when the point cloud was taken: from the
//                   TOOL pose measured by the robot (intToBase, frames.h), or
//                   from the joint angles (GetTransform(q), Gen3_model.h).
//   R_base_ee_now:  EE orientation in BASE at that moment (same source),
//                   used to choose between the two symmetric orientations.
//   angles_now_deg: EE Euler angles at that moment, as the robot shows them
//                   (theta_x, theta_y, theta_z). The result is written with
//                   the angles closest to these (grasp_to_ee.cpp, step 6).
//   ee_offset_z:    INT -> EE distance along Z_INT [m] (TOOL_OFFSET_Z_2F85 = 0.120).
EEPose graspToEE(const gpd::candidate::Hand &grasp, const Eigen::Vector3d &grasp_position,
                 double hand_depth,
                 const Eigen::Matrix4d &T_base_int,
                 const Eigen::Matrix3d &R_base_ee_now,
                 const Eigen::Vector3d &angles_now_deg, double ee_offset_z);
