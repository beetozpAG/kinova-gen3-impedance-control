// File:    frames.h
// Module:  0_Input
// Purpose: Gives the transforms (4x4, Eigen) between the frames BASE, TOOL,
//          INT, DEPTH and COLOR. With them, a grasp that GPD finds in the point
//          cloud (DEPTH frame) can be written in the BASE frame for the robot.
//          No robot connection and no forward kinematics: it uses the TOOL
//          pose that the robot already measured.
//
// Frames:
//   BASE:  robot base.
//   TOOL:  tool frame of the 2F-85, measured by the robot.
//   INT:   interface module (wrist), 0.120 m behind TOOL along Z.
//   DEPTH: depth sensor. The point cloud and the GPD grasps are in this frame.
//   COLOR: color sensor. The color image is in this frame.
//
// The DEPTH frame with respect to the BASE frame:
//   T_base_depth = T_base_tool * T_tool_int * T_int_depth
//   p_base = T_base_depth * p_depth
//
// Function names: xToY gives T_y_x, the transform that takes a point in the
// X frame to the Y frame (e.g. depthToInt gives T_int_depth).
//
// Why the TOOL pose measured by the robot and not Gen3_model.h: the robot
// uses its own factory calibration. On our robot the nominal model differs
// from it by about 5 to 9 mm and 1 deg (print_frames, 2026-10-13).
#pragma once

#include <string>

#include <Eigen/Dense>

// TOOL pose with the same values as the Kinova web app: position in meters,
// orientation in degrees (x-y-z extrinsic).
struct ToolPose {
  double x, y, z;
  double theta_x, theta_y, theta_z;
};

// toolToBase: the TOOL frame with respect to the BASE frame (T_base_tool),
// from the TOOL pose.
// Rotation x-y-z extrinsic: R = Rz(theta_z) * Ry(theta_y) * Rx(theta_x)
// (user guide r12.0, p.116).
Eigen::Matrix4d toolToBase(const ToolPose &pose);

// intToBase: the INT frame with respect to the BASE frame (T_base_int),
// from T_base_tool. TOOL is ee_offset_z ahead of INT along Z, with no
// rotation (Table 28, p.92), so
//   T_base_int = T_base_tool * T_tool_int,  T_tool_int = translation of -ee_offset_z along Z.
Eigen::Matrix4d intToBase(const Eigen::Matrix4d &T_base_tool, double ee_offset_z);

// depthToInt: the DEPTH frame with respect to the INT frame (T_int_depth):
//   T_int_depth = T_int_color * T_color_depth
// T_int_color: colorToInt. T_color_depth: translation depth_from_color_mm
// of CAMERA_CALIBRATION_FILE (both sensors are on the same board of the
// vision module, so only the mount of the module is calibrated).
// With the values of the manual (user guide r12.0, Table 92, p.225):
//
//                 | 1  0  0   0.02750 |
//   T_int_depth = | 0  1  0   0.06600 |
//                 | 0  0  1  -0.00305 |
//                 | 0  0  0   1       |
//
// R = identity: DEPTH has the same orientation as INT.
// t = origin of DEPTH (center of the left imager) seen from INT
//     (Figure 84, p.225): x = 27.50 mm, y = 66.00 mm, z = -3.05 mm.
// Point:     p_int = R * p_depth + t = p_depth + t
// Direction: d_int = R * d_depth = d_depth (e.g. the GPD approach)
// After a calibration (Calibration/README.md) R is no longer exactly the
// identity: use the whole matrix.
Eigen::Matrix4d depthToInt();

// colorToInt: the COLOR frame (color sensor) with respect to the INT frame
// (T_int_color), from color_position_mm and color_rotation_deg (x-y-z, as
// tool_pose) of CAMERA_CALIBRATION_FILE. The manual gives (user guide r12.0,
// Table 92, p.225; colorToIntManual):
//
//                 | 1  0  0   0       |
//   T_int_color = | 0  1  0   0.05639 |
//                 | 0  0  1  -0.00305 |
//                 | 0  0  0   1       |
//
// R = identity: COLOR has the same orientation as INT (and as DEPTH).
// t = origin of COLOR seen from INT (p.225): x = 0 mm, y = 56.39 mm,
//     z = -3.05 mm.
// So COLOR and DEPTH differ only by a translation, computed from the two
// matrices of Table 92 (it is not a value of the table):
//   T_color_depth = colorToInt().inverse() * depthToInt()
//                 = translation t_DEPTH - t_COLOR
//                 = (27.50 - 0, 66.00 - 56.39, -3.05 + 3.05) mm
//                 = (27.50, 9.61, 0) mm.
// This stays a translation after a calibration (depthToInt).
Eigen::Matrix4d colorToInt();

// colorToIntManual: T_int_color of the manual (Table 92, p.225), to compare
// a calibration with it.
Eigen::Matrix4d colorToIntManual();

// The camera mount in use: COLOR in INT and DEPTH in COLOR. Read once, the
// first time colorToInt or depthToInt is called. If the file is missing or
// a key is wrong, the values of the manual are used (and a line says so).
const std::string CAMERA_CALIBRATION_FILE = "../cfg/camera_calibration.cfg";

// readPoseFile: reads the TOOL pose and the joint angles from a NAME_pose.txt
// written by saveCapture (depth_camera.h). Lines used:
//   tool_pose = x y z theta_x theta_y theta_z
//   joint_angles_deg = q1 q2 q3 q4 q5 q6 q7
// Lines starting with "#" are comments.
// Returns false if the file cannot be opened or has no valid tool_pose line.
bool readPoseFile(const std::string &file_name, ToolPose &tool, double q_deg[7]);
