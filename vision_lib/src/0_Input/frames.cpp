// File:    frames.cpp
// Module:  0_Input
// Purpose: Gives the transforms (4x4, Eigen) between the frames BASE, TOOL,
//          INT and DEPTH. See frames.h for each transform and its source.
#include "0_Input/frames.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

// toolToBase: rotation from the three angles, then the position.
Eigen::Matrix4d toolToBase(const ToolPose &pose) {
  const double deg2rad = M_PI / 180.0;

  // Rotation about the fixed axes x, then y, then z.
  const Eigen::Matrix3d R =
      (Eigen::AngleAxisd(pose.theta_z * deg2rad, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pose.theta_y * deg2rad, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(pose.theta_x * deg2rad, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = Eigen::Vector3d(pose.x, pose.y, pose.z);
  return T;
}

// intToBase: moves back ee_offset_z along the Z axis of TOOL.
Eigen::Matrix4d intToBase(const Eigen::Matrix4d &T_base_tool, double ee_offset_z) {
  Eigen::Matrix4d T_tool_int = Eigen::Matrix4d::Identity();
  T_tool_int(2, 3) = -ee_offset_z;
  return T_base_tool * T_tool_int;
}

// CameraMount: the values of CAMERA_CALIBRATION_FILE.
struct CameraMount {
  Eigen::Matrix4d T_int_color;
  Eigen::Vector3d depth_from_color;  // [m]
};

// readCameraMount: "key = 3 values" lines; any key missing -> the manual.
static CameraMount readCameraMount() {
  CameraMount mount{colorToIntManual(), Eigen::Vector3d(0.02750, 0.00961, 0.0)};
  std::ifstream file(CAMERA_CALIBRATION_FILE);
  Eigen::Vector3d position, angles, depth;
  int keys = 0;
  std::string line;
  while (file.good() && std::getline(file, line)) {
    const size_t equal_sign = line.find('=');
    if (line.empty() || line[0] == '#' || equal_sign == std::string::npos) {
      continue;
    }
    std::istringstream key_stream(line.substr(0, equal_sign));
    std::string key;
    key_stream >> key;
    std::istringstream values(line.substr(equal_sign + 1));
    Eigen::Vector3d v;
    values >> v.x() >> v.y() >> v.z();
    if (values.fail()) {
      continue;
    }
    if (key == "color_position_mm") {
      position = v / 1000.0;
      keys |= 1;
    } else if (key == "color_rotation_deg") {
      angles = v;
      keys |= 2;
    } else if (key == "depth_from_color_mm") {
      depth = v / 1000.0;
      keys |= 4;
    }
  }
  if (keys != 7) {
    printf("Camera mount: %s missing or incomplete, using the manual (Table 92, p.225)\n",
           CAMERA_CALIBRATION_FILE.c_str());
    return mount;
  }
  mount.T_int_color = toolToBase(ToolPose{position.x(), position.y(), position.z(), angles.x(),
                                          angles.y(), angles.z()});
  mount.depth_from_color = depth;
  return mount;
}

// cameraMount: read once (static), the first time it is needed.
static const CameraMount &cameraMount() {
  static const CameraMount mount = readCameraMount();
  return mount;
}

// depthToInt: T_int_color * translation depth_from_color.
Eigen::Matrix4d depthToInt() {
  Eigen::Matrix4d T_color_depth = Eigen::Matrix4d::Identity();
  T_color_depth.block<3, 1>(0, 3) = cameraMount().depth_from_color;
  return cameraMount().T_int_color * T_color_depth;
}

// colorToInt: the mount of the cfg.
Eigen::Matrix4d colorToInt() {
  return cameraMount().T_int_color;
}

// colorToIntManual: identity rotation plus the translation of Table 92, p.225.
Eigen::Matrix4d colorToIntManual() {
  Eigen::Matrix4d T_int_color = Eigen::Matrix4d::Identity();
  T_int_color.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.05639, -0.00305);
  return T_int_color;
}

// readPoseFile: reads the file line by line and keeps the values of the
// keys tool_pose and joint_angles_deg.
bool readPoseFile(const std::string &file_name, ToolPose &tool, double q_deg[7]) {
  std::ifstream file(file_name);
  if (!file.good()) {
    return false;
  }

  bool has_tool_pose = false;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    // "key = values": split at the "=".
    const size_t equal_sign = line.find('=');
    if (equal_sign == std::string::npos) {
      continue;
    }
    std::istringstream key_stream(line.substr(0, equal_sign));
    std::string key;
    key_stream >> key;
    std::istringstream values(line.substr(equal_sign + 1));

    if (key == "tool_pose") {
      values >> tool.x >> tool.y >> tool.z >> tool.theta_x >> tool.theta_y >> tool.theta_z;
      has_tool_pose = !values.fail();
    } else if (key == "joint_angles_deg") {
      for (int i = 0; i < 7; i++) {
        values >> q_deg[i];
      }
    }
  }
  return has_tool_pose;
}
