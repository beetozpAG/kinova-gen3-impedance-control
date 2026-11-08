// grasp_to_ee: GPD grasp (DEPTH frame) -> EE pose in the BASE frame.
// Based on KinovaOpenCVCPP2026/PointCloudGPD_test.cpp:361-517 (same steps),
// with the palm depth computed from hand_depth instead of a fixed estimate.
#include "6_SelectGrasp/grasp_to_ee.h"

#include <cmath>

#include "0_Input/frames.h"

// TODO: verify on the robot (move the EE along X in the web app and look
// at the fingers).
// true  -> the 2F-85 fingers close along X_EE
// false -> the 2F-85 fingers close along Y_EE
const bool FINGERS_CLOSE_ALONG_X = true;

// Angle in degrees, wrapped to [-180, 180].
static double wrapAngle(double angle_deg) {
  return std::remainder(angle_deg, 360.0);
}

// How far apart two sets of Euler angles are [deg]: sum of the wrapped
// differences squared (179 and -179 are 2 deg apart, not 358).
static double angleDistance(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
  double sum = 0.0;
  for (int i = 0; i < 3; i++) {
    const double d = wrapAngle(a(i) - b(i));
    sum += d * d;
  }
  return sum;
}

Eigen::Vector3d eulerXYZFromRotation(const Eigen::Matrix3d &R) {
  // From writing out R = Rz(theta_z) * Ry(theta_y) * Rx(theta_x)
  // (user guide r12.0, p.116, also p.155, p.175):
  //   R(2,1) / R(2,2) = tan(theta_x)
  //   R(2,0) = -sin(theta_y),  sqrt(R(0,0)^2 + R(1,0)^2) = cos(theta_y)
  //   R(1,0) / R(0,0) = tan(theta_z)
  const double rad2deg = 180.0 / M_PI;
  return Eigen::Vector3d(
      std::atan2(R(2, 1), R(2, 2)) * rad2deg,
      std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0))) * rad2deg,
      std::atan2(R(1, 0), R(0, 0)) * rad2deg);
}

double fingertipAdvance(double opening) {
  // Openings go from large (index 0, open) to small (last, closed).
  if (opening >= TIP_ADVANCE_OPENING[0]) {
    return TIP_ADVANCE[0];
  }
  for (int i = 1; i < TIP_ADVANCE_POINTS; i++) {
    if (opening >= TIP_ADVANCE_OPENING[i]) {
      const double t = (TIP_ADVANCE_OPENING[i - 1] - opening) /
                       (TIP_ADVANCE_OPENING[i - 1] - TIP_ADVANCE_OPENING[i]);
      return TIP_ADVANCE[i - 1] + t * (TIP_ADVANCE[i] - TIP_ADVANCE[i - 1]);
    }
  }
  return TIP_ADVANCE[TIP_ADVANCE_POINTS - 1];
}

EEPose graspToEE(const gpd::candidate::Hand &grasp, const Eigen::Vector3d &grasp_position,
                 double hand_depth,
                 const Eigen::Matrix4d &T_base_int,
                 const Eigen::Matrix3d &R_base_ee_now,
                 const Eigen::Vector3d &angles_now_deg, double ee_offset_z) {
  EEPose ee;

  // GPD grasp, DEPTH frame (see third_party/gpd/readme/hand_frame.png):
  //   position: base of the fingers, at the center between them
  //             (third_party/gpd/src/gpd/candidate/hand.cpp:41-44); given
  //             as grasp_position (maybe centered on the object).
  //   approach: direction in which the hand moves to the object.
  //   binormal: direction in which the fingers close.
  //   axis:     approach x binormal.
  const Eigen::Vector3d approach = grasp.getApproach();
  const Eigen::Vector3d binormal = grasp.getBinormal();
  const Eigen::Vector3d hand_axis = grasp.getAxis();

  // ---------- 1. EE axes in the DEPTH frame ----------
  // Z_EE points out of the gripper, towards the object = approach.
  // The fingers close along X_EE or Y_EE (FINGERS_CLOSE_ALONG_X).
  // Both options are right-handed (X x Y = Z), because
  // (approach, binormal, axis) is right-handed in GPD.
  Eigen::Matrix3d R_depth_ee;
  if (FINGERS_CLOSE_ALONG_X) {
    R_depth_ee << binormal, hand_axis, approach;  // X = binormal, Y = axis
  } else {
    R_depth_ee << -hand_axis, binormal, approach;  // X = -axis, Y = binormal
  }

  // ---------- 2. EE origin in the DEPTH frame ----------
  // Along Z_INT (= approach), from the INT face:
  //   finger tips:          FINGERTIP_FROM_INT (0.1493 m)
  //   base of GPD fingers:  FINGERTIP_FROM_INT - hand_depth  (GPD fingers end at the tips)
  //   EE origin:            ee_offset_z (0.120 m)
  // So the EE origin is (ee_offset_z - palm_depth_from_int) after the
  // GPD position, along approach:
  //   hand_depth 0.038 -> palm 0.1113 m -> +0.0087 m
  //   hand_depth 0.061 -> palm 0.0883 m -> +0.0317 m
  ee.palm_depth_from_int = FINGERTIP_FROM_INT - hand_depth;
  // The gripper closes until its opening is the object width (GPD grasp
  // width). On the way the tips move forward by tip_advance, so the EE goes
  // that much back along the approach: when the gripper is closed, the tips
  // are where GPD planned them (init_bite into the object), not lower.
  ee.tip_advance = fingertipAdvance(grasp.getGraspWidth());
  const Eigen::Vector3d ee_position_depth =
      grasp_position + (ee_offset_z - ee.palm_depth_from_int - ee.tip_advance) * approach;

  // ---------- 3. DEPTH -> INT ----------
  // With the manual, only a translation: DEPTH and INT axes are parallel
  // (Table 92, p.225). After a calibration (cfg/camera_calibration.cfg)
  // there is a small rotation: R_int_ee = R_int_depth * R_depth_ee.
  const Eigen::Matrix4d T_int_depth = depthToInt();
  const Eigen::Vector3d ee_position_int = (T_int_depth * ee_position_depth.homogeneous()).head<3>();

  // ---------- 4. INT -> BASE ----------
  //   p_base = R_base_int * p_int + t_base_int
  //   R_base_ee = R_base_int * R_int_depth * R_depth_ee
  const Eigen::Matrix3d R_base_int = T_base_int.block<3, 3>(0, 0);
  const Eigen::Vector3d t_base_int = T_base_int.block<3, 1>(0, 3);
  ee.position = R_base_int * ee_position_int + t_base_int;
  Eigen::Matrix3d R_base_ee = R_base_int * T_int_depth.block<3, 3>(0, 0) * R_depth_ee;

  // ---------- 5. Choose between the two symmetric orientations ----------
  // The 2F-85 is symmetric: the same grasp rotated 180 deg about Z_EE
  // (X and Y change sign) is also valid. Keep the one closer to the current
  // EE orientation, so the wrist turns less.
  // trace(R_now^T * R) = 1 + 2 cos(angle between R_now and R):
  // the larger the trace, the smaller the rotation.
  const Eigen::Matrix3d R_flip_z = Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal();
  const Eigen::Matrix3d R_base_ee_flipped = R_base_ee * R_flip_z;
  if ((R_base_ee_now.transpose() * R_base_ee_flipped).trace() >
      (R_base_ee_now.transpose() * R_base_ee).trace()) {
    R_base_ee = R_base_ee_flipped;
  }
  ee.rotation = R_base_ee;

  // Pre-grasp: PREGRASP_DISTANCE back along the approach (Z_EE = column 2).
  ee.pregrasp_position = ee.position - PREGRASP_DISTANCE * ee.rotation.col(2);

  // ---------- 6. Euler angles, x-y-z extrinsic (degrees) ----------
  // One orientation can be written with two sets of angles:
  //   (tx, ty, tz)  and  (tx + 180, 180 - ty, tz + 180)
  // (both give the same Rz*Ry*Rx). The robot may show either one: for example
  // gripper looking down = (-0.7, -179.4, 0.3) = (179.3, -0.6, -179.7).
  // We write the set closest to the angles the robot shows now. If the user
  // types the new angles one by one in the Web App, the poses in between stay
  // close to the current one. Mixing the two sets (e.g. tx of one and ty of
  // the other) gives a different orientation: the gripper can point up and
  // hit the table.
  const Eigen::Vector3d angles_a = eulerXYZFromRotation(ee.rotation);
  const Eigen::Vector3d angles_b(wrapAngle(angles_a.x() + 180.0),
                                 wrapAngle(180.0 - angles_a.y()),
                                 wrapAngle(angles_a.z() + 180.0));
  const Eigen::Vector3d angles =
      (angleDistance(angles_b, angles_now_deg) < angleDistance(angles_a, angles_now_deg))
          ? angles_b
          : angles_a;
  ee.theta_x = angles.x();
  ee.theta_y = angles.y();
  ee.theta_z = angles.z();

  return ee;
}
