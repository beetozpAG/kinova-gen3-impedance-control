// grasps_from_file: detect grasps in a point cloud file (.pcd) with GPD and
// print, for each grasp, the end effector (EE) pose in the BASE frame.
//
// No robot and no camera are needed. The point cloud must be in the DEPTH
// frame. The robot pose when the cloud was taken is given in one of two ways,
// or not given at all:
//
// Usage (run from kinova_grasp/build/):
//   ./grasps_from_file ../cfg/gpd_params.cfg NAME.pcd NAME_pose.txt
//       TOOL pose measured by the robot, from the file written by
//       grasps_from_camera or view_cloud.
//       Use this one with clouds from the robot (more exact, see frames.h).
//   ./grasps_from_file ../cfg/gpd_params.cfg FILE.pcd q1 q2 q3 q4 q5 q6 q7
//       Joint angles in degrees (like the Kinova web app). The pose is
//       computed with Gen3_model.h (nominal model, ~5-9 mm error).
//   ./grasps_from_file ../cfg/gpd_params.cfg FILE.pcd
//       No robot pose: GPD on the whole cloud, as atenpas detect_grasps.cpp
//       does. Steps 1.0 and 5 are skipped and the grasps are printed in the
//       DEPTH frame (the frame of the cloud).
//   Add --color NAME (with a _pose.txt that has a color image) to look for
//   grasps only on the plate of that color (cfg/plate_colors.cfg), e.g.
//   ./grasps_from_file ../cfg/gpd_plate_params.cfg NAME.pcd NAME_pose.txt --color blue
//
// Steps:
//   1. Read the arguments (and, with a _pose.txt, the color image of the
//      capture if it has one: capture_file.h).
//   2. Read hand_depth from the hand geometry file of the config.
//   3. INT and EE poses in BASE (from the pose file or from Gen3_model.h).
//   4. Load the cloud, drop the points below the table (crop_table.cpp,
//      step 1.0), with --color keep the plate points as samples
//      (3_Sample/plate_samples.h) and detect grasps with GPD.
//   5. Convert each grasp to the EE pose in BASE (grasp_to_ee.cpp) and print it.
// Steps 2, 4 (detection) and 5 are in grasp_pipeline.cpp (shared with
// grasps_from_camera).

#include <cstdlib>
#include <iostream>
#include <string>

#include "Robot/Gen3_model.h"
#include "Pipeline/grasp_pipeline.h"
#include "6_SelectGrasp/grasp_to_ee.h"
#include "0_Input/capture_file.h"
#include "0_Input/frames.h"
#include "3_Sample/plate_samples.h"
#include "1_PreprocessCloud/crop_table.h"

#include <pcl/io/pcd_io.h>

// Matrix.h 4x4 -> Eigen 4x4.
Eigen::Matrix4d toEigen(const matrix::mat<4, 4> &T) {
  Eigen::Matrix4d E;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      E(i, j) = T[i][j];
    }
  }
  return E;
}

// printGraspsDepth: prints each grasp as GPD gives it, in the DEPTH frame
// (see third_party/gpd/readme/hand_frame.png):
//   score    : how good the grasp is, according to the neural network
//              (positive = likely to work, negative = likely to fail).
//   position : bottom center of the hand (where the fingers start), [m].
//   approach : direction in which the hand moves towards the object.
//   binormal : direction in which the fingers close.
//   axis     : third direction (approach x binormal).
//   width    : how much the gripper must open for this grasp, [m].
void printGraspsDepth(const Grasps &grasps) {
  std::cout << "\n========== GRASPS (DEPTH frame) ==========\n";
  std::cout << "Number of grasps: " << grasps.size() << " (best first)\n";
  for (size_t i = 0; i < grasps.size(); i++) {
    const gpd::candidate::Hand &g = *grasps[i];
    std::cout << "\nGrasp " << i << "\n"
              << "  score    : " << g.getScore() << "\n"
              << "  position : " << g.getPosition().transpose() << "\n"
              << "  approach : " << g.getApproach().transpose() << "\n"
              << "  binormal : " << g.getBinormal().transpose() << "\n"
              << "  axis     : " << g.getAxis().transpose() << "\n"
              << "  width    : " << g.getGraspWidth() << "\n";
  }
  std::cout << "==========================================\n";
}

int main(int argc, char *argv[]) {
  // ---------- 1. Read the arguments ----------
  // "--color NAME" can be anywhere; it is taken out first.
  const std::string color_name = takeColorOption(argc, argv);
  // argv[1] = config file, argv[2] = point cloud file, then either
  // argv[3] = pose file (4 arguments), argv[3..9] = joint angles (10 arguments)
  // or nothing (3 arguments, no robot pose).
  if (argc != 3 && argc != 4 && argc != 10) {
    std::cout << "Usage: grasps_from_file CONFIG_FILE PCD_FILE POSE_FILE\n"
              << "       (POSE_FILE = NAME_pose.txt written by grasps_from_camera or view_cloud)\n"
              << "   or: grasps_from_file CONFIG_FILE PCD_FILE q1 q2 q3 q4 q5 q6 q7\n"
              << "       (joint angles in degrees)\n"
              << "   or: grasps_from_file CONFIG_FILE PCD_FILE\n"
              << "       (no robot pose: grasps in the DEPTH frame, no table cut)\n"
              << "Add --color NAME (with POSE_FILE) to grasp only the plate of that color\n"
              << "Example (from build/): ./grasps_from_file ../cfg/gpd_params.cfg "
                 "../third_party/gpd/tutorials/krylon.pcd\n";
    return 1;
  }
  const std::string config_filename = argv[1];
  const std::string pcd_filename = argv[2];
  const bool has_pose = (argc != 3);
  const bool from_pose_file = (argc == 4);
  if (!color_name.empty() && !from_pose_file) {
    std::cout << "Error: --color needs POSE_FILE (the color image is saved with the pose)\n";
    return 1;
  }
  double q_deg[7] = {};
  ToolPose robot_tool{};
  Intrinsics color_intr{};
  cv::Mat color_image;
  if (from_pose_file) {
    if (!readPoseFile(argv[3], robot_tool, q_deg)) {
      std::cout << "Error: could not read a tool_pose line in " << argv[3] << "\n";
      return 1;
    }
    // Color image of the capture, if it has one (used with --color).
    if (readColorCapture(argv[3], color_intr, color_image)) {
      std::cout << "Color image: " << color_image.cols << "x" << color_image.rows
                << " (from " << argv[3] << ")\n";
    } else {
      std::cout << "No color image in this capture (saved before the color camera was added)\n";
      if (!color_name.empty()) {
        std::cout << "Error: --color needs a capture with a color image\n";
        return 1;
      }
    }
  } else if (has_pose) {
    for (int i = 0; i < 7; i++) {
      q_deg[i] = std::atof(argv[3 + i]);
    }
  }

  // ---------- 2. Read hand_depth (and camera position) ----------
  GpdSetup setup;
  if (!readGpdSetup(config_filename, setup)) {
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud_full;
  if (pcl::io::loadPCDFile(pcd_filename, cloud_full) != 0) {
    std::cout << "Error: could not read the point cloud file: " << pcd_filename << "\n";
    return 1;
  }

  // ---------- No robot pose: GPD on the whole cloud, grasps in DEPTH ----------
  if (!has_pose) {
    Grasps grasps;
    if (!detectGraspsInCloud(config_filename, cloud_full, setup, grasps)) {
      return 1;
    }
    printGraspsDepth(grasps);
    return 0;
  }

  // ---------- 3. INT and EE poses in BASE ----------
  //   T_base_int: INT in BASE.
  //   T_base_ee:  EE in BASE = T_base_int * (0.120 m on Z) (Table 28, p.92).
  //               Its rotation is used to choose between the two symmetric
  //               EE orientations (grasp_to_ee.cpp, step 5).
  Eigen::Matrix4d T_base_int;
  Eigen::Matrix4d T_base_ee;
  if (from_pose_file) {
    // EE pose measured by the robot -> INT pose (frames.h).
    T_base_ee = toolToBase(robot_tool);
    T_base_int = intToBase(T_base_ee, KinovaGen3::TOOL_OFFSET_Z_2F85);
  } else {
    // Forward kinematics of Gen3_model.h (Table 90, p.221-222), angles in radians.
    matrix::mat<7, 1> q{};
    const double deg2rad = M_PI / 180.0;
    for (int i = 0; i < 7; i++) {
      q[i][0] = static_cast<float>(q_deg[i] * deg2rad);
    }
    T_base_int = toEigen(KinovaGen3::GetTransform(q));
    T_base_ee = toEigen(KinovaGen3::GetToolTransform(q));
  }

  // ---------- 4. Drop the points below the table, detect ----------
  // Step 1.0 (crop_table.cpp) needs the DEPTH frame with respect to BASE.
  // Grasps are in the DEPTH frame.
  const Eigen::Matrix4d T_base_depth = T_base_int * depthToInt();
  const pcl::PointCloud<pcl::PointXYZ> cloud = cropBelowTable(cloud_full, T_base_depth);
  // With --color: GPD looks for grasps only on the plate of that color.
  Eigen::Matrix3Xd samples;
  if (!color_name.empty() &&
      !plateSamples(cloud, color_image, color_intr, color_name, samples)) {
    return 1;
  }
  Grasps grasps;
  if (!detectGraspsInCloud(config_filename, cloud, setup, grasps, samples)) {
    return 1;
  }

  // With --color: plate edge, and the grasp nearest to its middle first.
  PlateEdge edge;
  const bool has_edge =
      !color_name.empty() && plateEdgeOfGrasps(cloud, color_image, color_intr, color_name,
                                               samples, setup, grasps, edge);

  // ---------- 5. Convert and print ----------
  std::cout << "\n========== EE POSES w.r.t. BASE ==========\n";
  if (from_pose_file) {
    std::cout << "Robot pose: measured by the robot (" << argv[3] << ")\n"
              << "  EE now (m, deg): " << robot_tool.x << " " << robot_tool.y << " "
              << robot_tool.z << "  " << robot_tool.theta_x << " " << robot_tool.theta_y
              << " " << robot_tool.theta_z << "\n";
  } else {
    std::cout << "Robot pose: Gen3_model.h from the joint angles\n";
  }
  std::cout << "Joint angles (deg):";
  for (int i = 0; i < 7; i++) {
    std::cout << " " << q_deg[i];
  }
  std::cout << "\n";
  // EE angles now, as the robot shows them: from the pose file, or (joint
  // angles only) computed from Gen3_model.h.
  const Eigen::Vector3d ee_angles_now =
      from_pose_file
          ? Eigen::Vector3d(robot_tool.theta_x, robot_tool.theta_y, robot_tool.theta_z)
          : eulerXYZFromRotation(T_base_ee.block<3, 3>(0, 0));
  printEEPoses(grasps, setup, T_base_int, T_base_ee, ee_angles_now,
               KinovaGen3::TOOL_OFFSET_Z_2F85, samples, cloud);

  // With --color: length of the plate edge and where each grasp is on it.
  if (has_edge) {
    printPlateEdgeOfGrasps(edge, grasps, T_base_depth);
  }
  return 0;
}
