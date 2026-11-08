// grasp_pipeline: the GPD steps shared by grasps_from_file and
// grasps_from_camera.
//   1. readGpdSetup:   hand_depth and camera position from the GPD config.
//   2. detectGraspsInCloud: run GPD on a cloud in memory.
//   3. plateEdgeOfGrasps: with --color, the plate edge and the grasp nearest
//      to its middle first.
//   4. printEEPoses:   convert each grasp to the EE pose in BASE and print it.
// Moved from detect_ee_pose.cpp, now grasps_from_file.cpp (same code).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gpd/candidate/hand.h>

#include "0_Input/vision_module.h"
#include "2_GetROI/plate_edge.h"
#include "6_SelectGrasp/center_grasp.h"

// Values of the GPD config that our code needs too.
struct GpdSetup {
  std::string hand_geometry_filename;
  double hand_depth;            // GPD finger length [m]
  HandSize hand;                // finger width, outer diameter, depth, height [m]
  Eigen::Matrix3Xd view_points; // camera position in the cloud frame (camera_position)
  bool contour_frames;          // our key contour_frames: frames from the outline (3_Sample/contour_frames.h)
  Eigen::Vector3d approach_direction;  // direction (approach filter), cloud frame: the outline is seen along it
  bool select_plate_center;     // our key select_plate_center: grasp nearest to the plate middle first
  bool select_object_center;    // our key select_object_center: grasp nearest to the object middle first
};

// Grasps printed (the first ones); GPD may return more (num_selected).
const size_t GRASPS_PRINTED = 5;

// Grasps returned by GPD, best first.
using Grasps = std::vector<std::unique_ptr<gpd::candidate::Hand>>;

// Read hand_depth (from the hand geometry file) and camera_position from the
// GPD config. Same file and same reader that GPD uses (grasp_detector.cpp:13-18),
// so the conversion always uses the hand_depth that GPD used.
// Returns false (and prints why) if a file is missing.
bool readGpdSetup(const std::string &config_filename, GpdSetup &setup);

// Run GPD on a cloud in memory. Grasps are in the frame of the cloud (DEPTH frame).
// samples: where GPD looks for grasps (one column per point, DEPTH frame),
// for example the points of one plate (3_Sample/plate_samples.h). Empty:
// GPD chooses its samples itself (step 1.7). In both cases GPD gets the
// whole cloud, to check that the hand does not hit anything.
// With samples and contour_frames = 1, the samples are also the outline of
// the object: GPD turns the hand to its directions of minimum width
// (3_Sample/contour_directions.h, contour_frames.h).
// Returns false (and prints why) if the cloud is empty.
bool detectGraspsInCloud(const std::string &config_filename,
                         const pcl::PointCloud<pcl::PointXYZ> &points,
                         const GpdSetup &setup, Grasps &grasps,
                         const Eigen::Matrix3Xd &samples = Eigen::Matrix3Xd());

// takeColorOption: finds "--color NAME" among the arguments, removes both
// from argv (argc gets 2 smaller) and returns NAME, so the other arguments
// keep their usual positions. Returns "" if there is no --color (or no NAME
// after it).
std::string takeColorOption(int &argc, char *argv[]);

// plateEdgeOfGrasps: with --color, measures the edge of the plate of
// color_name in the color image (2_GetROI/plate_edge_image.h); if that
// fails, from its points (plate_points, DEPTH frame: the samples of
// plateSamples, 2_GetROI/plate_edge.h). With select_plate_center = 1 in the
// cfg, orders the grasps nearest to the middle of the edge first; with
// select_object_center = 1, nearest to the middle of the object along the
// hand axis (6_SelectGrasp/plate_center.h). Returns false if the edge could
// not be measured; the grasps then keep the GPD order.
bool plateEdgeOfGrasps(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &color_image,
                       const Intrinsics &color_intr, const std::string &color_name,
                       const Eigen::Matrix3Xd &plate_points, const GpdSetup &setup,
                       Grasps &grasps, PlateEdge &edge);

// printPlateEdgeOfGrasps: prints the length of the edge and where each
// printed grasp is along it (2_GetROI/plate_edge.h).
void printPlateEdgeOfGrasps(const PlateEdge &edge, const Grasps &grasps,
                            const Eigen::Matrix4d &T_base_depth);

// Convert the first GRASPS_PRINTED grasps to the EE pose in BASE
// (grasp_to_ee.cpp) and print them.
// With object points (--color), each grasp is first moved to the middle of
// the object (6_SelectGrasp/center_grasp.h).
//   object_points: points of the object, DEPTH frame (empty: no centering).
//   cloud:       the cloud given to GPD (DEPTH frame).
//   T_base_int:  INT in BASE when the cloud was taken.
//   T_base_ee:   EE in BASE at that moment (chooses the symmetric orientation).
//   ee_angles_now_deg: EE Euler angles at that moment, as the robot shows
//                them (the printed angles are written close to these).
//   ee_offset_z: INT -> EE distance along Z_INT (0.120 m, Table 28, p.92).
void printEEPoses(const Grasps &grasps, const GpdSetup &setup,
                  const Eigen::Matrix4d &T_base_int, const Eigen::Matrix4d &T_base_ee,
                  const Eigen::Vector3d &ee_angles_now_deg, double ee_offset_z,
                  const Eigen::Matrix3Xd &object_points,
                  const pcl::PointCloud<pcl::PointXYZ> &cloud);
