// File:    center_grasp.h
// Module:  6_SelectGrasp
// Purpose: Moves a GPD grasp along the closing direction so the object is in
//          the middle between the fingers. The 2F-85 closes both fingers
//          together: off center, one finger touches first and pushes the
//          object (bad when it stands in a holder or a fixture).
//
// Why: GPD places the hand at one of num_finger_placements positions where
// the fingers hit no point (finger_hand.cpp), and its position is the
// middle between the fingers, not the middle of the object
// (hand.cpp:40-44). Measured with contour frames: 13 to 20 mm off on the
// hexagon (camera_2410_180933), 7 to 12 mm on the plate (camera_2410_173940).
//
// How, in the hand frame of GPD (position at the base of the fingers):
//   1. Object points between the fingers: along the approach from 0 to
//      hand_depth, across within the opening, within hand_height.
//   2. shift = middle of their extent along the closing direction.
//   3. The fingers moved by shift must hit no point of the cloud (the same
//      finger boxes GPD checks). If one does, the GPD position is kept.
// Limits: only the fingers are checked again, not the base of the hand.
#pragma once

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gpd/candidate/hand.h>

// Hand sizes from the hand geometry .cfg [m].
struct HandSize {
  double finger_width;
  double outer_diameter;  // opening + 2 finger widths
  double depth;           // finger length (hand_depth)
  double height;          // finger height (hand_height)
};

// A grasp position moved to the middle of the object.
struct CenteredGrasp {
  Eigen::Vector3d position;  // DEPTH frame, base of the fingers [m]
  double shift;              // along the closing direction (binormal) [m]
  bool moved;                // false: GPD position kept (no object points or a finger would hit)
};

// centerGrasp: object_points are the points of the object (one column per
// point, DEPTH frame, e.g. the samples of plateSamples); cloud is the cloud
// given to GPD (DEPTH frame). With no object points between the fingers,
// the GPD position is kept.
CenteredGrasp centerGrasp(const gpd::candidate::Hand &grasp, const Eigen::Matrix3Xd &object_points,
                          const pcl::PointCloud<pcl::PointXYZ> &cloud, const HandSize &hand);
