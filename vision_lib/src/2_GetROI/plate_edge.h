// File:    plate_edge.h
// Module:  2_GetROI
// Purpose: Measures the edge of the plate (the colored face the camera
//          sees): its length, its thickness, and where each grasp is along
//          it. It tells if the grasp is near the middle of the plate.
//
// How: the points of the edge (the plate color, color_mask.h) form a long,
// thin strip. PCA of those points (covariance, eigenvectors):
//   - largest eigenvalue  -> axis along the edge (the length),
//   - middle eigenvalue   -> axis across the edge (the thickness),
//   - smallest eigenvalue -> normal of the edge face.
// The points are projected on each axis; the length is the spread along the
// first one. Percentiles 1 and 99 are used instead of the min and max, so a
// few loose points at the ends do not make the edge longer; their spread is
// divided by 0.98, the part of the points they keep.
//
// Limits: the camera widens the borders and the cloud has a point every few
// mm, so expect a few mm of error. The length is right only if the whole
// edge is seen (not cut by the image, not hidden by another object).
#pragma once

#include <vector>

#include <Eigen/Dense>

// Percentiles used as the ends of the edge (see above). They leave out 2%
// of the spread; the points are spread evenly along the edge, so the spread
// between them is divided by KEPT_FRACTION to get the full length back.
// Also used by plate_edge_image.h.
const double LOW_PERCENTILE = 0.01;
const double HIGH_PERCENTILE = 0.99;
const double KEPT_FRACTION = HIGH_PERCENTILE - LOW_PERCENTILE;

// percentile: value at fraction q (0 to 1) of the values.
double percentile(std::vector<double> values, double q);

// Edge of the plate, in the DEPTH frame (the frame of the cloud).
struct PlateEdge {
  Eigen::Vector3d center;  // middle of the edge [m]
  Eigen::Vector3d axis;    // unit vector along the edge
  double length;           // [m]
  double thickness;        // [m], across the edge (only measured, any value)
};

// measurePlateEdge: PCA of the edge points (one column per point, DEPTH
// frame, e.g. the samples of plateSamples, 3_Sample/plate_samples.h).
// Returns false (and prints why) if there are fewer than 10 points.
bool measurePlateEdge(const Eigen::Matrix3Xd &points, PlateEdge &edge);

// alongEdge: position of a point along the edge, from its center [m]
// (negative = toward the end at -axis).
double alongEdge(const PlateEdge &edge, const Eigen::Vector3d &point);

// printPlateEdge: prints the length and thickness of the edge, its center
// and axis in the BASE frame, and for each grasp position (DEPTH frame) its
// distance to the center and to both ends.
void printPlateEdge(const PlateEdge &edge, const Eigen::Matrix4d &T_base_depth,
                    const std::vector<Eigen::Vector3d> &grasp_positions);
