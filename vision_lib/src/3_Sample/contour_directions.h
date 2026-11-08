// File:    contour_directions.h
// Module:  3_Sample
// Purpose: The directions in which the fingers can close on the object, from
//          its outline seen along the approach (from above): the directions
//          of minimum width. GPD builds its local frames with them
//          (contour_frames.h), so the pads end parallel to the faces.
//
// Why: GPD turns the hand with a frame from the normals around each sample
// (local_frame.cpp:14-40). On a flat top (plate edge, bolt head) all
// normals point up and that frame is set by the noise. Measured on
// camera_2410_173940 (plate) and camera_2410_180933 (hexagon): 21 to 46 deg
// off the faces with the original frame, 0 to 2 deg with these directions.
//
// How (rotating calipers, Toussaint 1983): the object points projected on
// the plane normal to the view; for each angle 0 to 179 deg, the width of
// the outline (1st to 99th percentile, so loose points do not count). A
// direction of minimum width is perpendicular to a flat side:
//   plate standing   -> 1 (across the thickness)
//   hexagon          -> 3 (60 deg apart), octagon -> 4 (45 deg apart)
//   triangle         -> 3 (one pad on a side, the other on the corner)
//   round            -> none: GPD keeps its own frames
// Limits: convex objects whose outline does not change with the height
// (prisms). The object points must be the object alone (color mask and
// largest group, 3_Sample/plate_samples.h).
#pragma once

#include <vector>

#include <Eigen/Dense>

// Minimum angle between two directions of minimum width, on each side [deg].
const int CONTOUR_WINDOW_DEG = 15;
// A minimum must be this much narrower than the widest direction within
// 2 * CONTOUR_WINDOW_DEG of it (2%: a hexagon is 36 mm across the flats and
// about 39 mm across the corners once measured).
const double CONTOUR_MIN_CONTRAST = 1.02;

// contourDirections: the directions of minimum width of points (one column
// per point, cloud frame) seen along view (unit vector, cloud frame). Each
// direction is a unit vector normal to view; widths[i] is the width of the
// object along directions[i] [m]. Empty if there are fewer than 10 points
// or no minimum (round outline).
std::vector<Eigen::Vector3d> contourDirections(const Eigen::Matrix3Xd &points,
                                               const Eigen::Vector3d &view,
                                               std::vector<double> &widths);
