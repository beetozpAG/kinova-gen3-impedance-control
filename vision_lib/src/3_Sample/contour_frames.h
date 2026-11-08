// File:    contour_frames.h
// Module:  3_Sample
// Purpose: Gives GPD the closing directions of the object (contour_directions.h)
//          before it searches the hands. They are used by our copy of GPD's
//          frame estimator (gpd_frame_estimator_contour.cpp), built into
//          libgpd instead of third_party/gpd/src/gpd/candidate/frame_estimator.cpp
//          (see CMakeLists.txt).
//
// With directions set, each sample gets one local frame per direction
// instead of the frame from the normals: same normal (the hand still comes
// along minus the normal), and the fingers close along the direction (at
// the orientation -90 deg of GPD, the only one with num_orientations = 1,
// hand_search.cpp:151-155). GPD does the rest as before: finger placements,
// collisions, deepen_hand, filters, classifier and selection.
// With no directions set, the frames are the original ones of GPD.
#pragma once

#include <vector>

#include <Eigen/Dense>

// setContourDirections: directions in the cloud frame (unit vectors), used
// by the next detectGrasps. An empty list means the original frames.
void setContourDirections(const std::vector<Eigen::Vector3d> &directions);
