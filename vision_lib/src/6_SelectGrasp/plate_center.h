// File:    plate_center.h
// Module:  6_SelectGrasp
// Purpose: Chooses the grasp nearest to the middle of the plate edge.
//          GPD orders its grasps by the score of its network only, which
//          sees about 6 cm around the hand and not the whole plate:
//          measured, the best one was 25 to 50 mm from the middle on
//          approach_2610_182452, 182355, camera_2610_182558, 160732.
//
// How: the place of each grasp along the edge (alongEdge, plate_edge.h);
// the grasps are ordered by its size, nearest first. Grasps at the same
// place keep the GPD order (best score first).
// Limits: only the grasps GPD returns (num_selected in the cfg) are
// compared; none of them has to be near the middle.
//
// sortByObjectCenter does the same for a compact object (the head of a peg):
// the hand is centered across the closing direction (center_grasp.h), but
// not along the fingers, where it stays at its GPD sample. Measured on
// camera_2910_170114: the best score was 13 mm from the middle of the head,
// the second one 2 mm.
#pragma once

#include <memory>
#include <vector>

#include <gpd/candidate/hand.h>

#include "2_GetROI/plate_edge.h"

// sortByPlateCenter: the grasps (DEPTH frame) nearest to edge.center first.
void sortByPlateCenter(std::vector<std::unique_ptr<gpd::candidate::Hand>> &grasps,
                       const PlateEdge &edge);

// sortByObjectCenter: the grasps (DEPTH frame) nearest to center first,
// measured along the hand axis of each grasp (the direction centerGrasp
// does not correct).
void sortByObjectCenter(std::vector<std::unique_ptr<gpd::candidate::Hand>> &grasps,
                        const Eigen::Vector3d &center);
