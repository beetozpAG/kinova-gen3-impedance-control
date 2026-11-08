// File:    plate_center.cpp
// Module:  6_SelectGrasp
// Purpose: The grasp nearest to the middle of the plate edge first. See
//          plate_center.h.
#include "6_SelectGrasp/plate_center.h"

#include <algorithm>
#include <cmath>

// sortByPlateCenter: stable sort, so equal places keep the GPD order.
void sortByPlateCenter(std::vector<std::unique_ptr<gpd::candidate::Hand>> &grasps,
                       const PlateEdge &edge) {
  std::stable_sort(grasps.begin(), grasps.end(),
                   [&edge](const std::unique_ptr<gpd::candidate::Hand> &a,
                           const std::unique_ptr<gpd::candidate::Hand> &b) {
                     return std::abs(alongEdge(edge, a->getPosition())) <
                            std::abs(alongEdge(edge, b->getPosition()));
                   });
}

// axisDistance: distance from the grasp to center along the hand axis (along
// the fingers, across the approach and the closing direction). The closing
// direction is centered later (center_grasp.h), the approach is how far the
// hand goes in: only this direction is left as GPD placed it.
static double axisDistance(const gpd::candidate::Hand &grasp, const Eigen::Vector3d &center) {
  return std::abs((center - grasp.getPosition()).dot(grasp.getAxis()));
}

// sortByObjectCenter: stable sort, so equal distances keep the GPD order.
void sortByObjectCenter(std::vector<std::unique_ptr<gpd::candidate::Hand>> &grasps,
                        const Eigen::Vector3d &center) {
  std::stable_sort(grasps.begin(), grasps.end(),
                   [&center](const std::unique_ptr<gpd::candidate::Hand> &a,
                             const std::unique_ptr<gpd::candidate::Hand> &b) {
                     return axisDistance(*a, center) < axisDistance(*b, center);
                   });
}
