// File:    center_grasp.cpp
// Module:  6_SelectGrasp
// Purpose: Moves a GPD grasp to the middle of the object. See center_grasp.h.
#include "6_SelectGrasp/center_grasp.h"

#include <algorithm>
#include <cmath>

// Point in the hand frame: along the approach, along the closing direction
// (binormal) and along the hand axis, from the base of the fingers.
static Eigen::Vector3d toHand(const gpd::candidate::Hand &grasp, const Eigen::Vector3d &p) {
  const Eigen::Vector3d d = p - grasp.getPosition();
  return Eigen::Vector3d(d.dot(grasp.getApproach()), d.dot(grasp.getBinormal()),
                         d.dot(grasp.getAxis()));
}

// centerGrasp: extent of the object between the fingers, its middle, then
// the finger boxes at the new place.
CenteredGrasp centerGrasp(const gpd::candidate::Hand &grasp, const Eigen::Matrix3Xd &object_points,
                          const pcl::PointCloud<pcl::PointXYZ> &cloud, const HandSize &hand) {
  CenteredGrasp result{grasp.getPosition(), 0.0, false};
  const double half_outer = hand.outer_diameter / 2.0;
  const double half_height = hand.height / 2.0;

  // 1. Object points between the fingers.
  double low = 1e9;
  double high = -1e9;
  for (int i = 0; i < object_points.cols(); i++) {
    const Eigen::Vector3d h = toHand(grasp, object_points.col(i));
    if (h.x() >= 0.0 && h.x() <= hand.depth && std::abs(h.z()) <= half_height &&
        std::abs(h.y()) < half_outer - hand.finger_width) {
      low = std::min(low, h.y());
      high = std::max(high, h.y());
    }
  }
  if (low > high) {
    return result;
  }

  // 2. Middle of the object along the closing direction.
  const double shift = 0.5 * (low + high);

  // 3. Fingers at the new place: no point of the cloud inside them.
  for (const pcl::PointXYZ &point : cloud) {
    const Eigen::Vector3d h = toHand(grasp, point.getVector3fMap().cast<double>());
    const double across = std::abs(h.y() - shift);
    if (h.x() >= 0.0 && h.x() <= hand.depth && std::abs(h.z()) <= half_height &&
        across >= half_outer - hand.finger_width && across <= half_outer) {
      return result;
    }
  }
  result.position = grasp.getPosition() + shift * grasp.getBinormal();
  result.shift = shift;
  result.moved = true;
  return result;
}
