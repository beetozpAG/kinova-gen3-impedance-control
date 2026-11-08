// File:    plate_edge.cpp
// Module:  2_GetROI
// Purpose: Length and thickness of the plate edge, and where each grasp is
//          along it. See plate_edge.h for the method and its limits.
#include "2_GetROI/plate_edge.h"

#include <algorithm>
#include <cstdio>

// percentile: the k-th smallest value, k = q * (n - 1).
double percentile(std::vector<double> values, double q) {
  const size_t k = static_cast<size_t>(q * (values.size() - 1));
  std::nth_element(values.begin(), values.begin() + k, values.end());
  return values[k];
}

// measurePlateEdge: mean and covariance of the points, eigenvectors of the
// covariance (Eigen sorts the eigenvalues from small to large), then the
// spread of the points along the two largest axes.
bool measurePlateEdge(const Eigen::Matrix3Xd &points, PlateEdge &edge) {
  const int n = static_cast<int>(points.cols());
  if (n < 10) {
    printf("Plate edge: only %d points, too few to measure it.\n", n);
    return false;
  }
  const Eigen::Vector3d mean = points.rowwise().mean();
  const Eigen::Matrix3Xd centered = points.colwise() - mean;
  const Eigen::Matrix3d covariance = centered * centered.transpose() / n;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  const Eigen::Vector3d along = solver.eigenvectors().col(2);   // largest
  const Eigen::Vector3d across = solver.eigenvectors().col(1);  // middle

  std::vector<double> s(n), t(n);
  for (int i = 0; i < n; i++) {
    s[i] = along.dot(centered.col(i));
    t[i] = across.dot(centered.col(i));
  }
  const double s_low = percentile(s, LOW_PERCENTILE);
  const double s_high = percentile(s, HIGH_PERCENTILE);

  edge.axis = along;
  edge.length = (s_high - s_low) / KEPT_FRACTION;
  edge.center = mean + along * (s_low + s_high) / 2.0;
  edge.thickness =
      (percentile(t, HIGH_PERCENTILE) - percentile(t, LOW_PERCENTILE)) / KEPT_FRACTION;
  return true;
}

double alongEdge(const PlateEdge &edge, const Eigen::Vector3d &point) {
  return edge.axis.dot(point - edge.center);
}

void printPlateEdge(const PlateEdge &edge, const Eigen::Matrix4d &T_base_depth,
                    const std::vector<Eigen::Vector3d> &grasp_positions) {
  const Eigen::Vector3d center_base =
      T_base_depth.block<3, 3>(0, 0) * edge.center + T_base_depth.block<3, 1>(0, 3);
  const Eigen::Vector3d axis_base = T_base_depth.block<3, 3>(0, 0) * edge.axis;

  printf("\n========== PLATE EDGE ==========\n");
  printf("Length: %.0f mm   thickness: %.1f mm\n", edge.length * 1000.0,
         edge.thickness * 1000.0);
  printf("Center in BASE (m): x = %.4f  y = %.4f  z = %.4f\n", center_base.x(), center_base.y(),
         center_base.z());
  printf("Axis in BASE:       x = %.3f  y = %.3f  z = %.3f\n", axis_base.x(), axis_base.y(),
         axis_base.z());

  const double half = edge.length / 2.0;
  for (size_t i = 0; i < grasp_positions.size(); i++) {
    const double s = alongEdge(edge, grasp_positions[i]);
    printf("Grasp %zu: %.0f mm from the center (%.0f mm from end A, %.0f mm from end B)\n", i,
           s * 1000.0, (half + s) * 1000.0, (half - s) * 1000.0);
  }
  printf("(End A is at -axis, end B at +axis.)\n");
  printf("================================\n");
}
