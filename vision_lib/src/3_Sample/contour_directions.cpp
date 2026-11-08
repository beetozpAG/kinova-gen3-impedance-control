// File:    contour_directions.cpp
// Module:  3_Sample
// Purpose: Directions of minimum width of the object outline. See
//          contour_directions.h.
#include "3_Sample/contour_directions.h"

#include <algorithm>
#include <cmath>

// outlineWidth: width of the projected points along the angle (deg) and the
// basis e1, e2 of the plane [m], between the 1st and 99th percentiles.
static double outlineWidth(const std::vector<Eigen::Vector2d> &points, int angle_deg) {
  const double a = angle_deg * M_PI / 180.0;
  const Eigen::Vector2d u(std::cos(a), std::sin(a));
  std::vector<double> s(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    s[i] = points[i].dot(u);
  }
  std::sort(s.begin(), s.end());
  const size_t last = s.size() - 1;
  return s[static_cast<size_t>(0.99 * last)] - s[static_cast<size_t>(0.01 * last)];
}

// contourDirections: project, width per degree, then the local minima.
std::vector<Eigen::Vector3d> contourDirections(const Eigen::Matrix3Xd &points,
                                               const Eigen::Vector3d &view,
                                               std::vector<double> &widths) {
  std::vector<Eigen::Vector3d> directions;
  widths.clear();
  if (points.cols() < 10) {
    return directions;
  }

  // Plane normal to the view.
  const Eigen::Vector3d e1 = view.unitOrthogonal();
  const Eigen::Vector3d e2 = view.cross(e1);
  std::vector<Eigen::Vector2d> projected(points.cols());
  for (int i = 0; i < points.cols(); i++) {
    projected[i] = Eigen::Vector2d(points.col(i).dot(e1), points.col(i).dot(e2));
  }

  std::vector<double> w(180);
  for (int a = 0; a < 180; a++) {
    w[a] = outlineWidth(projected, a);
  }

  // A minimum: the narrowest within +-CONTOUR_WINDOW_DEG (only the first
  // angle of a flat stretch), and CONTOUR_MIN_CONTRAST narrower than the
  // widest within twice that window.
  for (int a = 0; a < 180; a++) {
    double narrowest = w[a];
    double widest = w[a];
    for (int k = -2 * CONTOUR_WINDOW_DEG; k <= 2 * CONTOUR_WINDOW_DEG; k++) {
      const double wk = w[(a + k + 180) % 180];
      if (std::abs(k) <= CONTOUR_WINDOW_DEG) {
        narrowest = std::min(narrowest, wk);
      }
      widest = std::max(widest, wk);
    }
    const bool same_as_previous = (w[(a + 179) % 180] == w[a]);
    if (w[a] > narrowest || same_as_previous || widest < CONTOUR_MIN_CONTRAST * w[a]) {
      continue;
    }
    const double angle = a * M_PI / 180.0;
    directions.push_back(std::cos(angle) * e1 + std::sin(angle) * e2);
    widths.push_back(w[a]);
  }
  return directions;
}
