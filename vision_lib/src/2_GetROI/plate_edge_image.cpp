// File:    plate_edge_image.cpp
// Module:  2_GetROI
// Purpose: Edge of the plate measured in the color image. See
//          plate_edge_image.h for the steps and their limits.
#include "2_GetROI/plate_edge_image.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "0_Input/color_cloud.h"

// Spots of the color smaller than this are erased before step 2 (morphological
// opening) [pixels]: on camera_2610_160811 the dark holder had loose
// spots that touched the plate and joined its region (thickness 36.9 mm
// instead of 11.4). The edge (10 mm) is more than 20 pixels wide up to 0.6 m.
const int OPEN_PIXELS = 9;

// The region needs this many pixels, and this many cloud points in it.
const int MIN_REGION_PIXELS = 100;
const int MIN_DEPTH_POINTS = 10;

// Cloud points of the region farther than this from the median depth are
// left out [m]: they are the background seen past the borders of the plate
// (camera_2610_160811: z = 0.51 m at one end, the plate is at 0.30 m).
const double DEPTH_WINDOW = 0.03;

// One cloud point seen in the region: place along the edge [pixels] and
// distance to the color camera z [m].
struct DepthSample {
  double s;
  double z;
};

// DepthLine: z = z0 + slope * s along the edge (a tilted camera sees the
// edge at different depths).
struct DepthLine {
  double z0, slope;
  double at(double s) const { return z0 + slope * s; }
};

// fitDepthLine: least squares line of the samples within DEPTH_WINDOW of
// their median z. Returns false if fewer than MIN_DEPTH_POINTS are kept.
static bool fitDepthLine(const std::vector<DepthSample> &samples, DepthLine &line) {
  std::vector<double> z;
  for (const DepthSample &d : samples) {
    z.push_back(d.z);
  }
  const double z_median = percentile(z, 0.5);
  double n = 0, s_sum = 0, z_sum = 0, ss_sum = 0, sz_sum = 0;
  for (const DepthSample &d : samples) {
    if (std::abs(d.z - z_median) < DEPTH_WINDOW) {
      n += 1;
      s_sum += d.s;
      z_sum += d.z;
      ss_sum += d.s * d.s;
      sz_sum += d.s * d.z;
    }
  }
  if (n < MIN_DEPTH_POINTS) {
    return false;
  }
  const double denominator = n * ss_sum - s_sum * s_sum;
  line.slope = (denominator > 0.0) ? (n * sz_sum - s_sum * z_sum) / denominator : 0.0;
  line.z0 = (z_sum - line.slope * s_sum) / n;
  return true;
}

bool measurePlateEdgeImage(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                           const cv::Mat &color_image, const Intrinsics &color_intr,
                           const ColorRange &range, PlateEdge &edge) {
  // ---------- 1. Pixels of the plate color ----------
  cv::Mat mask = colorMaskImage(color_image, range);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(OPEN_PIXELS, OPEN_PIXELS)));

  // ---------- 2. Largest connected region ----------
  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
  int region = 0;
  int region_pixels = 0;
  for (int k = 1; k < count; k++) {  // label 0 is the background
    const int pixels = stats.at<int>(k, cv::CC_STAT_AREA);
    if (pixels > region_pixels) {
      region = k;
      region_pixels = pixels;
    }
  }
  if (region_pixels < MIN_REGION_PIXELS) {
    printf("Plate edge (color image): only %d pixels of the color, too few.\n", region_pixels);
    return false;
  }

  // ---------- 3. PCA of the pixels (col, row) ----------
  std::vector<Eigen::Vector2d> pixels;
  pixels.reserve(region_pixels);
  for (int row = 0; row < labels.rows; row++) {
    for (int col = 0; col < labels.cols; col++) {
      if (labels.at<int>(row, col) == region) {
        pixels.emplace_back(col, row);
      }
    }
  }
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const Eigen::Vector2d &p : pixels) {
    mean += p;
  }
  mean /= static_cast<double>(pixels.size());
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const Eigen::Vector2d &p : pixels) {
    covariance += (p - mean) * (p - mean).transpose();
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  const Eigen::Vector2d along = solver.eigenvectors().col(1);   // largest
  const Eigen::Vector2d across = solver.eigenvectors().col(0);  // smallest

  std::vector<double> s(pixels.size()), t(pixels.size());
  for (size_t i = 0; i < pixels.size(); i++) {
    s[i] = along.dot(pixels[i] - mean);
    t[i] = across.dot(pixels[i] - mean);
  }
  const double s_low = percentile(s, LOW_PERCENTILE);
  const double s_high = percentile(s, HIGH_PERCENTILE);
  const double t_low = percentile(t, LOW_PERCENTILE);
  const double t_high = percentile(t, HIGH_PERCENTILE);
  const double length_px = (s_high - s_low) / KEPT_FRACTION;
  const double thickness_px = (t_high - t_low) / KEPT_FRACTION;
  const double s_mid = (s_low + s_high) / 2.0;
  const Eigen::Vector2d middle = mean + along * s_mid + across * (t_low + t_high) / 2.0;
  const Eigen::Vector2d end_a = middle - along * length_px / 2.0;
  const Eigen::Vector2d end_b = middle + along * length_px / 2.0;

  // ---------- 4. Pixels -> 3D ----------
  // The cloud points seen in the region give the depth (as colorCloud).
  const DepthToColor to_color(color_intr);
  std::vector<DepthSample> depth;
  for (const pcl::PointXYZ &p : cloud) {
    int col, row;
    double z;
    if (!to_color.pixel(Eigen::Vector3d(p.x, p.y, p.z), col, row, z) || col < 0 ||
        col >= labels.cols || row < 0 || row >= labels.rows ||
        labels.at<int>(row, col) != region) {
      continue;
    }
    depth.push_back(DepthSample{along.dot(Eigen::Vector2d(col, row) - mean), z});
  }
  DepthLine line;
  if (!fitDepthLine(depth, line)) {
    printf("Plate edge (color image): only %zu cloud points on the region, too few.\n",
           depth.size());
    return false;
  }

  // Middle and ends at their depth, back to the DEPTH frame.
  const double z_middle = line.at(s_mid);
  const Eigen::Vector3d a = to_color.toDepth(end_a.x(), end_a.y(), line.at(s_mid - length_px / 2.0));
  const Eigen::Vector3d b = to_color.toDepth(end_b.x(), end_b.y(), line.at(s_mid + length_px / 2.0));

  edge.center = to_color.toDepth(middle.x(), middle.y(), z_middle);
  edge.axis = (b - a).normalized();
  edge.length = (b - a).norm();
  edge.thickness = thickness_px * z_middle / color_intr.fx;
  printf("Plate edge measured in the color image: %d pixels, %zu cloud points on them.\n",
         region_pixels, depth.size());
  return true;
}
