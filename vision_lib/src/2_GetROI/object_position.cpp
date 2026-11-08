// File:    object_position.cpp
// Module:  2_GetROI
// Purpose: The centroid of the object of one color in BASE. See
//          object_position.h for how and its limits.
#include "2_GetROI/object_position.h"

#include <cstdio>
#include <vector>

#include "0_Input/color_cloud.h"
#include "2_GetROI/color_mask.h"
#include "2_GetROI/largest_cluster.h"

// objectPosition: color of each point (color_cloud), points of the color
// (color_mask), only the object itself (largest_cluster), then each point to
// BASE: the mean is the centroid, the min and max give the size.
bool objectPosition(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &color_image,
                    const Intrinsics &color_intr, const std::string &color_name,
                    const Eigen::Matrix4d &T_base_depth, ObjectPosition &object) {
  ColorRange range;
  if (!readColorRange(PLATE_COLORS_FILE, color_name, range)) {
    return false;
  }

  const pcl::PointCloud<pcl::PointXYZRGB> colored = colorCloud(cloud, color_image, color_intr);
  const std::vector<int> indices = largestCluster(cloud, colorMask(colored, range));
  if (indices.size() < MIN_OBJECT_POINTS) {
    printf("Only %zu points of color %s (fewer than %zu): no object of that color is seen.\n"
           "Check the object is in the image, or the range in %s.\n",
           indices.size(), color_name.c_str(), MIN_OBJECT_POINTS, PLATE_COLORS_FILE.c_str());
    return false;
  }

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d min_point = Eigen::Vector3d::Constant(1e9);
  Eigen::Vector3d max_point = Eigen::Vector3d::Constant(-1e9);
  for (int i : indices) {
    const pcl::PointXYZ &p = cloud[i];
    const Eigen::Vector3d in_base = (T_base_depth * Eigen::Vector4d(p.x, p.y, p.z, 1.0)).head<3>();
    sum += in_base;
    min_point = min_point.cwiseMin(in_base);
    max_point = max_point.cwiseMax(in_base);
  }
  object.points = indices.size();
  object.centroid = sum / static_cast<double>(indices.size());
  object.size = max_point - min_point;
  return true;
}
