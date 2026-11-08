// File:    plate_samples.cpp
// Module:  3_Sample
// Purpose: The points of one plate color, as GPD samples. See
//          plate_samples.h for what the function returns.
#include "3_Sample/plate_samples.h"

#include <cstdio>
#include <vector>

#include "0_Input/color_cloud.h"
#include "2_GetROI/color_mask.h"
#include "2_GetROI/largest_cluster.h"

// plateSamples: color of each point (color_cloud), points of the plate
// color (color_mask), only the plate itself (largest_cluster), then their
// x, y, z as columns.
bool plateSamples(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &color_image,
                  const Intrinsics &color_intr, const std::string &color_name,
                  Eigen::Matrix3Xd &samples) {
  ColorRange range;
  if (!readColorRange(PLATE_COLORS_FILE, color_name, range)) {
    return false;
  }

  const pcl::PointCloud<pcl::PointXYZRGB> colored = colorCloud(cloud, color_image, color_intr);
  const std::vector<int> plate = largestCluster(cloud, colorMask(colored, range));
  if (plate.empty()) {
    printf("No point of color %s in the cloud: check the plate is seen, or the range in %s.\n",
           color_name.c_str(), PLATE_COLORS_FILE.c_str());
    return false;
  }

  samples.resize(3, static_cast<int>(plate.size()));
  for (size_t i = 0; i < plate.size(); i++) {
    const pcl::PointXYZ &p = cloud[plate[i]];
    samples.col(static_cast<int>(i)) = Eigen::Vector3d(p.x, p.y, p.z);
  }
  printf("Plate %s: %zu points of %zu; GPD looks for grasps only around them.\n",
         color_name.c_str(), plate.size(), cloud.size());
  return true;
}
