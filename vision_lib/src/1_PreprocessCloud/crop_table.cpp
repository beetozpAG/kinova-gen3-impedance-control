// File:    crop_table.cpp
// Module:  1_PreprocessCloud
// Purpose: Drops the points that are below the table. See crop_table.h for
//          the table height and the margin.
#include "1_PreprocessCloud/crop_table.h"

#include <cstdio>

// cropBelowTable: only the height of each point in BASE is needed, so only
// the third row of T_base_depth is used:
//   z_base = T(2,0) * x + T(2,1) * y + T(2,2) * z + T(2,3)
pcl::PointCloud<pcl::PointXYZ> cropBelowTable(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                              const Eigen::Matrix4d &T_base_depth) {
  const double z_min = TABLE_Z_BASE - TABLE_MARGIN;
  pcl::PointCloud<pcl::PointXYZ> kept;

  for (const pcl::PointXYZ &p : cloud) {
    const double z_base = T_base_depth(2, 0) * p.x + T_base_depth(2, 1) * p.y +
                          T_base_depth(2, 2) * p.z + T_base_depth(2, 3);
    if (z_base > z_min) {
      kept.push_back(p);
    }
  }

  printf("Crop below the table (z in BASE > %.3f m): %zu of %zu points kept\n", z_min,
         kept.size(), cloud.size());
  return kept;
}
