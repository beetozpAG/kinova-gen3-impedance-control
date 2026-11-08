// File:    largest_cluster.cpp
// Module:  2_GetROI
// Purpose: The largest group of touching points of the plate color. See
//          largest_cluster.h for why and how.
#include "2_GetROI/largest_cluster.h"

#include <cstdio>

#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// largestCluster: copies the points, groups them with a kd-tree search
// (radius CLUSTER_TOLERANCE), then maps the largest group back to the
// indices of cloud. PCL returns the groups from the largest to the smallest.
std::vector<int> largestCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                const std::vector<int> &indices) {
  const size_t MIN_POINTS = 10;
  if (indices.size() < MIN_POINTS) {
    return indices;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i : indices) {
    points->push_back(cloud[i]);
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(points);

  std::vector<pcl::PointIndices> groups;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering;
  clustering.setClusterTolerance(CLUSTER_TOLERANCE);
  clustering.setMinClusterSize(1);
  clustering.setSearchMethod(tree);
  clustering.setInputCloud(points);
  clustering.extract(groups);
  if (groups.empty()) {
    return indices;
  }

  std::vector<int> kept;
  for (int j : groups[0].indices) {
    kept.push_back(indices[j]);
  }
  printf("Largest group: %zu of %zu points kept (%zu groups, %.0f mm apart).\n", kept.size(),
         indices.size(), groups.size(), CLUSTER_TOLERANCE * 1000.0);
  return kept;
}
