// File:    largest_cluster.h
// Module:  2_GetROI
// Purpose: Keeps only the plate itself among the points of the plate color:
//          the largest group of points that touch each other. Loose points
//          of the same color far from the plate are dropped.
//
// Why: the depth and color cameras are 2.9 cm apart (color_cloud.h), so some
// points of the background next to the edge of the plate take the color of
// the plate. They are few, but they are far from it, and they made the edge
// longer and thicker (camera_2110_101018, pink plate of 188 mm: 274 mm with
// them, 178 mm without them).
//
// How: Euclidean clustering of PCL (pcl::EuclideanClusterExtraction): two
// points are in the same group if they are closer than CLUSTER_TOLERANCE;
// the group with the most points is the plate.
#pragma once

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Two points closer than this are in the same group [m]. Larger than the
// distance between neighbor points of the plate (a few mm), smaller than the
// gap to the loose points (cm).
const double CLUSTER_TOLERANCE = 0.01;

// largestCluster: the indices (of cloud) of the largest group among the
// points given by indices. Prints how many points are kept. Returns indices
// unchanged if there are fewer than 10 of them.
std::vector<int> largestCluster(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                const std::vector<int> &indices);
