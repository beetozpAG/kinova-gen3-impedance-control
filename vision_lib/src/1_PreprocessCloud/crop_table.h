// File:    crop_table.h
// Module:  1_PreprocessCloud
// Purpose: Drops the points that are below the table (floor, things under the
//          table, false far points). The cut is a height in the BASE frame,
//          so it works with the camera at any height or tilt, with no value
//          to change for each capture.
#pragma once

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Height of the table top in the BASE frame [m], measured once.
// The table top is 1.5 cm below BASE: the robot stands on a ~1.5 cm plate.
// Measured on e1_full (2026-10-17): from -0.004 to -0.024 across the image,
// because the table is not level.
// Measure it again if the table or the robot is moved.
const double TABLE_Z_BASE = -0.015;  

// Points lower than TABLE_Z_BASE - TABLE_MARGIN are dropped [m].
// The margin keeps the whole table in the cloud (not level, camera noise):
// GPD needs the table to find it with RANSAC (step 1.6) and to reject hands
// that would hit it (step 3).
const double TABLE_MARGIN = 0.02;  

// cropBelowTable: keeps the points that are above the table.
// T_base_depth is the DEPTH frame with respect to the BASE frame when the
// cloud was taken (frames.h). The cloud returned stays in the DEPTH frame,
// as GPD needs it.
pcl::PointCloud<pcl::PointXYZ> cropBelowTable(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                              const Eigen::Matrix4d &T_base_depth);
