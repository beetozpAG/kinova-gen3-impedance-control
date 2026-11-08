// File:    plate_samples.h
// Module:  3_Sample
// Purpose: Chooses where GPD looks for grasps: only on the plate of one
//          color. The points of that color (2_GetROI/color_mask.h) become
//          the samples of GPD (Algorithm 1, step 3: "N samples in R").
//          GPD still gets the whole cloud, to check that the hand does not
//          hit anything; it only looks for grasps around these samples.
#pragma once

#include <string>

#include <Eigen/Dense>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "0_Input/vision_module.h"

// plateSamples: the points of cloud (DEPTH frame) whose color, taken from
// color_image, is inside the range named color_name in PLATE_COLORS_FILE
// (color_mask.h). One column per point, in the DEPTH frame, ready for
// detectGraspsInCloud (Pipeline/grasp_pipeline.h). Prints how many there are.
// Returns false (and prints why) if the color is not in the file or no
// point has that color.
bool plateSamples(const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &color_image,
                  const Intrinsics &color_intr, const std::string &color_name,
                  Eigen::Matrix3Xd &samples);
