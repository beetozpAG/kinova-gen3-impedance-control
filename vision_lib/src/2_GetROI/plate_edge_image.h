// File:    plate_edge_image.h
// Module:  2_GetROI
// Purpose: Measures the edge of the plate in the color image instead of the
//          depth cloud: the color image has about 4 times more pixels per
//          mm (1280 x 720, fx 1298, against 480 x 270, fx 334), and its
//          borders are sharp. Gives the same PlateEdge as plate_edge.h, so
//          the grasps can be compared with the middle of the edge.
//
// Steps:
//   1. Pixels of the plate color (colorMaskImage, color_mask.h).
//   2. Spots smaller than a few pixels erased (opening), then the largest
//      connected region of those pixels.
//   3. PCA of the pixels of the region: axis along the edge, ends (1 and 99
//      percentiles, as plate_edge.h), middle, thickness in pixels.
//   4. Pixels -> 3D: the cloud points that fall in the region (DepthToColor,
//      0_Input/color_cloud.h), without the background seen past its
//      borders, give a line z(s) along the edge; the middle and the ends
//      at their z, back to the DEPTH frame.
//
// No thickness is assumed: it is only measured and printed.
// Limits: the whole edge must be in the image; a false region of the same
// color that is larger than the plate is taken instead of it.
#pragma once

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "0_Input/vision_module.h"
#include "2_GetROI/color_mask.h"
#include "2_GetROI/plate_edge.h"

// measurePlateEdgeImage: edge of the plate of color range, in the DEPTH
// frame. cloud is in the DEPTH frame (it gives the depth of the region).
// Returns false (and prints why) if the region or its depth is too small.
bool measurePlateEdgeImage(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                           const cv::Mat &color_image, const Intrinsics &color_intr,
                           const ColorRange &range, PlateEdge &edge);
