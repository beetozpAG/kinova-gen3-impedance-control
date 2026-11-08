// File:    fuse_views.h
// Module:  Task (block 4: fusion)
// Purpose: Joins the views of a sweep (block 3, sweep_capture) into one
//          colored cloud, in the DEPTH frame of view 0, ready for GPD
//          (grasps_from_file with fused.pcd and fused_pose.txt).
//
// Why: one view has holes next to the object (stereo shadow) and even on
// it: on sweep_2410_200944, view 0 saw only 34 mm of the 160 mm plate edge
// in one piece (1298 blue points in 12 groups), view 1 saw 143 mm. With the
// 3 views, 99% of the 3 mm cells within 4 cm of the object have points
// (view 0 alone: 58%).
//
// How:
//   4a. Each view: points above the table (crop_table), colored with its
//       own color image (color_cloud), moved to BASE with its TOOL pose.
//   4b. Geometry: all of view 0. Then, view by view, only the points that
//       fall in 3 mm cells (seen from above, XY of BASE) with no point yet:
//       the holes are filled, the surfaces are not doubled. The views
//       differ by up to about 4 mm across the plate edge (measured), so
//       doubling would make the 5 mm edge look thicker.
//   4c. Back to the DEPTH frame of view 0.
// The color of each point comes from the view that saw it, so the points of
// the object color (the samples of GPD) cover the whole object.
// No ICP: on a flat table it slides and gave false corrections of 13 to
// 27 mm; the robot poses are used as they are.
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "0_Input/frames.h"
#include "0_Input/vision_module.h"

// Size of the cells, seen from above [m] (the voxel size of GPD, cfg).
const double FUSE_CELL = 0.003;

// One view of a sweep, as saved by saveCapture.
struct SweepView {
  pcl::PointCloud<pcl::PointXYZ> cloud;  // DEPTH frame [m]
  cv::Mat color_image;
  Intrinsics color_intr;
  ToolPose tool;
};

// readSweepView: reads FOLDER/view_K.pcd and FOLDER/view_K_pose.txt (and
// its color image). Returns false (and prints why) if a file is missing.
bool readSweepView(const std::string &folder, int k, SweepView &view);

// fuseViews: the fused colored cloud, in the DEPTH frame of views[0].
// ee_offset_z: TOOL ahead of INT along Z (as in intToBase, frames.h).
// added[k] is the number of points taken from view k (all for view 0).
pcl::PointCloud<pcl::PointXYZRGB> fuseViews(const std::vector<SweepView> &views,
                                            double ee_offset_z, std::vector<size_t> &added);
