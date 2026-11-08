// File:    color_mask.h
// Module:  2_GetROI
// Purpose: Finds the points of the cloud that have the color of a plate.
//          Those points are the region of interest: the plate to grasp.
//          The colors come from the color camera (0_Input/color_cloud.h)
//          and the range of each plate from cfg/plate_colors.cfg.
#pragma once

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// File with the color range of each plate. Path from kinova_grasp/build/,
// where the programs run (same as cfg/gpd_params.cfg).
const std::string PLATE_COLORS_FILE = "../cfg/plate_colors.cfg";

// Keys of the PCL viewer (its help, key h): a plate color cannot use them.
// n and t are ours: "no mask" and "depth filter on/off" in view_cloud.
// r (reset camera) and p (draw with points) are left out on purpose: red
// and pink use them, and their PCL action does no harm (r only moves the
// view back to the start).
const std::string VIEWER_KEYS = "wsjcfeqguolxhnt0123456789+-";

// Color range of one plate, in HSV (OpenCV units: H 0-180, S and V 0-255).
// If h_min > h_max the hue range wraps around 180 (red).
struct ColorRange {
  std::string name;
  char key;  // key that shows this color in view_cloud
  int h_min, h_max;
  int s_min, s_max;
  int v_min, v_max;
};

// readColorRanges: reads all the color ranges of a .cfg file (one line per
// plate: name key H_min H_max S_min S_max V_min V_max; "#" starts a
// comment). A line whose key is a PCL viewer key (VIEWER_KEYS) or the key
// of another color is skipped, with a message.
// Returns false (and prints why) if the file cannot be read or has no
// valid line.
bool readColorRanges(const std::string &file_name, std::vector<ColorRange> &ranges);

// readColorRange: the range called name in the .cfg file. Returns false (and
// prints the names that are in the file) if there is none with that name.
bool readColorRange(const std::string &file_name, const std::string &name, ColorRange &range);

// colorMask: indices of the points of cloud whose color is inside range.
// The indices are those of cloud, so they also point to the same points of
// the cloud without colors (colorCloud keeps the order).
std::vector<int> colorMask(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const ColorRange &range);

// colorMaskImage: the pixels of color_image (8 bits, BGR) whose color is
// inside range: 255 in the mask, 0 elsewhere. Same size as the image.
cv::Mat colorMaskImage(const cv::Mat &color_image, const ColorRange &range);
