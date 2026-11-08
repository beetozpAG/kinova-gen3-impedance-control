// File:    color_mask.cpp
// Module:  2_GetROI
// Purpose: Reads the color ranges of the plates and finds the points of each
//          color. See color_mask.h for what each function returns.
#include "2_GetROI/color_mask.h"

#include <cstdio>
#include <fstream>
#include <sstream>

#include <opencv2/imgproc.hpp>

// readColorRanges: one range per line that has a name and 6 numbers.
bool readColorRanges(const std::string &file_name, std::vector<ColorRange> &ranges) {
  std::ifstream file(file_name);
  if (!file.good()) {
    printf("Could not read the color file %s\n", file_name.c_str());
    return false;
  }
  ranges.clear();
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream values(line);
    ColorRange r;
    values >> r.name >> r.key >> r.h_min >> r.h_max >> r.s_min >> r.s_max >> r.v_min >> r.v_max;
    if (values.fail()) {
      continue;
    }
    if (VIEWER_KEYS.find(r.key) != std::string::npos) {
      printf("Color %s skipped: key %c is a key of the PCL viewer (%s).\n", r.name.c_str(), r.key,
             VIEWER_KEYS.c_str());
      continue;
    }
    bool repeated = false;
    for (const ColorRange &other : ranges) {
      repeated = repeated || other.key == r.key;
    }
    if (repeated) {
      printf("Color %s skipped: key %c is already used by another color.\n", r.name.c_str(),
             r.key);
      continue;
    }
    ranges.push_back(r);
  }
  if (ranges.empty()) {
    printf("No color range in %s\n", file_name.c_str());
    return false;
  }
  return true;
}

// readColorRange: reads all the ranges and keeps the one with that name.
bool readColorRange(const std::string &file_name, const std::string &name, ColorRange &range) {
  std::vector<ColorRange> ranges;
  if (!readColorRanges(file_name, ranges)) {
    return false;
  }
  for (const ColorRange &r : ranges) {
    if (r.name == name) {
      range = r;
      return true;
    }
  }
  printf("No color \"%s\" in %s. Colors in the file:", name.c_str(), file_name.c_str());
  for (const ColorRange &r : ranges) {
    printf(" %s", r.name.c_str());
  }
  printf("\n");
  return false;
}

// insideRange: is one HSV color inside the range? (hue wraps if h_min > h_max)
static bool insideRange(const cv::Vec3b &hsv, const ColorRange &range) {
  const int h = hsv[0], s = hsv[1], v = hsv[2];
  const bool wraps = range.h_min > range.h_max;
  const bool h_ok = wraps ? (h >= range.h_min || h <= range.h_max)
                          : (h >= range.h_min && h <= range.h_max);
  return h_ok && s >= range.s_min && s <= range.s_max && v >= range.v_min && v <= range.v_max;
}

// colorMask: all the colors go into one image of 1 x N pixels, so OpenCV
// converts them to HSV in one call; then each pixel is checked.
std::vector<int> colorMask(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const ColorRange &range) {
  std::vector<int> indices;
  if (cloud.empty()) {
    return indices;
  }
  cv::Mat bgr(1, static_cast<int>(cloud.size()), CV_8UC3);
  for (size_t i = 0; i < cloud.size(); i++) {
    bgr.at<cv::Vec3b>(0, static_cast<int>(i)) = cv::Vec3b(cloud[i].b, cloud[i].g, cloud[i].r);
  }
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  for (int i = 0; i < hsv.cols; i++) {
    if (insideRange(hsv.at<cv::Vec3b>(0, i), range)) {
      indices.push_back(i);
    }
  }
  return indices;
}

// colorMaskImage: the whole image to HSV, then each pixel is checked.
cv::Mat colorMaskImage(const cv::Mat &color_image, const ColorRange &range) {
  cv::Mat hsv;
  cv::cvtColor(color_image, hsv, cv::COLOR_BGR2HSV);
  cv::Mat mask(hsv.rows, hsv.cols, CV_8U, cv::Scalar(0));
  for (int row = 0; row < hsv.rows; row++) {
    for (int col = 0; col < hsv.cols; col++) {
      if (insideRange(hsv.at<cv::Vec3b>(row, col), range)) {
        mask.at<uint8_t>(row, col) = 255;
      }
    }
  }
  return mask;
}
