// File:    depth_filter.cpp
// Module:  1_PreprocessCloud
// Purpose: First-order low-pass filter of the depth image. See
//          depth_filter.h for the formula and when to use it.
#include "1_PreprocessCloud/depth_filter.h"

#include <cmath>
#include <cstdint>

// filterDepth: y is kept in float (mm), so the decimals are not lost from
// one step to the next (in integers y would stop a few mm from u).
void filterDepth(const cv::Mat &u, cv::Mat &y, double w, double delta) {
  // y(0) = u(0); also if the image size changed.
  if (y.empty() || y.size() != u.size()) {
    u.convertTo(y, CV_32F);
    return;
  }
  const float alpha = static_cast<float>(1.0 - std::exp(-w * delta));
  for (int row = 0; row < u.rows; row++) {
    for (int col = 0; col < u.cols; col++) {
      const float u_i = u.at<uint16_t>(row, col);
      float &y_i = y.at<float>(row, col);
      if (u_i == 0.0f) {
        continue;  // no measurement: keep y
      }
      if (y_i == 0.0f) {
        y_i = u_i;  // first measurement of this pixel
        continue;
      }
      y_i = (1.0f - alpha) * y_i + alpha * u_i;
    }
  }
}
