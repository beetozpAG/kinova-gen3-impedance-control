// File:    hand_eye_park.cpp
// Module:  Calibration (block C3b: method of Park and Martin)
// Purpose: Camera mount with Park and Martin (1994). See hand_eye_park.h.
#include "Calibration/hand_eye_park.h"

// handEyePark: OpenCV implements the two steps of the paper.
HandEyeResult handEyePark(const HandEyeData &data) {
  HandEyeResult result;
  result.method = "park";
  result.views = static_cast<int>(data.names.size());
  result.T_int_color = solveHandEye(data, cv::CALIB_HAND_EYE_PARK);
  return result;
}
