// File:    hand_eye_tsai.cpp
// Module:  Calibration (block C3a: method of Tsai and Lenz)
// Purpose: Camera mount with Tsai and Lenz (1989). See hand_eye_tsai.h.
#include "Calibration/hand_eye_tsai.h"

// handEyeTsai: OpenCV implements the two steps of the paper.
HandEyeResult handEyeTsai(const HandEyeData &data) {
  HandEyeResult result;
  result.method = "tsai";
  result.views = static_cast<int>(data.names.size());
  result.T_int_color = solveHandEye(data, cv::CALIB_HAND_EYE_TSAI);
  return result;
}
