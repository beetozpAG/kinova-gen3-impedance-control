// File:    hand_eye_daniilidis.cpp
// Module:  Calibration (block C3c: method of Daniilidis)
// Purpose: Camera mount with Daniilidis (1999). See hand_eye_daniilidis.h.
#include "Calibration/hand_eye_daniilidis.h"

// handEyeDaniilidis: OpenCV implements the steps of the paper.
HandEyeResult handEyeDaniilidis(const HandEyeData &data) {
  HandEyeResult result;
  result.method = "daniilidis";
  result.views = static_cast<int>(data.names.size());
  result.T_int_color = solveHandEye(data, cv::CALIB_HAND_EYE_DANIILIDIS);
  return result;
}
