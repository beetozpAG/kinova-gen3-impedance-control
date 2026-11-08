// hand_eye_tsai: block C3a of the camera calibration (src/Calibration/).
// Finds where the color camera is on the wrist (T_int_color) with the
// method of Tsai and Lenz (1989), from the photos of calib_capture. No robot needed.
// How the method works: Calibration/hand_eye_tsai.h.
//
// Usage (run from kinova_grasp/build/):
//   ./hand_eye_tsai FOLDER       e.g. ./hand_eye_tsai calib_0311_101500
//
// Writes FOLDER/hand_eye_tsai.txt (read by hand_eye_compare, block C4).
//
// Steps:
//   1. Read the photos and find the board in each one (hand_eye_data.h, C2).
//   2. Solve A X = X B with the method.
//   3. Save and print the result, next to the manual (Table 92, p.225).

#include <iostream>
#include <string>

#include "Robot/Gen3_model.h"  // only for TOOL_OFFSET_Z_2F85 (0.120 m)
#include "Calibration/hand_eye_tsai.h"

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "Usage: hand_eye_tsai FOLDER   (FOLDER from calib_capture)\n";
    return 1;
  }
  const std::string folder = argv[1];

  // ---------- 1. Photos ----------
  HandEyeData data;
  if (!readHandEyeData(folder, KinovaGen3::TOOL_OFFSET_Z_2F85, data)) {
    return 1;
  }

  // ---------- 2. Method ----------
  const HandEyeResult result = handEyeTsai(data);

  // ---------- 3. Save ----------
  return writeHandEyeResult(folder, result) ? 0 : 1;
}
