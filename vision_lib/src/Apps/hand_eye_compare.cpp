// hand_eye_compare: block C4 of the camera calibration (src/Calibration/).
// Takes the results of the three methods (hand_eye_tsai, hand_eye_park,
// hand_eye_daniilidis: blocks C3) and the photos, and says how good each
// one is and how much they agree (Calibration/compare_hand_eye.h). Writes
// the best one as FOLDER/camera_calibration_suggested.cfg (block C5).
// No robot needed.
//
// Usage (run from kinova_grasp/build/), after the three methods:
//   ./hand_eye_compare FOLDER       e.g. ./hand_eye_compare calib_0311_101500
//
// Steps:
//   1. Read the photos (as the methods) and the result files.
//   2. C4b. Validation of each result, and of the manual and the cfg in use.
//   3. C4a. Agreement between the methods.
//   4. Verdict and suggested cfg.

#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "Robot/Gen3_model.h"  // only for TOOL_OFFSET_Z_2F85 (0.120 m)
#include "0_Input/frames.h"
#include "Calibration/compare_hand_eye.h"

// printRow: one line of the table.
static void printRow(const std::string &label, const Eigen::Matrix4d &X, const HandEyeCheck &c) {
  const Eigen::Vector3d t = X.block<3, 1>(0, 3) * 1000.0;
  const Eigen::Vector3d r = anglesXYZ(X.block<3, 3>(0, 0));
  printf("  %-11s %7.2f %7.2f %7.2f | %6.3f %6.3f %6.3f | %6.2f %6.2f %6.3f | %5.2f\n",
         label.c_str(), t.x(), t.y(), t.z(), r.x(), r.y(), r.z(), c.spread_rms_mm, c.spread_max_mm,
         c.rotation_rms_deg, c.reprojection_px);
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cout << "Usage: hand_eye_compare FOLDER   (after hand_eye_tsai, _park, _daniilidis)\n";
    return 1;
  }
  const std::string folder = argv[1];

  // ---------- 1. Photos and results ----------
  HandEyeData data;
  if (!readHandEyeData(folder, KinovaGen3::TOOL_OFFSET_Z_2F85, data)) {
    return 1;
  }
  std::vector<HandEyeResult> results;
  for (const std::string method : {"tsai", "park", "daniilidis"}) {
    HandEyeResult result;
    if (readHandEyeResult(handEyeFile(folder, method), result)) {
      results.push_back(result);
    } else {
      printf("No result of %s (run ./hand_eye_%s %s first)\n", method.c_str(), method.c_str(),
             folder.c_str());
    }
  }
  if (results.empty()) {
    return 1;
  }

  // ---------- 2. Validation (C4b) ----------
  std::cout << "\n========== VALIDATION (" << data.names.size() << " photos) ==========\n"
            << "  COLOR in INT: position [mm] | rotation x-y-z [deg] | board in BASE: spread\n"
            << "  RMS, max [mm] and rotation RMS [deg] | reprojection [px]\n";
  printf("  %-11s %7s %7s %7s | %6s %6s %6s | %6s %6s %6s | %5s\n", "", "x", "y", "z", "rx", "ry",
         "rz", "rms", "max", "rot", "px");
  const HandEyeCheck manual = checkHandEye(data, colorToIntManual());
  printRow("manual", colorToIntManual(), manual);
  const bool cfg_is_manual = colorToInt().isApprox(colorToIntManual(), 1e-9);
  if (!cfg_is_manual) {
    printRow("cfg in use", colorToInt(), checkHandEye(data, colorToInt()));
  }
  std::vector<HandEyeCheck> checks;
  size_t best = 0;
  for (size_t m = 0; m < results.size(); m++) {
    checks.push_back(checkHandEye(data, results[m].T_int_color));
    printRow(results[m].method, results[m].T_int_color, checks[m]);
    if (checks[m].spread_rms_mm < checks[best].spread_rms_mm) {
      best = m;
    }
  }

  // ---------- 3. Agreement (C4a) ----------
  std::cout << "\n========== AGREEMENT ==========\n";
  bool agree = true;
  for (size_t a = 0; a < results.size(); a++) {
    for (size_t b = a + 1; b < results.size(); b++) {
      double mm = 0.0, deg = 0.0;
      motionDifference(results[a].T_int_color, results[b].T_int_color, mm, deg);
      const bool ok = mm <= AGREE_MM && deg <= AGREE_DEG;
      agree = agree && ok;
      printf("  %-10s vs %-10s %5.2f mm  %5.3f deg  %s\n", results[a].method.c_str(),
             results[b].method.c_str(), mm, deg, ok ? "OK" : "DIFFERENT");
    }
  }

  // ---------- 4. Verdict ----------
  const HandEyeCheck &c = checks[best];
  std::cout << "\n========== VERDICT ==========\n";
  printf("  Best: %s, board spread %.2f mm RMS (manual: %.2f mm)\n", results[best].method.c_str(),
         c.spread_rms_mm, manual.spread_rms_mm);
  printf("  Methods agree (%.1f mm, %.1f deg): %s\n", AGREE_MM, AGREE_DEG, agree ? "yes" : "NO");
  printf("  Spread below %.1f mm: %s (worst photo: %s, %.2f mm)\n", SPREAD_OK_MM,
         c.spread_rms_mm <= SPREAD_OK_MM ? "yes" : "NO", c.worst_view.c_str(), c.spread_max_mm);
  if (agree && c.spread_rms_mm <= SPREAD_OK_MM) {
    std::cout << "  Good: copy the suggested lines into cfg/camera_calibration.cfg.\n";
  } else {
    std::cout << "  Not good enough: see \"If the error is large\" in src/Calibration/README.md.\n";
  }
  writeSuggestedCfg(folder, results[best], c);
  std::cout << "=============================\n";
  return 0;
}
