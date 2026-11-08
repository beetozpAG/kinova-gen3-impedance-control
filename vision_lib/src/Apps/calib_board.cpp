// calib_board: block C0 of the camera calibration (src/Calibration/).
// Writes the ChArUco board to print, at its exact size, and checks that
// OpenCV finds all its corners in it. No robot needed.
//
// Usage (run from kinova_grasp/build/):
//   ./calib_board
//
// Writes:
//   charuco_board.pdf          to print (letter, landscape, 100% / Actual size)
//   charuco_board_preview.png  the board alone, to look at it on screen
//
// Steps (Calibration/charuco_board.h):
//   1. PDF at the printed size (BOARD_SQUARE_MM, BOARD_MARKER_MM).
//   2. Self-check: the board image must give all its inner corners.

#include <cstdio>
#include <iostream>

#include <opencv2/imgcodecs.hpp>

#include "Calibration/charuco_board.h"

int main(int argc, char *argv[]) {
  (void)argv;
  if (argc != 1) {
    std::cout << "Usage: calib_board   (writes charuco_board.pdf in the current folder)\n";
    return 1;
  }

  // ---------- 1. PDF ----------
  const std::string pdf_file = "charuco_board.pdf";
  if (!writeBoardPdf(pdf_file)) {
    return 1;
  }

  // ---------- 2. Self-check ----------
  // A preview with a white margin of one square (detection needs it).
  const BoardSize printed{BOARD_SQUARE_MM / 1000.0, BOARD_MARKER_MM / 1000.0};
  const cv::aruco::CharucoBoard board = makeBoard(printed);
  const int px_per_square = 100;
  cv::Mat preview;
  board.generateImage(cv::Size((BOARD_COLS + 2) * px_per_square, (BOARD_ROWS + 2) * px_per_square),
                      preview, px_per_square, 1);
  const std::string png_file = "charuco_board_preview.png";
  cv::imwrite(png_file, preview);

  std::vector<cv::Point2f> corners;
  std::vector<int> ids;
  cv::aruco::CharucoDetector(board).detectBoard(preview, corners, ids);
  const int expected = (BOARD_COLS - 1) * (BOARD_ROWS - 1);

  std::cout << "\n========== BOARD ==========\n";
  printf("  %d x %d squares of %.1f mm, markers %.1f mm, DICT_4X4_50\n", BOARD_COLS, BOARD_ROWS,
         BOARD_SQUARE_MM, BOARD_MARKER_MM);
  printf("  Board %.0f x %.0f mm, on a letter sheet, landscape\n", BOARD_COLS * BOARD_SQUARE_MM,
         BOARD_ROWS * BOARD_SQUARE_MM);
  printf("  Self-check: %zu of %d inner corners found %s\n", ids.size(), expected,
         static_cast<int>(ids.size()) == expected ? "(OK)" : "(ERROR)");
  std::cout << "Saved: " << pdf_file << ", " << png_file << "\n"
            << "Print at 100% / Actual size, measure it and write the sizes in "
            << BOARD_FILE << "\n"
            << "===========================\n";
  return static_cast<int>(ids.size()) == expected ? 0 : 1;
}
