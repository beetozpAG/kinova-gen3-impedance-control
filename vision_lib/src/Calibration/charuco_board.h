// File:    charuco_board.h
// Module:  Calibration (block C0: the board)
// Purpose: The printed pattern of the camera calibration: a ChArUco board
//          (a chessboard with an ArUco marker in each white square), and the
//          PDF to print it at its exact size.
//
// Why ChArUco and not a plain chessboard: each inner corner of the
// chessboard has a number (given by the markers around it), so the board
// is found even if part of it is out of the image or covered, and there is
// no doubt about which corner is which when the camera turns 180 deg.
// The pose uses the chessboard corners (very precise, sub-pixel); the
// markers only give the numbers.
// Reference: OpenCV, "Detection of ChArUco Boards",
// https://docs.opencv.org/4.10.0/df/d4a/tutorial_charuco_detection.html
//
// Size: 7 x 5 squares of 30 mm = 210 x 150 mm (fits a letter sheet). At
// 0.40 m the color camera (1280 x 720, fx = 1298 px) sees 0.39 x 0.22 m, so
// the whole board fits with room to tilt the camera.
#pragma once

#include <string>

#include <opencv2/objdetect/charuco_detector.hpp>

// Squares of the board: columns x rows. 5 rows (odd): the same board in
// OpenCV 4.6+ and older versions (the "legacy pattern" only changes boards
// with an even number of rows).
const int BOARD_COLS = 7;
const int BOARD_ROWS = 5;

// Printed size [mm]: the side of a square and of a marker (black border
// included). The marker is smaller than its white square, so the chessboard
// corners are clear.
const double BOARD_SQUARE_MM = 30.0;
const double BOARD_MARKER_MM = 22.0;

// ArUco dictionary: 4 x 4 bits, 50 markers (17 are used). Large bits, so
// they are read well from far.
const cv::aruco::PredefinedDictionaryType BOARD_DICTIONARY = cv::aruco::DICT_4X4_50;

// The measured board: the sizes measured with a ruler on the printed sheet
// (the printer can shrink it a little). Written by hand, no need to compile.
const std::string BOARD_FILE = "../cfg/charuco_board.cfg";

// Sizes of the board in use [m].
struct BoardSize {
  double square;  // side of a square
  double marker;  // side of a marker
};

// readBoardSize: square_mm and marker_mm of BOARD_FILE, in meters.
// Returns false (and prints why) if the file or a key is missing.
bool readBoardSize(const std::string &file_name, BoardSize &size);

// makeBoard: the OpenCV board with these sizes (BOARD_COLS x BOARD_ROWS,
// BOARD_DICTIONARY). Its frame (BOARD): origin at the outer corner of the
// first square, X along the columns, Y along the rows, Z into the board.
cv::aruco::CharucoBoard makeBoard(const BoardSize &size);

// writeBoardPdf: a one-page PDF (letter, landscape) with the board at the
// printed size BOARD_SQUARE_MM, BOARD_MARKER_MM: the page is in millimeters
// (1 mm = 72 / 25.4 points), so with "Actual size / 100%" the printer keeps
// the size. Below the board: a 100 mm bar to check the scale and the board
// data. Returns false (and prints why) if the file cannot be written.
bool writeBoardPdf(const std::string &file_name);
