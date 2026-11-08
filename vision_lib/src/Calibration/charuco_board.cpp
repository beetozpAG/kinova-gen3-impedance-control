// File:    charuco_board.cpp
// Module:  Calibration (block C0: the board)
// Purpose: The ChArUco board and its PDF. See charuco_board.h.
#include "Calibration/charuco_board.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>

// Pixels per millimeter of the board image in the PDF: 10 (254 dpi). 30 mm
// is exactly 300 px, so the edges of the squares fall on pixel edges.
static const int PX_PER_MM = 10;

// Points per millimeter in a PDF (1 point = 1/72 inch).
static const double PT_PER_MM = 72.0 / 25.4;

// Letter page, landscape [mm] (279.4 x 215.9).
static const double PAGE_W_MM = 279.4;
static const double PAGE_H_MM = 215.9;

// readBoardSize: "key = value" lines; "#" starts a comment.
bool readBoardSize(const std::string &file_name, BoardSize &size) {
  std::ifstream file(file_name);
  if (!file.good()) {
    printf("Could not read the board file %s\n", file_name.c_str());
    return false;
  }
  bool has_square = false, has_marker = false;
  std::string line;
  while (std::getline(file, line)) {
    const size_t equal_sign = line.find('=');
    if (line.empty() || line[0] == '#' || equal_sign == std::string::npos) {
      continue;
    }
    std::istringstream key_stream(line.substr(0, equal_sign));
    std::string key;
    key_stream >> key;
    std::istringstream values(line.substr(equal_sign + 1));
    double mm = 0.0;
    values >> mm;
    if (values.fail()) {
      continue;
    }
    if (key == "square_mm") {
      size.square = mm / 1000.0;
      has_square = true;
    } else if (key == "marker_mm") {
      size.marker = mm / 1000.0;
      has_marker = true;
    }
  }
  if (!has_square || !has_marker) {
    printf("%s needs square_mm and marker_mm\n", file_name.c_str());
    return false;
  }
  return true;
}

// makeBoard: OpenCV CharucoBoard with the sizes in meters.
cv::aruco::CharucoBoard makeBoard(const BoardSize &size) {
  return cv::aruco::CharucoBoard(cv::Size(BOARD_COLS, BOARD_ROWS), static_cast<float>(size.square),
                                 static_cast<float>(size.marker),
                                 cv::aruco::getPredefinedDictionary(BOARD_DICTIONARY));
}

// pdfText: one line of text (Helvetica) at (x, y) [mm from the bottom left].
static std::string pdfText(double x_mm, double y_mm, double size_pt, const std::string &text) {
  char buffer[512];
  snprintf(buffer, sizeof(buffer), "BT /F1 %.1f Tf %.3f %.3f Td (%s) Tj ET\n", size_pt,
           x_mm * PT_PER_MM, y_mm * PT_PER_MM, text.c_str());
  return buffer;
}

// pdfRect: a filled black rectangle [mm from the bottom left].
static std::string pdfRect(double x_mm, double y_mm, double w_mm, double h_mm) {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "%.3f %.3f %.3f %.3f re f\n", x_mm * PT_PER_MM, y_mm * PT_PER_MM,
           w_mm * PT_PER_MM, h_mm * PT_PER_MM);
  return buffer;
}

// writeBoardPdf: the board image from OpenCV (1 bit per pixel), placed at
// its size in mm, plus the check bar and the text. The PDF is written by
// hand (5 objects: catalog, pages, page, content, image, font).
bool writeBoardPdf(const std::string &file_name) {
  // 1. Board image: 2100 x 1500 px, no margin, marker border of 1 bit.
  const BoardSize printed{BOARD_SQUARE_MM / 1000.0, BOARD_MARKER_MM / 1000.0};
  const cv::aruco::CharucoBoard board = makeBoard(printed);
  const double board_w_mm = BOARD_COLS * BOARD_SQUARE_MM;
  const double board_h_mm = BOARD_ROWS * BOARD_SQUARE_MM;
  const cv::Size image_size(static_cast<int>(board_w_mm * PX_PER_MM),
                            static_cast<int>(board_h_mm * PX_PER_MM));
  cv::Mat image;
  board.generateImage(image_size, image, 0, 1);

  // 1 bit per pixel, rows padded to a whole byte; in DeviceGray 0 = black.
  const int row_bytes = (image.cols + 7) / 8;
  std::string bits(static_cast<size_t>(row_bytes) * image.rows, '\0');
  for (int r = 0; r < image.rows; r++) {
    for (int c = 0; c < image.cols; c++) {
      if (image.at<uchar>(r, c) > 127) {
        bits[r * row_bytes + c / 8] |= static_cast<char>(0x80 >> (c % 8));
      }
    }
  }

  // 2. Page content: board centered, 25 mm from the top; bar and text below.
  const double board_x = (PAGE_W_MM - board_w_mm) / 2.0;
  const double board_y = PAGE_H_MM - 25.0 - board_h_mm;
  std::string content;
  char cm[160];
  snprintf(cm, sizeof(cm), "q %.4f 0 0 %.4f %.4f %.4f cm /Im1 Do Q\n", board_w_mm * PT_PER_MM,
           board_h_mm * PT_PER_MM, board_x * PT_PER_MM, board_y * PT_PER_MM);
  content += cm;
  content += pdfText(board_x, PAGE_H_MM - 15.0, 11,
                     "ChArUco board for the hand-eye calibration (kinova_grasp, Calibration)");

  // Check bar: 100 mm, 0.5 mm thick, a tick every 10 mm (longer at 0, 50, 100).
  const double bar_x = board_x;
  const double bar_y = board_y - 15.0;
  content += pdfRect(bar_x, bar_y, 100.0, 0.5);
  for (int mm = 0; mm <= 100; mm += 10) {
    const double tick = (mm % 50 == 0) ? 5.0 : 3.0;
    content += pdfRect(bar_x + mm - 0.15, bar_y, 0.3, tick);
  }
  content += pdfText(bar_x + 104.0, bar_y, 9, "<- this bar must measure 100.0 mm");

  char line[256];
  snprintf(line, sizeof(line),
           "%d x %d squares of %.1f mm (board %.0f x %.0f mm), markers %.1f mm, DICT_4X4_50",
           BOARD_COLS, BOARD_ROWS, BOARD_SQUARE_MM, board_w_mm, board_h_mm, BOARD_MARKER_MM);
  content += pdfText(board_x, bar_y - 9.0, 9, line);
  content += pdfText(board_x, bar_y - 14.0, 9,
                     "Print at 100% / Actual size (NOT fit to page). Laser printer, matte paper.");
  content += pdfText(board_x, bar_y - 19.0, 9,
                     "Glue it flat on a rigid board. Measure 5 squares in a row and write"
                     " (length / 5) in cfg/charuco_board.cfg.");

  // 3. PDF objects; the offset of each one goes in the xref table.
  std::string pdf = "%PDF-1.4\n";
  std::vector<size_t> offsets;
  auto addObject = [&](const std::string &body) {
    offsets.push_back(pdf.size());
    pdf += std::to_string(offsets.size()) + " 0 obj\n" + body + "\nendobj\n";
  };
  char page[256];
  snprintf(page, sizeof(page),
           "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 %.3f %.3f] /Contents 4 0 R "
           "/Resources << /XObject << /Im1 5 0 R >> /Font << /F1 6 0 R >> >> >>",
           PAGE_W_MM * PT_PER_MM, PAGE_H_MM * PT_PER_MM);
  addObject("<< /Type /Catalog /Pages 2 0 R >>");
  addObject("<< /Type /Pages /Kids [3 0 R] /Count 1 >>");
  addObject(page);
  addObject("<< /Length " + std::to_string(content.size()) + " >>\nstream\n" + content +
            "endstream");
  addObject("<< /Type /XObject /Subtype /Image /Width " + std::to_string(image.cols) +
            " /Height " + std::to_string(image.rows) +
            " /ColorSpace /DeviceGray /BitsPerComponent 1 /Interpolate false /Length " +
            std::to_string(bits.size()) + " >>\nstream\n" + bits + "\nendstream");
  addObject("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>");

  const size_t xref = pdf.size();
  pdf += "xref\n0 " + std::to_string(offsets.size() + 1) + "\n0000000000 65535 f \n";
  for (size_t offset : offsets) {
    char entry[32];
    snprintf(entry, sizeof(entry), "%010zu 00000 n \n", offset);
    pdf += entry;
  }
  pdf += "trailer\n<< /Size " + std::to_string(offsets.size() + 1) + " /Root 1 0 R >>\nstartxref\n" +
         std::to_string(xref) + "\n%%EOF\n";

  std::ofstream file(file_name, std::ios::binary);
  file << pdf;
  if (!file.good()) {
    printf("Could not write %s\n", file_name.c_str());
    return false;
  }
  return true;
}
