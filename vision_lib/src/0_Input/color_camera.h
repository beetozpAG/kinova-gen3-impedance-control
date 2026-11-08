// File:    color_camera.h
// Module:  0_Input
// Purpose: Reads the color camera on the robot wrist: its parameters and its
//          video. With them, each point of the depth cloud can be given the
//          color of the scene (the plates are told apart by their color).
//          It only reads from the robot: it never changes its settings.
//          Its parameters are read with readIntrinsics (vision_module.h).
//
// Notes (user guide r12.0):
//   - Sensor: Omnivision OV5640 (p.21). Resolutions 320x240, 640x480,
//     1280x720 and 1920x1080 (p.22); the one in use is set in the Web App
//     (Configuration > Robot > Vision, Table 67, p.135).
//   - Focusing range: 30 cm to infinity (p.22). Closer than 30 cm the color
//     image is blurred, even if the depth camera still measures (18 cm).
//   - The color and depth videos are separate streams: their images are not
//     taken at the same instant. The robot and the scene must be still.
#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

// openColorStream: opens the color video of the robot camera. Each image is
// 8 bits per channel, BGR order (the OpenCV order).
// keep_only_latest = true for live views: old images are dropped, so the
// view does not fall behind. Returns false (and prints why) if it fails.
bool openColorStream(const std::string &ip_address, bool keep_only_latest, cv::VideoCapture &stream);

// captureColorImage: takes one color image (8 bits, BGR) from the robot
// camera. Returns false (and prints why) if it fails.
bool captureColorImage(const std::string &ip_address, cv::Mat &color_image);
