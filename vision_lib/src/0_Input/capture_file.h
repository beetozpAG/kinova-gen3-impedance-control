// File:    capture_file.h
// Module:  0_Input
// Purpose: Reads back the color part of a capture saved by saveCapture
//          (depth_camera.h), so the color steps can run on saved captures,
//          without the robot. The cloud is read with pcl::io::loadPCDFile
//          and the robot pose with readPoseFile (frames.h).
#pragma once

#include <string>

#include <opencv2/core.hpp>

#include "0_Input/vision_module.h"

// readColorCapture: reads the color camera parameters (color_intrinsics
// line) and the color image (color_image line, a .png in the same folder as
// the pose file) of NAME_pose.txt.
// Returns false if the file has no color lines (captures saved before the
// color camera was added) or if the image cannot be read; the second case
// prints why.
bool readColorCapture(const std::string &pose_file, Intrinsics &color_intr, cv::Mat &color_image);
