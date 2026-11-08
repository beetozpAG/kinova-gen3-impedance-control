// File:    depth_filter.h
// Module:  1_PreprocessCloud
// Purpose: First-order low-pass filter of the depth image, pixel by pixel.
//          It removes the noise of the depth camera (the depth of a pixel
//          changes a little from one image to the next, even with the
//          scene still), so the cloud built from it is smoother.
//          Full explanation: Documents/filtro_primer_orden.pdf.
//
// Filter: H(s) = w / (s + w), that is dy/dt = w (u - y). Discrete, exact for
// u constant during delta:
//   y_i = (1 - alpha) * y_(i-1) + alpha * u_i,   alpha = 1 - exp(-w * delta)
//   y_0 = u_0
// alpha is always between 0 and 1, for any w and delta (the time between
// images, measured: it changes from one image to the next).
//
// Why the depth image and not the cloud: depthToCloud drops the pixels with
// no depth, so point i of one cloud is not the same pixel as point i of the
// next. A pixel of the image is always the same direction of the camera.
// For one pixel, x and y are proportional to z (depth_camera.cpp), so
// filtering z is the same as filtering the point.
//
// Use it with the robot still: the camera is on the wrist, so if the arm
// moves the filter mixes different views for 3 to 5 tau (tau = 1 / w).
#pragma once

#include <opencv2/core.hpp>

// Cut-off frequency of the filter [rad/s]. Time to react tau = 1 / w
// (95% of a change after 3 tau); noise left sqrt(alpha / (2 - alpha)).
// (chosen) 2 rad/s: tau 0.5 s, 95% in 1.5 s; at 10 images/s alpha = 0.18
// and the noise drops to about 1/3. 0 = no filter.
const double DEPTH_FILTER_W = 3.0;

// filterDepth: one step of the filter.
//   u:     new depth image (CV_16UC1, mm, 0 = no measurement).
//   y:     filtered depth (CV_32FC1, mm), updated in place. Empty = first
//          image: y = u. Keep it between calls.
//   w:     cut-off frequency [rad/s].
//   delta: time since the previous image [s].
// Pixels with u = 0 keep their y (0 is "no measurement", not a depth).
// Pixels with y = 0 (never measured) take u.
void filterDepth(const cv::Mat &u, cv::Mat &y, double w, double delta);
