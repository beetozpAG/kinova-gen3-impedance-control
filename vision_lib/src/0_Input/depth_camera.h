// File:    depth_camera.h
// Module:  0_Input
// Purpose: Takes one point cloud from the depth camera on the robot wrist,
//          together with the robot configuration at that moment, and saves
//          both to files. It only reads from the robot: it never moves it
//          or changes its settings.
#pragma once

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <RouterClient.h>

#include "0_Input/frames.h"
#include "0_Input/vision_module.h"

// Sign of the X and Y axes of the cloud: x = -(z/fx)(col - cx), y = -(z/fy)(row - cy).
// Why (user guide r12.0):
//   - DEPTH has the same orientation as INT; its origin is the left imager,
//     at +27.5 mm along X_INT (Table 92, p.225).
//   - "Left" is seen from the sensor looking out at the scene (p.22), so
//     +X_INT is on the left of the image. Image columns grow to the right,
//     so a larger column is toward -X_INT: x needs the minus sign.
//   - Z is the same (the camera looks along +Z_INT), so y also needs the
//     minus sign to keep a right-handed frame.
// Checked on the robot (2026-10-16, with live_cloud, now view_cloud): an
// object moved toward +X_INT and +Y_INT appears at +X and +Y in the cloud.
const bool FLIP_XY = true;

// Number of images read and thrown away before the one we keep. The first
// images of a new video stream can be incomplete while the decoder starts.
const int FRAMES_TO_SKIP = 10;

// One capture: the point cloud, the color image and the robot configuration
// when they were taken.
struct Capture
{
    pcl::PointCloud<pcl::PointXYZ> cloud; // DEPTH frame [m]
    Intrinsics intr;                      // depth camera parameters
    cv::Mat color_image;                  // color image, 8 bits BGR (empty = not taken)
    Intrinsics color_intr;                // color camera parameters
    double q_deg[7];                      // joint angles [deg]
    ToolPose tool;                        // TOOL pose measured by the robot
    double depth_filter_w = 0.0;          // depth filter w [rad/s], 0 = one image, no filter
};

// openDepthStream: opens the depth video of the robot camera.
// keep_only_latest = true for live views: old images are dropped, so the
// view does not fall behind. Returns false (and prints why) if it fails.
bool openDepthStream(const std::string &ip_address, bool keep_only_latest, cv::VideoCapture &stream);

// captureDepthImage: takes one depth image (millimeters, 16 bits per pixel)
// from the robot camera. Returns false (and prints why) if it fails.
bool captureDepthImage(const std::string &ip_address, cv::Mat &depth_image);

// captureFilteredDepth: takes depth images for 3 tau = 3 / w seconds and
// filters them (first-order low-pass, 1_PreprocessCloud/depth_filter.h,
// cut-off w [rad/s]). After 3 tau the noise of a still scene is at its
// lowest. depth_image is the filtered depth, rounded to millimeters (16 bits
// per pixel, as captureDepthImage). The robot must be still the whole time.
// Returns false (and prints why) if it fails.
bool captureFilteredDepth(const std::string &ip_address, double w, cv::Mat &depth_image);

// depthToCloud: turns a depth image (millimeters) into a point cloud in the
// DEPTH frame [m]. Pixels with no depth (0) are dropped. Every other pixel is
// kept: the cloud is cut later by the GPD workspace (cfg/gpd_params.cfg).
pcl::PointCloud<pcl::PointXYZ> depthToCloud(const cv::Mat &depth_image, const Intrinsics &intr);

// captureFromRobot: one full capture. Connects to the robot, reads the
// parameters of both cameras, takes one depth image (or, with
// depth_filter_w > 0, filtered depth images for 3 / depth_filter_w seconds:
// captureFilteredDepth) and one color image, reads the joint angles and the
// TOOL pose right after them, and turns the depth image into a point cloud.
// The robot must be still: the images and the pose are taken a fraction of a
// second apart (about one second more with the filter), so if the arm moves,
// they do not match.
// Returns false (and prints why) if a step fails. The cloud can be empty:
// the caller checks it.
bool captureFromRobot(const std::string &ip_address, Capture &capture,
                      double depth_filter_w = 0.0);

// saveCapture: writes NAME.pcd (the cloud), NAME_color.png (the color image,
// if there is one) and NAME_pose.txt (joint angles, TOOL pose, parameters of
// both cameras, FLIP_XY and the depth filter w), and prints the names of the files written.
// NAME_pose.txt can be read back with readPoseFile (frames.h). Returns false
// (and prints why) if it fails.
bool saveCapture(const std::string &name, const Capture &capture);
