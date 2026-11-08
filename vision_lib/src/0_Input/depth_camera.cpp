// File:    depth_camera.cpp
// Module:  0_Input
// Purpose: Takes one point cloud from the depth camera on the robot wrist,
//          together with the robot configuration at that moment, and saves
//          both to files. See depth_camera.h for what each function returns.
#include "0_Input/depth_camera.h"

#include <chrono>
#include <cstdio>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

#include <pcl/io/pcd_io.h>

#include "0_Input/color_camera.h"
#include "1_PreprocessCloud/depth_filter.h"
#include "Robot/robot_connection.h"

namespace k_api = Kinova::Api;

// openDepthStream: opens the RTSP depth stream with GStreamer.
// Stream address and format: user guide r12.0, p.101 (rtsp://<ip>/depth,
// 16 bits per pixel). The pipeline is the depth stream example of p.217, with
// GRAY16_LE so OpenCV gets the 16-bit depth. Each pixel is the depth in
// millimeters (0 = no measurement).
bool openDepthStream(const std::string &ip_address, bool keep_only_latest, cv::VideoCapture &stream)
{
    // keep_only_latest: keep only the newest image (drop=true max-buffers=1).
    // For one capture we keep every image (drop=false).
    const std::string sink = keep_only_latest ? "appsink drop=true max-buffers=1"
                                              : "appsink drop=false";
    const std::string pipeline =
        "rtspsrc location=rtsp://" + ip_address + "/depth latency=0 ! "
        "rtpgstdepay ! videoconvert ! video/x-raw,format=GRAY16_LE ! " + sink;

    stream.open(pipeline, cv::CAP_GSTREAMER);
    if (!stream.isOpened())
    {
        printf("Could not open the depth stream:\n  %s\n", pipeline.c_str());
        return false;
    }
    return true;
}

// captureDepthImage: opens the stream, throws away FRAMES_TO_SKIP images and
// keeps the next one. Checks that it is 16 bits, 1 channel.
bool captureDepthImage(const std::string &ip_address, cv::Mat &depth_image)
{
    cv::VideoCapture capture;
    if (!openDepthStream(ip_address, false, capture))
    {
        return false;
    }

    for (int i = 0; i <= FRAMES_TO_SKIP; i++)
    {
        if (!capture.read(depth_image))
        {
            printf("Could not read a depth image.\n");
            return false;
        }
    }

    if (depth_image.type() != CV_16UC1)
    {
        printf("The depth image is not 16 bits, 1 channel (OpenCV type %d).\n", depth_image.type());
        return false;
    }
    return true;
}

// captureFilteredDepth: same start as captureDepthImage (FRAMES_TO_SKIP
// images thrown away), then each image goes through filterDepth with the
// time since the previous one (delta, measured), until 3 / w seconds.
bool captureFilteredDepth(const std::string &ip_address, double w, cv::Mat &depth_image)
{
    cv::VideoCapture capture;
    if (!openDepthStream(ip_address, false, capture))
    {
        return false;
    }
    cv::Mat u;
    for (int i = 0; i <= FRAMES_TO_SKIP; i++)
    {
        if (!capture.read(u))
        {
            printf("Could not read a depth image.\n");
            return false;
        }
    }
    if (u.type() != CV_16UC1)
    {
        printf("The depth image is not 16 bits, 1 channel (OpenCV type %d).\n", u.type());
        return false;
    }

    const double seconds = 3.0 / w;
    cv::Mat y;  // filter state, CV_32FC1 [mm]
    filterDepth(u, y, w, 0.0);  // y(0) = u(0)
    const auto start = std::chrono::steady_clock::now();
    auto last_time = start;
    int images = 1;
    while (std::chrono::duration<double>(last_time - start).count() < seconds)
    {
        if (!capture.read(u))
        {
            printf("Could not read a depth image.\n");
            return false;
        }
        const auto now = std::chrono::steady_clock::now();
        filterDepth(u, y, w, std::chrono::duration<double>(now - last_time).count());
        last_time = now;
        images++;
    }
    y.convertTo(depth_image, CV_16U);  // rounded to mm
    printf("Depth filter: %d images in %.2f s (w = %.1f rad/s)\n", images,
           std::chrono::duration<double>(last_time - start).count(), w);
    return true;
}

// depthToCloud: pinhole model. A pixel (row, col) with depth z is at
//   x = (z / fx) * (col - cx),   y = (z / fy) * (row - cy)
// with the signs of FLIP_XY.
// No distortion correction: the robot gives k1 = k2 = k3 = p1 = p2 = 0 for
// the depth sensor (read on 2026-10-16), because the depth image is already
// rectified.
pcl::PointCloud<pcl::PointXYZ> depthToCloud(const cv::Mat &depth_image, const Intrinsics &intr)
{
    const float sign = FLIP_XY ? -1.0f : 1.0f;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int row = 0; row < depth_image.rows; row++)
    {
        for (int col = 0; col < depth_image.cols; col++)
        {
            const float z = depth_image.at<uint16_t>(row, col) / 1000.0f;
            if (z == 0.0f)
            {
                continue;
            }
            const float x = sign * (z / intr.fx) * (col - intr.cx);
            const float y = sign * (z / intr.fy) * (row - intr.cy);
            cloud.push_back(pcl::PointXYZ(x, y, z));
        }
    }
    return cloud;
}

// captureFromRobot: runs the 5 steps below in order.
bool captureFromRobot(const std::string &ip_address, Capture &capture, double depth_filter_w)
{
    // ---------- 1. Connect to the robot ----------
    // The connection closes by itself at the end of this function.
    printf("Connecting to the robot at %s (read only, the robot does not move)...\n", ip_address.c_str());
    RobotConnection robot(ip_address);
    if (!robot.isConnected())
    {
        return false;
    }

    // ---------- 2. Camera parameters (both cameras, one call) ----------
    Intrinsics &intr = capture.intr;
    Intrinsics &color_intr = capture.color_intr;
    bool ok;
    try
    {
        ok = readIntrinsics(robot.router(), intr, color_intr);
    }
    catch (k_api::KDetailedException &ex)
    {
        printf("Kortex error (camera parameters): %s\n", ex.what());
        return false;
    }
    if (!ok)
    {
        return false;
    }
    printf("Depth camera: %dx%d  fx = %.3f  fy = %.3f  cx = %.3f  cy = %.3f\n",
           intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy);
    printf("Color camera: %dx%d  fx = %.3f  fy = %.3f  cx = %.3f  cy = %.3f\n",
           color_intr.width, color_intr.height, color_intr.fx, color_intr.fy, color_intr.cx,
           color_intr.cy);

    // ---------- 3. Depth image ----------
    printf("Reading the depth stream...\n");
    cv::Mat depth_image;
    capture.depth_filter_w = depth_filter_w;
    const bool depth_ok = (depth_filter_w > 0.0)
                              ? captureFilteredDepth(ip_address, depth_filter_w, depth_image)
                              : captureDepthImage(ip_address, depth_image);
    if (!depth_ok)
    {
        return false;
    }
    // The parameters are only valid for the image size they were given for.
    if (depth_image.cols != intr.width || depth_image.rows != intr.height)
    {
        printf("Image size %dx%d does not match the camera parameters (%dx%d).\n",
               depth_image.cols, depth_image.rows, intr.width, intr.height);
        return false;
    }

    // ---------- 3b. Color image ----------
    printf("Reading the color stream...\n");
    if (!captureColorImage(ip_address, capture.color_image))
    {
        return false;
    }
    if (capture.color_image.cols != color_intr.width || capture.color_image.rows != color_intr.height)
    {
        printf("Color image size %dx%d does not match the camera parameters (%dx%d).\n",
               capture.color_image.cols, capture.color_image.rows, color_intr.width,
               color_intr.height);
        return false;
    }

    // ---------- 4. Robot configuration, right after the images ----------
    if (!robot.readJointAngles(capture.q_deg) || !robot.readToolPose(capture.tool))
    {
        return false;
    }

    // ---------- 5. Point cloud ----------
    capture.cloud = depthToCloud(depth_image, intr);
    printf("Point cloud: %zu points\n", capture.cloud.size());
    return true;
}

// saveCapture: writes the cloud as a binary .pcd, the color image as a .png
// (lossless) and the robot configuration as "key = values" lines (the format
// readPoseFile reads).
bool saveCapture(const std::string &name, const Capture &capture)
{
    const std::string pcd_file = name + ".pcd";
    const std::string color_file = name + "_color.png";
    const std::string pose_file = name + "_pose.txt";
    if (pcl::io::savePCDFileBinary(pcd_file, capture.cloud) != 0)
    {
        printf("Could not write %s\n", pcd_file.c_str());
        return false;
    }
    const bool has_color = !capture.color_image.empty();
    if (has_color && !cv::imwrite(color_file, capture.color_image))
    {
        printf("Could not write %s\n", color_file.c_str());
        return false;
    }

    FILE *file = std::fopen(pose_file.c_str(), "w");
    if (file == nullptr)
    {
        printf("Could not write %s\n", pose_file.c_str());
        return false;
    }
    const double *q = capture.q_deg;
    const ToolPose &tool = capture.tool;
    const Intrinsics &intr = capture.intr;
    std::fprintf(file, "# Robot configuration when %s was taken (point cloud in the DEPTH frame).\n", pcd_file.c_str());
    std::fprintf(file, "joint_angles_deg = %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                 q[0], q[1], q[2], q[3], q[4], q[5], q[6]);
    std::fprintf(file, "# TOOL pose measured by the robot: x y z [m], theta_x theta_y theta_z [deg, x-y-z extrinsic]\n");
    std::fprintf(file, "tool_pose = %.5f %.5f %.5f %.3f %.3f %.3f\n",
                 tool.x, tool.y, tool.z, tool.theta_x, tool.theta_y, tool.theta_z);
    std::fprintf(file, "# Depth camera: width height fx fy cx cy [pixels]\n");
    std::fprintf(file, "depth_intrinsics = %d %d %.3f %.3f %.3f %.3f\n",
                 intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy);
    std::fprintf(file, "flip_xy = %d\n", FLIP_XY ? 1 : 0);
    std::fprintf(file, "# Depth filter w [rad/s] (1_PreprocessCloud/depth_filter.h), 0 = one image, no filter\n");
    std::fprintf(file, "depth_filter_w = %.2f\n", capture.depth_filter_w);
    if (has_color)
    {
        const Intrinsics &color_intr = capture.color_intr;
        std::fprintf(file, "# Color camera: width height fx fy cx cy [pixels], and its image\n");
        std::fprintf(file, "color_intrinsics = %d %d %.3f %.3f %.3f %.3f\n", color_intr.width,
                     color_intr.height, color_intr.fx, color_intr.fy, color_intr.cx, color_intr.cy);
        // Only the file name: readColorCapture looks for it in the folder of
        // the pose file, so the capture can be moved to another folder.
        const size_t slash = color_file.find_last_of('/');
        std::fprintf(file, "color_image = %s\n", color_file.substr(slash + 1).c_str());
    }
    std::fclose(file);

    printf("Saved:\n");
    printf("  %s   (point cloud, %zu points)\n", pcd_file.c_str(), capture.cloud.size());
    if (has_color)
    {
        printf("  %s   (color image, %dx%d)\n", color_file.c_str(), capture.color_image.cols,
               capture.color_image.rows);
    }
    else
    {
        printf("  (no color image)\n");
    }
    printf("  %s   (robot pose and camera parameters)\n", pose_file.c_str());
    return true;
}
