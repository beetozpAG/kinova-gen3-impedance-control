// view_cloud: show the point cloud of the robot's wrist depth camera in real
// time, exactly as grasps_from_camera captures it (same conversion).
// Useful to place the camera: the Web App camera view is the color sensor,
// which does not see the same area as the depth sensor.
//
// Usage (run from kinova_grasp/build/):
//   ./view_cloud               robot at 192.168.1.10, files view_ddmm_hhmmss
//   ./view_cloud NAME          robot at 192.168.1.10, files NAME_1, NAME_2, ...
//   ./view_cloud NAME <ip>     robot at another IP address
// Keys in the window:
//   s   save the cloud shown now as FILE.pcd / FILE_pose.txt
//       (same files as grasps_from_camera; keep the robot still when
//       saving).
//       The terminal shows the number of points and the commands to see
//       the file and to detect grasps on it.
//   b, r, p  plate mask: show only the points of one plate color in the
//       "colors" window (the rest in dark gray): blue, red, pink. The key of
//       each color is in cfg/plate_colors.cfg (the terminal lists them).
//       r also resets the camera of the window (a PCL viewer key).
//       Use it to set the color ranges under the real light (color_mask.h).
//   n   no mask: all colors again.
//   t   depth filter on/off (on at the start), to compare the cloud with
//       and without it (1_PreprocessCloud/depth_filter.h).
//   q   quit
//
// The program ONLY READS: it does not move the robot and does not change
// its mode or its camera settings.
//
// What the windows show (three windows):
//   - "view_cloud": the cloud in the DEPTH frame, colored by distance to the
//     camera (z); the DEPTH axes at the camera (red = X, green = Y,
//     blue = Z = view direction); the number of points and the time
//     between two images (delta).
//   - "view_cloud: colors": the same cloud, each point with the color of the
//     scene (color_cloud.h; gray = outside the color image). If the colors
//     fall on the right objects, the depth and color cameras are matched
//     right.
//   - "color": the image of the color camera (color_camera.h).
//   The depth is filtered before the cloud is built (first-order low-pass,
//   1_PreprocessCloud/depth_filter.h, w = DEPTH_FILTER_W): keep the robot
//   still, or the cloud mixes views for about a second after it moves.
//   "s" saves the filtered cloud.
//   The keys work in both 3D windows. "s" saves the cloud and the color
//   image (saveCapture, depth_camera.h).

#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>

#include <pcl/visualization/pcl_visualizer.h>

#include "0_Input/color_camera.h"
#include "0_Input/color_cloud.h"
#include "0_Input/depth_camera.h"
#include "1_PreprocessCloud/depth_filter.h"
#include "2_GetROI/color_mask.h"
#include "Robot/robot_connection.h"

// Color of the points outside the plate mask (R, G, B).
const uint8_t MASK_OFF_RGB[3] = {40, 40, 40};

// File name for a saved view.
// No NAME: from the current date and time, view_ddmm_hhmmss.
// With NAME: NAME_1, NAME_2, ... the first number whose .pcd does not exist
// yet, so an older capture is never overwritten.
static std::string viewName(const std::string &name)
{
    if (name.empty())
    {
        const std::time_t now = std::time(nullptr);
        char text[32];
        std::strftime(text, sizeof(text), "view_%d%m_%H%M%S", std::localtime(&now));
        return text;
    }
    for (int i = 1;; i++)
    {
        const std::string candidate = name + "_" + std::to_string(i);
        if (!std::ifstream(candidate + ".pcd").good())
        {
            return candidate;
        }
    }
}

int main(int argc, char *argv[])
{
    if (argc > 3)
    {
        printf("Usage: %s [NAME] [ip]   (default: files view_ddmm_hhmmss, robot at 192.168.1.10)\n",
               argv[0]);
        return 1;
    }
    const std::string name = (argc >= 2) ? argv[1] : "";
    const std::string ip_address = (argc == 3) ? argv[2] : "192.168.1.10";

    // ---------- 1. Connect and read the camera parameters (once) ----------
    printf("Connecting to the robot at %s (read only, the robot does not move)...\n", ip_address.c_str());
    RobotConnection robot(ip_address);
    if (!robot.isConnected())
    {
        return 1;
    }
    Capture capture;
    Intrinsics color_intr;
    try
    {
        // Both sensors in one call (one Kortex client per session).
        if (!readIntrinsics(robot.router(), capture.intr, color_intr))
        {
            return 1;
        }
    }
    catch (Kinova::Api::KBasicException &ex)
    {
        printf("Kortex error (camera parameters): %s\n", ex.what());
        return 1;
    }
    const Intrinsics &intr = capture.intr;
    printf("Depth camera: %dx%d  fx = %.3f  fy = %.3f  cx = %.3f  cy = %.3f\n",
           intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy);
    printf("Color camera: %dx%d  fx = %.3f  fy = %.3f  cx = %.3f  cy = %.3f\n",
           color_intr.width, color_intr.height, color_intr.fx, color_intr.fy, color_intr.cx,
           color_intr.cy);

    // ---------- 2. Open the depth and color streams (keep only the newest image) ----------
    cv::VideoCapture stream;
    cv::VideoCapture color_stream;
    if (!openDepthStream(ip_address, true, stream) ||
        !openColorStream(ip_address, true, color_stream))
    {
        return 1;
    }

    // ---------- 3. Windows ----------
    // Two 3D windows with the same settings: z colors and scene colors.
    pcl::visualization::PCLVisualizer viewer("view_cloud (s = save, q = quit)");
    pcl::visualization::PCLVisualizer color_viewer("view_cloud: colors (s = save, q = quit)");
    bool save_requested = false;
    char mask_key = 0;  // last color key pressed (0 = none)
    for (pcl::visualization::PCLVisualizer *v : {&viewer, &color_viewer})
    {
        v->setBackgroundColor(0.0, 0.0, 0.0);
        v->addCoordinateSystem(0.05, "depth_frame");  // 5 cm axes at the camera
        // Look from the camera, along +Z (view direction); -Y up, like the image.
        v->setCameraPosition(0.0, 0.0, -0.1, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
        v->registerKeyboardCallback(
            [&save_requested, &mask_key](const pcl::visualization::KeyboardEvent &event)
            {
                if (event.keyDown() && event.getKeySym() == "s")
                {
                    save_requested = true;
                }
                else if (event.keyDown() && event.getKeyCode() != 0)
                {
                    mask_key = event.getKeyCode();  // checked in the loop
                }
            });
    }

    // Plate colors, each with its key (-1 = no mask, all colors shown).
    std::vector<ColorRange> plate_colors;
    if (readColorRanges(PLATE_COLORS_FILE, plate_colors))
    {
        printf("Color keys (\"colors\" window):");
        for (const ColorRange &c : plate_colors)
        {
            printf("  %c = %s", c.key, c.name.c_str());
        }
        printf("  n = all colors\n");
    }
    printf("Depth filter key: t = on/off (now on, w = %.1f rad/s)\n", DEPTH_FILTER_W);
    int mask = -1;

    // ---------- 4. Loop: read images -> colored cloud -> show ----------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr shown(new pcl::PointCloud<pcl::PointXYZRGB>);
    bool cloud_added = false;
    cv::Mat depth_image;
    cv::Mat color_image;
    cv::Mat filtered_depth;  // filter state y (CV_32FC1, mm), depth_filter.h
    cv::Mat depth_to_show;   // filtered depth back to CV_16UC1 for depthToCloud
    bool filter_on = true;   // key t

    // delta = time between two images [s]: one time stamp per loop, the
    // difference with the previous one. steady_clock never jumps back.
    auto last_time = std::chrono::steady_clock::now();
    while (!viewer.wasStopped() && !color_viewer.wasStopped())
    {
        const auto now = std::chrono::steady_clock::now();
        const double delta = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        if (!stream.read(depth_image) || depth_image.type() != CV_16UC1)
        {
            printf("Could not read a depth image.\n");
            break;
        }
        // The parameters are only valid for the image size they were given for
        // (same check as captureFromRobot, depth_camera.cpp).
        if (depth_image.cols != intr.width || depth_image.rows != intr.height)
        {
            printf("Image size %dx%d does not match the camera parameters (%dx%d).\n",
                   depth_image.cols, depth_image.rows, intr.width, intr.height);
            break;
        }
        if (!color_stream.read(color_image) || color_image.type() != CV_8UC3)
        {
            printf("Could not read a color image.\n");
            break;
        }
        if (color_image.cols != color_intr.width || color_image.rows != color_intr.height)
        {
            printf("Color image size %dx%d does not match the camera parameters (%dx%d).\n",
                   color_image.cols, color_image.rows, color_intr.width, color_intr.height);
            break;
        }
        // Depth filter (depth_filter.h), then the cloud from the filtered depth.
        // Filter off (key t): the cloud from the image as it comes.
        if (filter_on)
        {
            filterDepth(depth_image, filtered_depth, DEPTH_FILTER_W, delta);
            filtered_depth.convertTo(depth_to_show, CV_16U);  // rounded to mm
            *cloud = depthToCloud(depth_to_show, intr);
        }
        else
        {
            *cloud = depthToCloud(depth_image, intr);
        }
        *colored = colorCloud(*cloud, color_image, color_intr);

        // Color key: only that plate color; n: all colors again. Other keys
        // are left to the PCL viewer.
        if (mask_key != 0)
        {
            for (size_t c = 0; c < plate_colors.size(); c++)
            {
                if (plate_colors[c].key == mask_key)
                {
                    mask = static_cast<int>(c);
                    printf("Mask: %s\n", plate_colors[c].name.c_str());
                }
            }
            if (mask_key == 'n')
            {
                mask = -1;
                printf("Mask: off (all colors)\n");
            }
            // t: filter on/off. Its memory is cleared, so when it is turned
            // on again it starts from the image of that moment (y = u).
            if (mask_key == 't')
            {
                filter_on = !filter_on;
                filtered_depth.release();
                printf("Depth filter: %s\n", filter_on ? "on" : "off");
            }
            mask_key = 0;
        }

        // Window 2 shows all the colors, or only the points of the plate
        // color (the rest in dark gray).
        *shown = *colored;
        size_t mask_points = 0;
        if (mask >= 0)
        {
            const std::vector<int> plate = colorMask(*colored, plate_colors[mask]);
            mask_points = plate.size();
            for (pcl::PointXYZRGB &p : shown->points)
            {
                p.r = MASK_OFF_RGB[0];
                p.g = MASK_OFF_RGB[1];
                p.b = MASK_OFF_RGB[2];
            }
            for (int i : plate)
            {
                (*shown)[i].r = (*colored)[i].r;
                (*shown)[i].g = (*colored)[i].g;
                (*shown)[i].b = (*colored)[i].b;
            }
        }

        // Window 1: color by z = distance along the camera axis.
        // Window 2: each point with its own color (r, g, b fields).
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> z_color(cloud, "z");
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_color(shown);
        if (!cloud_added)
        {
            viewer.addPointCloud<pcl::PointXYZ>(cloud, z_color, "cloud");
            color_viewer.addPointCloud<pcl::PointXYZRGB>(shown, rgb_color, "cloud");
            cloud_added = true;
        }
        else
        {
            viewer.updatePointCloud<pcl::PointXYZ>(cloud, z_color, "cloud");
            color_viewer.updatePointCloud<pcl::PointXYZRGB>(shown, rgb_color, "cloud");
        }

        char text[128];
        char filter_text[64] = "filter off";
        if (filter_on)
        {
            std::snprintf(filter_text, sizeof(filter_text), "filter on (w = %.1f rad/s)",
                          DEPTH_FILTER_W);
        }
        std::snprintf(text, sizeof(text), "%zu points, %.0f ms per image, %s", cloud->size(),
                      delta * 1000.0, filter_text);
        for (pcl::visualization::PCLVisualizer *v : {&viewer, &color_viewer})
        {
            if (!v->updateText(text, 10, 10, "info"))
            {
                v->addText(text, 10, 10, "info");
            }
        }
        // Points of the plate color, in the colors window only.
        char mask_text[128] = "";
        if (mask >= 0)
        {
            std::snprintf(mask_text, sizeof(mask_text), "mask %s: %zu points",
                          plate_colors[mask].name.c_str(), mask_points);
        }
        if (!color_viewer.updateText(mask_text, 10, 30, "mask"))
        {
            color_viewer.addText(mask_text, 10, 30, "mask");
        }

        // ---------- 5. Save on "s" ----------
        if (save_requested)
        {
            save_requested = false;
            capture.cloud = *cloud;
            capture.color_image = color_image.clone();
            capture.color_intr = color_intr;
            capture.depth_filter_w = filter_on ? DEPTH_FILTER_W : 0.0;
            const std::string file = viewName(name);
            if (robot.readJointAngles(capture.q_deg) && robot.readToolPose(capture.tool) &&
                saveCapture(file, capture))
            {
                printf("  To see the cloud:  pcl_viewer -ax 0.1 %s.pcd\n", file.c_str());
                printf("  To detect grasps:  ./grasps_from_file ../cfg/gpd_params.cfg %s.pcd %s_pose.txt\n",
                       file.c_str(), file.c_str());
            }
        }

        // Color image, in its own window.
        cv::imshow("color", color_image);
        cv::waitKey(1);  // draw the color window, 1 ms

        viewer.spinOnce(10);        // draw and handle keys, 10 ms
        color_viewer.spinOnce(10);  // same for the colors window
    }
    return 0;
}
