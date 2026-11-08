// File:    capture_file.cpp
// Module:  0_Input
// Purpose: Reads back the color part of a saved capture. See capture_file.h
//          for what each function returns.
#include "0_Input/capture_file.h"

#include <cstdio>
#include <fstream>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

// readColorCapture: reads the file line by line ("key = values", the format
// written by saveCapture), keeps color_intrinsics and color_image, then loads
// the image. The image name is relative to the folder of the pose file.
bool readColorCapture(const std::string &pose_file, Intrinsics &color_intr, cv::Mat &color_image)
{
    std::ifstream file(pose_file);
    if (!file.good())
    {
        return false;
    }

    bool has_intrinsics = false;
    std::string image_name;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        // "key = values": split at the "=".
        const size_t equal_sign = line.find('=');
        if (equal_sign == std::string::npos)
        {
            continue;
        }
        std::istringstream key_stream(line.substr(0, equal_sign));
        std::string key;
        key_stream >> key;
        std::istringstream values(line.substr(equal_sign + 1));

        if (key == "color_intrinsics")
        {
            values >> color_intr.width >> color_intr.height >> color_intr.fx >> color_intr.fy >>
                color_intr.cx >> color_intr.cy;
            has_intrinsics = !values.fail();
        }
        else if (key == "color_image")
        {
            values >> image_name;
        }
    }
    if (!has_intrinsics || image_name.empty())
    {
        return false;
    }

    // Folder of the pose file ("" if it is in the current folder).
    const size_t slash = pose_file.find_last_of('/');
    const std::string folder = (slash == std::string::npos) ? "" : pose_file.substr(0, slash + 1);
    const std::string image_file = folder + image_name;

    color_image = cv::imread(image_file, cv::IMREAD_COLOR);
    if (color_image.empty())
    {
        printf("Could not read the color image %s\n", image_file.c_str());
        return false;
    }
    if (color_image.cols != color_intr.width || color_image.rows != color_intr.height)
    {
        printf("Color image size %dx%d does not match the camera parameters (%dx%d) in %s\n",
               color_image.cols, color_image.rows, color_intr.width, color_intr.height,
               pose_file.c_str());
        return false;
    }
    return true;
}
