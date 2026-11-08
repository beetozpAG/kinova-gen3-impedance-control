// File:    color_camera.cpp
// Module:  0_Input
// Purpose: Reads the parameters and the video of the color camera on the
//          robot wrist. See color_camera.h for what each function returns.
#include "0_Input/color_camera.h"

#include <cstdio>

#include "0_Input/depth_camera.h"

// openColorStream: opens the RTSP color stream with GStreamer.
// Stream address and format: user guide r12.0, p.101 (rtsp://<ip>/color,
// H.264). The pipeline is the color stream example of p.217, with
// videoconvert to BGR so OpenCV gets a normal color image.
bool openColorStream(const std::string &ip_address, bool keep_only_latest, cv::VideoCapture &stream)
{
    // keep_only_latest: keep only the newest image (drop=true max-buffers=1).
    // For one capture we keep every image (drop=false).
    const std::string sink = keep_only_latest ? "appsink drop=true max-buffers=1"
                                              : "appsink drop=false";
    const std::string pipeline =
        "rtspsrc location=rtsp://" + ip_address + "/color latency=0 ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! " + sink;

    stream.open(pipeline, cv::CAP_GSTREAMER);
    if (!stream.isOpened())
    {
        printf("Could not open the color stream:\n  %s\n", pipeline.c_str());
        return false;
    }
    return true;
}

// captureColorImage: opens the stream, throws away FRAMES_TO_SKIP images
// (depth_camera.h) and keeps the next one. Checks that it is 8 bits, 3 channels.
bool captureColorImage(const std::string &ip_address, cv::Mat &color_image)
{
    cv::VideoCapture capture;
    if (!openColorStream(ip_address, false, capture))
    {
        return false;
    }

    for (int i = 0; i <= FRAMES_TO_SKIP; i++)
    {
        if (!capture.read(color_image))
        {
            printf("Could not read a color image.\n");
            return false;
        }
    }

    if (color_image.type() != CV_8UC3)
    {
        printf("The color image is not 8 bits, 3 channels (OpenCV type %d).\n", color_image.type());
        return false;
    }
    return true;
}
