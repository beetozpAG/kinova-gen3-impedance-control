// File:    vision_module.h
// Module:  0_Input
// Purpose: What the depth camera and the color camera of the robot wrist
//          (the vision module) have in common: their parameters (pinhole
//          model) and how to read them from the robot. It only reads from
//          the robot: it never changes its settings.
#pragma once

#include <RouterClient.h>
#include <VisionConfig.pb.h>

// Camera parameters, in pixels (pinhole model).
struct Intrinsics
{
    float fx, fy; // focal lengths
    float cx, cy; // principal point (image center)
    int width, height;
};

// readIntrinsics: reads the parameters of both sensors of the vision module
// from the robot: fx, fy, cx, cy and the image size of the resolution in use.
// Both are read in one call because Kortex allows only one client of each
// service (VisionConfig, DeviceManager) per session: a second client on the
// same router fails with "notification callback is already registered".
// So call it once per session. Needs an open session (router).
// Returns false (and prints why) if it fails.
bool readIntrinsics(Kinova::Api::RouterClient &router, Intrinsics &depth, Intrinsics &color);
