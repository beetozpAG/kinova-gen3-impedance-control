// File:    vision_module.cpp
// Module:  0_Input
// Purpose: Reads the parameters of the depth and color sensors of the
//          vision module. See vision_module.h for what each function returns.
#include "0_Input/vision_module.h"

#include <cstdio>

#include <DeviceManagerClientRpc.h>
#include <VisionConfigClientRpc.h>

namespace k_api = Kinova::Api;

// resolutionSize: image size (width, height) of a Kortex resolution value.
// Depth: 424x240 or 480x270. Color: 320x240, 640x480, 1280x720 or 1920x1080
// (user guide r12.0, Table 67, p.135). Returns false for an unknown value.
static bool resolutionSize(k_api::VisionConfig::Resolution resolution, int &width, int &height)
{
    switch (resolution)
    {
    case k_api::VisionConfig::RESOLUTION_320x240: width = 320; height = 240; return true;
    case k_api::VisionConfig::RESOLUTION_424x240: width = 424; height = 240; return true;
    case k_api::VisionConfig::RESOLUTION_480x270: width = 480; height = 270; return true;
    case k_api::VisionConfig::RESOLUTION_640x480: width = 640; height = 480; return true;
    case k_api::VisionConfig::RESOLUTION_1280x720: width = 1280; height = 720; return true;
    case k_api::VisionConfig::RESOLUTION_1920x1080: width = 1920; height = 1080; return true;
    default: return false;
    }
}

// sensorIntrinsics: asks one sensor of the vision module for its parameters.
static bool sensorIntrinsics(k_api::VisionConfig::VisionConfigClient &vision_config,
                             uint32_t vision_device_id, k_api::VisionConfig::Sensor sensor,
                             Intrinsics &intr)
{
    k_api::VisionConfig::SensorIdentifier sensor_id;
    sensor_id.set_sensor(sensor);
    auto params = vision_config.GetIntrinsicParameters(sensor_id, vision_device_id);

    intr.fx = params.focal_length_x();
    intr.fy = params.focal_length_y();
    intr.cx = params.principal_point_x();
    intr.cy = params.principal_point_y();
    if (!resolutionSize(params.resolution(), intr.width, intr.height))
    {
        printf("Unknown %s resolution: %s\n", k_api::VisionConfig::Sensor_Name(sensor).c_str(),
               k_api::VisionConfig::Resolution_Name(params.resolution()).c_str());
        return false;
    }
    return true;
}

// readIntrinsics: creates the two Kortex clients once, finds the vision
// module among the robot devices, then asks each sensor for its parameters.
bool readIntrinsics(k_api::RouterClient &router, Intrinsics &depth, Intrinsics &color)
{
    k_api::DeviceManager::DeviceManagerClient device_manager(&router);
    k_api::VisionConfig::VisionConfigClient vision_config(&router);

    uint32_t vision_device_id = 0;
    auto devices = device_manager.ReadAllDevices();
    for (int i = 0; i < devices.device_handle_size(); i++)
    {
        const auto &handle = devices.device_handle(i);
        if (handle.device_type() == k_api::Common::DeviceTypes::VISION)
        {
            vision_device_id = handle.device_identifier();
            break;
        }
    }
    if (vision_device_id == 0)
    {
        printf("The robot has no vision module.\n");
        return false;
    }

    return sensorIntrinsics(vision_config, vision_device_id, k_api::VisionConfig::SENSOR_DEPTH,
                            depth) &&
           sensorIntrinsics(vision_config, vision_device_id, k_api::VisionConfig::SENSOR_COLOR,
                            color);
}
