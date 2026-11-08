// robot_connection: one connection to the Kinova Gen3 (Kortex API, TCP),
// shared by every program that reads the robot.
//
// Creating a RobotConnection opens the TCP connection and the session.
// Destroying it (end of the block) closes them, even after an error.
// Example:
//   RobotConnection robot("192.168.1.10");
//   if (!robot.isConnected()) return 1;
//   robot.readJointAngles(q_deg);
//   robot.readToolPose(tool);
//
// Only "Get" functions are used here: nothing is sent that moves the robot
// or changes its mode.
#pragma once

#include <string>

#include <BaseClientRpc.h>
#include <RouterClient.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>

#include "0_Input/frames.h"

class RobotConnection
{
public:
    // Connect to the robot at ip_address and open a session.
    // If it fails, it prints why and isConnected() returns false.
    // If the robot is not reachable, the TCP connection waits for the
    // system timeout (~2 min).
    explicit RobotConnection(const std::string &ip_address);

    // Close the session and the TCP connection.
    ~RobotConnection();

    // Only one owner of the connection: no copies.
    RobotConnection(const RobotConnection &) = delete;
    RobotConnection &operator=(const RobotConnection &) = delete;

    // True if the TCP connection and the session are open.
    bool isConnected() const { return session_open_; }

    // Router of the connection, for other Kortex clients
    // (for example readIntrinsics in vision_module.h).
    Kinova::Api::RouterClient &router() { return router_; }

    // Joint angles in degrees, one per actuator (like the web app).
    // Returns false (and prints why) if the reading fails.
    bool readJointAngles(double q_deg[7]);

    // TOOL pose computed by the robot with its own calibration
    // (meters and degrees, x-y-z extrinsic, the web app values).
    // It uses the tool configured in the robot (web app: Configurations > Robot > Tool).
    // It must be the 2F-85 tool: X = Y = 0, Z = 0.120 m, angles 0
    // ("Robotiq 2F-85 Gripper tool configuration", Table 29), because
    // intToBase (frames.h) subtracts that same 0.120 m.
    // Checked in the web app on 2026-10-16.
    // Returns false (and prints why) if the reading fails.
    bool readToolPose(ToolPose &tool);

private:
    // Declared in this order on purpose: members are built in this order,
    // and each one needs the one before it.
    Kinova::Api::TransportClientTcp transport_;
    Kinova::Api::RouterClient router_;
    Kinova::Api::SessionManager session_manager_;
    // Reads joint angles and poses. Created ONCE per connection: each Kortex
    // client registers its service in the router when it is created
    // (RouterClient::registerNotificationCallback, RouterClient.h:52), and the
    // API has no way to remove it. A second BaseClient on the same router
    // throws "notification callback is already registered".
    // The same rule applies to any other client (DeviceManager, VisionConfig...).
    Kinova::Api::Base::BaseClient base_;
    bool tcp_open_ = false;
    bool session_open_ = false;
};
