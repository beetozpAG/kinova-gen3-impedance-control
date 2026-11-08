// robot_connection: open/close the Kortex connection and read the robot.
// Errors: KBasicException is caught (KDetailedException derives from it,
// KDetailedException.h:18), so any Kortex error is printed instead of
// stopping the program.
// The connection steps are the same as in
// KinovaOpenCVCPP2026/KinovaRobot/kinova_robot.cpp:15-36 (TCP part only).
#include "Robot/robot_connection.h"

#include <cstdio>

namespace k_api = Kinova::Api;

// TCP port for the Kortex API calls (KinovaOpenCVCPP2026, Gripper_test.cpp:83).
static const uint32_t PORT_TCP = 10000;

// Called by the Kortex API when the TCP connection has an error.
static void printTcpError(k_api::KError error)
{
    printf("TCP error: %s\n", error.toString().c_str());
}

RobotConnection::RobotConnection(const std::string &ip_address)
    : router_(&transport_, printTcpError), session_manager_(&router_), base_(&router_)
{
    if (!transport_.connect(ip_address, PORT_TCP))
    {
        printf("Could not connect to the robot at %s:%u\n", ip_address.c_str(), PORT_TCP);
        return;
    }
    tcp_open_ = true;

    // Same user, password and timeouts as KinovaOpenCVCPP2026
    // (Gripper_test.cpp:136-137, kinova_robot.cpp:22-28).
    k_api::Session::CreateSessionInfo session_info;
    session_info.set_username("admin");
    session_info.set_password("admin");
    session_info.set_session_inactivity_timeout(60000);   // ms
    session_info.set_connection_inactivity_timeout(2000); // ms
    try
    {
        session_manager_.CreateSession(session_info);
        session_open_ = true;
    }
    catch (k_api::KBasicException &ex)
    {
        printf("Kortex error (create session): %s\n", ex.what());
    }
}

RobotConnection::~RobotConnection()
{
    if (session_open_)
    {
        // A destructor must not throw: catch any Kortex error here.
        try
        {
            session_manager_.CloseSession();
        }
        catch (k_api::KBasicException &ex)
        {
            printf("Kortex error (close session): %s\n", ex.what());
        }
    }
    if (tcp_open_)
    {
        router_.SetActivationStatus(false);
        transport_.disconnect();
    }
}

bool RobotConnection::readJointAngles(double q_deg[7])
{
    if (!session_open_)
    {
        printf("Not connected to the robot.\n");
        return false;
    }
    try
    {
        const k_api::Base::JointAngles joint_angles = base_.GetMeasuredJointAngles();
        if (joint_angles.joint_angles_size() != 7)
        {
            printf("The robot reported %d joints, expected 7\n", joint_angles.joint_angles_size());
            return false;
        }
        for (int i = 0; i < 7; i++)
        {
            q_deg[i] = joint_angles.joint_angles(i).value();
        }
        return true;
    }
    catch (k_api::KBasicException &ex)
    {
        printf("Kortex error (joint angles): %s\n", ex.what());
        return false;
    }
}

bool RobotConnection::readToolPose(ToolPose &tool)
{
    if (!session_open_)
    {
        printf("Not connected to the robot.\n");
        return false;
    }
    try
    {
        const k_api::Base::Pose pose = base_.GetMeasuredCartesianPose();
        tool = {pose.x(), pose.y(), pose.z(), pose.theta_x(), pose.theta_y(), pose.theta_z()};
        return true;
    }
    catch (k_api::KBasicException &ex)
    {
        printf("Kortex error (TOOL pose): %s\n", ex.what());
        return false;
    }
}
