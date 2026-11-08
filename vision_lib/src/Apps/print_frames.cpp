// Print the pose of three frames with respect to the BASE frame:
//   1. INT:  interface module frame (end of the arm).
//   2. TOOL: end effector frame of the Robotiq 2F-85 = INT + 0.120 m along Z_INT
//            (Kinova Gen3 user guide r12.0, Table 28, p.92).
//   3. DEPTH: depth sensor frame = TOOL measured by the robot - 0.120 m along
//            Z_TOOL, then + (27.50, 66.00, -3.05) mm in INT (Table 92, p.225).
//            Only with the robot. The point clouds are in this frame.
// INT and TOOL are computed from the 7 joint angles with Gen3_model.h.
//
// Usage (from build/):
//   ./print_frames                         read the joint angles from the robot (192.168.1.10)
//   ./print_frames <ip>                    same, robot at another IP address
//   ./print_frames q1 q2 q3 q4 q5 q6 q7    no robot: joint angles given in degrees
//
// With the robot, the program ONLY READS. It does not move the robot and does
// not change its mode: it opens a session, asks for the joint angles and the
// TOOL pose measured by the robot, prints them and closes the session.
// The robot's TOOL pose is the one shown in the Kinova web app. It is printed
// next to ours to check Gen3_model.h (Matlab or Manual version, see GetTransform).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "0_Input/frames.h"
#include "Robot/Gen3_model.h"
#include "Robot/robot_connection.h"

const float deg2rad = static_cast<float>(M_PI) / 180.0f;
const float rad2deg = 180.0f / static_cast<float>(M_PI);

// Pose with the same values as the Kinova web app:
// position in meters, orientation in degrees (x-y-z extrinsic).
struct Pose
{
    float x, y, z;
    float theta_x, theta_y, theta_z;
};

// Pose from a 4x4 transform. Euler angles in the x-y-z extrinsic convention:
// rotation about the fixed x axis, then fixed y, then fixed z,
// so R = Rz(theta_z) * Ry(theta_y) * Rx(theta_x).
// This is the convention of the Kinova web app and Kortex API
// (user guide r12.0, p.116, also p.155 and p.175).
// Each angle comes from writing out Rz*Ry*Rx:
//   R[2][1] / R[2][2] = tan(theta_x)
//   R[2][0] = -sin(theta_y),  sqrt(R[0][0]^2 + R[1][0]^2) = cos(theta_y)
//   R[1][0] / R[0][0] = tan(theta_z)
Pose poseFromTransform(const matrix::mat<4, 4> &T)
{
    Pose p;
    p.x = T[0][3];
    p.y = T[1][3];
    p.z = T[2][3];
    p.theta_x = std::atan2(T[2][1], T[2][2]) * rad2deg;
    p.theta_y = std::atan2(-T[2][0], std::sqrt(T[0][0] * T[0][0] + T[1][0] * T[1][0])) * rad2deg;
    p.theta_z = std::atan2(T[1][0], T[0][0]) * rad2deg;
    return p;
}

// Print the position and the orientation of a frame.
void printPose(const char *name, const Pose &p)
{
    printf("%s with respect to BASE:\n", name);
    printf("  position (m):        x = %9.5f   y = %9.5f   z = %9.5f\n", p.x, p.y, p.z);
    printf("  orientation (deg):  tx = %9.3f  ty = %9.3f  tz = %9.3f\n\n", p.theta_x, p.theta_y, p.theta_z);
}

// Print the rotation matrix (3x3 part) of a transform.
void printRotation(const matrix::mat<4, 4> &T)
{
    printf("  rotation matrix:\n");
    for (int i = 0; i < 3; i++)
    {
        printf("    [%8.4f %8.4f %8.4f]\n", T[i][0], T[i][1], T[i][2]);
    }
    printf("\n");
}

// Angle difference a - b in degrees, wrapped to [-180, 180]
// (for example 179 and -179 are only 2 degrees apart).
float angleDiff(float a, float b)
{
    return std::remainder(a - b, 360.0f);
}

// Read the joint angles (degrees) and the measured TOOL pose from the robot.
// Only "Get" functions are used: nothing is sent that moves the robot.
// The connection opens and closes inside RobotConnection (robot_connection.h).
// Returns false if the connection or a reading fails.
bool readRobot(const std::string &ip_address, float q_deg[7], Pose &robot_tool)
{
    RobotConnection robot(ip_address);
    double q[7];
    ToolPose tool;
    if (!robot.isConnected() || !robot.readJointAngles(q) || !robot.readToolPose(tool))
    {
        return false;
    }
    for (int i = 0; i < 7; i++)
    {
        q_deg[i] = static_cast<float>(q[i]);
    }
    robot_tool = {static_cast<float>(tool.x), static_cast<float>(tool.y),
                  static_cast<float>(tool.z), static_cast<float>(tool.theta_x),
                  static_cast<float>(tool.theta_y), static_cast<float>(tool.theta_z)};
    return true;
}

int main(int argc, char *argv[])
{
    // 1. Get the 7 joint angles (degrees): from the robot or from the command line.
    float q_deg[7] = {};
    Pose robot_tool{};
    bool from_robot = (argc == 1 || argc == 2);

    if (from_robot)
    {
        std::string ip_address = (argc == 2) ? argv[1] : "192.168.1.10";
        printf("Reading the robot at %s (read only, the robot does not move)...\n", ip_address.c_str());
        // If the robot is not reachable, the TCP connection waits for the system timeout (~2 min).
        printf("If nothing happens, check the cable and the IP address (Ctrl+C to stop).\n\n");
        if (!readRobot(ip_address, q_deg, robot_tool))
        {
            return 1;
        }
    }
    else if (argc == 8)
    {
        for (int i = 0; i < 7; i++)
        {
            q_deg[i] = std::atof(argv[i + 1]);
        }
    }
    else
    {
        printf("Usage: %s                         (read the robot at 192.168.1.10)\n", argv[0]);
        printf("       %s <ip>                    (read the robot at <ip>)\n", argv[0]);
        printf("       %s q1 q2 q3 q4 q5 q6 q7    (no robot, joint angles in degrees)\n", argv[0]);
        return 1;
    }

    printf("Joint angles (deg):");
    for (int i = 0; i < 7; i++)
    {
        printf(" %.2f", q_deg[i]);
    }
    printf("\n\n");

    // 2. Forward kinematics (Gen3_model.h), joint angles in radians.
    //    GetTransform:     T_base_int  (product of Table 90 matrices, p.221-222)
    //    GetToolTransform: T_base_tool = T_base_int * T_int_tool (0.120 m on Z, Table 28)
    matrix::mat<7, 1> q{};
    for (int i = 0; i < 7; i++)
    {
        q[i][0] = q_deg[i] * deg2rad;
    }
    matrix::mat<4, 4> T_base_int = KinovaGen3::GetTransform(q);
    matrix::mat<4, 4> T_base_tool = KinovaGen3::GetToolTransform(q);
    Pose int_pose = poseFromTransform(T_base_int);
    Pose tool_pose = poseFromTransform(T_base_tool);

    // 3. Frames from the joint angles (Gen3_model.h).
    printf("========== From the joint angles (Gen3_model.h) ==========\n\n");
    printPose("INT  (interface module)", int_pose);
    printRotation(T_base_int);
    printPose("TOOL (Robotiq 2F-85, INT + 0.120 m on Z)", tool_pose);
    printRotation(T_base_tool);

    if (from_robot)
    {
        // 4. Frames from the TOOL pose measured by the robot.
        //    DEPTH frame with respect to BASE frame (frames.h):
        //    T_base_depth = T_base_tool * T_tool_int * T_int_depth
        const ToolPose measured_tool = {robot_tool.x, robot_tool.y, robot_tool.z,
                                        robot_tool.theta_x, robot_tool.theta_y, robot_tool.theta_z};
        const Eigen::Matrix4d T = intToBase(toolToBase(measured_tool), KinovaGen3::TOOL_OFFSET_Z_2F85) * depthToInt();
        matrix::mat<4, 4> T_base_depth{};
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                T_base_depth[i][j] = static_cast<float>(T(i, j));
            }
        }
        printf("========== From the TOOL pose measured by the robot ==========\n\n");
        printPose("TOOL measured by the robot (web app values)", robot_tool);
        printPose("DEPTH (depth sensor, from the measured TOOL)", poseFromTransform(T_base_depth));
        printRotation(T_base_depth);

        // 5. Check Gen3_model.h: our TOOL pose minus the one measured by the robot.
        printf("========== Check: TOOL from the joint angles - TOOL measured ==========\n\n");
        printf("  position (mm):      dx = %8.2f  dy = %8.2f  dz = %8.2f\n",
               (tool_pose.x - robot_tool.x) * 1000.0f,
               (tool_pose.y - robot_tool.y) * 1000.0f,
               (tool_pose.z - robot_tool.z) * 1000.0f);
        printf("  orientation (deg): dtx = %8.3f dty = %8.3f dtz = %8.3f\n",
               angleDiff(tool_pose.theta_x, robot_tool.theta_x),
               angleDiff(tool_pose.theta_y, robot_tool.theta_y),
               angleDiff(tool_pose.theta_z, robot_tool.theta_z));
    }

    return 0;
}
