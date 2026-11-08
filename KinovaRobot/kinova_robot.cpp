#include "kinova_robot.h"
#include "ActuatorConfig.pb.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <typeinfo>

namespace kinova_robot {

using namespace matrix;
using enum ControlMode;

constexpr float rad2deg = 180.0f / 3.141592653589793f;
constexpr float deg2rad = 3.141592653589793f / 180.0f;

KinovaRobotLowLevel::KinovaRobotLowLevel(string ip_address, string username,
                                         string password)
    : routerTcp{&transportTcp,
                [](KError error) {
                  cout << "TCP error: " << error.toString() << endl;
                }},
      routerUdp{&transportUdp,
                [](KError error) {
                  cout << "UDP error: " << error.toString() << endl;
                }},
      session_info{[](string username, string password) {
        Session::CreateSessionInfo session_info;
        session_info.set_username(username);
        session_info.set_password(password);
        session_info.set_session_inactivity_timeout(60000 /*ms*/);
        session_info.set_connection_inactivity_timeout(2000 /*ms*/);
        return session_info;
      }(username, password)},
      sessionManagerTcp{
          (transportTcp.connect(ip_address, port_tcp), &routerTcp)},
      sessionManagerUdp{
          (transportUdp.connect(ip_address, port_udp), &routerUdp)},
      base{(sessionManagerTcp.CreateSession(session_info), &routerTcp)},
      actuatorConfig{&routerTcp},
      baseCyclic{(sessionManagerUdp.CreateSession(session_info), &routerUdp)},
      actuatorCount{base.GetActuatorCount().count()} {
  base.ClearFaults();
  auto servoing_mode = Base::ServoingModeInformation{};
  servoing_mode.set_servoing_mode(Kinova::Api::Base::LOW_LEVEL_SERVOING);
  base.SetServoingMode(servoing_mode);
  updateLocal(baseCyclic.RefreshFeedback(), steady_clock::now());
  gripperPosCommand = gripperPos;
  gripperVelCommand = {};
  sendPos(pos);
}

KinovaRobotLowLevel::~KinovaRobotLowLevel() {
  try {
    // CLOSE : CHANGE CONTRL MODES
    setControlMode({POS, POS, POS, POS, POS, POS, POS});
    sendPos(getPos(true));

    auto servoing_mode = Base::ServoingModeInformation{};
    servoing_mode.set_servoing_mode(Kinova::Api::Base::SINGLE_LEVEL_SERVOING);
    base.SetServoingMode(servoing_mode);
    sessionManagerTcp.CloseSession();
    sessionManagerUdp.CloseSession();
    routerTcp.SetActivationStatus(false);
    transportTcp.disconnect();
    routerUdp.SetActivationStatus(false);
    transportUdp.disconnect();
    cout << "Destroyed" << endl;
  } catch (KDetailedException &ex) {
    cout << "Kortex Exception on destroy" << ex.what() << endl;
  } catch (std::runtime_error &ex) {
    cout << "Runtime error on destroy" << ex.what() << endl;
  } catch (...) {
    cout << "Unknown error on destroy" << endl;
  }
}

steady_clock::time_point KinovaRobotLowLevel::waitForRefresh() {
  steady_clock::time_point now = steady_clock::now();
  while (!timeout &&
         ((now = steady_clock::now()) - lastFrameTime) < milliseconds(1)) {
    wait_callback(wait_callback_user_data);
  }
  return now;
}

void KinovaRobotLowLevel::updateLocal(const BaseCyclic::Feedback &feedback,
                                      steady_clock::time_point now) {
  lastFrameTime = now;
  for (size_t i = 0; i < actuatorCount; i++) {
    pos[i][0] = feedback.actuators(i).position() * deg2rad;
    vel[i][0] = feedback.actuators(i).velocity() * deg2rad;
    tau[i][0] = feedback.actuators(i).torque();
    cur[i][0] = feedback.actuators(i).current_motor();
    command[i][0] = float(feedback.actuators(i).command_id() & 0xFFFF);
  }
  // TOOL pose measured by the robot (read only).
  if (feedback.has_base()) {
    const auto &b = feedback.base();
    toolPose[0][0] = b.tool_pose_x();
    toolPose[1][0] = b.tool_pose_y();
    toolPose[2][0] = b.tool_pose_z();
    toolPose[3][0] = b.tool_pose_theta_x();
    toolPose[4][0] = b.tool_pose_theta_y();
    toolPose[5][0] = b.tool_pose_theta_z();
    toolPoseValid = true;
  }
  // Gripper: only if the feedback carries its motor.
  if (feedback.has_interconnect() &&
      feedback.interconnect().has_gripper_feedback() &&
      feedback.interconnect().gripper_feedback().motor_size() > 0) {
    gripperDetected = true;
    gripperPos[0][0] =
        feedback.interconnect().gripper_feedback().motor(0).position();
    gripperVel[0][0] =
        feedback.interconnect().gripper_feedback().motor(0).velocity();
  }
}

void KinovaRobotLowLevel::sendCommand() {
  BaseCyclic::Command command;
  // Gripper: without a gripper the frame is the same as before.
  // Non-finite commands keep the measured position with zero velocity.
  if (gripperDetected) {
    const bool finite = std::isfinite(gripperPosCommand[0][0]) &&
                        std::isfinite(gripperVelCommand[0][0]) &&
                        std::isfinite(gripperForceCommand);
    auto &gripperCommand = *command.mutable_interconnect()
                                ->mutable_gripper_command()
                                ->add_motor_cmd();
    gripperCommand.set_position(
        finite ? std::clamp(gripperPosCommand[0][0], 0.0f, 100.0f)
               : gripperPos[0][0]);
    gripperCommand.set_velocity(
        finite ? std::clamp(gripperVelCommand[0][0], 0.0f, 100.0f) : 0.0f);
    gripperCommand.set_force(
        finite ? std::clamp(gripperForceCommand, 0.0f, 100.0f) : 0.0f);
  }
  lastFrameId = lastFrameId >= 65535 ? 0 : lastFrameId + 1;
  command.set_frame_id(lastFrameId);
  for (unsigned i = 0; i < actuatorCount; i++) {
    command.add_actuators();
    command.mutable_actuators(i)->set_command_id(lastFrameId);
    command.mutable_actuators(i)->set_current_motor(curCommand[i][0]);
    command.mutable_actuators(i)->set_position(posCommand[i][0] * rad2deg);
    command.mutable_actuators(i)->set_velocity(velCommand[i][0] * rad2deg);
  }
  auto now = waitForRefresh();
  try {
    updateLocal(baseCyclic.Refresh(command), now);
    timeout = false;
  } catch (KDetailedException &ex) {
    cout << "Kortex Exception on refresh" << ex.what() << endl;

  } catch (std::runtime_error &ex) {
    if (string(ex.what()).find("timeout") != size_t(-1)) {
      timeout = true;
    }
    cout << "Runtime error on refresh" << ex.what() << endl;
  } catch (...) {
    cout << "Unknown error on refresh" << endl;
  }
}

vec<7> KinovaRobotLowLevel::getPos(bool refresh) {
  steady_clock::time_point now;
  if (refresh &&
      (now = steady_clock::now()) - lastFrameTime >= milliseconds(1)) {
    updateLocal(baseCyclic.RefreshFeedback(), now);
  }
  return pos;
}

vec<7> KinovaRobotLowLevel::getVel(bool refresh) {
  steady_clock::time_point now;
  if (refresh &&
      (now = steady_clock::now()) - lastFrameTime >= milliseconds(1)) {
    updateLocal(baseCyclic.RefreshFeedback(), now);
  }
  return vel;
}

vec<7> KinovaRobotLowLevel::getTau(bool refresh) {
  steady_clock::time_point now;
  if (refresh &&
      (now = steady_clock::now()) - lastFrameTime >= milliseconds(1)) {
    updateLocal(baseCyclic.RefreshFeedback(), now);
  }
  return tau;
}

vec<7> KinovaRobotLowLevel::getCur(bool refresh) {
  steady_clock::time_point now;
  if (refresh &&
      (now = steady_clock::now()) - lastFrameTime >= milliseconds(1)) {
    updateLocal(baseCyclic.RefreshFeedback(), now);
  }
  return cur;
}

void KinovaRobotLowLevel::sendPos(const vec<7> &posCom) {
  for (unsigned i = 0; i < actuatorCount; i++) {
    switch (controlMode[i]) {
    case ControlMode::CUR:
      posCommand[i] = pos[i];
      velCommand[i] = {};
      break;
    case ControlMode::POS:
      curCommand[i] = {};
      posCommand[i] = posCom[i];
      velCommand[i] = {};
      break;
    case ControlMode::VEL:
      curCommand[i] = {};
      posCommand[i] = pos[i];
      break;
    }
  }
  sendCommand();
}

void KinovaRobotLowLevel::sendVel(const vec<7> &velCom) {
  for (unsigned i = 0; i < actuatorCount; i++) {
    switch (controlMode[i]) {
    case ControlMode::CUR:
      velCommand[i] = {};
      posCommand[i] = pos[i];
      break;
    case ControlMode::POS:
      curCommand[i] = {};
      velCommand[i] = {};
      break;
    case ControlMode::VEL:
      curCommand[i] = {};
      posCommand[i] = pos[i];
      velCommand[i] = velCom[i];
      break;
    }
  }
  sendCommand();
}

void KinovaRobotLowLevel::sendCur(const vec<7> &curCom) {
  for (unsigned i = 0; i < actuatorCount; i++) {
    switch (controlMode[i]) {
    case ControlMode::CUR:
      curCommand[i] = curCom[i];
      velCommand[i] = {};
      posCommand[i] = pos[i];
      break;
    case ControlMode::POS:
      curCommand[i] = {};
      velCommand[i] = {};
      break;
    case ControlMode::VEL:
      curCommand[i] = {};
      posCommand[i] = pos[i];
      break;
    }
  }
  sendCommand();
}

void KinovaRobotLowLevel::setControlMode(
    const array<ControlMode, 7> &newModes) {
  controlMode = newModes;
  for (unsigned i = 0; i < actuatorCount; i++) {
    switch (controlMode[i]) {
    case CUR:
      velCommand[i] = {};
      posCommand[i] = pos[i];
      break;
    case POS:
      velCommand[i] = {};
      curCommand[i] = {};
      break;
    case VEL:
      curCommand[i] = {};
      posCommand[i] = pos[i];
      break;
    }
  }
  ActuatorConfig::ControlModeInformation control_mode_message{};
  try {
    switch (actuatorCount) {
    case 7:
      for (unsigned i = 0; i < 7; i++) {
        ActuatorConfig::ControlMode mode{};
        switch (controlMode[i]) {
        case CUR:
          mode = ActuatorConfig::ControlMode::CURRENT;
          break;
        case POS:
          mode = ActuatorConfig::ControlMode::POSITION;
          break;
        case VEL:
          mode = ActuatorConfig::ControlMode::VELOCITY;
          break;
        }
        control_mode_message.set_control_mode(mode);
        actuatorConfig.SetControlMode(control_mode_message, i + 1);
      }
      break;
    case 6:
      for (unsigned i = 0; i < 6; i++) {
        ActuatorConfig::ControlMode mode{};
        switch (controlMode[i]) {
        case CUR:
          mode = ActuatorConfig::ControlMode::CURRENT;
          break;
        case POS:
          mode = ActuatorConfig::ControlMode::POSITION;
          break;
        case VEL:
          mode = ActuatorConfig::ControlMode::VELOCITY;
          break;
        }
        control_mode_message.set_control_mode(mode);
        actuatorConfig.SetControlMode(control_mode_message, i + (i == 5) + 1);
      }
    }
  } catch (...) {
  }
}
} // namespace kinova_robot
