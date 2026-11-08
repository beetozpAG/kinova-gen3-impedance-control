// Kinova Headers
#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include "matrix.h"

namespace kinova_robot {

using namespace matrix;

using namespace Kinova::Api;
using namespace std;

/***********************************************
 *  Enum for the different low level control modes supported by the robots.
 ***********************************************/
enum class ControlMode : int {
  CUR = 0, /*!< value 1 */
  POS = 1, /*!< value 2 */
  VEL = 2, /*!< value 3 */
};

/*! \enum ControlMode
 * A description of the enum type.
 */

/*! \var ControlMode::CUR
 * A description of the enum type.
 */

/*! \var ControlMode::POS
 * A description of the enum type.
 */

/*! \var ControlMode::VEL
 * A description of the enum type.
 */

/***********************************************
 *  Represents a Kinova Gen 3 or Gen 3 Lite robot. This class is responsible for
 *  connecting to the robot and also abstracts the process of controling the
 *robot and getting it's status
 ***********************************************/
class KinovaRobotLowLevel {
  static constexpr unsigned port_tcp = 10000;
  static constexpr unsigned port_udp = 10001;
  TransportClientTcp transportTcp{};
  RouterClient routerTcp;
  TransportClientUdp transportUdp{};
  RouterClient routerUdp;
  Session::CreateSessionInfo session_info;
  SessionManager sessionManagerTcp;
  SessionManager sessionManagerUdp;
  Base::BaseClient base;
  ActuatorConfig::ActuatorConfigClient actuatorConfig;
  BaseCyclic::BaseCyclicClient baseCyclic;
  unsigned actuatorCount;
  steady_clock::time_point lastFrameTime;
  unsigned lastFrameId{};
  vec<7> pos{};
  vec<7> vel{};
  vec<7> tau{};
  vec<7> cur{};
  vec<7> posCommand{};
  vec<7> curCommand{};
  vec<7> velCommand{};
  array<ControlMode, 7> controlMode{};

public:
  // Gripper (Robotiq 2F-85) through the interconnect, same scheme as
  // KinovaOpenCVCPP2026/KinovaRobot. Kortex units: 0-100 %
  // (position 0 = open, 100 = closed).
  // The gripper command is only sent if the feedback reports a gripper.
  // At startup: position command = measured position, velocity = 0, so the
  // gripper does not move until a program changes these commands.
  bool gripperDetected = false;
  vec<1> gripperPos{};             // measured position [%]
  vec<1> gripperVel{};             // measured velocity [%]
  vec<1> gripperPosCommand{};      // desired position [%]
  vec<1> gripperVelCommand{};      // desired velocity [%] (0 = do not move)
  float gripperForceCommand = 1.0f; // force [%], same value as the lab code

  // TOOL pose computed by the robot (its own calibration, Tool configuration
  // of the web app): x y z [m], theta_x theta_y theta_z [deg, x-y-z
  // extrinsic]. Same values as the web app. Read only (used by vision).
  bool toolPoseValid = false;
  vec<6> toolPose{};

  vec<7> command{};
  bool timeout = false;
  std::function<void(void *)> wait_callback = [](void *) {};
  void *wait_callback_user_data = {};
  KinovaRobotLowLevel(string ip_address = "192.168.1.10",
                      string username = "admin", string password = "admin");
  ~KinovaRobotLowLevel();

private:
  steady_clock::time_point waitForRefresh();
  void updateLocal(const BaseCyclic::Feedback &, steady_clock::time_point);
  void sendCommand();

public:
  void setControlMode(const array<ControlMode, 7> &);
  vec<7> getPos(bool refresh = false);
  vec<7> getVel(bool refresh = false);
  vec<7> getTau(bool refresh = false);
  vec<7> getCur(bool refresh = false);
  void sendPos(const vec<7> &);
  void sendVel(const vec<7> &);
  void sendCur(const vec<7> &);
};

} // namespace kinova_robot
