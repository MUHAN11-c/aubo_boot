#include <array>
#include <iomanip>
#include <iostream>
#include <string>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace
{
constexpr int kJointCount = aubo_robot_namespace::ARM_DOF;

const char * robot_state_name(aubo_robot_namespace::RobotState state)
{
  switch (state) {
    case aubo_robot_namespace::RobotStopped:
      return "stopped";
    case aubo_robot_namespace::RobotRunning:
      return "running";
    case aubo_robot_namespace::RobotPaused:
      return "paused";
    case aubo_robot_namespace::RobotResumed:
      return "resumed";
  }
  return "unknown";
}

template<typename T>
void print_result(const char * name, int rc, const T & value)
{
  std::cout << name << ": rc=" << rc;
  if (rc == aubo_robot_namespace::InterfaceCallSuccCode) {
    std::cout << ", value=" << value;
  }
  std::cout << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string host = argc > 1 ? argv[1] : "169.254.10.98";
  const int port = argc > 2 ? std::stoi(argv[2]) : 8899;

  std::cout << "READ-ONLY SDK TEST: no startup, power, brake, IO, or motion calls\n";
  std::cout << "target=" << host << ':' << port << '\n';

  ServiceInterface service;
  const int login_rc = service.robotServiceLogin(
    host.c_str(), port, "aubo", "123456");
  std::cout << "login: rc=" << login_rc << '\n';
  if (login_rc != aubo_robot_namespace::InterfaceCallSuccCode) {
    return 2;
  }

  bool connected = false;
  service.robotServiceGetConnectStatus(connected);
  std::cout << "connected=" << std::boolalpha << connected << '\n';

  bool real_robot_exists = false;
  print_result(
    "real_robot_exists",
    service.robotServiceGetIsRealRobotExist(real_robot_exists),
    real_robot_exists);

  bool mac_communication = false;
  print_result(
    "mac_communication",
    service.robotServiceGetMacCommunicationStatus(mac_communication),
    mac_communication);

  aubo_robot_namespace::RobotWorkMode work_mode{};
  const int mode_rc = service.robotServiceGetRobotWorkMode(work_mode);
  print_result(
    "work_mode", mode_rc,
    work_mode == aubo_robot_namespace::RobotModeReal ? "real" : "simulator");

  aubo_robot_namespace::RobotState robot_state{};
  const int state_rc = service.robotServiceGetRobotCurrentState(robot_state);
  print_result("robot_state", state_rc, robot_state_name(robot_state));

  aubo_robot_namespace::JointParam joint_angles{};
  const int angles_rc = service.robotServiceGetJointAngleInfo(joint_angles);
  std::cout << "joint_angles: rc=" << angles_rc;
  if (angles_rc == aubo_robot_namespace::InterfaceCallSuccCode) {
    std::cout << ", rad=[";
    for (int i = 0; i < kJointCount; ++i) {
      if (i != 0) {
        std::cout << ", ";
      }
      std::cout << std::fixed << std::setprecision(6) << joint_angles.jointPos[i];
    }
    std::cout << ']';
  }
  std::cout << '\n';

  std::array<aubo_robot_namespace::JointStatus, kJointCount> joints{};
  const int joints_rc =
    service.robotServiceGetRobotJointStatus(joints.data(), joints.size());
  std::cout << "joint_status: rc=" << joints_rc << '\n';
  if (joints_rc == aubo_robot_namespace::InterfaceCallSuccCode) {
    for (int i = 0; i < kJointCount; ++i) {
      std::cout << "  J" << i + 1
                << ": pos_rad=" << joints[i].jointPosJ
                << ", speed=" << joints[i].jointSpeedMoto
                << ", current=" << joints[i].jointCurrentI
                << ", temp=" << joints[i].jointCurTemp
                << ", error=" << joints[i].jointErrorNum << '\n';
    }
  }

  aubo_robot_namespace::RobotDiagnosis diagnosis{};
  const int diagnosis_rc =
    service.robotServiceGetRobotDiagnosisInfo(diagnosis);
  std::cout << "diagnosis: rc=" << diagnosis_rc;
  if (diagnosis_rc == aubo_robot_namespace::InterfaceCallSuccCode) {
    std::cout << ", power=" << diagnosis.armPowerStatus
              << ", brake=" << diagnosis.brakeStuats
              << ", soft_emergency=" << diagnosis.softEmergency
              << ", remote_emergency=" << diagnosis.remoteEmergency
              << ", collision=" << diagnosis.robotCollision
              << ", joint_error=" << diagnosis.jointErrorStatus
              << ", canbus_mask=" << static_cast<int>(diagnosis.armCanbusStatus)
              << ", end_speed=" << diagnosis.robotEndSpeed;
  }
  std::cout << '\n';

  const int logout_rc = service.robotServiceLogout();
  std::cout << "logout: rc=" << logout_rc << '\n';
  return logout_rc == aubo_robot_namespace::InterfaceCallSuccCode ? 0 : 3;
}
