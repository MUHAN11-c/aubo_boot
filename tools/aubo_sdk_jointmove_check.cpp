// jointMove 物理到位验证：测量运动前后实际关节角变化。仅 home <-> camera_pose。
// Build:
//   SDK=src/aubo_e5_hardware/vendor/aubo_sdk_2_5_3
//   g++ -std=c++17 -I $SDK/include tools/aubo_sdk_jointmove_check.cpp
//     -o build/diagnostics/jointmove_check -L $SDK/lib -laubo_sdk -Wl,-rpath,$PWD/$SDK/lib
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace an = aubo_robot_namespace;
constexpr int kDof = an::ARM_DOF;
const double kHomePose[kDof] = {0.0, -0.0334, 1.236, -0.3675, 1.5701, 0.0};
const double kCameraPose[kDof] = {
  -0.27411168813705444, 0.4963911175727844, 1.7700852155685425,
  -0.2978658676147461, 1.571584939956665, -0.2750104069709778};

bool readJoints(ServiceInterface &svc, double out[kDof])
{
  an::JointParam jp{};
  if (svc.robotServiceGetJointAngleInfo(jp) != an::InterfaceCallSuccCode) {
    return false;
  }
  for (int i = 0; i < kDof; ++i) out[i] = jp.jointPos[i];
  return true;
}

double maxDiff(const double a[kDof], const double b[kDof])
{
  double d = 0.0;
  for (int i = 0; i < kDof; ++i) d = std::max(d, std::fabs(a[i] - b[i]));
  return d;
}

void leg(ServiceInterface &svc, const char *name, const double *target_pose)
{
  double before[kDof], after[kDof], target[kDof];
  readJoints(svc, before);
  for (int i = 0; i < kDof; ++i) target[i] = target_pose[i];
  const auto t0 = std::chrono::steady_clock::now();
  const int rc = svc.robotServiceJointMove(target, true);
  const double secs = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - t0).count();
  readJoints(svc, after);
  std::cout << name << ": rc=" << rc
            << " duration_s=" << std::fixed << std::setprecision(2) << secs
            << " moved_rad=" << std::setprecision(4) << maxDiff(after, before)
            << " err_to_target=" << std::setprecision(5)
            << maxDiff(after, target_pose) << '\n';
}

int main(int argc, char **argv)
{
  const std::string host = argc > 1 ? argv[1] : "169.254.10.98";
  ServiceInterface svc;
  const int login = svc.robotServiceLogin(host.c_str(), 8899, "aubo", "123456");
  std::cout << "login=" << login << '\n';
  if (login != 0) return 10;

  std::cout << "init=" << svc.robotServiceInitGlobalMoveProfile() << '\n';
  an::JointVelcAccParam v{}, a{};
  for (int i = 0; i < kDof; ++i) {
    v.jointPara[i] = 0.3;
    a.jointPara[i] = 1.0;
  }
  svc.robotServiceSetGlobalMoveJointMaxVelc(v);
  svc.robotServiceSetGlobalMoveJointMaxAcc(a);

  leg(svc, "to_camera_pose", kCameraPose);
  leg(svc, "to_home", kHomePose);

  std::cout << "logout=" << svc.robotServiceLogout() << '\n';
  return 0;
}
