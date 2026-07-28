// jointSpeedMoto 语义实测：静止与微动下 轮询 vs 推送 的速度读数对比。
// 排查 JTC 用推送速度做样条初值导致开局冲刺（走廊 cause=2）的嫌疑。
//
// Build:
//   SDK=src/aubo_e5_hardware/vendor/aubo_sdk_2_5_3
//   g++ -std=c++17 -I $SDK/include tools/aubo_sdk_speed_check.cpp
//     -o build/diagnostics/speed_check -L $SDK/lib -laubo_sdk -Wl,-rpath,$PWD/$SDK/lib
// Usage: speed_check [host]

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

namespace an = aubo_robot_namespace;

int main(int argc, char **argv)
{
  const char *host = argc > 1 ? argv[1] : "169.254.10.98";
  ServiceInterface svc;
  if (svc.robotServiceLogin(host, 8899, "aubo", "123456") != 0) {
    std::cout << "login failed\n";
    return 10;
  }

  std::cout << std::fixed << std::setprecision(6);
  double last_pos[6] = {0};
  bool has_last = false;
  auto last_t = std::chrono::steady_clock::now();

  for (int k = 0; k < 40; ++k) {
    an::JointStatus st[6]{};
    if (svc.robotServiceGetRobotJointStatus(st, 6) ==
        an::InterfaceCallSuccCode) {
      const auto now = std::chrono::steady_clock::now();
      std::cout << "J2 pos=" << st[1].jointPosJ
                << " speedMoto=" << st[1].jointSpeedMoto
                << " tagSpeed=" << st[1].jointTagSpeedMoto;
      if (has_last) {
        const double dt =
          std::chrono::duration<double>(now - last_t).count();
        if (dt > 1e-3) {
          std::cout << " diff_vel=" << (st[1].jointPosJ - last_pos[1]) / dt;
        }
      }
      std::cout << '\n';
      for (int i = 0; i < 6; ++i) last_pos[i] = st[i].jointPosJ;
      last_t = now;
      has_last = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  svc.robotServiceLogout();
  return 0;
}
