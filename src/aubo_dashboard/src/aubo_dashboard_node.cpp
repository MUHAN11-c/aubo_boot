// Copyright 2026, aubo_e5_ros2_ws authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// ============================================================================
// aubo_dashboard_node.cpp —— AUBO 控制柜慢速操作的独立服务节点。
// 蓝本：aubo_boot aubo_dashboard_node（自带 ServiceInterface 连接、
// sdk_mutex_ 串行化，/home/wjz/桌面/aubo_boot）。
// 只提供非运动类服务：TCP2CAN 激活期间，SDK 运动类 API 由硬件插件独占，
// 本节点不得触碰（避免与 sendLoop/ioLoop 的调用竞争同一控制通道）。
// 线程模型：单线程 spin；所有服务回调在 rclcpp 执行线程上跑，经
// sdk_mutex_ 串行化后访问 sdk_（ServiceInterface 单实例不并发）。
// ============================================================================
#include <csignal>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "aubo_msgs/srv/get_fk.hpp"
#include "aubo_msgs/srv/get_ik.hpp"
#include "aubo_msgs/srv/set_payload.hpp"

#include "aubo_driver/serviceinterface.h"
#include "aubo_driver/AuboRobotMetaType.h"

class AuboDashboardNode : public rclcpp::Node
{
public:
  AuboDashboardNode()
  : Node("aubo_dashboard")
  {
    robot_ip_ = declare_parameter("robot_ip", "169.254.10.98");
    server_port_ = declare_parameter("server_port", 8899);
    username_ = declare_parameter("sdk_username", std::string("aubo"));
    password_ = declare_parameter("sdk_password", std::string("123456"));

    // 蓝本：登录最多重试 5 次。
    for (int i = 0; i < 5 && !connected_; ++i) {
      connected_ = sdk_.robotServiceLogin(
          robot_ip_.c_str(), server_port_, username_.c_str(), password_.c_str()) ==
        aubo_robot_namespace::InterfaceCallSuccCode;
      if (!connected_) {
        RCLCPP_WARN(get_logger(), "login attempt %d/5 failed", i + 1);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }
    RCLCPP_INFO(get_logger(), connected_ ? "connected to %s:%d" : "NOT connected (%s:%d)",
                robot_ip_.c_str(), server_port_);

    using Trigger = std_srvs::srv::Trigger;
    // 登录失败（5 次重试后）仍创建全部服务，但每个回调入口检查 connected_：
    // 未连接时不触碰 SDK（对未登录的 ServiceInterface 发起调用的行为 SDK
    // 未定义），Trigger 类返回 success=false + "not connected"，FK/IK 无
    // success 字段的服务直接丢弃请求（与参数非法的既有处理一致）。
    startup_srv_ = create_service<Trigger>(
        "~/startup", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
        if (!connected_) {
          resp->success = false;
          resp->message = "not connected";
          return;
        }
        std::lock_guard<std::mutex> lock(sdk_mutex_);
          // 蓝本 aubo_boot aubo_dashboard_node.cpp:245-251：动力学参数全零
          // （无工具）、碰撞等级 6、readPose=true、staticCollisionDetect=true、
          // boardMaxAcc=1000、IsBlock=true（阻塞等待启动完成）。
        aubo_robot_namespace::ToolDynamicsParam dyn;
        std::memset(&dyn, 0, sizeof(dyn));
        aubo_robot_namespace::ROBOT_SERVICE_STATE result;
        int ret = sdk_.rootServiceRobotStartup(dyn, 6, true, true, 1000, result, true);
        resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode) &&
        (result == aubo_robot_namespace::ROBOT_SERVICE_WORKING);
        resp->message = resp->success ? "startup ok" : "startup failed";
        });
    shutdown_srv_ = create_service<Trigger>(
        "~/shutdown", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
        if (!connected_) {
          resp->success = false;
          resp->message = "not connected";
          return;
        }
        std::lock_guard<std::mutex> lock(sdk_mutex_);
          // 蓝本顺序：先停运动（RobotMoveStop），再关机。
        sdk_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveStop);
        int ret = sdk_.robotServiceRobotShutdown(true);
        resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
        resp->message = resp->success ? "shutdown ok" : "shutdown failed";
        });
    release_brake_srv_ = create_service<Trigger>(
        "~/release_brake",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
        if (!connected_) {
          resp->success = false;
          resp->message = "not connected";
          return;
        }
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        int ret = sdk_.robotServiceReleaseBrake();
        resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
        resp->message = resp->success ? "brake released" : "release brake failed";
        });
    stop_srv_ = create_service<Trigger>(
        "~/stop", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
        if (!connected_) {
          resp->success = false;
          resp->message = "not connected";
          return;
        }
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        int ret = sdk_.rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveStop);
        resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
        resp->message = resp->success ? "motion stopped" : "stop failed";
        });
    fast_stop_srv_ = create_service<Trigger>(
        "~/fast_stop",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
        if (!connected_) {
          resp->success = false;
          resp->message = "not connected";
          return;
        }
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        int ret = sdk_.robotMoveFastStop();
        resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
        resp->message = resp->success ? "fast stop ok" : "fast stop failed";
        });
    collision_recover_srv_ = create_service<Trigger>(
        "~/collision_recover",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr resp) {
        if (!connected_) {
          resp->success = false;
          resp->message = "not connected";
          return;
        }
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        int ret = sdk_.robotServiceCollisionRecover();
        resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
        resp->message = resp->success ? "collision recovered" : "collision recover failed";
        });

    get_fk_srv_ = create_service<aubo_msgs::srv::GetFK>(
        "~/get_fk", [this](const aubo_msgs::srv::GetFK::Request::SharedPtr req,
      aubo_msgs::srv::GetFK::Response::SharedPtr resp) {
        if (!connected_ || req->joint.size() < 6) {return;}
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        double joint[6];
        for (int i = 0; i < 6; ++i) {joint[i] = req->joint[i];}
        aubo_robot_namespace::wayPoint_S wp;
        int ret = sdk_.robotServiceRobotFk(joint, 6, wp);
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
          resp->pos = {static_cast<float>(wp.cartPos.position.x),
            static_cast<float>(wp.cartPos.position.y),
            static_cast<float>(wp.cartPos.position.z)};
          resp->ori = {static_cast<float>(wp.orientation.w),
            static_cast<float>(wp.orientation.x),
            static_cast<float>(wp.orientation.y),
            static_cast<float>(wp.orientation.z)};
        }
        });
    get_ik_srv_ = create_service<aubo_msgs::srv::GetIK>(
        "~/get_ik", [this](const aubo_msgs::srv::GetIK::Request::SharedPtr req,
      aubo_msgs::srv::GetIK::Response::SharedPtr resp) {
        if (!connected_ || req->ref_joint.size() < 6 || req->pos.size() < 3 ||
        req->ori.size() < 4)
        {
          return;
        }
        std::lock_guard<std::mutex> lock(sdk_mutex_);
        double ref_joint[6];
        for (int i = 0; i < 6; ++i) {ref_joint[i] = req->ref_joint[i];}
        aubo_robot_namespace::Pos position;
        position.x = req->pos[0];
        position.y = req->pos[1];
        position.z = req->pos[2];
        aubo_robot_namespace::Ori ori;
        ori.w = req->ori[0];
        ori.x = req->ori[1];
        ori.y = req->ori[2];
        ori.z = req->ori[3];
        aubo_robot_namespace::wayPoint_S wp;
        int ret = sdk_.robotServiceRobotIk(ref_joint, position, ori, wp);
        if (ret == aubo_robot_namespace::InterfaceCallSuccCode) {
          resp->joint.resize(6);
          for (int i = 0; i < 6; ++i) {resp->joint[i] = static_cast<float>(wp.jointpos[i]);}
        }
        });
    set_payload_srv_ = create_service<aubo_msgs::srv::SetPayload>(
        "~/set_payload", [this](const aubo_msgs::srv::SetPayload::Request::SharedPtr req,
      aubo_msgs::srv::SetPayload::Response::SharedPtr resp) {
        if (!connected_) {
          resp->success = false;
          return;
        }
        std::lock_guard<std::mutex> lock(sdk_mutex_);
          // 注意（SDK 约束）：工具动力学参数按设计应在上电（startup）前
          // 设置 —— startup 流程本身就会写入一份全零参数。运行中调用本
          // 服务可能不生效（固件可忽略或延迟采纳），此处只 WARN 提示、
          // 不改变既有行为。
        RCLCPP_WARN(
              get_logger(),
              "set_payload while running: the SDK expects tool dynamics to be set "
              "before power-on; this call may not take effect until next startup");
          // 只改负载质量，其余动力学字段保持全零（同 startup 的无工具约定）。
        aubo_robot_namespace::ToolDynamicsParam param;
        std::memset(&param, 0, sizeof(param));
        param.payload = req->payload;
        int ret = sdk_.robotServiceSetToolDynamicsParam(param);
        resp->success = (ret == aubo_robot_namespace::InterfaceCallSuccCode);
        });
  }

  ~AuboDashboardNode() override
  {
    if (connected_) {sdk_.robotServiceLogout();}
  }

private:
  ServiceInterface sdk_;
  std::mutex sdk_mutex_;  // ServiceInterface 单实例不并发，回调间串行化
  std::string robot_ip_;
  int server_port_{8899};
  std::string username_, password_;
  bool connected_{false};

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr startup_srv_, shutdown_srv_,
    release_brake_srv_, stop_srv_, fast_stop_srv_, collision_recover_srv_;
  rclcpp::Service<aubo_msgs::srv::GetFK>::SharedPtr get_fk_srv_;
  rclcpp::Service<aubo_msgs::srv::GetIK>::SharedPtr get_ik_srv_;
  rclcpp::Service<aubo_msgs::srv::SetPayload>::SharedPtr set_payload_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AuboDashboardNode>();
  rclcpp::spin(node);
  node.reset();  // 保证 SIGINT 后先析构节点（登出），再 shutdown
  rclcpp::shutdown();
  return 0;
}
