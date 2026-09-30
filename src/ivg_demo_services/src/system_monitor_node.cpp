// system_monitor_node — IVG 演示栈健康监控。
// 源自 aubo_boot demo_driver/system_monitor_node（2026-09-30 移植）：
//  - RobotStatus 源从 ivg /robot_status 改为 aubo_msgs
//    /aubo_io_controller/robot_status（本仓 IO 控制器发布）
//  - 在线判定：drives_powered!=0 且无急停
#include <chrono>
#include <string>

#include <aubo_msgs/msg/robot_status.hpp>
#include <ivg_interfaces/msg/node_status.hpp>
#include <ivg_interfaces/msg/system_log.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace ivg_demo_services
{

class SystemMonitorNode : public rclcpp::Node
{
public:
  SystemMonitorNode()
  : Node("system_monitor_node")
  {
    status_pub_ = create_publisher<ivg_interfaces::msg::NodeStatus>(
      "/system/node_status", rclcpp::QoS(1).transient_local());
    log_pub_ = create_publisher<ivg_interfaces::msg::SystemLog>(
      "/system/log", 10);

    robot_status_sub_ = create_subscription<aubo_msgs::msg::RobotStatus>(
      "/aubo_io_controller/robot_status", 10,
      std::bind(&SystemMonitorNode::onRobotStatus, this, std::placeholders::_1));

    timer_ = create_wall_timer(1s, std::bind(&SystemMonitorNode::publishStatus, this));

    publishLog(ivg_interfaces::msg::SystemLog::LEVEL_INFO, "system_monitor", "System Monitor 已启动");
    RCLCPP_INFO(get_logger(), "System Monitor 已启动（订 /aubo_io_controller/robot_status）");
  }

private:
  void onRobotStatus(const aubo_msgs::msg::RobotStatus::SharedPtr msg)
  {
    last_robot_status_ = now();
    robot_powered_ = msg->drives_powered != 0;
    robot_estopped_ = msg->e_stopped != 0;
    robot_in_error_ = msg->in_error != 0;
  }

  void publishStatus()
  {
    auto now = this->now();
    ivg_interfaces::msg::NodeStatus status;

    // 机械臂驱动状态（映射到 aubo_dashboard_node/驱动侧健康度）
    status.node_name = "aubo_driver";
    status.node_type = "driver";
    status.last_heartbeat = now;
    auto elapsed = now - last_robot_status_;
    if (last_robot_status_.nanoseconds() > 0 && elapsed > 5s) {
      status.status = "offline";
      status.error_message = "无 RobotStatus 更新超过 5s";
    } else if (robot_estopped_ || robot_in_error_) {
      status.status = "error";
      status.error_message = robot_estopped_ ? "急停有效" : "机器人处于错误态";
    } else if (!robot_powered_) {
      status.status = "degraded";
      status.error_message = "臂电未上";
    } else {
      status.status = "online";
    }
    status.uptime_sec = uptime_sec();
    status_pub_->publish(status);
  }

  void publishLog(uint8_t level, const std::string & node, const std::string & msg)
  {
    ivg_interfaces::msg::SystemLog log;
    log.timestamp = now();
    log.level = level;
    log.node_name = node;
    log.message = msg;
    log_pub_->publish(log);
  }

  double uptime_sec() const
  {
    return (now() - start_time_).seconds();
  }

  rclcpp::Publisher<ivg_interfaces::msg::NodeStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<ivg_interfaces::msg::SystemLog>::SharedPtr log_pub_;
  rclcpp::Subscription<aubo_msgs::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_{now()};
  rclcpp::Time last_robot_status_{0, 0, RCL_ROS_TIME};
  bool robot_powered_{false};
  bool robot_estopped_{false};
  bool robot_in_error_{false};
};

}  // namespace ivg_demo_services

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ivg_demo_services::SystemMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
