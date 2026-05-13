#include <chrono>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ivg_interfaces/msg/node_status.hpp>
#include <ivg_interfaces/msg/system_log.hpp>
#include <ivg_interfaces/msg/robot_status.hpp>

using namespace std::chrono_literals;

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

        // 订阅关键组件状态
        robot_status_sub_ = create_subscription<ivg_interfaces::msg::RobotStatus>(
            "/aubo_driver/robot_status", 10,
            std::bind(&SystemMonitorNode::onRobotStatus, this, std::placeholders::_1));

        // 定时发布聚合状态 (1Hz)
        timer_ = create_wall_timer(1s, std::bind(&SystemMonitorNode::publishStatus, this));

        publishLog(ivg_interfaces::msg::SystemLog::LEVEL_INFO, "system_monitor", "System Monitor 已启动");
        RCLCPP_INFO(get_logger(), "System Monitor 已启动");
    }

private:
    void onRobotStatus(const ivg_interfaces::msg::RobotStatus::SharedPtr msg)
    {
        last_robot_status_ = now();
        robot_online_ = msg->is_online;
        robot_enabled_ = msg->enable;
    }

    void publishStatus()
    {
        auto now = this->now();
        auto status = ivg_interfaces::msg::NodeStatus();

        // 机械臂驱动状态
        status.node_name = "aubo_dashboard_node";
        status.node_type = "driver";
        status.last_heartbeat = now;
        auto elapsed = now - last_robot_status_;
        if (elapsed > 5s && last_robot_status_.nanoseconds() > 0) {
            status.status = "offline";
            status.error_message = "无 RobotStatus 更新超过 5s";
        } else if (!robot_online_) {
            status.status = "degraded";
            status.error_message = "机械臂离线";
        } else {
            status.status = "online";
        }
        status.uptime_sec = uptime_sec();
        status_pub_->publish(status);
    }

    void publishLog(uint8_t level, const std::string& node, const std::string& msg)
    {
        auto log = ivg_interfaces::msg::SystemLog();
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
    rclcpp::Subscription<ivg_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_{now()};
    rclcpp::Time last_robot_status_{0, 0, RCL_ROS_TIME};
    bool robot_online_ = false;
    bool robot_enabled_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SystemMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
