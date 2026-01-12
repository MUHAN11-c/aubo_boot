/*
 * 机器人状态订阅示例
 * 演示如何订阅 /robot_status 话题
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/msg/robot_status.hpp>
#include <memory>
#include <iomanip>
#include <sstream>

class RobotStatusSubscriber : public rclcpp::Node
{
public:
    RobotStatusSubscriber() : Node("robot_status_subscriber")
    {
        subscription_ = this->create_subscription<demo_interface::msg::RobotStatus>(
            "/robot_status", 10,
            std::bind(&RobotStatusSubscriber::status_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "已订阅 /robot_status 话题");
    }

private:
    void status_callback(const demo_interface::msg::RobotStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "\n=== 机器人状态更新 ===");
        RCLCPP_INFO(this->get_logger(), "时间戳: %d.%09d",
                   msg->header.stamp.sec, msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "在线状态: %s", msg->is_online ? "在线" : "离线");
        RCLCPP_INFO(this->get_logger(), "使能状态: %s", msg->enable ? "已使能" : "未使能");
        RCLCPP_INFO(this->get_logger(), "运动状态: %s", msg->in_motion ? "运动中" : "静止");
        RCLCPP_INFO(this->get_logger(), "规划状态: %s", msg->planning_status.c_str());

        // 打印关节位置
        std::vector<std::string> joint_names = {
            "shoulder", "upperArm", "foreArm", "wrist1", "wrist2", "wrist3"
        };
        RCLCPP_INFO(this->get_logger(), "\n关节位置 (度):");
        for (size_t i = 0; i < msg->joint_position_deg.size() && i < joint_names.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  %s: %.2f°", joint_names[i].c_str(),
                       msg->joint_position_deg[i]);
        }

        // 打印笛卡尔位姿
        RCLCPP_INFO(this->get_logger(), "\n笛卡尔位姿:");
        const auto& pos = msg->cartesian_position.position;
        const auto& ori = msg->cartesian_position.orientation;
        RCLCPP_INFO(this->get_logger(), "  位置: x=%.4f m, y=%.4f m, z=%.4f m",
                   pos.x, pos.y, pos.z);
        RCLCPP_INFO(this->get_logger(), "  姿态 (四元数): w=%.4f, x=%.4f, y=%.4f, z=%.4f",
                   ori.w, ori.x, ori.y, ori.z);
        RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
    }

    rclcpp::Subscription<demo_interface::msg::RobotStatus>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto subscriber = std::make_shared<RobotStatusSubscriber>();

    RCLCPP_INFO(subscriber->get_logger(), "等待接收机器人状态消息...");
    RCLCPP_INFO(subscriber->get_logger(), "按 Ctrl+C 退出\n");

    try {
        rclcpp::spin(subscriber);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(subscriber->get_logger(), "异常: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

