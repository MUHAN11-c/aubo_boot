/*
 * 获取当前机械臂位姿客户端
 * 演示如何调用 /get_current_state 服务获取当前位姿
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/get_current_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <memory>
#include <iomanip>
#include <sstream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

class GetCurrentPoseClient : public rclcpp::Node
{
public:
    GetCurrentPoseClient() : Node("get_current_pose_client")
    {
        client_ = this->create_client<demo_interface::srv::GetCurrentState>("/get_current_state");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "等待服务 /get_current_state 可用...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "服务 /get_current_state 不可用");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务 /get_current_state 已就绪");
        return true;
    }

    bool get_current_pose(geometry_msgs::msg::Pose& pose)
    {
        auto request = std::make_shared<demo_interface::srv::GetCurrentState::Request>();

        RCLCPP_INFO(this->get_logger(), "正在获取当前机械臂位姿...");

        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                pose = response->cartesian_position;
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "获取位姿失败: %s", response->message.c_str());
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
            return false;
        }
    }

    void print_pose(const geometry_msgs::msg::Pose& pose)
    {
        const auto& pos = pose.position;
        const auto& ori = pose.orientation;
        
        RCLCPP_INFO(this->get_logger(), "\n=== 当前机械臂位姿 ===");
        RCLCPP_INFO(this->get_logger(), "位置 (米):");
        RCLCPP_INFO(this->get_logger(), "  x = %.6f", pos.x);
        RCLCPP_INFO(this->get_logger(), "  y = %.6f", pos.y);
        RCLCPP_INFO(this->get_logger(), "  z = %.6f", pos.z);
        
        RCLCPP_INFO(this->get_logger(), "\n姿态 (四元数):");
        RCLCPP_INFO(this->get_logger(), "  w = %.6f", ori.w);
        RCLCPP_INFO(this->get_logger(), "  x = %.6f", ori.x);
        RCLCPP_INFO(this->get_logger(), "  y = %.6f", ori.y);
        RCLCPP_INFO(this->get_logger(), "  z = %.6f", ori.z);
        
        // 计算并显示欧拉角（可选）
        double roll, pitch, yaw;
        quaternion_to_euler(ori.x, ori.y, ori.z, ori.w, roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "\n姿态 (欧拉角，度):");
        RCLCPP_INFO(this->get_logger(), "  roll  (绕x轴) = %.2f°", roll * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  pitch (绕y轴) = %.2f°", pitch * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  yaw   (绕z轴) = %.2f°", yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "========================\n");
    }

private:
    void quaternion_to_euler(double x, double y, double z, double w, 
                             double& roll, double& pitch, double& yaw)
    {
        // 四元数转欧拉角 (ZYX顺序)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    rclcpp::Client<demo_interface::srv::GetCurrentState>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<GetCurrentPoseClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 获取当前位姿
    geometry_msgs::msg::Pose current_pose;
    if (client->get_current_pose(current_pose)) {
        client->print_pose(current_pose);
    } else {
        RCLCPP_ERROR(client->get_logger(), "获取位姿失败");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}

