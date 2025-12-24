/*
 * 获取当前状态服务调用示例
 * 演示如何调用 /get_current_state 服务
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/get_current_state.hpp>
#include <chrono>
#include <memory>
#include <iomanip>
#include <sstream>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

class GetCurrentStateClient : public rclcpp::Node
{
public:
    GetCurrentStateClient() : Node("get_current_state_client")
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

    std::shared_ptr<demo_interface::srv::GetCurrentState::Response> call_service()
    {
        auto request = std::make_shared<demo_interface::srv::GetCurrentState::Request>();

        RCLCPP_INFO(this->get_logger(), "发送获取当前状态请求...");

        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✓ 成功: %s", response->message.c_str());
                print_state(response);
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ 失败: %s", response->message.c_str());
            }
            return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
            return nullptr;
        }
    }

private:
    void print_state(const std::shared_ptr<demo_interface::srv::GetCurrentState::Response>& response)
    {
        RCLCPP_INFO(this->get_logger(), "\n=== 机器人当前状态 ===");
        
        // 打印关节位置
        std::stringstream joint_rad_ss, joint_deg_ss;
        joint_rad_ss << "关节位置 (弧度): [";
        joint_deg_ss << "关节位置 (度): [";
        for (size_t i = 0; i < response->joint_position_rad.size(); ++i) {
            if (i > 0) {
                joint_rad_ss << ", ";
                joint_deg_ss << ", ";
            }
            joint_rad_ss << std::fixed << std::setprecision(4) << response->joint_position_rad[i];
            joint_deg_ss << std::fixed << std::setprecision(2)
                         << (response->joint_position_rad[i] * 180.0 / M_PI);
        }
        joint_rad_ss << "]";
        joint_deg_ss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", joint_rad_ss.str().c_str());
        RCLCPP_INFO(this->get_logger(), "%s", joint_deg_ss.str().c_str());

        // 打印关节速度
        if (!response->velocity.empty()) {
            std::stringstream vel_ss;
            vel_ss << "关节速度: [";
            for (size_t i = 0; i < response->velocity.size(); ++i) {
                if (i > 0) vel_ss << ", ";
                vel_ss << std::fixed << std::setprecision(4) << response->velocity[i];
            }
            vel_ss << "]";
            RCLCPP_INFO(this->get_logger(), "%s", vel_ss.str().c_str());
        }

        // 打印笛卡尔位姿
        RCLCPP_INFO(this->get_logger(), "\n笛卡尔位姿:");
        const auto& pos = response->cartesian_position.position;
        const auto& ori = response->cartesian_position.orientation;
        RCLCPP_INFO(this->get_logger(), "  位置: x=%.4f, y=%.4f, z=%.4f",
                   pos.x, pos.y, pos.z);
        RCLCPP_INFO(this->get_logger(), "  姿态 (四元数): w=%.4f, x=%.4f, y=%.4f, z=%.4f",
                   ori.w, ori.x, ori.y, ori.z);
    }

    rclcpp::Client<demo_interface::srv::GetCurrentState>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<GetCurrentStateClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 示例: 获取当前状态
    RCLCPP_INFO(client->get_logger(), "\n=== 示例: 获取机器人当前状态 ===");
    client->call_service();

    rclcpp::shutdown();
    return 0;
}

