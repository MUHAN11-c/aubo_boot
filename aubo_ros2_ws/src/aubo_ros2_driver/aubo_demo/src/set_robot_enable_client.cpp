/*
 * 设置机器人使能状态服务调用示例
 * 演示如何调用 /set_robot_enable 服务
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/set_robot_enable.hpp>
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

class SetRobotEnableClient : public rclcpp::Node
{
public:
    SetRobotEnableClient() : Node("set_robot_enable_client")
    {
        client_ = this->create_client<demo_interface::srv::SetRobotEnable>("/set_robot_enable");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "等待服务 /set_robot_enable 可用...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "服务 /set_robot_enable 不可用");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务 /set_robot_enable 已就绪");
        return true;
    }

    std::shared_ptr<demo_interface::srv::SetRobotEnable::Response> call_service(bool enable)
    {
        auto request = std::make_shared<demo_interface::srv::SetRobotEnable::Request>();
        request->enable = enable;

        const char* action = enable ? "使能" : "禁用";
        RCLCPP_INFO(this->get_logger(), "发送%s机器人请求...", action);

        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✓ 成功: %s", response->message.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ 失败 (错误码: %d): %s",
                            response->error_code, response->message.c_str());
            }
            return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "服务调用超时");
            return nullptr;
        }
    }

private:
    rclcpp::Client<demo_interface::srv::SetRobotEnable>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<SetRobotEnableClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 示例1: 使能机器人
    RCLCPP_INFO(client->get_logger(), "\n=== 示例1: 使能机器人 ===");
    client->call_service(true);

    // 等待一段时间
    std::this_thread::sleep_for(2s);

    // 示例2: 禁用机器人
    RCLCPP_INFO(client->get_logger(), "\n=== 示例2: 禁用机器人 ===");
    client->call_service(false);

    rclcpp::shutdown();
    return 0;
}

