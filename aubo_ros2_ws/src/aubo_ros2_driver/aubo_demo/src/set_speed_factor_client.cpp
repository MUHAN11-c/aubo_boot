/*
 * 设置速度因子服务调用示例
 * 演示如何调用 /set_speed_factor 服务
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/set_speed_factor.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class SetSpeedFactorClient : public rclcpp::Node
{
public:
    SetSpeedFactorClient() : Node("set_speed_factor_client")
    {
        client_ = this->create_client<demo_interface::srv::SetSpeedFactor>("/set_speed_factor");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "等待服务 /set_speed_factor 可用...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "服务 /set_speed_factor 不可用");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务 /set_speed_factor 已就绪");
        return true;
    }

    std::shared_ptr<demo_interface::srv::SetSpeedFactor::Response> call_service(float velocity_factor)
    {
        if (velocity_factor < 0.0f || velocity_factor > 1.0f) {
            RCLCPP_ERROR(this->get_logger(), "速度因子必须在 0.0-1.0 之间，当前值: %.2f", velocity_factor);
            return nullptr;
        }

        auto request = std::make_shared<demo_interface::srv::SetSpeedFactor::Request>();
        request->velocity_factor = velocity_factor;

        RCLCPP_INFO(this->get_logger(), "发送设置速度因子请求: %.2f", velocity_factor);

        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✓ 成功: %s", response->message.c_str());
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
    rclcpp::Client<demo_interface::srv::SetSpeedFactor>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<SetSpeedFactorClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 示例1: 设置低速度
    RCLCPP_INFO(client->get_logger(), "\n=== 示例1: 设置低速度 (20%%) ===");
    client->call_service(0.2f);

    // 示例2: 设置中等速度
    RCLCPP_INFO(client->get_logger(), "\n=== 示例2: 设置中等速度 (50%%) ===");
    client->call_service(0.5f);

    // 示例3: 设置高速度
    RCLCPP_INFO(client->get_logger(), "\n=== 示例3: 设置高速度 (80%%) ===");
    client->call_service(0.8f);

    rclcpp::shutdown();
    return 0;
}

