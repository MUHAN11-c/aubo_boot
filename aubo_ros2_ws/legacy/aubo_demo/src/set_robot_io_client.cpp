/*
 * 设置机器人IO服务调用示例
 * 演示如何调用 /demo_driver/set_io 服务
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/set_robot_io.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class SetRobotIOClient : public rclcpp::Node
{
public:
    SetRobotIOClient() : Node("set_robot_io_client")
    {
        client_ = this->create_client<demo_interface::srv::SetRobotIO>("/demo_driver/set_io");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "等待服务 /demo_driver/set_io 可用...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "服务 /demo_driver/set_io 不可用");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务 /demo_driver/set_io 已就绪");
        return true;
    }

    std::shared_ptr<demo_interface::srv::SetRobotIO::Response> call_service(
        const std::string& io_type, int32_t io_index, double value)
    {
        auto request = std::make_shared<demo_interface::srv::SetRobotIO::Request>();
        request->io_type = io_type;
        request->io_index = io_index;
        request->value = value;

        RCLCPP_INFO(this->get_logger(), "发送设置IO请求: 类型=%s, 索引=%d, 值=%.2f",
                   io_type.c_str(), io_index, value);

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
    rclcpp::Client<demo_interface::srv::SetRobotIO>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<SetRobotIOClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 示例1: 设置数字输出为高电平
    RCLCPP_INFO(client->get_logger(), "\n=== 示例1: 设置数字输出 #0 为 HIGH ===");
    client->call_service("digital_output", 0, 1.0);

    // 示例2: 设置数字输出为低电平
    RCLCPP_INFO(client->get_logger(), "\n=== 示例2: 设置数字输出 #0 为 LOW ===");
    client->call_service("digital_output", 1, 0.0);

    // 示例3: 设置模拟输出
    RCLCPP_INFO(client->get_logger(), "\n=== 示例3: 设置模拟输出 #0 为 3.3V ===");
    client->call_service("analog_output", 2, 3.3);

    // 示例4: 设置工具IO
    RCLCPP_INFO(client->get_logger(), "\n=== 示例4: 设置工具IO #0 为 HIGH ===");
    client->call_service("tool_io", 0, 1.0);

    // 示例5: 设置工具IO为输入模式
    RCLCPP_INFO(client->get_logger(), "\n=== 示例5: 设置工具IO #1 为输入模式 ===");
    client->call_service("tool_io", 1, -1.0);

    rclcpp::shutdown();
    return 0;
}

