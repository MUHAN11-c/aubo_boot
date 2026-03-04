/*
 * 读取机器人IO服务调用示例
 * 演示如何调用 /read_robot_io 服务
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/srv/read_robot_io.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ReadRobotIOClient : public rclcpp::Node
{
public:
    ReadRobotIOClient() : Node("read_robot_io_client")
    {
        client_ = this->create_client<demo_interface::srv::ReadRobotIO>("/demo_driver/read_robot_io");
    }

    bool wait_for_service(std::chrono::seconds timeout = 5s)
    {
        RCLCPP_INFO(this->get_logger(), "等待服务 /read_robot_io 可用...");
        if (!client_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "服务 /read_robot_io 不可用");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "服务 /read_robot_io 已就绪");
        return true;
    }

    std::shared_ptr<demo_interface::srv::ReadRobotIO::Response> call_service(
        const std::string& io_type, int32_t io_index)
    {
        auto request = std::make_shared<demo_interface::srv::ReadRobotIO::Request>();
        request->io_type = io_type;
        request->io_index = io_index;

        RCLCPP_INFO(this->get_logger(), "发送读取IO请求: 类型=%s, 索引=%d",
                   io_type.c_str(), io_index);

        auto future = client_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "✓ 成功: %s", response->message.c_str());
                RCLCPP_INFO(this->get_logger(), "  IO值: %.2f", response->value);
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
    rclcpp::Client<demo_interface::srv::ReadRobotIO>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto client = std::make_shared<ReadRobotIOClient>();

    if (!client->wait_for_service()) {
        RCLCPP_ERROR(client->get_logger(), "无法连接到服务，退出");
        rclcpp::shutdown();
        return 1;
    }

    // 示例1: 读取数字输入
    RCLCPP_INFO(client->get_logger(), "\n=== 示例1: 读取数字输入 #0 ===");
    client->call_service("digital_input", 0);

    // 示例2: 读取数字输出
    RCLCPP_INFO(client->get_logger(), "\n=== 示例2: 读取数字输出 #0 ===");
    client->call_service("digital_output", 0);

    // 示例3: 读取模拟输入
    RCLCPP_INFO(client->get_logger(), "\n=== 示例3: 读取模拟输入 #0 ===");
    client->call_service("analog_input", 0);

    // 示例4: 读取模拟输出
    RCLCPP_INFO(client->get_logger(), "\n=== 示例4: 读取模拟输出 #0 ===");
    client->call_service("analog_output", 0);

    // 示例5: 读取工具IO
    RCLCPP_INFO(client->get_logger(), "\n=== 示例5: 读取工具IO #0 ===");
    client->call_service("tool_io", 0);

    rclcpp::shutdown();
    return 0;
}

