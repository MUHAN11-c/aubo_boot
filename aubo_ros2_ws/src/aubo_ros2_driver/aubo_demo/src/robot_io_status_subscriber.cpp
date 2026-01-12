/*
 * 机器人IO状态订阅示例
 * 演示如何订阅 /robot_io_status 话题
 */

#include <rclcpp/rclcpp.hpp>
#include <demo_interface/msg/robot_io_status.hpp>
#include <memory>
#include <iomanip>

class RobotIOStatusSubscriber : public rclcpp::Node
{
public:
    RobotIOStatusSubscriber() : Node("robot_io_status_subscriber")
    {
        subscription_ = this->create_subscription<demo_interface::msg::RobotIOStatus>(
            "/robot_io_status", 10,
            std::bind(&RobotIOStatusSubscriber::io_status_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "已订阅 /robot_io_status 话题");
    }

private:
    void io_status_callback(const demo_interface::msg::RobotIOStatus::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "\n=== 机器人IO状态更新 ===");
        RCLCPP_INFO(this->get_logger(), "时间戳: %d.%09d",
                   msg->header.stamp.sec, msg->header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), "连接状态: %s", msg->is_connected ? "已连接" : "未连接");

        // 打印数字输入
        RCLCPP_INFO(this->get_logger(), "\n数字输入 (%zu 个):", msg->digital_inputs.size());
        size_t max_di = (msg->digital_inputs.size() > 16) ? 16 : msg->digital_inputs.size();
        for (size_t i = 0; i < max_di; ++i) {
            RCLCPP_INFO(this->get_logger(), "  DI[%zu]: %s", i,
                       msg->digital_inputs[i] ? "HIGH" : "LOW");
        }

        // 打印数字输出
        RCLCPP_INFO(this->get_logger(), "\n数字输出 (%zu 个):", msg->digital_outputs.size());
        size_t max_do = (msg->digital_outputs.size() > 16) ? 16 : msg->digital_outputs.size();
        for (size_t i = 0; i < max_do; ++i) {
            RCLCPP_INFO(this->get_logger(), "  DO[%zu]: %s", i,
                       msg->digital_outputs[i] ? "HIGH" : "LOW");
        }

        // 打印模拟输入
        if (!msg->analog_inputs.empty()) {
            RCLCPP_INFO(this->get_logger(), "\n模拟输入 (%zu 个):", msg->analog_inputs.size());
            size_t max_ai = (msg->analog_inputs.size() > 8) ? 8 : msg->analog_inputs.size();
            for (size_t i = 0; i < max_ai; ++i) {
                RCLCPP_INFO(this->get_logger(), "  AI[%zu]: %.3f V", i, msg->analog_inputs[i]);
            }
        }

        // 打印模拟输出
        if (!msg->analog_outputs.empty()) {
            RCLCPP_INFO(this->get_logger(), "\n模拟输出 (%zu 个):", msg->analog_outputs.size());
            size_t max_ao = (msg->analog_outputs.size() > 8) ? 8 : msg->analog_outputs.size();
            for (size_t i = 0; i < max_ao; ++i) {
                RCLCPP_INFO(this->get_logger(), "  AO[%zu]: %.3f V", i, msg->analog_outputs[i]);
            }
        }

        // 打印工具IO状态
        RCLCPP_INFO(this->get_logger(), "\n工具IO状态:");
        const auto& tool = msg->tool_io_status;
        if (!tool.digital_inputs.empty()) {
            RCLCPP_INFO(this->get_logger(), "  数字输入 (%zu 个):", tool.digital_inputs.size());
            size_t max_tool_di = (tool.digital_inputs.size() > 4) ? 4 : tool.digital_inputs.size();
            for (size_t i = 0; i < max_tool_di; ++i) {
                RCLCPP_INFO(this->get_logger(), "    Tool_DI[%zu]: %s", i,
                           tool.digital_inputs[i] ? "HIGH" : "LOW");
            }
        }
        if (!tool.digital_outputs.empty()) {
            RCLCPP_INFO(this->get_logger(), "  数字输出 (%zu 个):", tool.digital_outputs.size());
            size_t max_tool_do = (tool.digital_outputs.size() > 4) ? 4 : tool.digital_outputs.size();
            for (size_t i = 0; i < max_tool_do; ++i) {
                RCLCPP_INFO(this->get_logger(), "    Tool_DO[%zu]: %s", i,
                           tool.digital_outputs[i] ? "HIGH" : "LOW");
            }
        }

        RCLCPP_INFO(this->get_logger(), "--------------------------------------------------");
    }

    rclcpp::Subscription<demo_interface::msg::RobotIOStatus>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto subscriber = std::make_shared<RobotIOStatusSubscriber>();

    RCLCPP_INFO(subscriber->get_logger(), "等待接收机器人IO状态消息...");
    RCLCPP_INFO(subscriber->get_logger(), "按 Ctrl+C 退出\n");

    try {
        rclcpp::spin(subscriber);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(subscriber->get_logger(), "异常: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}

