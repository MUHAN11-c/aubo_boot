/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 *
 * 接口与逻辑参考 ROS1 实现：
 *   aubo_ws/src/aubo_robot/aubo_robot/demo_driver/src/set_robot_io_server.cpp
 * 服务名 /demo_driver/set_io，内部调用 /aubo_driver/set_io（aubo_msgs/SetIO）。
 */

#include "demo_driver/set_robot_io_server.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>

namespace demo_driver
{

/**
 * @brief 构造函数，初始化服务客户端和服务服务器
 */
SetRobotIOServer::SetRobotIOServer()
    : Node("set_robot_io_server_node")
    , aubo_set_io_service_name_("/aubo_driver/set_io")
{
    // 获取参数
    this->declare_parameter("aubo_set_io_service", std::string("/aubo_driver/set_io"));
    this->get_parameter("aubo_set_io_service", aubo_set_io_service_name_);

    // 初始化服务客户端
    aubo_set_io_client_ = this->create_client<aubo_msgs::srv::SetIO>(aubo_set_io_service_name_);

    // 等待服务可用
    if (!aubo_set_io_client_->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_WARN(this->get_logger(), "Aubo SetIO service not available: %s", aubo_set_io_service_name_.c_str());
        RCLCPP_WARN(this->get_logger(), "Make sure aubo_driver is running");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Connected to Aubo SetIO service: %s", aubo_set_io_service_name_.c_str());
    }

    // 初始化服务服务器（与 ROS1 接口一致：/demo_driver/set_io）
    set_robot_io_service_ = this->create_service<demo_interface::srv::SetRobotIO>(
        "/demo_driver/set_io",
        std::bind(&SetRobotIOServer::setRobotIOCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "SetRobotIOServer initialized, service '/demo_driver/set_io' is ready");
}

SetRobotIOServer::~SetRobotIOServer()
{
}

/**
 * @brief 服务回调函数
 * @param req 服务请求
 * @param res 服务响应
 */
void SetRobotIOServer::setRobotIOCallback(
    const std::shared_ptr<demo_interface::srv::SetRobotIO::Request> req,
    std::shared_ptr<demo_interface::srv::SetRobotIO::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "Received set_robot_io request");
    RCLCPP_INFO(this->get_logger(), "IO type: %s, index: %d, value: %.2f",
             req->io_type.c_str(), req->io_index, req->value);

    // 验证输入参数
    if (req->io_index < 0)
    {
        res->success = false;
        res->error_code = -1;
        res->message = "Invalid io_index, must be >= 0";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    // 设置机器人IO
    int32_t error_code = 0;
    std::string message;
    bool success = setRobotIO(req->io_type, req->io_index, req->value, error_code, message);

    res->success = success;
    res->error_code = error_code;
    res->message = message;

    if (success)
    {
        RCLCPP_INFO(this->get_logger(), "Set robot IO succeeded: %s", message.c_str());
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "Set robot IO failed: %s (error_code: %d)", message.c_str(), error_code);
    }
}

/**
 * @brief 设置机器人IO
 * @param io_type IO类型（digital_input/digital_output/analog_input/analog_output/tool_io）
 * @param io_index IO点索引
 * @param value 要设置的值
 * @param error_code 输出错误代码
 * @param message 输出消息
 * @return 成功返回true
 */
bool SetRobotIOServer::setRobotIO(const std::string& io_type,
                                  int32_t io_index,
                                  double value,
                                  int32_t& error_code,
                                  std::string& message)
{
    // 检查服务是否可用
    if (!aubo_set_io_client_->service_is_ready())
    {
        error_code = -100;
        message = "Aubo SetIO service not available: " + aubo_set_io_service_name_;
        return false;
    }

    try
    {
        // 将IO类型字符串转换为小写以便比较
        std::string io_type_lower = io_type;
        std::transform(io_type_lower.begin(), io_type_lower.end(), io_type_lower.begin(),
                      [](unsigned char c) { return std::tolower(c); });

        auto request = std::make_shared<aubo_msgs::srv::SetIO::Request>();
        bool valid_type = false;

        // 根据IO类型设置功能码和参数
        if (io_type_lower == "digital_output" || io_type_lower == "digitaloutput")
        {
            // 数字输出：fun=1 (RobotBoardUserDO)
            request->fun = 1;
            request->pin = static_cast<int8_t>(io_index);
            // 数字IO：0.0表示低电平，1.0表示高电平
            request->state = (value != 0.0) ? 1.0f : 0.0f;
            valid_type = true;
            RCLCPP_INFO(this->get_logger(), "Setting digital output pin %d to %s", io_index,
                     (request->state != 0.0) ? "HIGH" : "LOW");
        }
        else if (io_type_lower == "analog_output" || io_type_lower == "analogoutput")
        {
            // 模拟输出：fun=2 (RobotBoardUserAO)
            request->fun = 2;
            request->pin = static_cast<int8_t>(io_index);
            request->state = static_cast<float>(value);
            valid_type = true;
            RCLCPP_INFO(this->get_logger(), "Setting analog output pin %d to %.2f", io_index, value);
        }
        else if (io_type_lower == "tool_io" || io_type_lower == "toolio")
        {
            // 工具数字IO：fun=3 (ToolDigitalIO)
            request->fun = 3;
            request->pin = static_cast<int8_t>(io_index);
            // 如果value为-1，设置为输入模式；否则设置为输出模式并设置值
            if (value == -1.0)
            {
                request->state = -1.0f;  // 设置为输入模式
                RCLCPP_INFO(this->get_logger(), "Setting tool IO pin %d to input mode", io_index);
            }
            else
            {
                request->state = (value != 0.0) ? 1.0f : 0.0f;  // 设置为输出模式并设置值
                RCLCPP_INFO(this->get_logger(), "Setting tool IO pin %d to output mode with value %s", io_index,
                         (request->state != 0.0) ? "HIGH" : "LOW");
            }
            valid_type = true;
        }
        else if (io_type_lower == "tool_analog_output" || io_type_lower == "toolanalogoutput")
        {
            // 工具模拟输出：fun=4 (RobotToolAO)
            request->fun = 4;
            request->pin = static_cast<int8_t>(io_index);
            request->state = static_cast<float>(value);
            valid_type = true;
            RCLCPP_INFO(this->get_logger(), "Setting tool analog output pin %d to %.2f", io_index, value);
        }
        else if (io_type_lower == "digital_input" || io_type_lower == "digitalinput" ||
                 io_type_lower == "analog_input" || io_type_lower == "analoginput")
        {
            // 输入类型不能设置，只能读取
            error_code = -2;
            message = "Cannot set input IO. Input IO can only be read, not written. "
                     "Use 'digital_output' or 'analog_output' for setting values.";
            return false;
        }
        else
        {
            error_code = -3;
            message = "Invalid io_type: " + io_type +
                     ". Valid types: digital_output, analog_output, tool_io, tool_analog_output";
            return false;
        }

        if (!valid_type)
        {
            error_code = -4;
            message = "Failed to determine IO type";
            return false;
        }

        // 调用 aubo_driver 的 IO 设置服务（与 ROS1 set_robot_io_server 行为一致）
        auto result = aubo_set_io_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            if (response->success)
            {
                error_code = 0;
                message = "Successfully set " + io_type + " pin " +
                         std::to_string(io_index) + " to " + std::to_string(value);
                return true;
            }
            else
            {
                error_code = -5;
                message = "Aubo SetIO service returned failure";
                return false;
            }
        }
        else
        {
            error_code = -6;
            message = "Failed to call Aubo SetIO service";
            return false;
        }
    }
    catch (const std::exception& e)
    {
        error_code = -200;
        message = std::string("Exception occurred: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

/**
 * @brief 主循环，保持节点运行
 */
void SetRobotIOServer::spin()
{
    rclcpp::spin(this->shared_from_this());
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<demo_driver::SetRobotIOServer>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("set_robot_io_server_node"),
                    "set_robot_io_server_node 异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
