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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/qos.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>

namespace demo_driver
{

// aubo_driver 协议功能码，与 aubo_driver_ros2.cpp setIO 实现一致
constexpr int8_t AUBO_FUN_DIGITAL_OUT = 1;   // RobotBoardUserDO
constexpr int8_t AUBO_FUN_ANALOG_OUT = 2;    // RobotBoardUserAO
constexpr int8_t AUBO_FUN_TOOL_DIGITAL = 3;  // ToolDigitalIO
constexpr int8_t AUBO_FUN_TOOL_ANALOG = 4;   // RobotToolAO

// tool_io 专用：value=-1 表示将 pin 设为输入模式
constexpr double TOOL_IO_VALUE_INPUT_MODE = -1.0;

// pin 字段为 int8_t，有效范围 0~17
constexpr int32_t PIN_INDEX_MAX = 17;

/**
 * @brief 构造函数，初始化服务客户端和服务服务器
 */
SetRobotIOServer::SetRobotIOServer()
    : Node("set_robot_io_server_node")
    , aubo_set_io_service_name_("/aubo_driver/set_io")
    , service_wait_timeout_(5)
    , call_timeout_(5)
{
    // 参数：服务名、超时
    this->declare_parameter("aubo_set_io_service", std::string("/aubo_driver/set_io"));
    this->declare_parameter("service_wait_timeout_sec", 5);
    this->declare_parameter("call_timeout_sec", 5);
    this->get_parameter("aubo_set_io_service", aubo_set_io_service_name_);
    int wait_sec = 0, call_sec = 0;
    this->get_parameter("service_wait_timeout_sec", wait_sec);
    this->get_parameter("call_timeout_sec", call_sec);
    service_wait_timeout_ = std::chrono::seconds(std::max(1, wait_sec));
    call_timeout_ = std::chrono::seconds(std::max(1, call_sec));

    // 回调组：服务回调独立，避免阻塞
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // 服务客户端（异步初始化，不阻塞启动），QoS 使用默认
    aubo_set_io_client_ = this->create_client<aubo_msgs::srv::SetIO>(aubo_set_io_service_name_);
    RCLCPP_INFO(this->get_logger(), "SetRobotIOServer 已启动，aubo 服务: %s", aubo_set_io_service_name_.c_str());

    // 服务服务器，QoS 使用默认，callback_group 独立
    set_robot_io_service_ = this->create_service<demo_interface::srv::SetRobotIO>(
        "/demo_driver/set_io",
        std::bind(&SetRobotIOServer::setRobotIOCallback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_);

    RCLCPP_INFO(this->get_logger(), "服务 /demo_driver/set_io 已就绪");
}

SetRobotIOServer::~SetRobotIOServer()
{
    set_robot_io_service_.reset();
    aubo_set_io_client_.reset();
    callback_group_.reset();
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
    if (!req || !res)
    {
        RCLCPP_ERROR(this->get_logger(), "回调参数无效");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "收到请求: io_type=%s, io_index=%d, value=%.2f",
             req->io_type.c_str(), req->io_index, req->value);

    // 严格校验：io_type 非空
    if (req->io_type.empty())
    {
        res->success = false;
        res->error_code = static_cast<int32_t>(SetRobotIOServer::ErrorCode::kInvalidIoType);
        res->message = "io_type 不能为空";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    // 严格校验：io_index 范围 0~127
    if (req->io_index < 0)
    {
        res->success = false;
        res->error_code = static_cast<int32_t>(SetRobotIOServer::ErrorCode::kInvalidIoIndex);
        res->message = "io_index 非法，需 >= 0";
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }
    if (req->io_index > PIN_INDEX_MAX)
    {
        res->success = false;
        res->error_code = static_cast<int32_t>(SetRobotIOServer::ErrorCode::kIoIndexOutOfRange);
        res->message = "io_index 超出范围 0~" + std::to_string(PIN_INDEX_MAX);
        RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        return;
    }

    // 设置机器人IO
    int32_t error_code = static_cast<int32_t>(SetRobotIOServer::ErrorCode::kSuccess);
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
    // 状态检查：客户端有效
    if (!aubo_set_io_client_)
    {
        error_code = static_cast<int32_t>(ErrorCode::kAuboServiceNotReady);
        message = "aubo_driver 客户端未初始化";
        return false;
    }

    // 状态检查：等待服务就绪
    if (!aubo_set_io_client_->service_is_ready() &&
        !aubo_set_io_client_->wait_for_service(service_wait_timeout_))
    {
        error_code = static_cast<int32_t>(ErrorCode::kAuboServiceNotReady);
        message = "aubo_driver 服务不可用";
        return false;
    }

    try
    {
        // 将IO类型字符串转换为小写以便比较
        std::string io_type_lower = io_type;
        std::transform(io_type_lower.begin(), io_type_lower.end(), io_type_lower.begin(),
                      [](unsigned char c) { return std::tolower(c); });

        auto request = std::make_shared<aubo_msgs::srv::SetIO::Request>();

        // 根据IO类型设置功能码和参数
        if (io_type_lower == "digital_output")
        {
            request->fun = AUBO_FUN_DIGITAL_OUT;
            request->pin = static_cast<int8_t>(io_index);
            // 数字IO：0.0表示低电平，1.0表示高电平
            request->state = (value != 0.0) ? 1.0f : 0.0f;
            RCLCPP_INFO(this->get_logger(), "Setting digital output pin %d to %s", io_index,
                     (request->state != 0.0) ? "HIGH" : "LOW");
        }
        else if (io_type_lower == "analog_output")
        {
            request->fun = AUBO_FUN_ANALOG_OUT;
            request->pin = static_cast<int8_t>(io_index);
            request->state = static_cast<float>(value);
            RCLCPP_INFO(this->get_logger(), "Setting analog output pin %d to %.2f", io_index, value);
        }
        else if (io_type_lower == "tool_io")
        {
            request->fun = AUBO_FUN_TOOL_DIGITAL;
            request->pin = static_cast<int8_t>(io_index);
            if (value == TOOL_IO_VALUE_INPUT_MODE)
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
        }
        else if (io_type_lower == "tool_analog_output")
        {
            request->fun = AUBO_FUN_TOOL_ANALOG;
            request->pin = static_cast<int8_t>(io_index);
            request->state = static_cast<float>(value);
            RCLCPP_INFO(this->get_logger(), "Setting tool analog output pin %d to %.2f", io_index, value);
        }
        else if (io_type_lower == "digital_input" || io_type_lower == "analog_input")
        {
            // 输入类型不能设置，只能读取
            error_code = static_cast<int32_t>(ErrorCode::kInputTypeNotWritable);
            message = "输入类型不可写";
            return false;
        }
        else
        {
            error_code = static_cast<int32_t>(ErrorCode::kInvalidIoType);
            message = "无效 io_type: " + io_type;
            return false;
        }

        // 调用 aubo_driver 的 IO 设置服务
        auto result = aubo_set_io_client_->async_send_request(request);
        if (result.wait_for(call_timeout_) != std::future_status::ready)
        {
            error_code = static_cast<int32_t>(ErrorCode::kAuboServiceCallTimeout);
            message = "调用超时";
            return false;
        }
        auto response = result.get();
        if (response->success)
        {
            error_code = static_cast<int32_t>(ErrorCode::kSuccess);
            message = "已设置 " + io_type + " pin " +
                     std::to_string(io_index) + " = " + std::to_string(value);
            return true;
        }
        else
        {
            error_code = static_cast<int32_t>(ErrorCode::kAuboServiceReturnedFailure);
            message = "aubo_driver 返回失败";
            return false;
        }
    }
    catch (const std::exception& e)
    {
        error_code = static_cast<int32_t>(ErrorCode::kInternalException);
        message = std::string("异常: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
        return false;
    }
}

} // namespace demo_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<demo_driver::SetRobotIOServer>();
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node);
        executor.spin();
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
