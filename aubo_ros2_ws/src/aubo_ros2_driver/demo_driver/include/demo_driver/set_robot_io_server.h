/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024
 * All rights reserved.
 */

#ifndef DEMO_DRIVER_SET_ROBOT_IO_SERVER_H_
#define DEMO_DRIVER_SET_ROBOT_IO_SERVER_H_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <demo_interface/srv/set_robot_io.hpp>
#include <aubo_msgs/srv/set_io.hpp>
#include <string>
#include <memory>
#include <cstdint>
#include <chrono>

namespace demo_driver
{

/**
 * @brief 设置机器人IO服务服务器类
 * 提供设置机器人IO状态的服务，支持数字输入/输出、模拟输入/输出和工具IO
 */
class SetRobotIOServer : public rclcpp::Node
{
public:
    SetRobotIOServer();  // 构造函数
    ~SetRobotIOServer(); // 析构函数

    /**
     * @brief SetRobotIO 服务统一错误码定义
     *
     * 约定：
     * - 0 表示成功；
     * - 负数表示失败；
     * - -1~-99 为业务校验/流程错误；
     * - <= -100 为底层服务状态或异常错误。
     *
     * 注意：srv 响应字段仍为 int32，本枚举用于提升可读性，赋值时请使用 static_cast<int32_t>(ErrorCode::Xxx)。
     */
    enum class ErrorCode : int32_t
    {
        kSuccess = 0,                  // 成功
        kInvalidIoIndex = -1,          // io_index 非法（必须 >= 0）
        kInputTypeNotWritable = -2,    // 输入类型不可写（digital_input/analog_input）
        kInvalidIoType = -3,           // io_type 不在支持列表
        kFailedToResolveIoType = -4,   // io_type 解析异常（理论上不应出现）
        kAuboServiceReturnedFailure = -5, // 下游 /aubo_driver/set_io 返回 success=false
        kAuboServiceCallTimeout = -6,  // 调用下游服务超时
        kIoIndexOutOfRange = -7,       // io_index 超出 pin 有效范围（0~127，对应 int8_t）
        kAuboServiceNotReady = -100,   // 下游服务不可用/未就绪
        kInternalException = -200      // 本节点内部异常（捕获到 std::exception）
    };

private:
    // 服务客户端
    rclcpp::Client<aubo_msgs::srv::SetIO>::SharedPtr aubo_set_io_client_;

    // 服务服务器
    rclcpp::Service<demo_interface::srv::SetRobotIO>::SharedPtr set_robot_io_service_;

    // 回调组：服务回调独立，避免阻塞其他回调
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // 服务回调函数
    void setRobotIOCallback(
        const std::shared_ptr<demo_interface::srv::SetRobotIO::Request> req,
        std::shared_ptr<demo_interface::srv::SetRobotIO::Response> res);

    // 辅助函数
    bool setRobotIO(const std::string& io_type,
                   int32_t io_index,
                   double value,
                   int32_t& error_code,
                   std::string& message);

    // 参数
    std::string aubo_set_io_service_name_;
    std::chrono::seconds service_wait_timeout_;
    std::chrono::seconds call_timeout_;
};

} // namespace demo_driver

#endif // DEMO_DRIVER_SET_ROBOT_IO_SERVER_H_
