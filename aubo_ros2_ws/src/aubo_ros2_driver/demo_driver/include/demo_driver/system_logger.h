#pragma once

#include <chrono>
#include <string>

#include <ivg_interfaces/msg/system_log.hpp>
#include <rclcpp/rclcpp.hpp>

/** 结构化日志发布 mixin
 *
 * 包装 RCLCPP_* 的同时发布到 /system/log 话题。
 * 用法: 在 Node 派生类中组合使用，调用 logInfo/logWarn/logError 即可。
 *
 * 不改变现有 RCLCPP_* 调用，仅作为并行发布通道。
 */
class SystemLogger
{
public:
    explicit SystemLogger(rclcpp::Node* node)
        : node_(node)
    {
        log_pub_ = node_->create_publisher<ivg_interfaces::msg::SystemLog>(
            "/system/log", 10);
    }

    void logInfo(const std::string& message, const std::string& file = "", int line = 0)
    {
        publish(ivg_interfaces::msg::SystemLog::LEVEL_INFO, message, file, line);
    }

    void logWarn(const std::string& message, const std::string& file = "", int line = 0)
    {
        publish(ivg_interfaces::msg::SystemLog::LEVEL_WARN, message, file, line);
    }

    void logError(const std::string& message, const std::string& file = "", int line = 0)
    {
        publish(ivg_interfaces::msg::SystemLog::LEVEL_ERROR, message, file, line);
    }

    void logDebug(const std::string& message, const std::string& file = "", int line = 0)
    {
        publish(ivg_interfaces::msg::SystemLog::LEVEL_DEBUG, message, file, line);
    }

private:
    void publish(uint8_t level, const std::string& msg, const std::string& file, int line)
    {
        auto log = ivg_interfaces::msg::SystemLog();
        log.timestamp = node_->now();
        log.level = level;
        log.node_name = std::string(node_->get_name());
        log.message = msg;
        log.file = file;
        log.line = line;
        log_pub_->publish(log);
    }

    rclcpp::Node* node_;
    rclcpp::Publisher<ivg_interfaces::msg::SystemLog>::SharedPtr log_pub_;
};
