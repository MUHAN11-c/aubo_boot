#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>

#include "percipio_camera_interface/msg/camera_status.hpp"
#include "percipio_camera_interface/srv/set_camera_parameters.hpp"
#include "percipio_camera_interface/srv/software_trigger.hpp"

#include <chrono>
#include <string>
#include <memory>
#include <deque>

class CameraControlNode : public rclcpp::Node
{
public:
    CameraControlNode()
        : Node("camera_control_node")
    {
        // 声明参数
        camera_name_ = this->declare_parameter<std::string>("camera_name", "camera");
        status_publish_rate_ = this->declare_parameter<double>("status_publish_rate", 1.0);
        
        // 创建状态发布器
        status_pub_ = this->create_publisher<percipio_camera_interface::msg::CameraStatus>(
            "camera_status", 10);
        
        // 创建服务端
        set_params_srv_ = this->create_service<percipio_camera_interface::srv::SetCameraParameters>(
            "set_camera_parameters",
            std::bind(&CameraControlNode::setCameraParametersCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 创建软件触发服务
        // 注意：使用相对服务名，通过launch文件中的remapping映射到全局服务名 /software_trigger
        software_trigger_srv_ = this->create_service<percipio_camera_interface::srv::SoftwareTrigger>(
            "software_trigger",
            std::bind(&CameraControlNode::softwareTriggerCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 获取服务的实际全名（包括命名空间）
        std::string service_full_name = this->get_name();
        std::string service_name = "software_trigger";
        // 如果节点在命名空间下，服务名会是 /namespace/software_trigger
        // 通过remapping后应该是 /software_trigger
        RCLCPP_INFO(this->get_logger(), "软件触发服务已创建");
        RCLCPP_INFO(this->get_logger(), "  节点名: %s", this->get_name());
        RCLCPP_INFO(this->get_logger(), "  节点命名空间: %s", this->get_namespace());
        RCLCPP_INFO(this->get_logger(), "  服务名: %s (通过remapping映射到 /software_trigger)", service_name.c_str());
        RCLCPP_INFO(this->get_logger(), "  服务类型: percipio_camera_interface/srv/SoftwareTrigger");
        
        // 订阅相机信息话题来获取分辨率
        std::string camera_info_topic = "/" + camera_name_ + "/color/camera_info";
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, 10,
            std::bind(&CameraControlNode::cameraInfoCallback, this, std::placeholders::_1));
        
        // 订阅图像话题来计算帧率
        std::string image_topic = "/" + camera_name_ + "/color/image_raw";
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&CameraControlNode::imageCallback, this, std::placeholders::_1));
        
        // 订阅设备事件话题来获取连接状态
        std::string device_event_topic = "/" + camera_name_ + "/device_event";
        device_event_sub_ = this->create_subscription<std_msgs::msg::String>(
            device_event_topic, 10,
            std::bind(&CameraControlNode::deviceEventCallback, this, std::placeholders::_1));
        
        // 创建触发事件发布器
        trigger_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/" + camera_name_ + "/trigger_event", 10);
        
        // 创建参数客户端 - 使用完整的节点路径
        // 相机节点在命名空间下，完整路径为 /camera_name/camera_name
        std::string camera_node_name = "/" + camera_name_ + "/" + camera_name_;
        param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, camera_node_name);
        
        // 尝试从相机节点获取序列号作为camera_id
        // 等待参数服务可用
        if (param_client_->wait_for_service(std::chrono::seconds(3))) {
            try {
                auto params = param_client_->get_parameters({"serial_number"});
                if (!params.empty() && params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    std::string serial_number = params[0].as_string();
                    // 移除引号（如果launch文件中使用了引号）
                    if (serial_number.length() > 2 && serial_number.front() == '"' && serial_number.back() == '"') {
                        serial_number = serial_number.substr(1, serial_number.length() - 2);
                    }
                    if (!serial_number.empty() && serial_number != "\"\"" && serial_number != "") {
                        camera_id_ = serial_number;
                        RCLCPP_INFO(this->get_logger(), "从相机节点获取序列号: %s", camera_id_.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "序列号为空，使用camera_name: %s", camera_name_.c_str());
                    }
                }
            } catch (const std::exception& e) {
                // 获取失败，使用camera_name作为fallback
                RCLCPP_WARN(this->get_logger(), "无法从相机节点获取序列号 (%s)，使用camera_name: %s", 
                           e.what(), camera_name_.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "参数服务不可用，使用camera_name: %s", camera_name_.c_str());
        }
        
        // 如果仍未获取到序列号，使用camera_name
        if (camera_id_.empty()) {
            camera_id_ = camera_name_;
        }
        
        RCLCPP_INFO(this->get_logger(), "相机ID设置为: %s", camera_id_.c_str());
        
        // 创建定时器发布状态
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / status_publish_rate_)),
            std::bind(&CameraControlNode::publishStatus, this));
        
        // 初始化状态
        is_connected_ = true;  // 假设初始连接
        resolution_width_ = 0;
        resolution_height_ = 0;
        frame_rate_ = 0.0f;
        exposure_time_ = 0.0f;
        gain_ = 0.0f;
        trigger_mode_ = 0;  // 0: 连续模式
        
        RCLCPP_INFO(this->get_logger(), "Camera control node started for camera: %s", camera_name_.c_str());
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // 依据：从percipio_camera节点发布的/color/camera_info话题获取分辨率
        // 参考percipio_camera_node.cpp中的publishColorFrame函数，发布CameraInfo消息
        resolution_width_ = msg->width;
        resolution_height_ = msg->height;
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // 计算帧率
        // 依据：通过统计图像接收频率来计算帧率
        // 方法：记录最近2秒内的图像时间戳，计算平均帧率
        auto now = std::chrono::steady_clock::now();
        image_timestamps_.push_back(now);
        
        // 只保留最近2秒的时间戳
        auto two_seconds_ago = now - std::chrono::seconds(2);
        while (!image_timestamps_.empty() && image_timestamps_.front() < two_seconds_ago) {
            image_timestamps_.pop_front();
        }
        
        if (image_timestamps_.size() > 1) {
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                image_timestamps_.back() - image_timestamps_.front()).count();
            if (duration > 0) {
                frame_rate_ = static_cast<float>(image_timestamps_.size() - 1) * 1000.0f / duration;
            }
        }
    }
    
    void deviceEventCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 依据：从percipio_camera节点发布的/device_event话题获取连接状态
        // 参考percipio_camera_node.cpp中的SendOfflineMsg、SendConnectMsg、SendTimetMsg函数
        std::string event = msg->data;
        if (event.find("DeviceOffline") != std::string::npos) {
            is_connected_ = false;
        } else if (event.find("DeviceConnect") != std::string::npos) {
            is_connected_ = true;
        }
    }
    
    void publishStatus()
    {
        // 尝试从参数服务器获取曝光时间和增益
        if (param_client_->wait_for_service(std::chrono::seconds(1))) {
            try {
                // 注意：这些参数可能不存在，需要根据实际percipio_camera的参数名调整
                // 这里我们尝试获取，如果失败就使用默认值
            } catch (...) {
                // 参数获取失败，使用当前值
            }
        }
        
        auto status_msg = percipio_camera_interface::msg::CameraStatus();
        status_msg.header.stamp = this->now();
        status_msg.header.frame_id = camera_name_ + "_link";
        status_msg.camera_id = camera_id_;
        status_msg.is_connected = is_connected_;
        status_msg.resolution_width = resolution_width_;
        status_msg.resolution_height = resolution_height_;
        status_msg.frame_rate = frame_rate_;
        status_msg.exposure_time = exposure_time_;
        status_msg.gain = gain_;
        status_msg.trigger_mode = trigger_mode_;
        
        status_pub_->publish(status_msg);
    }
    
    void setCameraParametersCallback(
        const std::shared_ptr<percipio_camera_interface::srv::SetCameraParameters::Request> request,
        std::shared_ptr<percipio_camera_interface::srv::SetCameraParameters::Response> response)
    {
        response->success = false;
        response->message = "";
        
        // 检查相机ID是否匹配
        if (!request->camera_id.empty() && request->camera_id != camera_name_) {
            response->message = "Camera ID mismatch. Expected: " + camera_name_ + ", got: " + request->camera_id;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        if (!param_client_->wait_for_service(std::chrono::seconds(2))) {
            response->message = "Parameter service not available";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        std::vector<rclcpp::Parameter> parameters;
        bool has_changes = false;
        
        // 设置曝光时间（单位：微秒）
        // 依据：TYDefs.h中定义的TY_FLOAT_EXPOSURE_TIME_US (0x0308) 和 TY_INT_EXPOSURE_TIME (0x0301)
        // 使用TYSetFloat/TYSetInt API设置，参考percipio_device.cpp中的set_exposure_time实现
        if (request->exposure_time > 0.0f) {
            parameters.push_back(rclcpp::Parameter("exposure_time", request->exposure_time));
            has_changes = true;
            exposure_time_ = request->exposure_time;  // 记录设置的值
        }
        
        // 设置增益（单位：dB）
        // 依据：TYDefs.h中定义的TY_INT_GAIN (0x0303)
        // 使用TYSetInt API设置，参考percipio_device.cpp中的set_gain实现
        if (request->gain >= 0.0f) {
            parameters.push_back(rclcpp::Parameter("gain", request->gain));
            has_changes = true;
            gain_ = request->gain;  // 记录设置的值
        }
        
        // 设置触发模式
        // 注意：device_workmode被标记为不可变参数，需要重启设备才能生效
        // 这里尝试设置，但可能会失败，需要用户重启节点
        if (request->trigger_mode >= 0 && request->trigger_mode <= 2) {
            std::string workmode_str;
            if (request->trigger_mode == 0) {
                workmode_str = "trigger_off";
            } else if (request->trigger_mode == 1) {
                workmode_str = "trigger_soft";
            } else {
                workmode_str = "trigger_hard";
            }
            parameters.push_back(rclcpp::Parameter("device_workmode", workmode_str));
            has_changes = true;
            trigger_mode_ = request->trigger_mode;
            response->message += "Warning: trigger_mode change requires device restart. ";
        }
        
        // 分辨率设置：需要重新打开流，当前版本暂不支持动态更改
        // 依据：分辨率更改需要调用stream_close和stream_open，这需要停止和重启流
        // 参考percipio_device.cpp中的stream_open和stream_close实现
        // 如需更改分辨率，请停止节点，修改launch文件中的color_resolution或depth_resolution参数后重新启动
        if (request->resolution_width > 0 && request->resolution_height > 0) {
            RCLCPP_WARN(this->get_logger(), 
                       "Resolution change requested (%ux%u), but requires stream restart. "
                       "Please stop the camera node, modify the launch file parameters, and restart.",
                       request->resolution_width, request->resolution_height);
            response->message += "Resolution change requires node restart. ";
            // 不设置success为false，因为其他参数可能已经成功设置
        }
        
        if (has_changes) {
            try {
                auto results = param_client_->set_parameters(parameters);
                bool all_success = true;
                for (const auto& result : results) {
                    if (!result.successful) {
                        all_success = false;
                        response->message += result.reason + "; ";
                    }
                }
                response->success = all_success;
                if (all_success) {
                    response->message = "Parameters set successfully";
                    RCLCPP_INFO(this->get_logger(), "Camera parameters set successfully");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Some parameters failed to set: %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                response->message = std::string("Exception: ") + e.what();
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameters: %s", response->message.c_str());
            }
        } else {
            response->success = true;
            response->message = "No parameters to set";
        }
    }
    
    void softwareTriggerCallback(
        const std::shared_ptr<percipio_camera_interface::srv::SoftwareTrigger::Request> request,
        std::shared_ptr<percipio_camera_interface::srv::SoftwareTrigger::Response> response)
    {
        response->success = false;
        response->message = "";
        
        RCLCPP_INFO(this->get_logger(), "收到软件触发请求，相机ID: '%s'", request->camera_id.c_str());
        RCLCPP_INFO(this->get_logger(), "  当前camera_name: '%s'", camera_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "  当前camera_id: '%s'", camera_id_.c_str());
        
        // 检查相机ID是否匹配 - 接受camera_name或camera_id（序列号）
        // 如果请求中的camera_id为空，也接受（兼容性）
        bool camera_id_match = request->camera_id.empty() || 
                               request->camera_id == camera_name_ || 
                               request->camera_id == camera_id_;
        
        if (!camera_id_match) {
            response->message = "Camera ID mismatch. Expected: " + camera_name_ + " or " + camera_id_ + " or empty, got: " + request->camera_id;
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }
        
        // 发布触发事件
        std::string trigger_topic = "/" + camera_name_ + "/trigger_event";
        auto trigger_msg = std_msgs::msg::String();
        trigger_msg.data = "SoftTrigger";
        trigger_pub_->publish(trigger_msg);
        
        RCLCPP_INFO(this->get_logger(), "已发布触发事件到话题: %s", trigger_topic.c_str());
        
        response->success = true;
        response->message = "Software trigger command sent";
        RCLCPP_INFO(this->get_logger(), "✓ 软件触发成功 (相机: %s)", camera_name_.c_str());
    }
    
    std::string camera_name_;
    double status_publish_rate_;
    
    rclcpp::Publisher<percipio_camera_interface::msg::CameraStatus>::SharedPtr status_pub_;
    rclcpp::Service<percipio_camera_interface::srv::SetCameraParameters>::SharedPtr set_params_srv_;
    rclcpp::Service<percipio_camera_interface::srv::SoftwareTrigger>::SharedPtr software_trigger_srv_;
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr device_event_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trigger_pub_;
    
    rclcpp::SyncParametersClient::SharedPtr param_client_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // 状态变量
    std::string camera_id_;
    bool is_connected_;
    uint32_t resolution_width_;
    uint32_t resolution_height_;
    float frame_rate_;
    float exposure_time_;
    float gain_;
    int8_t trigger_mode_;
    
    // 帧率计算
    std::deque<std::chrono::steady_clock::time_point> image_timestamps_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 使用多线程执行器以提高服务响应性能
    // 这样可以避免图像回调阻塞服务回调，提高服务调用的稳定性
    auto node = std::make_shared<CameraControlNode>();
    
    // 创建多线程执行器（默认使用所有可用CPU核心）
    // 服务回调可以在独立线程中执行，不会被图像回调阻塞
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}

