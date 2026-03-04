// 视觉姿态估计节点主程序
// 负责初始化ROS2节点，加载配置，并启动视觉姿态估计服务

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <filesystem>
#include <unistd.h>
#include <chrono>
#include "visual_pose_estimation/ros2_communication.hpp"
#include "visual_pose_estimation/config_reader.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace visual_pose_estimation;

// 视觉姿态估计ROS2节点类
class VisualPoseEstimationNode : public rclcpp::Node {
public:
    // 构造函数：初始化节点并声明参数
    VisualPoseEstimationNode() : Node("visual_pose_estimation") {
        // 声明参数
        this->declare_parameter<std::string>("config_file", "");
        this->declare_parameter<std::string>("calib_file", "");
        this->declare_parameter<std::string>("template_root", "");
        this->declare_parameter<bool>("debug", false);
        
        // 获取参数
        std::string config_file;
        std::string calib_file;
        std::string template_root;
        bool debug = false;
        
        try {
            if (this->has_parameter("config_file")) {
                config_file = this->get_parameter("config_file").as_string();
            }
            if (this->has_parameter("calib_file")) {
                calib_file = this->get_parameter("calib_file").as_string();
            }
            if (this->has_parameter("template_root")) {
                template_root = this->get_parameter("template_root").as_string();
            }
            if (this->has_parameter("debug")) {
                debug = this->get_parameter("debug").as_bool();
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "参数获取失败，使用默认值: %s", e.what());
        }
        
        // 如果参数为空，使用默认路径
        if (config_file.empty()) {
            try {
                std::string pkg_share = ament_index_cpp::get_package_share_directory("visual_pose_estimation");
                config_file = pkg_share + "/configs/default.yaml";
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "无法获取包路径: %s", e.what());
            }
        }
        
        if (template_root.empty()) {
            template_root = "/home/nvidia/RVG_ws/templates";
        }
        
        // 保存参数供延迟初始化使用
        config_file_ = config_file;
        template_root_ = template_root;
        calib_file_ = calib_file;
        debug_ = debug;
        
        // 使用定时器延迟初始化，确保对象已被shared_ptr管理
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                this->delayedInitialize();
                init_timer_->cancel();  // 只执行一次
            });
    }
    
    // 延迟初始化函数：在节点完全创建后执行初始化
    // 使用定时器延迟执行，确保shared_from_this()可以安全使用
    void delayedInitialize() {
        // 创建配置读取器
        auto config_reader = std::make_shared<ConfigReader>();
        if (!config_file_.empty() && std::filesystem::exists(config_file_)) {
            if (config_reader->loadFromFile(config_file_)) {
                RCLCPP_INFO(this->get_logger(), "配置文件加载成功: %s", config_file_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "配置文件加载失败: %s", config_file_.c_str());
            }
        }
        
        // 创建ROS2通信对象（现在可以安全使用shared_from_this()）
        ros2_communication_ = std::make_shared<ROS2Communication>(shared_from_this());
        
        // 初始化ROS2通信
        if (!ros2_communication_->initialize(config_reader, template_root_, calib_file_, debug_)) {
            RCLCPP_ERROR(this->get_logger(), "ROS2通信初始化失败");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "视觉姿态估计节点启动成功");
        RCLCPP_INFO(this->get_logger(), "配置文件: %s", config_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "标定文件: %s", calib_file_.empty() ? "(未指定，将在工件目录下查找)" : calib_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "模板根目录: %s", template_root_.c_str());
        RCLCPP_INFO(this->get_logger(), "调试模式: %s", debug_ ? "开启" : "关闭");
    }
    
    // 析构函数：节点关闭时调用
    ~VisualPoseEstimationNode() {
        RCLCPP_INFO(this->get_logger(), "视觉姿态估计节点关闭");
    }

private:
    std::shared_ptr<ROS2Communication> ros2_communication_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::string config_file_;
    std::string template_root_;
    std::string calib_file_;
    bool debug_;
};

// 主函数：程序入口
// @param argc 命令行参数个数
// @param argv 命令行参数数组
// @return 程序退出码
int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点实例
    auto node = std::make_shared<VisualPoseEstimationNode>();
    
    RCLCPP_INFO(node->get_logger(), "开始运行视觉姿态估计节点...");
    
    // 运行节点，阻塞直到节点关闭
    rclcpp::spin(node);
    
    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}

