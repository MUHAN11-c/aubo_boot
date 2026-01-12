/*
 * 使用 MoveIt2 直接计算当前机械臂位姿
 * 独立节点，不依赖服务调用，直接使用 MoveIt2 API
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class GetCurrentPoseMoveIt2 : public rclcpp::Node
{
public:
    GetCurrentPoseMoveIt2() : Node("get_current_pose_moveit2")
    {
        // 获取参数（默认规划组名称是 manipulator，根据 SRDF 文件）
        this->declare_parameter("planning_group_name", std::string("manipulator"));
        this->declare_parameter("base_frame", std::string("base_link"));
        
        this->get_parameter("planning_group_name", planning_group_name_);
        this->get_parameter("base_frame", base_frame_);
        
        RCLCPP_INFO(this->get_logger(), "节点已创建，准备初始化 MoveIt2 接口...");
        RCLCPP_INFO(this->get_logger(), "规划组名称: %s", planning_group_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "基础坐标系: %s", base_frame_.c_str());
    }

    bool wait_for_robot_description(int timeout_seconds = 30)
    {
        RCLCPP_INFO(this->get_logger(), "等待机器人描述参数和 MoveIt2 服务可用...");
        
        auto start_time = std::chrono::steady_clock::now();
        int check_count = 0;
        
        // 尝试从其他节点获取 robot_description 参数
        // MoveGroupInterface 需要参数在节点命名空间下，所以我们需要从其他节点复制过来
        std::vector<std::string> source_nodes = {"/move_group", "/robot_state_publisher"};
        bool param_copied = false;
        
        for (const auto& source_node : source_nodes) {
            try {
                auto remote_param_client = std::make_shared<rclcpp::SyncParametersClient>(
                    shared_from_this(), source_node);
                
                if (remote_param_client->wait_for_service(std::chrono::seconds(2))) {
                    std::vector<rclcpp::Parameter> params = remote_param_client->get_parameters(
                        {"robot_description", "robot_description_semantic"});
                    
                    if (!params.empty()) {
                        // 将参数设置到当前节点
                        for (const auto& param : params) {
                            this->declare_parameter(param.get_name(), param.get_parameter_value());
                            this->set_parameter(param);
                        }
                        param_copied = true;
                        RCLCPP_INFO(this->get_logger(), "成功从 %s 节点复制 robot_description 参数", source_node.c_str());
                        break;
                    }
                }
            } catch (const std::exception& e) {
                // 继续尝试下一个节点
            }
        }
        
        if (!param_copied) {
            RCLCPP_WARN(this->get_logger(), "无法从其他节点获取 robot_description 参数，将尝试直接使用 MoveGroupInterface");
        }
        
        while (rclcpp::ok())
        {
            // 尝试直接初始化 MoveGroupInterface 来检查参数是否可用
            // 如果参数不存在或服务未准备好，这会抛出异常
            try
            {
                // 创建一个临时的 MoveGroupInterface 来测试参数是否可用
                auto test_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), planning_group_name_);
                
                // 尝试获取规划框架来验证是否完全初始化
                std::string test_frame = test_move_group->getPlanningFrame();
                
                if (!test_frame.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "机器人描述参数和 MoveIt2 服务已就绪");
                    RCLCPP_INFO(this->get_logger(), "规划框架: %s", test_frame.c_str());
                    return true;
                }
            }
            catch (const std::exception& e)
            {
                // 参数或服务还未准备好，继续等待
                check_count++;
                if (check_count % 10 == 0)  // 每5秒打印一次
                {
                    auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                        std::chrono::steady_clock::now() - start_time).count();
                    RCLCPP_INFO(this->get_logger(), "仍在等待... (已等待 %ld 秒, 错误: %s)",
                               elapsed_seconds, e.what());
                }
            }
            
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() >= timeout_seconds)
            {
                RCLCPP_ERROR(this->get_logger(), "等待超时 (%d 秒)", timeout_seconds);
                RCLCPP_ERROR(this->get_logger(), "请确保:");
                RCLCPP_ERROR(this->get_logger(), "  1. MoveIt2 已启动: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py");
                RCLCPP_ERROR(this->get_logger(), "  2. 规划组名称正确: 当前使用 '%s'", planning_group_name_.c_str());
                RCLCPP_ERROR(this->get_logger(), "  3. move_group 节点正在运行");
                return false;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        return false;
    }

    bool initialize(int max_retries = 3, int retry_delay_seconds = 2)
    {
        RCLCPP_INFO(this->get_logger(), "初始化 MoveIt2 接口...");
        
        // 等待机器人描述参数
        if (!wait_for_robot_description(30))
        {
            return false;
        }
        
        // 等待一段时间，确保参数完全加载
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // 尝试初始化，带重试机制
        for (int attempt = 1; attempt <= max_retries; ++attempt)
        {
            try
            {
                RCLCPP_INFO(this->get_logger(), "尝试初始化 MoveIt2 (第 %d/%d 次)...", attempt, max_retries);
                
                // 初始化 MoveIt2 接口（在对象创建后调用，此时可以使用 shared_from_this()）
                move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), planning_group_name_);
                planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
                
                // 设置参考坐标系
                move_group_->setPoseReferenceFrame(base_frame_);
                
                // 获取末端执行器链接名称
                end_effector_link_ = move_group_->getEndEffectorLink();
                RCLCPP_INFO(this->get_logger(), "末端执行器链接: %s", end_effector_link_.c_str());
                
                // 获取规划框架
                std::string planning_frame = move_group_->getPlanningFrame();
                RCLCPP_INFO(this->get_logger(), "规划框架: %s", planning_frame.c_str());
                
                RCLCPP_INFO(this->get_logger(), "MoveIt2 接口初始化成功");
                return true;
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "MoveIt2 接口初始化失败 (第 %d/%d 次): %s", 
                           attempt, max_retries, e.what());
                
                if (attempt < max_retries)
                {
                    RCLCPP_INFO(this->get_logger(), "等待 %d 秒后重试...", retry_delay_seconds);
                    std::this_thread::sleep_for(std::chrono::seconds(retry_delay_seconds));
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "MoveIt2 接口初始化最终失败");
                    RCLCPP_ERROR(this->get_logger(), "请检查:");
                    RCLCPP_ERROR(this->get_logger(), "  1. MoveIt2 是否已启动: ros2 launch aubo_moveit_config aubo_moveit_bridge_ros1.launch.py");
                    RCLCPP_ERROR(this->get_logger(), "  2. 规划组名称是否正确: 当前使用 '%s' (应该是 'manipulator')", 
                               planning_group_name_.c_str());
                    RCLCPP_ERROR(this->get_logger(), "  3. move_group 节点是否在运行: ros2 node list | grep move_group");
                    RCLCPP_ERROR(this->get_logger(), "  4. 机器人描述参数是否存在: ros2 param list | grep robot_description");
                    return false;
                }
            }
        }
        
        return false;
    }

    bool get_current_pose(geometry_msgs::msg::Pose& pose)
    {
        if (!move_group_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveIt2 接口未初始化");
            return false;
        }

        try
        {
            // 使用 MoveIt2 获取当前位姿（相对于规划框架，通常是 world）
            geometry_msgs::msg::PoseStamped current_pose_stamped = 
                move_group_->getCurrentPose(end_effector_link_);
            
            std::string source_frame = current_pose_stamped.header.frame_id;
            RCLCPP_INFO(this->get_logger(), "成功获取当前位姿 (原始坐标系: %s)", source_frame.c_str());
            
            // 如果原始坐标系不是 base_link，需要转换
            if (source_frame != base_frame_)
            {
                // 创建 TF buffer 和 listener（如果还没有创建）
                if (!tf_buffer_)
                {
                    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
                    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
                }
                
                // 等待 TF 变换可用
                if (tf_buffer_->canTransform(base_frame_, source_frame, rclcpp::Time(0), std::chrono::seconds(2)))
                {
                    // 转换位姿到 base_link 坐标系
                    geometry_msgs::msg::PoseStamped pose_in_base;
                    try
                    {
                        pose_in_base = tf_buffer_->transform(current_pose_stamped, base_frame_);
                        pose = pose_in_base.pose;
                        RCLCPP_INFO(this->get_logger(), "位姿已从 %s 转换到 %s 坐标系", 
                                   source_frame.c_str(), base_frame_.c_str());
                    }
                    catch (const tf2::TransformException& ex)
                    {
                        RCLCPP_WARN(this->get_logger(), "TF 转换失败: %s，使用原始位姿", ex.what());
                        pose = current_pose_stamped.pose;
                    }
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "无法获取从 %s 到 %s 的 TF 变换，使用原始位姿", 
                               source_frame.c_str(), base_frame_.c_str());
                    pose = current_pose_stamped.pose;
                }
            }
            else
            {
                pose = current_pose_stamped.pose;
            }
            
            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "获取位姿失败: %s", e.what());
            return false;
        }
    }

    bool get_current_joint_positions(std::vector<double>& joint_positions)
    {
        if (!move_group_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveIt2 接口未初始化");
            return false;
        }

        try
        {
            // 获取当前关节状态
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();
            const moveit::core::JointModelGroup* joint_model_group = 
                current_state->getJointModelGroup(move_group_->getName());
            
            if (!joint_model_group)
            {
                RCLCPP_ERROR(this->get_logger(), "无法获取关节模型组");
                return false;
            }
            
            // 复制关节位置
            current_state->copyJointGroupPositions(joint_model_group, joint_positions);
            
            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "获取关节位置失败: %s", e.what());
            return false;
        }
    }

    void print_pose(const geometry_msgs::msg::Pose& pose)
    {
        const auto& pos = pose.position;
        const auto& ori = pose.orientation;
        
        RCLCPP_INFO(this->get_logger(), "\n=== 当前机械臂位姿 (MoveIt2 计算) ===");
        RCLCPP_INFO(this->get_logger(), "位置 (米):");
        RCLCPP_INFO(this->get_logger(), "  x = %.6f", pos.x);
        RCLCPP_INFO(this->get_logger(), "  y = %.6f", pos.y);
        RCLCPP_INFO(this->get_logger(), "  z = %.6f", pos.z);
        
        RCLCPP_INFO(this->get_logger(), "\n姿态 (四元数):");
        RCLCPP_INFO(this->get_logger(), "  w = %.6f", ori.w);
        RCLCPP_INFO(this->get_logger(), "  x = %.6f", ori.x);
        RCLCPP_INFO(this->get_logger(), "  y = %.6f", ori.y);
        RCLCPP_INFO(this->get_logger(), "  z = %.6f", ori.z);
        
        // 计算并显示欧拉角 (ZYX顺序，即 roll-pitch-yaw)
        double roll, pitch, yaw;
        quaternion_to_euler(ori.x, ori.y, ori.z, ori.w, roll, pitch, yaw);
        RCLCPP_INFO(this->get_logger(), "\n姿态 (欧拉角，度，ZYX顺序):");
        RCLCPP_INFO(this->get_logger(), "  roll  (绕x轴) = %.2f°", roll * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  pitch (绕y轴) = %.2f°", pitch * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  yaw   (绕z轴) = %.2f°", yaw * 180.0 / M_PI);
        
        // 也尝试 XYZ 顺序（可能示教器使用这个顺序）
        double rx_xyz, ry_xyz, rz_xyz;
        quaternion_to_euler_xyz(ori.x, ori.y, ori.z, ori.w, rx_xyz, ry_xyz, rz_xyz);
        RCLCPP_INFO(this->get_logger(), "\n姿态 (欧拉角，度，XYZ顺序，可能用于示教器):");
        RCLCPP_INFO(this->get_logger(), "  RX (绕x轴) = %.2f°", rx_xyz * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  RY (绕y轴) = %.2f°", ry_xyz * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  RZ (绕z轴) = %.2f°", rz_xyz * 180.0 / M_PI);
        
        RCLCPP_INFO(this->get_logger(), "==========================================\n");
    }

    void print_joint_positions(const std::vector<double>& joint_positions)
    {
        std::vector<std::string> joint_names = {
            "shoulder_joint", "upperArm_joint", "foreArm_joint",
            "wrist1_joint", "wrist2_joint", "wrist3_joint"
        };
        
        RCLCPP_INFO(this->get_logger(), "\n=== 当前关节位置 ===");
        RCLCPP_INFO(this->get_logger(), "关节位置 (弧度):");
        for (size_t i = 0; i < joint_positions.size() && i < joint_names.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(), "  %s = %.6f rad (%.2f°)", 
                       joint_names[i].c_str(), 
                       joint_positions[i],
                       joint_positions[i] * 180.0 / M_PI);
        }
        RCLCPP_INFO(this->get_logger(), "===================\n");
    }

private:
    void quaternion_to_euler(double x, double y, double z, double w, 
                             double& roll, double& pitch, double& yaw)
    {
        // 四元数转欧拉角 (ZYX顺序，即 roll-pitch-yaw)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    
    void quaternion_to_euler_xyz(double x, double y, double z, double w,
                                  double& rx, double& ry, double& rz)
    {
        // 四元数转欧拉角 (XYZ顺序，固定轴旋转)
        // 这通常用于示教器显示
        double sinr = 2 * (w * x + y * z);
        double cosr = 1 - 2 * (x * x + y * y);
        rx = std::atan2(sinr, cosr);
        
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            ry = std::copysign(M_PI / 2, sinp);
        else
            ry = std::asin(sinp);
        
        double siny = 2 * (w * z + x * y);
        double cosy = 1 - 2 * (y * y + z * z);
        rz = std::atan2(siny, cosy);
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string end_effector_link_;
    std::string planning_group_name_;
    std::string base_frame_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        // 创建节点（此时对象已被 shared_ptr 管理）
        auto node = std::make_shared<GetCurrentPoseMoveIt2>();
        
        // 先初始化 MoveIt2 接口（此时节点还未添加到 executor，可以创建参数客户端）
        // 初始化 MoveIt2 接口（此时可以安全使用 shared_from_this()）
        if (!node->initialize())
        {
            RCLCPP_ERROR(node->get_logger(), "MoveIt2 初始化失败");
            rclcpp::shutdown();
            return 1;
        }
        
        // 现在将节点添加到 executor（参数已获取，MoveIt2 已初始化）
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        
        // 在后台线程中持续运行 executor
        std::atomic<bool> executor_running(true);
        std::thread executor_thread([&executor, &executor_running]() {
            while (executor_running && rclcpp::ok()) {
                executor.spin_some(std::chrono::milliseconds(100));
            }
        });
        
        // 等待 MoveIt2 完全初始化
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        // 获取当前位姿
        geometry_msgs::msg::Pose current_pose;
        if (node->get_current_pose(current_pose))
        {
            node->print_pose(current_pose);
        }
        else
        {
            executor_running = false;
            executor_thread.join();
            RCLCPP_ERROR(node->get_logger(), "获取位姿失败");
            rclcpp::shutdown();
            return 1;
        }
        
        // 获取当前关节位置
        std::vector<double> joint_positions;
        if (node->get_current_joint_positions(joint_positions))
        {
            node->print_joint_positions(joint_positions);
        }
        
        executor_running = false;
        executor_thread.join();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("get_current_pose_moveit2"), 
                    "程序异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}

