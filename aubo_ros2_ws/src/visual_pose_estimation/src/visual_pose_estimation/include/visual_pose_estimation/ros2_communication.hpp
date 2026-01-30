#ifndef VISUAL_POSE_ESTIMATION_ROS2_COMMUNICATION_HPP
#define VISUAL_POSE_ESTIMATION_ROS2_COMMUNICATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <interface/srv/estimate_pose.hpp>
#include <interface/srv/process_debug_step.hpp>
#include <interface/srv/list_templates.hpp>
#include <interface/srv/visualize_grasp_pose.hpp>
#include <interface/srv/standardize_template.hpp>
#include <interface/msg/image_data.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include "visual_pose_estimation/preprocessor.hpp"
#include "visual_pose_estimation/feature_extractor.hpp"
#include "visual_pose_estimation/template_standardizer.hpp"
#include "visual_pose_estimation/config_reader.hpp"
#include "visual_pose_estimation/pose_estimator.hpp"

namespace visual_pose_estimation {

/**
 * @brief ROS2通信和消息转换类
 * 
 * 负责处理ROS2消息的发布、订阅和服务调用，以及消息格式的转换
 */
class ROS2Communication {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     */
    explicit ROS2Communication(rclcpp::Node::SharedPtr node);

    /**
     * @brief 析构函数
     */
    ~ROS2Communication() = default;

    /**
     * @brief 初始化ROS2通信
     * @param config_reader 配置读取器
     * @param template_root 模板根目录
     * @param calib_file 手眼标定文件路径
     * @param debug 是否启用调试模式
     * @return 是否初始化成功
     */
    bool initialize(std::shared_ptr<ConfigReader> config_reader, const std::string& template_root, const std::string& calib_file = "", bool debug = false);

    /**
     * @brief 发布系统状态
     * @param status 状态信息
     */
    void publishSystemStatus(const std::string& status);

    /**
     * @brief 获取图像数据
     * @return 最新的图像数据
     */
    cv::Mat getLatestImage();

    /**
     * @brief 检查是否有新图像
     * @return 是否有新图像
     */
    bool hasNewImage();

private:
    rclcpp::Node::SharedPtr node_;
    
    // 服务
    rclcpp::Service<interface::srv::EstimatePose>::SharedPtr estimate_pose_service_;
    rclcpp::Service<interface::srv::ProcessDebugStep>::SharedPtr debug_step_service_;
    rclcpp::Service<interface::srv::ListTemplates>::SharedPtr list_templates_service_;
    rclcpp::Service<interface::srv::VisualizeGraspPose>::SharedPtr visualize_grasp_pose_service_;
    rclcpp::Service<interface::srv::StandardizeTemplate>::SharedPtr standardize_template_service_;
    
    // 发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    
    // 订阅者
    rclcpp::Subscription<interface::msg::ImageData>::SharedPtr image_subscriber_;
    
    // 状态变量
    cv::Mat latest_image_;
    bool has_new_image_;
    std::mutex image_mutex_;
    
    // 算法组件
    std::shared_ptr<Preprocessor> preprocessor_;
    std::shared_ptr<FeatureExtractor> feature_extractor_;
    std::shared_ptr<TemplateStandardizer> template_standardizer_;
    std::shared_ptr<ConfigReader> config_reader_;
    std::string template_root_;
    std::string calib_file_;
    bool debug_;
    
    // 暴力匹配参数
    bool brute_force_matching_enabled_;
    double brute_force_angle_step_deg_;
    int brute_force_max_threads_;
    double brute_force_rejection_threshold_;   // 舍弃阈值
    double brute_force_acceptance_threshold_;  // 通过阈值
    double brute_force_angle_matching_scale_;  // 角度匹配时缩小尺寸的比例
    
    // 辅助函数
    bool loadHandEyeCalibration(
        const std::string& calibration_path,
        cv::Mat& camera_matrix,
        cv::Mat& dist_coeffs,
        cv::Mat& T_E_C);
    
    bool loadPoseJSON(
        const std::string& pose_path,
        cv::Mat& T_B_E);
    
    /**
     * @brief 从JSON文件加载CartesianPosition（包含关节坐标）
     * @param json_path JSON文件路径
     * @param cartesian_pos 输出的CartesianPosition消息
     * @return 是否成功加载
     */
    bool loadCartesianPositionJSON(
        const std::string& json_path,
        interface::msg::CartesianPosition& cartesian_pos);
    
    cv::Vec4d rotationMatrixToQuaternion(const cv::Mat& R);
    
    /**
     * @brief 从旋转矩阵计算欧拉角（RPY，ZYX顺序）
     * @param R 旋转矩阵（3x3）
     * @return 欧拉角向量 [roll, pitch, yaw]（弧度）
     */
    cv::Vec3d rotationMatrixToEulerRPY(const cv::Mat& R);
    
    /**
     * @brief 在图像上绘制手抓可视化
     * @param image 输入输出图像
     * @param T_C_E 相机坐标系到末端执行器坐标系的变换矩阵
     * @param camera_matrix 相机内参矩阵
     * @param dist_coeffs 畸变系数
     * @param gripper_opening_mm 夹爪开口大小（毫米）
     * @param gripper_length_mm 夹爪长度（毫米）
     */
    void drawGripperOnImage(
        cv::Mat& image,
        const cv::Mat& T_C_E,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        double gripper_opening_mm = 50.0,
        double gripper_length_mm = 100.0
    );
    
    /**
     * @brief 计算两个掩膜的重合度（IoU - Intersection over Union）
     * @param mask1 第一个掩膜（二值图像，0或255）
     * @param mask2 第二个掩膜（二值图像，0或255）
     * @return 重合度值（0.0-1.0），1.0表示完全重合
     */
    double calculateMaskIoU(const cv::Mat& mask1, const cv::Mat& mask2);
    
    /**
     * @brief 通过旋转搜索找到最佳掩膜对齐角度和重合度
     * @param template_mask 模板掩膜（标准化后的掩膜，工件中心在掩膜中心）
     * @param target_mask 待估计工件的掩膜（输入图像坐标系）
     * @param target_center 待估计工件中心在输入图像中的位置
     * @param scale 缩放比例（基于外接圆半径）
     * @param initial_angle_deg 初始角度（度），用于确定搜索范围
     * @param angle_range_deg 搜索角度范围（度），在初始角度两侧各搜索此范围
     * @param angle_step_deg 角度搜索步长（度）
     * @param best_angle_deg 输出的最佳角度（度）
     * @param best_confidence 输出的最佳重合度（置信度）
     * @param best_aligned_mask 输出的最佳对齐后的掩膜
     * @return 是否成功找到最佳匹配
     */
    bool findBestMaskAlignment(
        const cv::Mat& template_mask,
        const cv::Mat& target_mask,
        const cv::Point2f& target_center,
        double scale,
        double initial_angle_deg,
        double angle_range_deg,
        double angle_step_deg,
        double& best_angle_deg,
        double& best_confidence,
        cv::Mat& best_aligned_mask
    );
    
    /**
     * @brief 暴力匹配：遍历所有模板，找到最佳匹配
     * @param templates 模板列表（前向声明，实际类型在cpp中定义）
     * @param workpiece_template_dir 工件模板目录
     * @param target_mask 待匹配工件掩膜
     * @param target_center 待匹配工件中心
     * @param target_standardized_angle_rad 目标工件的标准化角度（弧度），从工件中心到阀体中心的角度
     * @param best_template_idx 输出的最佳模板索引
     * @param best_angle_deg 输出的最佳角度（度）
     * @param best_confidence 输出的最佳重合度
     * @param best_aligned_mask 输出的最佳对齐后的掩膜
     * @return 是否成功找到匹配
     * 
     * 注意：此函数在cpp文件中实现，使用局部定义的TemplateItem结构
     * 模板掩膜已经标准化，阀体在正右侧（角度0）
     * 目标工件的阀体在target_standardized_angle_rad位置
     * 匹配时需要考虑这个角度差异
     */
    bool bruteForceTemplateMatching(
        const void* templates_ptr,  // 使用void*避免前向声明问题，实际类型为std::vector<TemplateItem>*
        const std::string& workpiece_template_dir,
        const cv::Mat& target_mask,
        const cv::Point2f& target_center,
        double target_standardized_angle_rad,
        double target_radius,  // 目标工件的外接圆半径
        const std::filesystem::path& debug_dir,  // debug文件夹路径
        int& best_template_idx,
        double& best_angle_deg,
        double& best_confidence,
        cv::Mat& best_aligned_mask
    );
    
    // 服务回调函数
    void handleEstimatePose(
        const std::shared_ptr<interface::srv::EstimatePose::Request> request,
        std::shared_ptr<interface::srv::EstimatePose::Response> response);
    
    void handleProcessDebugStep(
        const std::shared_ptr<interface::srv::ProcessDebugStep::Request> request,
        std::shared_ptr<interface::srv::ProcessDebugStep::Response> response);
    
    void handleListTemplates(
        const std::shared_ptr<interface::srv::ListTemplates::Request> request,
        std::shared_ptr<interface::srv::ListTemplates::Response> response);
    
    void handleVisualizeGraspPose(
        const std::shared_ptr<interface::srv::VisualizeGraspPose::Request> request,
        std::shared_ptr<interface::srv::VisualizeGraspPose::Response> response);
    
    void handleStandardizeTemplate(
        const std::shared_ptr<interface::srv::StandardizeTemplate::Request> request,
        std::shared_ptr<interface::srv::StandardizeTemplate::Response> response);
    
    // 订阅回调函数
    void imageCallback(const interface::msg::ImageData::SharedPtr msg);
};

} // namespace visual_pose_estimation

#endif // VISUAL_POSE_ESTIMATION_ROS2_COMMUNICATION_HPP

