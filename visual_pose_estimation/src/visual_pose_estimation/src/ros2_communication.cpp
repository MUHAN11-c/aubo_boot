#include "visual_pose_estimation/ros2_communication.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <mutex>
#include <exception>
#include <fstream>
#include <sstream>
#include <jsoncpp/json/json.h>
#include <filesystem>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <thread>
#include <algorithm>
#include <atomic>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace visual_pose_estimation {

ROS2Communication::ROS2Communication(rclcpp::Node::SharedPtr node)
    : node_(node), has_new_image_(false) {
}

bool ROS2Communication::initialize(std::shared_ptr<ConfigReader> config_reader, const std::string& template_root, const std::string& calib_file, bool debug) {
    config_reader_ = config_reader;
    template_root_ = template_root;
    calib_file_ = calib_file;
    debug_ = debug;
    
    // 创建算法组件
    preprocessor_ = std::make_shared<Preprocessor>();
    feature_extractor_ = std::make_shared<FeatureExtractor>();
    template_standardizer_ = std::make_shared<TemplateStandardizer>(preprocessor_, feature_extractor_);
    
    // 从配置文件加载参数
    if (config_reader_) {
        // 加载预处理参数
        std::map<std::string, double> preprocess_params;
        preprocess_params["scale_factor"] = config_reader_->getDouble("preprocess.scale_factor", 1.0);
        // 加载背景去除参数
        preprocess_params["bg_border_ratio"] = config_reader_->getDouble("preprocess.background.border_ratio", 0.08);
        preprocess_params["bg_hue_margin"] = config_reader_->getDouble("preprocess.background.hue_margin", 12.0);
        preprocess_params["bg_hue_std_mul"] = config_reader_->getDouble("preprocess.background.hue_std_mul", 3.0);
        preprocess_params["bg_sat_margin"] = config_reader_->getDouble("preprocess.background.sat_margin", 25.0);
        preprocess_params["bg_sat_std_mul"] = config_reader_->getDouble("preprocess.background.sat_std_mul", 2.0);
        preprocess_params["bg_val_margin"] = config_reader_->getDouble("preprocess.background.val_margin", 35.0);
        preprocess_params["bg_val_std_mul"] = config_reader_->getDouble("preprocess.background.val_std_mul", 2.0);
        preprocess_params["bg_lab_threshold"] = config_reader_->getDouble("preprocess.background.lab_threshold", 3.5);
        preprocess_params["bg_cleanup_kernel"] = config_reader_->getDouble("preprocess.background.cleanup_kernel", 7.0);
        preprocess_params["bg_foreground_close_kernel"] = config_reader_->getDouble("preprocess.background.foreground_close_kernel", 9.0);
        preprocess_params["bg_median_ksize"] = config_reader_->getDouble("preprocess.background.median_ksize", 5.0);
        preprocess_params["bg_enable_classic_hsv"] = config_reader_->getBool("preprocess.background.enable_classic_hsv", true) ? 1.0 : 0.0;
        preprocess_params["bg_use_histogram"] = config_reader_->getBool("preprocess.background.use_histogram", true) ? 1.0 : 0.0;
        preprocess_params["bg_min_noise_area"] = config_reader_->getDouble("preprocess.background.min_noise_area", 100.0);
        preprocess_params["bg_erode_before_dilate"] = config_reader_->getBool("preprocess.background.erode_before_dilate", false) ? 1.0 : 0.0;
        preprocess_params["bg_border_bg_ratio_threshold"] = config_reader_->getDouble("preprocess.background.border_bg_ratio_threshold", 0.7);
        preprocess_params["bg_component_min_area"] = config_reader_->getDouble("preprocess.background.component_min_area", 1500.0);
        preprocess_params["bg_component_max_area"] = config_reader_->getDouble("preprocess.background.component_max_area", 15000000.0);
        preprocess_params["bg_component_min_aspect_ratio"] = config_reader_->getDouble("preprocess.background.component_min_aspect_ratio", 0.2);
        preprocess_params["bg_component_max_aspect_ratio"] = config_reader_->getDouble("preprocess.background.component_max_aspect_ratio", 5.0);
        preprocess_params["bg_component_min_width"] = config_reader_->getDouble("preprocess.background.component_min_width", 40.0);
        preprocess_params["bg_component_min_height"] = config_reader_->getDouble("preprocess.background.component_min_height", 40.0);
        preprocess_params["bg_component_max_count"] = config_reader_->getDouble("preprocess.background.component_max_count", 3.0);
        preprocessor_->setParameters(preprocess_params);
        
        // 加载特征提取参数
        std::map<std::string, double> feature_params;
        feature_params["min_component_area"] = config_reader_->getDouble("preprocess.feature_extraction.min_component_area", 2000.0);
        feature_params["max_component_area"] = config_reader_->getDouble("preprocess.feature_extraction.max_component_area", 10000000.0);
        feature_params["big_circle.combine_contours"] = config_reader_->getBool("preprocess.feature_extraction.big_circle.combine_contours", true) ? 1.0 : 0.0;
        feature_params["big_circle.min_area"] = config_reader_->getDouble("preprocess.feature_extraction.big_circle.min_area", 100.0);
        feature_params["small_circle.erode_kernel"] = config_reader_->getDouble("preprocess.feature_extraction.small_circle.erode_kernel", 11.0);
        feature_params["small_circle.erode_iterations"] = config_reader_->getDouble("preprocess.feature_extraction.small_circle.erode_iterations", 5.0);
        feature_params["small_circle.largest_cc"] = config_reader_->getBool("preprocess.feature_extraction.small_circle.largest_cc", true) ? 1.0 : 0.0;
        feature_params["small_circle.dilate_kernel"] = config_reader_->getDouble("preprocess.feature_extraction.small_circle.dilate_kernel", 9.0);
        feature_params["small_circle.dilate_iterations"] = config_reader_->getDouble("preprocess.feature_extraction.small_circle.dilate_iterations", 1.0);
        feature_params["max_threads"] = config_reader_->getDouble("preprocess.feature_extraction.max_threads", 8.0);
        feature_extractor_->setParameters(feature_params);
        
        // 加载暴力匹配参数
        brute_force_matching_enabled_ = config_reader_->getBool("algorithm.brute_force_matching.enabled", false);
        brute_force_angle_step_deg_ = config_reader_->getDouble("algorithm.brute_force_matching.angle_step_deg", 0.5);
        brute_force_max_threads_ = config_reader_->getInt("algorithm.brute_force_matching.max_threads", 18);
        brute_force_rejection_threshold_ = config_reader_->getDouble("algorithm.brute_force_matching.rejection_threshold", 0.3);
        brute_force_acceptance_threshold_ = config_reader_->getDouble("algorithm.brute_force_matching.acceptance_threshold", 0.85);
        brute_force_angle_matching_scale_ = config_reader_->getDouble("algorithm.brute_force_matching.angle_matching_scale", 0.2);
        // 限制缩小比例在合理范围内
        brute_force_angle_matching_scale_ = std::clamp(brute_force_angle_matching_scale_, 0.1, 1.0);
    } else {
        brute_force_matching_enabled_ = false;
        brute_force_angle_step_deg_ = 0.5;
        brute_force_max_threads_ = 18;
        brute_force_rejection_threshold_ = 0.3;
        brute_force_acceptance_threshold_ = 0.85;
        brute_force_angle_matching_scale_ = 0.2;
    }
    
    // 创建服务
    estimate_pose_service_ = node_->create_service<interface::srv::EstimatePose>(
        "/estimate_pose",
        std::bind(&ROS2Communication::handleEstimatePose, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    debug_step_service_ = node_->create_service<interface::srv::ProcessDebugStep>(
        "/process_debug_step",
        std::bind(&ROS2Communication::handleProcessDebugStep, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    list_templates_service_ = node_->create_service<interface::srv::ListTemplates>(
        "/list_templates",
        std::bind(&ROS2Communication::handleListTemplates, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    visualize_grasp_pose_service_ = node_->create_service<interface::srv::VisualizeGraspPose>(
        "/visualize_grasp_pose",
        std::bind(&ROS2Communication::handleVisualizeGraspPose, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    standardize_template_service_ = node_->create_service<interface::srv::StandardizeTemplate>(
        "/standardize_template",
        std::bind(&ROS2Communication::handleStandardizeTemplate, this,
                  std::placeholders::_1, std::placeholders::_2));
    
    // 创建发布者
    status_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        "/visual_pose_estimation/status", 10);
    
    // 创建订阅者
    image_subscriber_ = node_->create_subscription<interface::msg::ImageData>(
        "/camera_0/image_data", 10,
        std::bind(&ROS2Communication::imageCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "ROS2通信初始化完成");
    return true;
}

void ROS2Communication::publishSystemStatus(const std::string& status) {
    auto msg = std_msgs::msg::String();
    msg.data = status;
    status_publisher_->publish(msg);
}

cv::Mat ROS2Communication::getLatestImage() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    has_new_image_ = false;
    return latest_image_.clone();
}

bool ROS2Communication::hasNewImage() {
    std::lock_guard<std::mutex> lock(image_mutex_);
    return has_new_image_;
}

void ROS2Communication::imageCallback(const interface::msg::ImageData::SharedPtr msg) {
    try {
        // 将ROS2图像消息转换为OpenCV格式
        // 这里需要根据实际的图像数据格式进行转换
        // 算法实现时将完善此部分
        
        std::lock_guard<std::mutex> lock(image_mutex_);
        has_new_image_ = true;
        // latest_image_ = ... (算法实现时添加)
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "图像回调处理失败: %s", e.what());
    }
}

void ROS2Communication::handleEstimatePose(
    const std::shared_ptr<interface::srv::EstimatePose::Request> request,
    std::shared_ptr<interface::srv::EstimatePose::Response> response) {
    
    // 记录开始时间
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 辅助函数：计算耗时并设置到响应中（保留3位小数）
    auto setProcessingTime = [&]() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double time_sec = duration.count() / 1000000.0;
        // 保留3位小数
        response->processing_time_sec = static_cast<float>(std::round(time_sec * 1000.0) / 1000.0);
    };
    
    RCLCPP_INFO(node_->get_logger(), "收到姿态估计请求: object_id=%s", request->object_id.c_str());
    
    // Debug模式：创建保存目录
    std::filesystem::path debug_base_dir = "/home/nvidia/RVG_ws/debug";
    std::filesystem::path debug_dir;
    if (debug_) {
        // 创建带时间戳的子文件夹
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        debug_dir = debug_base_dir / ("estimate_pose_" + ss.str() + "_" + request->object_id);
        std::filesystem::create_directories(debug_dir);
        RCLCPP_INFO(node_->get_logger(), "Debug模式：保存路径: %s", debug_dir.string().c_str());
    }
    
    try {
        // 初始化响应
        response->success_num = 0;
        response->confidence.clear();
        response->position.clear();
        response->preparation_position.clear();
        response->grab_position.clear();
        response->preplace_position.clear();
        response->place_position.clear();
        response->pose_image.clear();
        response->processing_time_sec = 0.0f;
        
        // 1. 转换ROS2图像消息为OpenCV图像
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(request->image, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "图像转换失败: %s", e.what());
            setProcessingTime();
            return;
        }
        
        cv::Mat input_image = cv_ptr->image;
        if (input_image.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "输入图像为空");
            setProcessingTime();
            return;
        }
        
        // Debug模式：保存原始输入图像
        if (debug_) {
            std::filesystem::path original_path = debug_dir / "00_original_input_image.jpg";
            cv::imwrite(original_path.string(), input_image);
        }
        
        // 2. 检查必要的组件
        if (!preprocessor_ || !feature_extractor_) {
            RCLCPP_ERROR(node_->get_logger(), "算法组件未初始化");
            setProcessingTime();
            return;
        }
        
        // 3. 构建模板目录路径
        std::string workpiece_template_dir = template_root_ + "/" + request->object_id;
        if (!std::filesystem::exists(workpiece_template_dir)) {
            RCLCPP_ERROR(node_->get_logger(), "工件模板目录不存在: %s", workpiece_template_dir.c_str());
            setProcessingTime();
            return;
        }
        
        // 4. 加载手眼标定参数
        // 优先使用启动时指定的calib_file，如果为空或不存在，则从模板文件夹查找
        std::filesystem::path calibration_path;
        if (!calib_file_.empty() && std::filesystem::exists(calib_file_)) {
            calibration_path = calib_file_;
            RCLCPP_INFO(node_->get_logger(), "使用指定的手眼标定文件: %s", calibration_path.string().c_str());
        } else {
            // 回退到从模板文件夹查找
            calibration_path = std::filesystem::path(workpiece_template_dir) / "hand_eye_calibration.xml";
            if (!std::filesystem::exists(calibration_path)) {
                RCLCPP_ERROR(node_->get_logger(), "手眼标定文件不存在: %s (指定的calib_file: %s)", 
                           calibration_path.string().c_str(), calib_file_.c_str());
                setProcessingTime();
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "使用模板文件夹中的手眼标定文件: %s", calibration_path.string().c_str());
        }
        
        cv::Mat camera_matrix, dist_coeffs, T_E_C;
        if (!loadHandEyeCalibration(calibration_path.string(), camera_matrix, dist_coeffs, T_E_C)) {
            RCLCPP_ERROR(node_->get_logger(), "加载手眼标定参数失败: %s", calibration_path.string().c_str());
            setProcessingTime();
            return;
        }
        
        // 5. 预处理和特征提取
        std::vector<cv::Mat> components = preprocessor_->preprocess(input_image);
        if (components.empty()) {
            RCLCPP_WARN(node_->get_logger(), "未检测到任何工件");
            setProcessingTime();
            return;
        }
        
        // Debug模式：保存预处理结果
        if (debug_) {
            // 保存所有连通域组件
            for (size_t i = 0; i < components.size(); i++) {
                std::filesystem::path comp_path = debug_dir / ("01_component_" + std::to_string(i) + ".jpg");
                cv::imwrite(comp_path.string(), components[i]);
            }
        }
        
        // 6. 加载模板库（所有连通域共享）
        // 使用pose_estimator.hpp中定义的TemplateItem结构
        std::vector<TemplateItem> templates;
        for (const auto& entry : std::filesystem::directory_iterator(workpiece_template_dir)) {
            if (!entry.is_directory()) continue;
            
            std::string pose_dir_name = entry.path().filename().string();
            if (pose_dir_name.find("pose_") != 0) continue;
            
            std::filesystem::path template_info_path = entry.path() / "template_info.json";
            if (!std::filesystem::exists(template_info_path)) continue;
            
            // 读取template_info.json
            std::ifstream template_file(template_info_path.string());
            Json::Value template_info;
            Json::Reader reader;
            if (!reader.parse(template_file, template_info)) continue;
            template_file.close();
            
            if (!template_info.isMember("feature_parameters")) continue;
            Json::Value feature_params = template_info["feature_parameters"];
            
            // 创建模板项
            TemplateItem template_item;
            template_item.id = pose_dir_name;
            
            // 设置特征
            template_item.feature.workpiece_center = cv::Point2f(
                feature_params.get("workpiece_center_x_original", 0.0).asDouble(),
                feature_params.get("workpiece_center_y_original", 0.0).asDouble());
            template_item.feature.workpiece_radius = feature_params.get("workpiece_radius_original", 0.0).asDouble();
            template_item.feature.valve_center = cv::Point2f(
                feature_params.get("final_valve_center_x", 0.0).asDouble(),
                feature_params.get("final_valve_center_y", 0.0).asDouble());
            template_item.feature.valve_radius = feature_params.get("final_valve_radius", 0.0).asDouble();
            template_item.feature.standardized_angle = feature_params.get("rotation_angle_rad", 0.0).asDouble();
            template_item.feature.workpiece_area = feature_params.get("workpiece_area", 0.0).asDouble();
            
            // 读取相机姿态
            cv::Mat T_B_E_camera;
            std::filesystem::path camera_pose_path = entry.path() / "camera_pose.json";
            std::filesystem::path prep_pose_path = entry.path() / "preparation_position.json";
            if (std::filesystem::exists(camera_pose_path)) {
                loadPoseJSON(camera_pose_path.string(), T_B_E_camera);
            } else if (std::filesystem::exists(prep_pose_path)) {
                loadPoseJSON(prep_pose_path.string(), T_B_E_camera);
            }
            
            if (!T_B_E_camera.empty()) {
                cv::Mat T_B_C_template = T_B_E_camera * T_E_C;
                cv::Mat T_C_template_B = T_B_C_template.inv();
                
                // 读取标准化抓取姿态
                if (template_info.isMember("standardized_grasp_pose")) {
                    Json::Value grasp_pose = template_info["standardized_grasp_pose"]["cartesian_position"];
                    Json::Value pos = grasp_pose["position"];
                    Json::Value ori = grasp_pose["orientation"];
                    
                    cv::Mat T_B_E_grasp = cv::Mat::eye(4, 4, CV_64F);
                    T_B_E_grasp.at<double>(0, 3) = pos.get("x", 0.0).asDouble();
                    T_B_E_grasp.at<double>(1, 3) = pos.get("y", 0.0).asDouble();
                    T_B_E_grasp.at<double>(2, 3) = pos.get("z", 0.0).asDouble();
                    
                    double qx = ori.get("x", 0.0).asDouble();
                    double qy = ori.get("y", 0.0).asDouble();
                    double qz = ori.get("z", 0.0).asDouble();
                    double qw = ori.get("w", 1.0).asDouble();
                    
                    cv::Vec4d quat(qx, qy, qz, qw);
                    double qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz, qw2 = qw * qw;
                    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
                    R.at<double>(0, 0) = qw2 + qx2 - qy2 - qz2;
                    R.at<double>(0, 1) = 2.0 * (qx * qy - qw * qz);
                    R.at<double>(0, 2) = 2.0 * (qx * qz + qw * qy);
                    R.at<double>(1, 0) = 2.0 * (qx * qy + qw * qz);
                    R.at<double>(1, 1) = qw2 - qx2 + qy2 - qz2;
                    R.at<double>(1, 2) = 2.0 * (qy * qz - qw * qx);
                    R.at<double>(2, 0) = 2.0 * (qx * qz - qw * qy);
                    R.at<double>(2, 1) = 2.0 * (qy * qz + qw * qx);
                    R.at<double>(2, 2) = qw2 - qx2 - qy2 + qz2;
                    R.copyTo(T_B_E_grasp(cv::Rect(0, 0, 3, 3)));
                    
                    template_item.T_C_E_grasp = T_C_template_B * T_B_E_grasp;
                }
            }
            
            templates.push_back(template_item);
        }
        
        if (templates.empty()) {
            RCLCPP_WARN(node_->get_logger(), "未找到模板");
            setProcessingTime();
            return;
        }
        
        // 7. 定义单个连通域的估计结果结构
        struct ComponentEstimationResult {
            double confidence = 0.0;
            geometry_msgs::msg::Point position;
            interface::msg::CartesianPosition preparation_position;
            interface::msg::CartesianPosition grab_position;
            interface::msg::CartesianPosition preplace_position;
            interface::msg::CartesianPosition place_position;
            sensor_msgs::msg::Image pose_image;
            cv::Mat vis_image;
            std::string template_id;
            ComponentFeature feature;
            int component_index;
        };
        
        std::vector<ComponentEstimationResult> all_results;
        
        // 8. 对每个连通域逐个执行模板匹配
        for (size_t comp_idx = 0; comp_idx < components.size(); ++comp_idx) {
            const cv::Mat& component = components[comp_idx];
            
            // Debug模式：为每个连通域创建子文件夹
            std::filesystem::path component_debug_dir = debug_dir;
            if (debug_) {
                component_debug_dir = debug_dir / ("component_" + std::to_string(comp_idx));
                std::filesystem::create_directories(component_debug_dir);
            }
            
            // 8.1 提取特征
            std::vector<cv::Mat> single_component = {component};
            std::vector<ComponentFeature> features = feature_extractor_->extractFeatures(single_component);
            
            if (features.empty()) {
                RCLCPP_WARN(node_->get_logger(), "连通域 %zu 特征提取失败，跳过", comp_idx);
                continue;
            }
            
            const ComponentFeature& target_feature = features[0];
            
            // Debug模式：保存特征提取可视化
            if (debug_) {
                cv::Mat feature_vis = input_image.clone();
                // 绘制工件外接圆
                cv::circle(feature_vis, target_feature.workpiece_center, 
                          static_cast<int>(target_feature.workpiece_radius), 
                          cv::Scalar(0, 255, 0), 3);
                cv::circle(feature_vis, target_feature.workpiece_center, 5, cv::Scalar(0, 255, 0), -1);
                // 绘制阀体圆
                if (target_feature.valve_radius > 0) {
                    cv::circle(feature_vis, target_feature.valve_center, 
                              static_cast<int>(target_feature.valve_radius), 
                              cv::Scalar(255, 0, 0), 3);
                    cv::circle(feature_vis, target_feature.valve_center, 5, cv::Scalar(255, 0, 0), -1);
                    // 绘制两圆心连线
                    cv::line(feature_vis, target_feature.workpiece_center, target_feature.valve_center,
                            cv::Scalar(0, 255, 255), 2);
                }
                std::filesystem::path feature_path = component_debug_dir / "03_feature_extraction_visualization.jpg";
                cv::imwrite(feature_path.string(), feature_vis);
            }
            
            // 8.2 选择最佳匹配模板
            int best_idx = -1;
            double best_distance = std::numeric_limits<double>::max();
            double mask_alignment_confidence = 0.0;
            double best_alignment_angle_deg = 0.0;
            cv::Mat best_aligned_mask;
            
            // 获取待估计工件的掩膜
            cv::Mat target_mask = component.clone();
            
            // 确保目标掩膜与输入图像尺寸一致
            if (target_mask.size() != input_image.size()) {
                cv::Mat resized_target_mask;
                cv::resize(target_mask, resized_target_mask, input_image.size());
                target_mask = resized_target_mask;
            }
            
            // 目标工件中心在输入图像中的位置
            cv::Point2f target_center_input(
                target_feature.workpiece_center.x,
                target_feature.workpiece_center.y
            );
            
            if (brute_force_matching_enabled_) {
                // 暴力匹配模式：遍历所有模板，进行旋转匹配
                RCLCPP_INFO(node_->get_logger(), "连通域 %zu: 使用暴力匹配模式，角度步进: %.2f度，最大线程数: %d", 
                           comp_idx, brute_force_angle_step_deg_, brute_force_max_threads_);
                RCLCPP_INFO(node_->get_logger(), "连通域 %zu: 目标工件标准化角度: %.2f度 (%.4f弧度)", 
                           comp_idx, target_feature.standardized_angle * 180.0 / M_PI, target_feature.standardized_angle);
                
                bool brute_force_success = bruteForceTemplateMatching(
                    &templates,  // 传递指针
                    workpiece_template_dir,
                    target_mask,
                    target_center_input,
                    target_feature.standardized_angle,  // 传递目标工件的标准化角度
                    target_feature.workpiece_radius,    // 传递目标工件的外接圆半径
                    component_debug_dir,  // 传递debug文件夹路径
                    best_idx,
                    best_alignment_angle_deg,
                    mask_alignment_confidence,
                    best_aligned_mask
                );
                
                if (!brute_force_success || best_idx < 0 || best_idx >= static_cast<int>(templates.size())) {
                    RCLCPP_WARN(node_->get_logger(), "连通域 %zu: 暴力匹配未找到匹配模板（可能低于舍弃阈值），跳过", comp_idx);
                    continue;  // 跳过当前连通域，处理下一个
                }
                
                // 再次检查置信度是否低于舍弃阈值（双重保险）
                if (mask_alignment_confidence < brute_force_rejection_threshold_) {
                    RCLCPP_WARN(node_->get_logger(), 
                               "连通域 %zu: 匹配置信度 %.4f 低于舍弃阈值 %.4f，跳过", 
                               comp_idx, mask_alignment_confidence, brute_force_rejection_threshold_);
                    continue;  // 跳过当前连通域，处理下一个
                }
                
                best_distance = 1.0 - mask_alignment_confidence;  // 转换为距离（置信度越高，距离越小）
            } else {
                // 原有匹配模式：使用特征距离
                for (size_t i = 0; i < templates.size(); ++i) {
                    const auto& tmpl = templates[i];
                    // 计算特征距离（简化：使用外接圆半径和角度的差异）
                    double radius_diff = std::abs(target_feature.workpiece_radius - tmpl.feature.workpiece_radius);
                    double angle_diff = std::abs(target_feature.standardized_angle - tmpl.feature.standardized_angle);
                    double distance = radius_diff / 100.0 + angle_diff;  // 归一化
                    
                    if (distance < best_distance) {
                        best_distance = distance;
                        best_idx = i;
                    }
                }
                
                if (best_idx < 0 || best_idx >= static_cast<int>(templates.size())) {
                    RCLCPP_WARN(node_->get_logger(), "连通域 %zu: 未找到匹配模板，跳过", comp_idx);
                    continue;  // 跳过当前连通域，处理下一个
                }
            }
            
            const TemplateItem& best_template = templates[best_idx];
            if (brute_force_matching_enabled_) {
                RCLCPP_INFO(node_->get_logger(), "连通域 %zu: 暴力匹配模板: %s, 置信度: %.4f, 角度: %.2f度", 
                           comp_idx, best_template.id.c_str(), mask_alignment_confidence, best_alignment_angle_deg);
            } else {
                RCLCPP_INFO(node_->get_logger(), "连通域 %zu: 匹配模板: %s, 距离: %.2f", comp_idx, best_template.id.c_str(), best_distance);
            }
            
            // Debug模式：保存模板匹配信息
            if (debug_) {
                Json::Value match_info;
                match_info["component_index"] = static_cast<int>(comp_idx);
                match_info["matched_template_id"] = best_template.id;
                match_info["matching_mode"] = brute_force_matching_enabled_ ? "brute_force" : "feature_distance";
                match_info["match_distance"] = best_distance;
                if (brute_force_matching_enabled_) {
                    match_info["match_confidence"] = mask_alignment_confidence;
                    match_info["best_angle_deg"] = best_alignment_angle_deg;
                    match_info["angle_step_deg"] = brute_force_angle_step_deg_;
                }
                match_info["target_feature"] = Json::Value(Json::objectValue);
                match_info["target_feature"]["workpiece_center_x"] = target_feature.workpiece_center.x;
                match_info["target_feature"]["workpiece_center_y"] = target_feature.workpiece_center.y;
                match_info["target_feature"]["workpiece_radius"] = target_feature.workpiece_radius;
                match_info["target_feature"]["valve_center_x"] = target_feature.valve_center.x;
                match_info["target_feature"]["valve_center_y"] = target_feature.valve_center.y;
                match_info["target_feature"]["valve_radius"] = target_feature.valve_radius;
                match_info["target_feature"]["standardized_angle"] = target_feature.standardized_angle;
                match_info["template_feature"] = Json::Value(Json::objectValue);
                match_info["template_feature"]["workpiece_center_x"] = best_template.feature.workpiece_center.x;
                match_info["template_feature"]["workpiece_center_y"] = best_template.feature.workpiece_center.y;
                match_info["template_feature"]["workpiece_radius"] = best_template.feature.workpiece_radius;
                match_info["template_feature"]["valve_center_x"] = best_template.feature.valve_center.x;
                match_info["template_feature"]["valve_center_y"] = best_template.feature.valve_center.y;
                match_info["template_feature"]["valve_radius"] = best_template.feature.valve_radius;
                match_info["template_feature"]["standardized_angle"] = best_template.feature.standardized_angle;
                
                std::filesystem::path match_info_path = component_debug_dir / "04_template_matching_info.json";
                std::ofstream match_file(match_info_path.string());
                Json::StreamWriterBuilder builder;
                builder["indentation"] = "  ";
                std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
                writer->write(match_info, &match_file);
                match_file.close();
            }
            
            // 8.3 计算姿态（简化版：假设相机位姿与模板拍摄时相同）
            // 获取模板的相机位姿
            std::filesystem::path best_template_dir = std::filesystem::path(workpiece_template_dir) / best_template.id;
            cv::Mat T_B_E_camera_template;
            std::filesystem::path camera_pose_path = best_template_dir / "camera_pose.json";
            if (std::filesystem::exists(camera_pose_path)) {
                loadPoseJSON(camera_pose_path.string(), T_B_E_camera_template);
            } else {
                std::filesystem::path prep_pose_path = best_template_dir / "preparation_position.json";
                if (std::filesystem::exists(prep_pose_path)) {
                    loadPoseJSON(prep_pose_path.string(), T_B_E_camera_template);
                }
            }
            
            if (T_B_E_camera_template.empty()) {
                RCLCPP_WARN(node_->get_logger(), "连通域 %zu: 无法获取模板相机位姿，跳过", comp_idx);
                continue;  // 跳过当前连通域，处理下一个
            }
            
            cv::Mat T_B_C_template = T_B_E_camera_template * T_E_C;
            
            // 读取模板的抓取姿态
            std::filesystem::path template_info_path = best_template_dir / "template_info.json";
            std::ifstream template_file(template_info_path.string());
            Json::Value template_info;
            Json::Reader reader;
            if (!reader.parse(template_file, template_info)) {
                RCLCPP_WARN(node_->get_logger(), "连通域 %zu: 无法解析template_info.json，跳过", comp_idx);
                continue;  // 跳过当前连通域，处理下一个
            }
            template_file.close();
            
            cv::Mat T_B_E_grasp_template;
            if (template_info.isMember("standardized_grasp_pose")) {
                Json::Value grasp_pose = template_info["standardized_grasp_pose"]["cartesian_position"];
                Json::Value pos = grasp_pose["position"];
                Json::Value ori = grasp_pose["orientation"];
                
                T_B_E_grasp_template = cv::Mat::eye(4, 4, CV_64F);
                T_B_E_grasp_template.at<double>(0, 3) = pos.get("x", 0.0).asDouble();
                T_B_E_grasp_template.at<double>(1, 3) = pos.get("y", 0.0).asDouble();
                T_B_E_grasp_template.at<double>(2, 3) = pos.get("z", 0.0).asDouble();
                
                double qx = ori.get("x", 0.0).asDouble();
                double qy = ori.get("y", 0.0).asDouble();
                double qz = ori.get("z", 0.0).asDouble();
                double qw = ori.get("w", 1.0).asDouble();
                
                double qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz, qw2 = qw * qw;
                cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
                R.at<double>(0, 0) = qw2 + qx2 - qy2 - qz2;
                R.at<double>(0, 1) = 2.0 * (qx * qy - qw * qz);
                R.at<double>(0, 2) = 2.0 * (qx * qz + qw * qy);
                R.at<double>(1, 0) = 2.0 * (qx * qy + qw * qz);
                R.at<double>(1, 1) = qw2 - qx2 + qy2 - qz2;
                R.at<double>(1, 2) = 2.0 * (qy * qz - qw * qx);
                R.at<double>(2, 0) = 2.0 * (qx * qz - qw * qy);
                R.at<double>(2, 1) = 2.0 * (qy * qz + qw * qx);
                R.at<double>(2, 2) = qw2 - qx2 - qy2 + qz2;
                R.copyTo(T_B_E_grasp_template(cv::Rect(0, 0, 3, 3)));
            } else {
                RCLCPP_WARN(node_->get_logger(), "连通域 %zu: 模板缺少抓取姿态，跳过", comp_idx);
                continue;  // 跳过当前连通域，处理下一个
            }
            
            // 角度差将在掩膜对齐后根据最佳匹配角度计算
            // 初始值设为0，掩膜对齐后会更新
            double dtheta = 0.0;  // 初始角度，掩膜对齐后会更新
            
            // 如果是暴力匹配模式，立即根据匹配结果计算dtheta
            if (brute_force_matching_enabled_) {
                // 暴力匹配模式下，已经得到了最佳对齐角度
                // best_alignment_angle_deg 是原始相对角度（不是归一化到0-360度的）
                // 将最佳角度从度转换为弧度
                double best_alignment_angle_rad = best_alignment_angle_deg * M_PI / 180.0;
                
                // 直接使用暴力匹配找到的角度（不取反，因为匹配角度已经表示正确的旋转方向）
                dtheta = best_alignment_angle_rad;
                
                // 归一化到[-π, π]范围
                while (dtheta > M_PI) dtheta -= 2 * M_PI;
                while (dtheta < -M_PI) dtheta += 2 * M_PI;
                
                RCLCPP_INFO(node_->get_logger(), "连通域 %zu: 暴力匹配：计算dtheta，最佳相对角度: %.2f度 (%.4f弧度), dtheta: %.2f度 (%.4f弧度)", 
                           comp_idx, best_alignment_angle_deg, best_alignment_angle_rad,
                           dtheta * 180.0 / M_PI, dtheta);
            }
            
            // 参考visual_pose_estimation_cpp的实现，重新梳理姿态计算流程：
            // 1. 通过相机拍照姿态和手眼标定矩阵，计算工件中心在机器人坐标系的位置
            // 2. 与模板相应的位置做比较，得到新抓取位置的偏移量
            // 3. 通过目标工件与模板匹配旋转的角度，计算抓取姿态
            
            double fx = camera_matrix.at<double>(0, 0);
            double fy = camera_matrix.at<double>(1, 1);
            double cx = camera_matrix.at<double>(0, 2);
            double cy = camera_matrix.at<double>(1, 2);
            
            // 步骤1: 获取模板抓取位置（在基座坐标系中）
            cv::Vec3d template_grasp_pos(
                T_B_E_grasp_template.at<double>(0, 3),
                T_B_E_grasp_template.at<double>(1, 3),
                T_B_E_grasp_template.at<double>(2, 3)
            );
            
            // 步骤2: 计算模板抓取姿态在相机坐标系下的深度（用于计算工件中心位置）
            cv::Mat T_C_template_B = T_B_C_template.inv();
            cv::Mat T_C_template_E_grasp = T_C_template_B * T_B_E_grasp_template;
            double z_template_camera = T_C_template_E_grasp.at<double>(2, 3);
            
            if (z_template_camera <= 0) {
                RCLCPP_WARN(node_->get_logger(), "连通域 %zu: 模板抓取姿态的Z坐标无效 (%.3f)，跳过", comp_idx, z_template_camera);
                continue;  // 跳过当前连通域，处理下一个
            }
            
            // 步骤3: 计算模板工件中心在基座坐标系下的位置（用于计算偏移）
            double template_center_u = best_template.feature.workpiece_center.x;
            double template_center_v = best_template.feature.workpiece_center.y;
            double template_center_x_camera = (template_center_u - cx) * z_template_camera / fx;
            double template_center_y_camera = (template_center_v - cy) * z_template_camera / fy;
            double template_center_z_camera = z_template_camera;
            
            // 转换到基座坐标系（使用模板拍摄时的相机位姿）
            cv::Mat template_center_camera = cv::Mat::eye(4, 4, CV_64F);
            template_center_camera.at<double>(0, 3) = template_center_x_camera;
            template_center_camera.at<double>(1, 3) = template_center_y_camera;
            template_center_camera.at<double>(2, 3) = template_center_z_camera;
            cv::Mat template_center_base = T_B_C_template * template_center_camera;
            cv::Vec3d template_center_base_pos(
                template_center_base.at<double>(0, 3),
                template_center_base.at<double>(1, 3),
                template_center_base.at<double>(2, 3)
            );
            
            // 步骤4: 计算模板抓取位置相对于模板中心的XY偏移（在基座坐标系中，只取XY）
            cv::Vec3d offset_template_base = template_grasp_pos - template_center_base_pos;
            
            // 步骤5: 计算目标工件中心在机器人基座坐标系下的位置
            // 假设当前拍摄时的相机位姿与模板拍摄时相同（眼在手上系统）
            cv::Mat T_B_C_current = T_B_C_template;  // 当前拍摄时的相机位姿与模板拍摄时相同
            
            double target_center_u = target_feature.workpiece_center.x;
            double target_center_v = target_feature.workpiece_center.y;
            
            // 使用模板工件中心的深度来计算目标工件中心位置
            double target_center_x_camera = (target_center_u - cx) * template_center_z_camera / fx;
            double target_center_y_camera = (target_center_v - cy) * template_center_z_camera / fy;
            double target_center_z_camera = template_center_z_camera;
            
            // 转换到基座坐标系
            cv::Mat target_center_camera = cv::Mat::eye(4, 4, CV_64F);
            target_center_camera.at<double>(0, 3) = target_center_x_camera;
            target_center_camera.at<double>(1, 3) = target_center_y_camera;
            target_center_camera.at<double>(2, 3) = target_center_z_camera;
            cv::Mat target_center_base = T_B_C_current * target_center_camera;
            cv::Vec3d target_center_base_pos(
                target_center_base.at<double>(0, 3),
                target_center_base.at<double>(1, 3),
                target_center_base.at<double>(2, 3)
            );
            
            // 步骤6: 通过目标工件与模板匹配旋转的角度，计算抓取姿态（在dtheta计算之后执行）
            // 按照用户指定的新计算逻辑：
            // 1. 计算模板中心与抓取位置的偏差（在基座坐标系中）
            // 2. 对目标中心的XY坐标应用这个偏差，得到中间数值
            // 3. 将中间数值绕目标工件中心的Z轴旋转dtheta角度，得到目标工件的抓取XY值
            // 4. Z轴：直接使用模板抓取姿态的Z坐标
            // 5. 旋转：将模板抓取姿态的旋转矩阵绕Z轴旋转dtheta角度
            
            // 构建绕Z轴的旋转矩阵（在基座坐标系中，逆时针为正）
            cv::Mat R_z_base = cv::Mat::eye(3, 3, CV_64F);
            R_z_base.at<double>(0, 0) = std::cos(dtheta);
            R_z_base.at<double>(0, 1) = -std::sin(dtheta);
            R_z_base.at<double>(1, 0) = std::sin(dtheta);
            R_z_base.at<double>(1, 1) = std::cos(dtheta);
            
            // 步骤1: 计算模板中心与抓取位置的偏差（在基座坐标系中，只取XY）
            cv::Vec3d template_grasp_offset_xy(
                offset_template_base[0],  // X偏差
                offset_template_base[1],  // Y偏差
                0.0                       // Z偏差设为0
            );
            
            // 步骤2: 对目标中心的XY坐标应用这个偏差，得到中间数值
            cv::Vec3d intermediate_pos(
                target_center_base_pos[0] + template_grasp_offset_xy[0],  // X = 目标中心X + 偏差X
                target_center_base_pos[1] + template_grasp_offset_xy[1],  // Y = 目标中心Y + 偏差Y
                target_center_base_pos[2]                                  // Z保持不变
            );
            
            // 步骤3: 将中间数值绕目标工件中心的Z轴旋转dtheta角度
            // 直接对中间数值进行旋转（相对于目标中心）
            cv::Vec3d intermediate_offset = intermediate_pos - target_center_base_pos;
            cv::Mat intermediate_offset_mat = cv::Mat::zeros(3, 1, CV_64F);
            intermediate_offset_mat.at<double>(0, 0) = intermediate_offset[0];
            intermediate_offset_mat.at<double>(1, 0) = intermediate_offset[1];
            intermediate_offset_mat.at<double>(2, 0) = intermediate_offset[2];
            cv::Mat rotated_intermediate_offset_mat = R_z_base * intermediate_offset_mat;
            
            // 计算目标抓取位置：目标中心 + 旋转后的偏移，Z直接使用模板的Z
            cv::Vec3d target_grasp_pos_base(
                target_center_base_pos[0] + rotated_intermediate_offset_mat.at<double>(0, 0),  // X = 目标中心X + 旋转后的偏移X
                target_center_base_pos[1] + rotated_intermediate_offset_mat.at<double>(1, 0),  // Y = 目标中心Y + 旋转后的偏移Y
                template_grasp_pos[2]                                                           // Z = 模板抓取位置的Z（直接使用）
            );
            
            // ========== 抓取姿态的旋转计算 ==========
            // 姿态计算：直接将模板的姿态绕Z轴旋转模板与目标工件的偏差角度dtheta
            // 获取模板抓取姿态的旋转矩阵
            cv::Mat R_template_grasp = T_B_E_grasp_template(cv::Rect(0, 0, 3, 3));
            
            // 将模板抓取姿态的旋转矩阵绕Z轴旋转dtheta角度
            // R_z_base是绕Z轴旋转dtheta的旋转矩阵，直接左乘模板姿态即可
            cv::Mat R_target_grasp = R_z_base * R_template_grasp;
            
            // 构建最终的抓取姿态
            cv::Mat T_B_E_grasp_current = cv::Mat::eye(4, 4, CV_64F);
            R_target_grasp.copyTo(T_B_E_grasp_current(cv::Rect(0, 0, 3, 3)));
            T_B_E_grasp_current.at<double>(0, 3) = target_grasp_pos_base[0];
            T_B_E_grasp_current.at<double>(1, 3) = target_grasp_pos_base[1];
            T_B_E_grasp_current.at<double>(2, 3) = target_grasp_pos_base[2];
            
            // 计算准备姿态
            cv::Mat T_B_E_prep_current;
            if (template_info.isMember("standardized_preparation_pose")) {
                // 读取模板的准备姿态
                Json::Value prep_pose = template_info["standardized_preparation_pose"]["cartesian_position"];
                Json::Value prep_pos = prep_pose["position"];
                Json::Value prep_ori = prep_pose["orientation"];
                
                cv::Mat T_B_E_prep_template = cv::Mat::eye(4, 4, CV_64F);
                T_B_E_prep_template.at<double>(0, 3) = prep_pos.get("x", 0.0).asDouble();
                T_B_E_prep_template.at<double>(1, 3) = prep_pos.get("y", 0.0).asDouble();
                T_B_E_prep_template.at<double>(2, 3) = prep_pos.get("z", 0.0).asDouble();
                
                double qx = prep_ori.get("x", 0.0).asDouble();
                double qy = prep_ori.get("y", 0.0).asDouble();
                double qz = prep_ori.get("z", 0.0).asDouble();
                double qw = prep_ori.get("w", 1.0).asDouble();
                
                double qx2 = qx * qx, qy2 = qy * qy, qz2 = qz * qz, qw2 = qw * qw;
                cv::Mat R_prep = cv::Mat::eye(3, 3, CV_64F);
                R_prep.at<double>(0, 0) = qw2 + qx2 - qy2 - qz2;
                R_prep.at<double>(0, 1) = 2.0 * (qx * qy - qw * qz);
                R_prep.at<double>(0, 2) = 2.0 * (qx * qz + qw * qy);
                R_prep.at<double>(1, 0) = 2.0 * (qx * qy + qw * qz);
                R_prep.at<double>(1, 1) = qw2 - qx2 + qy2 - qz2;
                R_prep.at<double>(1, 2) = 2.0 * (qy * qz - qw * qx);
                R_prep.at<double>(2, 0) = 2.0 * (qx * qz - qw * qy);
                R_prep.at<double>(2, 1) = 2.0 * (qy * qz + qw * qx);
                R_prep.at<double>(2, 2) = qw2 - qx2 - qy2 + qz2;
                R_prep.copyTo(T_B_E_prep_template(cv::Rect(0, 0, 3, 3)));
                
                // 使用与抓取姿态相同的新逻辑计算准备姿态
                // 步骤1: 计算模板准备位置与模板中心的偏差（在基座坐标系中，只取XY）
                cv::Vec3d template_prep_pos(
                    T_B_E_prep_template.at<double>(0, 3),
                    T_B_E_prep_template.at<double>(1, 3),
                    T_B_E_prep_template.at<double>(2, 3)
                );
                cv::Vec3d offset_prep_base = template_prep_pos - template_center_base_pos;
                cv::Vec3d template_prep_offset_xy(
                    offset_prep_base[0],  // X偏差
                    offset_prep_base[1],  // Y偏差
                    0.0                   // Z偏差设为0
                );
                
                // 步骤2: 对目标中心的XY坐标应用这个偏差，得到中间数值
                cv::Vec3d intermediate_prep_pos(
                    target_center_base_pos[0] + template_prep_offset_xy[0],  // X = 目标中心X + 偏差X
                    target_center_base_pos[1] + template_prep_offset_xy[1],  // Y = 目标中心Y + 偏差Y
                    target_center_base_pos[2]                                  // Z保持不变
                );
                
                // 步骤3: 将中间数值绕目标工件中心的Z轴旋转dtheta角度
                cv::Vec3d intermediate_prep_offset = intermediate_prep_pos - target_center_base_pos;
                cv::Mat intermediate_prep_offset_mat = cv::Mat::zeros(3, 1, CV_64F);
                intermediate_prep_offset_mat.at<double>(0, 0) = intermediate_prep_offset[0];
                intermediate_prep_offset_mat.at<double>(1, 0) = intermediate_prep_offset[1];
                intermediate_prep_offset_mat.at<double>(2, 0) = intermediate_prep_offset[2];
                cv::Mat rotated_intermediate_prep_offset_mat = R_z_base * intermediate_prep_offset_mat;
                cv::Vec3d rotated_intermediate_prep_offset(
                    rotated_intermediate_prep_offset_mat.at<double>(0, 0),
                    rotated_intermediate_prep_offset_mat.at<double>(1, 0),
                    rotated_intermediate_prep_offset_mat.at<double>(2, 0)
                );
                
                // 计算目标准备位置：目标中心 + 旋转后的偏移，Z直接使用模板的Z
                cv::Vec3d target_prep_pos_base(
                    target_center_base_pos[0] + rotated_intermediate_prep_offset[0],  // X = 目标中心X + 旋转后的偏移X
                    target_center_base_pos[1] + rotated_intermediate_prep_offset[1],  // Y = 目标中心Y + 旋转后的偏移Y
                    template_prep_pos[2]                                                // Z = 模板准备位置的Z（直接使用）
                );
                
                // ========== 预抓取姿态的旋转计算 ==========
                // 姿态计算：直接将模板的姿态绕Z轴旋转模板与目标工件的偏差角度dtheta
                // 获取模板预抓取姿态的旋转矩阵
                cv::Mat R_template_prep = T_B_E_prep_template(cv::Rect(0, 0, 3, 3));
                
                // 将模板预抓取姿态的旋转矩阵绕Z轴旋转dtheta角度
                // R_z_base是绕Z轴旋转dtheta的旋转矩阵，直接左乘模板姿态即可
                cv::Mat R_target_prep = R_z_base * R_template_prep;
                
                // 构建最终的准备姿态
                T_B_E_prep_current = cv::Mat::eye(4, 4, CV_64F);
                R_target_prep.copyTo(T_B_E_prep_current(cv::Rect(0, 0, 3, 3)));
                T_B_E_prep_current.at<double>(0, 3) = target_prep_pos_base[0];
                T_B_E_prep_current.at<double>(1, 3) = target_prep_pos_base[1];
                T_B_E_prep_current.at<double>(2, 3) = target_prep_pos_base[2];
            } else {
                // 如果没有准备姿态，使用抓取姿态
                T_B_E_prep_current = T_B_E_grasp_current.clone();
            }
            
            // 8.4 收集当前连通域的估计结果
            ComponentEstimationResult result;
            result.component_index = static_cast<int>(comp_idx);
            result.template_id = best_template.id;
            result.feature = target_feature;
            
            // 使用掩膜对齐的置信度作为输出置信度
            if (mask_alignment_confidence > 0.0) {
                result.confidence = mask_alignment_confidence;
            } else {
                result.confidence = 1.0 / (1.0 + best_distance);
            }
            
            // 工件中心位置（相机坐标系）
            result.position.x = target_center_x_camera;
            result.position.y = target_center_y_camera;
            result.position.z = target_center_z_camera;
            
            // 转换姿态到CartesianPosition
            result.grab_position.position.x = T_B_E_grasp_current.at<double>(0, 3);
            result.grab_position.position.y = T_B_E_grasp_current.at<double>(1, 3);
            result.grab_position.position.z = T_B_E_grasp_current.at<double>(2, 3);
            
            result.preparation_position.position.x = T_B_E_prep_current.at<double>(0, 3);
            result.preparation_position.position.y = T_B_E_prep_current.at<double>(1, 3);
            result.preparation_position.position.z = T_B_E_prep_current.at<double>(2, 3);
            
            // 转换旋转矩阵到四元数
            cv::Mat grab_rot = T_B_E_grasp_current(cv::Rect(0, 0, 3, 3));
            cv::Mat prep_rot = T_B_E_prep_current(cv::Rect(0, 0, 3, 3));
            
            cv::Vec4d grab_quat = rotationMatrixToQuaternion(grab_rot);
            cv::Vec4d prep_quat = rotationMatrixToQuaternion(prep_rot);
            
            // 四元数取反（正负号相反）
            result.grab_position.orientation.x = -grab_quat[0];
            result.grab_position.orientation.y = -grab_quat[1];
            result.grab_position.orientation.z = -grab_quat[2];
            result.grab_position.orientation.w = -grab_quat[3];
            
            result.preparation_position.orientation.x = -prep_quat[0];
            result.preparation_position.orientation.y = -prep_quat[1];
            result.preparation_position.orientation.z = -prep_quat[2];
            result.preparation_position.orientation.w = -prep_quat[3];
            
            // 计算欧拉角（RPY，ZYX顺序）
            cv::Vec3d grab_euler_rad = rotationMatrixToEulerRPY(grab_rot);
            cv::Vec3d prep_euler_rad = rotationMatrixToEulerRPY(prep_rot);
            
            // 设置欧拉角（弧度）
            result.grab_position.euler_orientation_rpy_rad[0] = grab_euler_rad[0];  // roll
            result.grab_position.euler_orientation_rpy_rad[1] = grab_euler_rad[1];  // pitch
            result.grab_position.euler_orientation_rpy_rad[2] = grab_euler_rad[2];  // yaw
            
            result.preparation_position.euler_orientation_rpy_rad[0] = prep_euler_rad[0];  // roll
            result.preparation_position.euler_orientation_rpy_rad[1] = prep_euler_rad[1];  // pitch
            result.preparation_position.euler_orientation_rpy_rad[2] = prep_euler_rad[2];  // yaw
            
            // 设置欧拉角（度）
            result.grab_position.euler_orientation_rpy_deg[0] = grab_euler_rad[0] * 180.0 / M_PI;  // roll
            result.grab_position.euler_orientation_rpy_deg[1] = grab_euler_rad[1] * 180.0 / M_PI;  // pitch
            result.grab_position.euler_orientation_rpy_deg[2] = grab_euler_rad[2] * 180.0 / M_PI;  // yaw
            
            result.preparation_position.euler_orientation_rpy_deg[0] = prep_euler_rad[0] * 180.0 / M_PI;  // roll
            result.preparation_position.euler_orientation_rpy_deg[1] = prep_euler_rad[1] * 180.0 / M_PI;  // pitch
            result.preparation_position.euler_orientation_rpy_deg[2] = prep_euler_rad[2] * 180.0 / M_PI;  // yaw
            
            // 计算预放置姿态和放置姿态（直接使用模板原始值）
            // 读取模板的预放置位置
            std::filesystem::path preplace_path = best_template_dir / "preplace_position.json";
            if (std::filesystem::exists(preplace_path)) {
                cv::Mat T_B_E_preplace_template;
                if (loadPoseJSON(preplace_path.string(), T_B_E_preplace_template)) {
                    // 直接使用模板的预放置位置和姿态
                    result.preplace_position.position.x = T_B_E_preplace_template.at<double>(0, 3);
                    result.preplace_position.position.y = T_B_E_preplace_template.at<double>(1, 3);
                    result.preplace_position.position.z = T_B_E_preplace_template.at<double>(2, 3);
                    
                    cv::Mat R_template_preplace = T_B_E_preplace_template(cv::Rect(0, 0, 3, 3));
                    cv::Vec4d preplace_quat = rotationMatrixToQuaternion(R_template_preplace);
                    result.preplace_position.orientation.x = -preplace_quat[0];
                    result.preplace_position.orientation.y = -preplace_quat[1];
                    result.preplace_position.orientation.z = -preplace_quat[2];
                    result.preplace_position.orientation.w = -preplace_quat[3];
                    
                    // 计算欧拉角（RPY，ZYX顺序）
                    cv::Vec3d preplace_euler_rad = rotationMatrixToEulerRPY(R_template_preplace);
                    result.preplace_position.euler_orientation_rpy_rad[0] = preplace_euler_rad[0];  // roll
                    result.preplace_position.euler_orientation_rpy_rad[1] = preplace_euler_rad[1];  // pitch
                    result.preplace_position.euler_orientation_rpy_rad[2] = preplace_euler_rad[2];  // yaw
                    result.preplace_position.euler_orientation_rpy_deg[0] = preplace_euler_rad[0] * 180.0 / M_PI;  // roll
                    result.preplace_position.euler_orientation_rpy_deg[1] = preplace_euler_rad[1] * 180.0 / M_PI;  // pitch
                    result.preplace_position.euler_orientation_rpy_deg[2] = preplace_euler_rad[2] * 180.0 / M_PI;  // yaw
                }
            }
            
            // 读取模板的放置位置
            std::filesystem::path place_path = best_template_dir / "place_position.json";
            if (std::filesystem::exists(place_path)) {
                cv::Mat T_B_E_place_template;
                if (loadPoseJSON(place_path.string(), T_B_E_place_template)) {
                    // 直接使用模板的放置位置和姿态
                    result.place_position.position.x = T_B_E_place_template.at<double>(0, 3);
                    result.place_position.position.y = T_B_E_place_template.at<double>(1, 3);
                    result.place_position.position.z = T_B_E_place_template.at<double>(2, 3);
                    
                    cv::Mat R_template_place = T_B_E_place_template(cv::Rect(0, 0, 3, 3));
                    cv::Vec4d place_quat = rotationMatrixToQuaternion(R_template_place);
                    result.place_position.orientation.x = -place_quat[0];
                    result.place_position.orientation.y = -place_quat[1];
                    result.place_position.orientation.z = -place_quat[2];
                    result.place_position.orientation.w = -place_quat[3];
                    
                    // 计算欧拉角（RPY，ZYX顺序）
                    cv::Vec3d place_euler_rad = rotationMatrixToEulerRPY(R_template_place);
                    result.place_position.euler_orientation_rpy_rad[0] = place_euler_rad[0];  // roll
                    result.place_position.euler_orientation_rpy_rad[1] = place_euler_rad[1];  // pitch
                    result.place_position.euler_orientation_rpy_rad[2] = place_euler_rad[2];  // yaw
                    result.place_position.euler_orientation_rpy_deg[0] = place_euler_rad[0] * 180.0 / M_PI;  // roll
                    result.place_position.euler_orientation_rpy_deg[1] = place_euler_rad[1] * 180.0 / M_PI;  // pitch
                    result.place_position.euler_orientation_rpy_deg[2] = place_euler_rad[2] * 180.0 / M_PI;  // yaw
                }
            }
            
            // 创建可视化图像（单个连通域）
            cv::Mat vis_image = input_image.clone();
            
            // 在可视化图像上绘制特征
            // 绘制工件外接圆
            cv::circle(vis_image, target_feature.workpiece_center, 
                      static_cast<int>(target_feature.workpiece_radius), 
                      cv::Scalar(0, 255, 0), 3);
            cv::circle(vis_image, target_feature.workpiece_center, 5, cv::Scalar(0, 255, 0), -1);
            // 绘制阀体圆
            if (target_feature.valve_radius > 0) {
                cv::circle(vis_image, target_feature.valve_center, 
                          static_cast<int>(target_feature.valve_radius), 
                          cv::Scalar(255, 0, 0), 3);
                cv::circle(vis_image, target_feature.valve_center, 5, cv::Scalar(255, 0, 0), -1);
                cv::line(vis_image, target_feature.workpiece_center, target_feature.valve_center,
                        cv::Scalar(0, 255, 255), 2);
            }
            
            // 掩膜对齐和可视化
            if (brute_force_matching_enabled_ && !best_aligned_mask.empty()) {
                // 暴力匹配模式下，已经得到了最佳对齐的掩膜
                // 使用最佳对齐的掩膜进行可视化
                // 半透明叠加掩膜（使用青色，透明度40%）
                cv::Mat vis_image_float;
                vis_image.convertTo(vis_image_float, CV_32F, 1.0 / 255.0);
                
                double alpha = 0.4;  // 透明度
                cv::Vec3f mask_color(0.0f, 1.0f, 1.0f);  // 青色 (BGR)
                
                for (int y = 0; y < vis_image.rows; ++y) {
                    for (int x = 0; x < vis_image.cols; ++x) {
                        if (best_aligned_mask.at<uchar>(y, x) > 128) {  // 掩膜区域
                            cv::Vec3f& pixel = vis_image_float.at<cv::Vec3f>(y, x);
                            pixel = pixel * (1.0 - alpha) + mask_color * alpha;
                        }
                    }
                }
                
                vis_image_float.convertTo(vis_image, CV_8U, 255.0);
                
                // Debug模式：保存对齐后的掩膜
                if (debug_) {
                    std::filesystem::path aligned_mask_path = component_debug_dir / "08_aligned_template_mask.jpg";
                    cv::imwrite(aligned_mask_path.string(), best_aligned_mask);
                }
            }
            
            // 计算当前相机坐标系下的抓取姿态（用于可视化）
            cv::Mat T_C_current_B = T_B_C_current.inv();
            cv::Mat T_C_current_E_grasp = T_C_current_B * T_B_E_grasp_current;
            
            // 绘制抓取姿态可视化
            drawGripperOnImage(vis_image, T_C_current_E_grasp, camera_matrix, dist_coeffs, 50.0, 100.0);
            
            // Debug模式：保存可视化图像
            if (debug_) {
                std::filesystem::path vis_path = component_debug_dir / "05_visualization_with_features.jpg";
                cv::imwrite(vis_path.string(), vis_image);
            }
            
            // 保存可视化图像到结果
            result.vis_image = vis_image;
            
            // 转换可视化图像为ROS消息
            sensor_msgs::msg::Image pose_img_msg;
            try {
                cv_bridge::CvImage pose_cv_bridge;
                pose_cv_bridge.image = vis_image;
                pose_cv_bridge.encoding = sensor_msgs::image_encodings::BGR8;
                pose_img_msg = *pose_cv_bridge.toImageMsg();
                result.pose_image = pose_img_msg;
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "连通域 %zu: 位姿图像转换失败: %s", comp_idx, e.what());
            }
            
            // 将结果添加到结果列表
            all_results.push_back(result);
        }  // 结束连通域循环
        
        // 9. 统一构建响应和可视化
        if (all_results.empty()) {
            RCLCPP_WARN(node_->get_logger(), "未找到任何有效的姿态估计结果");
            response->success_num = 0;
            setProcessingTime();
            return;
        }
        
        // 按置信度排序（从高到低）
        std::sort(all_results.begin(), all_results.end(), 
                  [](const ComponentEstimationResult& a, const ComponentEstimationResult& b) {
                      return a.confidence > b.confidence;
                  });
        
        // 填充响应
        response->success_num = static_cast<int32_t>(all_results.size());
        response->confidence.clear();
        response->position.clear();
        response->preparation_position.clear();
        response->grab_position.clear();
        response->preplace_position.clear();
        response->place_position.clear();
        response->pose_image.clear();
        
        // 创建综合可视化图像（包含所有连通域）
        cv::Mat combined_vis_image = input_image.clone();
        
        for (const auto& result : all_results) {
            // 填充置信度
            response->confidence.push_back(static_cast<float>(result.confidence));
            
            // 填充位置
            response->position.push_back(result.position);
            
            // 填充姿态
            response->preparation_position.push_back(result.preparation_position);
            response->grab_position.push_back(result.grab_position);
            response->preplace_position.push_back(result.preplace_position);
            response->place_position.push_back(result.place_position);
            
            // 填充位姿图像
            response->pose_image.push_back(result.pose_image);
            
            // 在综合可视化图像上绘制特征（使用不同颜色区分不同连通域）
            cv::Scalar color(0, 255, 0);  // 默认绿色
            if (result.component_index % 3 == 0) {
                color = cv::Scalar(0, 255, 0);  // 绿色
            } else if (result.component_index % 3 == 1) {
                color = cv::Scalar(255, 0, 0);  // 蓝色
            } else {
                color = cv::Scalar(0, 0, 255);  // 红色
            }
            
            // 绘制工件外接圆
            cv::circle(combined_vis_image, result.feature.workpiece_center, 
                      static_cast<int>(result.feature.workpiece_radius), 
                      color, 3);
            cv::circle(combined_vis_image, result.feature.workpiece_center, 5, color, -1);
            
            // 绘制阀体圆
            if (result.feature.valve_radius > 0) {
                cv::circle(combined_vis_image, result.feature.valve_center, 
                          static_cast<int>(result.feature.valve_radius), 
                          color, 3);
                cv::circle(combined_vis_image, result.feature.valve_center, 5, color, -1);
                cv::line(combined_vis_image, result.feature.workpiece_center, result.feature.valve_center,
                        color, 2);
            }
            
            // 绘制置信度文本
            std::string conf_text = "C" + std::to_string(result.component_index) + ": " + 
                                   std::to_string(result.confidence).substr(0, 4);
            cv::putText(combined_vis_image, conf_text, 
                       cv::Point(result.feature.workpiece_center.x - 30, 
                                result.feature.workpiece_center.y - static_cast<int>(result.feature.workpiece_radius) - 10),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
        }
        
        // 转换综合可视化图像为ROS消息
        sensor_msgs::msg::Image vis_image_msg;
        try {
            cv_bridge::CvImage vis_cv_bridge;
            vis_cv_bridge.image = combined_vis_image;
            vis_cv_bridge.encoding = sensor_msgs::image_encodings::BGR8;
            vis_image_msg = *vis_cv_bridge.toImageMsg();
            response->vis_image = vis_image_msg;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "可视化图像转换失败: %s", e.what());
        }
        
        // Debug模式：保存综合可视化图像
        if (debug_) {
            std::filesystem::path combined_vis_path = debug_dir / "06_combined_visualization.jpg";
            cv::imwrite(combined_vis_path.string(), combined_vis_image);
        }
        
        // Debug模式：保存最终结果JSON
        if (debug_) {
            // 先计算耗时（用于保存到JSON）
            setProcessingTime();
            
            Json::Value result_json;
            result_json["success_num"] = response->success_num;
            result_json["object_id"] = request->object_id;
            result_json["processing_time_sec"] = response->processing_time_sec;
            result_json["confidence"] = Json::Value(Json::arrayValue);
            for (const auto& conf : response->confidence) {
                result_json["confidence"].append(conf);
            }
            result_json["positions"] = Json::Value(Json::arrayValue);
            for (const auto& pos : response->position) {
                Json::Value pos_obj;
                pos_obj["x"] = pos.x;
                pos_obj["y"] = pos.y;
                pos_obj["z"] = pos.z;
                result_json["positions"].append(pos_obj);
            }
            result_json["grab_positions"] = Json::Value(Json::arrayValue);
            for (const auto& grab_pos : response->grab_position) {
                Json::Value grab_obj;
                grab_obj["position"]["x"] = grab_pos.position.x;
                grab_obj["position"]["y"] = grab_pos.position.y;
                grab_obj["position"]["z"] = grab_pos.position.z;
                grab_obj["orientation"]["x"] = grab_pos.orientation.x;
                grab_obj["orientation"]["y"] = grab_pos.orientation.y;
                grab_obj["orientation"]["z"] = grab_pos.orientation.z;
                grab_obj["orientation"]["w"] = grab_pos.orientation.w;
                result_json["grab_positions"].append(grab_obj);
            }
            result_json["preparation_positions"] = Json::Value(Json::arrayValue);
            for (const auto& prep_pos : response->preparation_position) {
                Json::Value prep_obj;
                prep_obj["position"]["x"] = prep_pos.position.x;
                prep_obj["position"]["y"] = prep_pos.position.y;
                prep_obj["position"]["z"] = prep_pos.position.z;
                prep_obj["orientation"]["x"] = prep_pos.orientation.x;
                prep_obj["orientation"]["y"] = prep_pos.orientation.y;
                prep_obj["orientation"]["z"] = prep_pos.orientation.z;
                prep_obj["orientation"]["w"] = prep_pos.orientation.w;
                result_json["preparation_positions"].append(prep_obj);
            }
            
            std::filesystem::path result_path = debug_dir / "06_estimation_result.json";
            std::ofstream result_file(result_path.string());
            Json::StreamWriterBuilder builder;
            builder["indentation"] = "  ";
            std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
            writer->write(result_json, &result_file);
            result_file.close();
            
            // 保存最终可视化图像
            std::filesystem::path final_vis_path = debug_dir / "07_final_visualization.jpg";
            cv::imwrite(final_vis_path.string(), combined_vis_image);
            
            RCLCPP_INFO(node_->get_logger(), "Debug模式：已保存所有过程文件到 %s", debug_dir.string().c_str());
        }
        
        // 计算并设置处理耗时
        setProcessingTime();
        
        RCLCPP_INFO(node_->get_logger(), "姿态估计服务处理完成，检测到 %d 个目标，耗时: %.3f 秒", 
                   response->success_num, response->processing_time_sec);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "姿态估计服务异常: %s", e.what());
        response->success_num = 0;
        setProcessingTime();
    }
}

bool ROS2Communication::loadHandEyeCalibration(
    const std::string& calibration_path,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs,
    cv::Mat& T_E_C) {
    
    try {
        std::ifstream file(calibration_path);
        if (!file.is_open()) {
            return false;
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string file_content = buffer.str();
        file.close();
        
        // 提取矩阵数据
        auto extractMatrixData = [](const std::string& content, const std::string& matrix_name) -> std::vector<double> {
            std::vector<double> values;
            std::string start_tag = "<" + matrix_name;
            std::string end_tag = "</" + matrix_name + ">";
            
            size_t start_pos = content.find(start_tag);
            if (start_pos == std::string::npos) {
                return values;
            }
            
            size_t data_start = content.find("<data>", start_pos);
            if (data_start == std::string::npos) {
                return values;
            }
            
            size_t data_end = content.find("</data>", data_start);
            if (data_end == std::string::npos) {
                return values;
            }
            
            std::string data_str = content.substr(data_start + 6, data_end - data_start - 6);
            data_str.erase(0, data_str.find_first_not_of(" \t\n\r"));
            data_str.erase(data_str.find_last_not_of(" \t\n\r") + 1);
            
            std::istringstream iss(data_str);
            double val;
            while (iss >> val) {
                values.push_back(val);
            }
            
            return values;
        };
        
        // 提取CameraMatrix
        std::vector<double> camera_matrix_data = extractMatrixData(file_content, "CameraMatrix");
        if (camera_matrix_data.size() == 9) {
            camera_matrix = cv::Mat(3, 3, CV_64F);
            for (int i = 0; i < 9; i++) {
                camera_matrix.at<double>(i / 3, i % 3) = camera_matrix_data[i];
            }
        } else {
            return false;
        }
        
        // 提取DistortionCoefficients
        std::vector<double> dist_data = extractMatrixData(file_content, "DistortionCoefficients");
        if (dist_data.size() >= 4) {
            dist_coeffs = cv::Mat(dist_data.size(), 1, CV_64F);
            for (size_t i = 0; i < dist_data.size(); i++) {
                dist_coeffs.at<double>(i, 0) = dist_data[i];
            }
        } else {
            return false;
        }
        
        // 提取TransformationMatrix
        std::vector<double> transform_data = extractMatrixData(file_content, "TransformationMatrix");
        if (transform_data.size() == 16) {
            T_E_C = cv::Mat(4, 4, CV_64F);
            for (int i = 0; i < 16; i++) {
                T_E_C.at<double>(i / 4, i % 4) = transform_data[i];
            }
        } else {
            return false;
        }
        
        return true;
        
    } catch (const std::exception&) {
        return false;
    }
}

bool ROS2Communication::loadPoseJSON(
    const std::string& pose_path,
    cv::Mat& T_B_E) {
    
    try {
        std::ifstream file(pose_path);
        if (!file.is_open()) {
            return false;
        }
        
        Json::Value json_data;
        Json::Reader reader;
        if (!reader.parse(file, json_data)) {
            file.close();
            return false;
        }
        file.close();
        
        // 读取位置
        const Json::Value& pos = json_data["cartesian_position"]["position"];
        double x = pos["x"].asDouble();
        double y = pos["y"].asDouble();
        double z = pos["z"].asDouble();
        
        // 读取四元数
        const Json::Value& orient = json_data["cartesian_position"]["orientation"];
        double qx = orient["x"].asDouble();
        double qy = orient["y"].asDouble();
        double qz = orient["z"].asDouble();
        double qw = orient["w"].asDouble();
        
        // 将四元数转换为旋转矩阵
        double qx2 = qx * qx;
        double qy2 = qy * qy;
        double qz2 = qz * qz;
        double qw2 = qw * qw;
        
        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        R.at<double>(0, 0) = qw2 + qx2 - qy2 - qz2;
        R.at<double>(0, 1) = 2.0 * (qx * qy - qw * qz);
        R.at<double>(0, 2) = 2.0 * (qx * qz + qw * qy);
        R.at<double>(1, 0) = 2.0 * (qx * qy + qw * qz);
        R.at<double>(1, 1) = qw2 - qx2 + qy2 - qz2;
        R.at<double>(1, 2) = 2.0 * (qy * qz - qw * qx);
        R.at<double>(2, 0) = 2.0 * (qx * qz - qw * qy);
        R.at<double>(2, 1) = 2.0 * (qy * qz + qw * qx);
        R.at<double>(2, 2) = qw2 - qx2 - qy2 + qz2;
        
        // 构建4x4齐次变换矩阵
        T_B_E = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(T_B_E(cv::Rect(0, 0, 3, 3)));
        T_B_E.at<double>(0, 3) = x;
        T_B_E.at<double>(1, 3) = y;
        T_B_E.at<double>(2, 3) = z;
        
        return true;
        
    } catch (const std::exception&) {
        return false;
    }
}

cv::Vec4d ROS2Communication::rotationMatrixToQuaternion(const cv::Mat& R) {
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    double w, x, y, z;
    
    if (trace > 0) {
        double s = std::sqrt(trace + 1.0) * 2.0;
        w = 0.25 * s;
        x = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
        y = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
        z = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
    } else if ((R.at<double>(0, 0) > R.at<double>(1, 1)) && (R.at<double>(0, 0) > R.at<double>(2, 2))) {
        double s = std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2)) * 2.0;
        w = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
        x = 0.25 * s;
        y = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        z = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
    } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
        double s = std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2)) * 2.0;
        w = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
        x = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        y = 0.25 * s;
        z = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
    } else {
        double s = std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1)) * 2.0;
        w = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
        x = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
        y = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
        z = 0.25 * s;
    }
    
    return cv::Vec4d(x, y, z, w);
}

cv::Vec3d ROS2Communication::rotationMatrixToEulerRPY(const cv::Mat& R) {
    // 从旋转矩阵计算欧拉角（ZYX顺序，即Roll-Pitch-Yaw）
    // Roll (绕X轴旋转)
    double roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    
    // Pitch (绕Y轴旋转)
    double pitch = -std::asin(R.at<double>(2, 0));
    
    // Yaw (绕Z轴旋转)
    double yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    
    return cv::Vec3d(roll, pitch, yaw);
}

bool ROS2Communication::loadCartesianPositionJSON(
    const std::string& json_path,
    interface::msg::CartesianPosition& cartesian_pos) {
    
    try {
        std::ifstream file(json_path);
        if (!file.is_open()) {
            RCLCPP_WARN(node_->get_logger(), "无法打开文件: %s", json_path.c_str());
            return false;
        }
        
        Json::Value root;
        Json::Reader reader;
        if (!reader.parse(file, root)) {
            RCLCPP_WARN(node_->get_logger(), "无法解析JSON文件: %s", json_path.c_str());
            file.close();
            return false;
        }
        file.close();
        
        // 读取笛卡尔坐标
        if (root.isMember("cartesian_position")) {
            Json::Value cart_pos = root["cartesian_position"];
            
            // 位置
            if (cart_pos.isMember("position")) {
                Json::Value pos = cart_pos["position"];
                cartesian_pos.position.x = pos.get("x", 0.0).asDouble();
                cartesian_pos.position.y = pos.get("y", 0.0).asDouble();
                cartesian_pos.position.z = pos.get("z", 0.0).asDouble();
            }
            
            // 姿态四元数
            if (cart_pos.isMember("orientation")) {
                Json::Value ori = cart_pos["orientation"];
                cartesian_pos.orientation.x = ori.get("x", 0.0).asDouble();
                cartesian_pos.orientation.y = ori.get("y", 0.0).asDouble();
                cartesian_pos.orientation.z = ori.get("z", 0.0).asDouble();
                cartesian_pos.orientation.w = ori.get("w", 1.0).asDouble();
            }
            
            // 欧拉角（弧度）
            if (cart_pos.isMember("euler_orientation_rpy_rad")) {
                Json::Value euler_rad = cart_pos["euler_orientation_rpy_rad"];
                if (euler_rad.isArray() && euler_rad.size() >= 3) {
                    cartesian_pos.euler_orientation_rpy_rad[0] = euler_rad[0].asDouble();
                    cartesian_pos.euler_orientation_rpy_rad[1] = euler_rad[1].asDouble();
                    cartesian_pos.euler_orientation_rpy_rad[2] = euler_rad[2].asDouble();
                }
            }
            
            // 欧拉角（度）
            if (cart_pos.isMember("euler_orientation_rpy_deg")) {
                Json::Value euler_deg = cart_pos["euler_orientation_rpy_deg"];
                if (euler_deg.isArray() && euler_deg.size() >= 3) {
                    cartesian_pos.euler_orientation_rpy_deg[0] = euler_deg[0].asDouble();
                    cartesian_pos.euler_orientation_rpy_deg[1] = euler_deg[1].asDouble();
                    cartesian_pos.euler_orientation_rpy_deg[2] = euler_deg[2].asDouble();
                }
            }
        }
        
        // 读取关节坐标
        if (root.isMember("joint_position_rad")) {
            Json::Value joint_rad = root["joint_position_rad"];
            if (joint_rad.isArray() && joint_rad.size() >= 6) {
                for (int i = 0; i < 6 && i < static_cast<int>(joint_rad.size()); i++) {
                    cartesian_pos.joint_position_rad[i] = joint_rad[i].asDouble();
                }
            }
        }
        
        if (root.isMember("joint_position_deg")) {
            Json::Value joint_deg = root["joint_position_deg"];
            if (joint_deg.isArray() && joint_deg.size() >= 6) {
                for (int i = 0; i < 6 && i < static_cast<int>(joint_deg.size()); i++) {
                    cartesian_pos.joint_position_deg[i] = joint_deg[i].asDouble();
                }
            }
        }
        
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(), "加载CartesianPosition JSON异常: %s", e.what());
        return false;
    }
}

void ROS2Communication::drawGripperOnImage(
    cv::Mat& image,
    const cv::Mat& T_C_E,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    double gripper_opening_mm,
    double gripper_length_mm) {
    
    if (T_C_E.empty() || camera_matrix.empty()) {
        return;
    }
    
    // 提取末端执行器原点在相机坐标系下的位置
    cv::Point3d ee_pos_camera(
        T_C_E.at<double>(0, 3),
        T_C_E.at<double>(1, 3),
        T_C_E.at<double>(2, 3)
    );
    
    // 提取末端执行器坐标系的三个轴在相机坐标系下的方向
    cv::Vec3d gripper_x_axis(
        T_C_E.at<double>(0, 0),
        T_C_E.at<double>(1, 0),
        T_C_E.at<double>(2, 0)
    );
    
    cv::Vec3d gripper_y_axis(
        T_C_E.at<double>(0, 1),
        T_C_E.at<double>(1, 1),
        T_C_E.at<double>(2, 1)
    );
    
    cv::Vec3d gripper_z_axis(
        T_C_E.at<double>(0, 2),
        T_C_E.at<double>(1, 2),
        T_C_E.at<double>(2, 2)
    );
    
    // 归一化
    double x_norm = cv::norm(gripper_x_axis);
    double y_norm = cv::norm(gripper_y_axis);
    double z_norm = cv::norm(gripper_z_axis);
    if (x_norm > 1e-6) {
        gripper_x_axis = gripper_x_axis / x_norm;
    }
    if (y_norm > 1e-6) {
        gripper_y_axis = gripper_y_axis / y_norm;
    }
    if (z_norm > 1e-6) {
        gripper_z_axis = gripper_z_axis / z_norm;
    }
    
    // 夹爪坐标系定义：Z轴=指向方向，Y轴=开口方向
    cv::Vec3d pointing_direction = gripper_z_axis;
    cv::Vec3d opening_direction = gripper_y_axis;
    
    // 计算夹爪的两个手指位置
    cv::Point3d finger_center = ee_pos_camera;
    
    // 两个手指分别沿着开口方向的正负方向偏移
    cv::Point3d finger1_center = finger_center + cv::Point3d(
        opening_direction[0] * gripper_opening_mm / 2.0,
        opening_direction[1] * gripper_opening_mm / 2.0,
        opening_direction[2] * gripper_opening_mm / 2.0
    );
    
    cv::Point3d finger2_center = finger_center + cv::Point3d(
        -opening_direction[0] * gripper_opening_mm / 2.0,
        -opening_direction[1] * gripper_opening_mm / 2.0,
        -opening_direction[2] * gripper_opening_mm / 2.0
    );
    
    // 手指沿着指向方向延伸
    cv::Point3d finger1_tip = finger1_center + cv::Point3d(
        pointing_direction[0] * gripper_length_mm,
        pointing_direction[1] * gripper_length_mm,
        pointing_direction[2] * gripper_length_mm
    );
    
    cv::Point3d finger2_tip = finger2_center + cv::Point3d(
        pointing_direction[0] * gripper_length_mm,
        pointing_direction[1] * gripper_length_mm,
        pointing_direction[2] * gripper_length_mm
    );
    
    // 将3D点投影到2D像素坐标
    std::vector<cv::Point3d> object_points = {
        finger_center, finger1_center, finger1_tip, finger2_center, finger2_tip
    };
    
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    
    std::vector<cv::Point2d> image_points;
    for (const auto& pt_3d : object_points) {
        if (pt_3d.z > 0) {  // 确保点在相机前方
            double u = fx * pt_3d.x / pt_3d.z + cx;
            double v = fy * pt_3d.y / pt_3d.z + cy;
            image_points.push_back(cv::Point2d(u, v));
        } else {
            image_points.push_back(cv::Point2d(-1, -1));  // 无效点
        }
    }
    
    // 检查点是否有效（在图像范围内）
    cv::Rect image_rect(0, 0, image.cols, image.rows);
    auto is_valid_point = [&image_rect](const cv::Point2d& pt) {
        return pt.x >= 0 && pt.y >= 0 && 
               pt.x < image_rect.width && pt.y < image_rect.height;
    };
    
    // 绘制夹爪（红色）
    cv::Scalar red_color(0, 0, 255);  // BGR格式，红色
    int thickness = 3;
    int circle_radius = 5;
    
    // 绘制中心点
    if (image_points.size() > 0 && is_valid_point(image_points[0])) {
        cv::circle(image, image_points[0], circle_radius, red_color, -1);
    }
    
    // 绘制第一个手指（从中心到指尖）
    if (image_points.size() > 2 && is_valid_point(image_points[1]) && is_valid_point(image_points[2])) {
        cv::line(image, image_points[1], image_points[2], red_color, thickness);
        cv::circle(image, image_points[2], circle_radius, red_color, -1);
    }
    
    // 绘制第二个手指（从中心到指尖）
    if (image_points.size() > 4 && is_valid_point(image_points[3]) && is_valid_point(image_points[4])) {
        cv::line(image, image_points[3], image_points[4], red_color, thickness);
        cv::circle(image, image_points[4], circle_radius, red_color, -1);
    }
    
    // 绘制连接两个手指中心的线
    if (image_points.size() > 3 && is_valid_point(image_points[1]) && is_valid_point(image_points[3])) {
        cv::line(image, image_points[1], image_points[3], red_color, thickness);
    }
}

double ROS2Communication::calculateMaskIoU(const cv::Mat& mask1, const cv::Mat& mask2) {
    if (mask1.empty() || mask2.empty() || mask1.size() != mask2.size()) {
        return 0.0;
    }
    
    // 确保掩膜是二值图像
    cv::Mat m1, m2;
    if (mask1.channels() == 1) {
        // 对于单通道图像，使用阈值127进行二值化（0-127为背景，128-255为前景）
        cv::threshold(mask1, m1, 127, 255, cv::THRESH_BINARY);
        m1 = (m1 > 0);  // 转换为布尔掩膜（0或1）
    } else {
        cv::cvtColor(mask1, m1, cv::COLOR_BGR2GRAY);
        cv::threshold(m1, m1, 127, 255, cv::THRESH_BINARY);
        m1 = (m1 > 0);
    }
    
    if (mask2.channels() == 1) {
        cv::threshold(mask2, m2, 127, 255, cv::THRESH_BINARY);
        m2 = (m2 > 0);  // 转换为布尔掩膜（0或1）
    } else {
        cv::cvtColor(mask2, m2, cv::COLOR_BGR2GRAY);
        cv::threshold(m2, m2, 127, 255, cv::THRESH_BINARY);
        m2 = (m2 > 0);
    }
    
    // 计算交集和并集
    cv::Mat intersection, union_mask;
    cv::bitwise_and(m1, m2, intersection);
    cv::bitwise_or(m1, m2, union_mask);
    
    int intersection_area = cv::countNonZero(intersection);
    int union_area = cv::countNonZero(union_mask);
    
    if (union_area == 0) {
        return 0.0;
    }
    
    return static_cast<double>(intersection_area) / static_cast<double>(union_area);
}

bool ROS2Communication::findBestMaskAlignment(
    const cv::Mat& template_mask,
    const cv::Mat& target_mask,
    const cv::Point2f& target_center,
    double scale,
    double initial_angle_deg,
    double angle_range_deg,
    double angle_step_deg,
    double& best_angle_deg,
    double& best_confidence,
    cv::Mat& best_aligned_mask) {
    
    if (template_mask.empty() || target_mask.empty()) {
        return false;
    }
    
    // 模板掩膜中心（标准化后的掩膜，工件中心在掩膜中心）
    cv::Point2f template_mask_center(
        template_mask.cols / 2.0f,
        template_mask.rows / 2.0f
    );
    
    // 初始化最佳值
    best_confidence = 0.0;
    best_angle_deg = initial_angle_deg;
    best_aligned_mask = cv::Mat();
    
    // 计算搜索角度范围
    double min_angle = initial_angle_deg - angle_range_deg;
    double max_angle = initial_angle_deg + angle_range_deg;
    
    // 遍历所有角度
    for (double angle_deg = min_angle; angle_deg <= max_angle; angle_deg += angle_step_deg) {
        // 创建旋转变换矩阵（以模板掩膜中心为旋转中心）
        cv::Mat transform = cv::getRotationMatrix2D(template_mask_center, angle_deg, scale);
        
        // 计算平移量：目标中心 - 变换后的掩膜中心
        // 由于以掩膜中心为旋转中心，旋转和缩放后掩膜中心位置不变
        transform.at<double>(0, 2) += target_center.x - template_mask_center.x;
        transform.at<double>(1, 2) += target_center.y - template_mask_center.y;
        
        // 变换模板掩膜到目标图像坐标系
        cv::Mat aligned_mask;
        // 使用INTER_NEAREST保持二值掩膜的特性
        cv::warpAffine(template_mask, aligned_mask, transform, target_mask.size(),
                      cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
        // 重新二值化，确保掩膜是纯二值的（0或255）
        cv::threshold(aligned_mask, aligned_mask, 127, 255, cv::THRESH_BINARY);
        
        // 计算重合度（IoU）
        double confidence = calculateMaskIoU(aligned_mask, target_mask);
        
        // 更新最佳匹配
        if (confidence > best_confidence) {
            best_confidence = confidence;
            best_angle_deg = angle_deg;
            best_aligned_mask = aligned_mask.clone();
        }
    }
    
    return best_confidence > 0.0;
}

bool ROS2Communication::bruteForceTemplateMatching(
    const void* templates_ptr,
    const std::string& workpiece_template_dir,
    const cv::Mat& target_mask,
    const cv::Point2f& target_center,
    double target_standardized_angle_rad,
    double target_radius,
    const std::filesystem::path& debug_dir,
    int& best_template_idx,
    double& best_angle_deg,
    double& best_confidence,
    cv::Mat& best_aligned_mask) {
    
    // 转换void*为实际的模板列表类型
    const std::vector<TemplateItem>* templates = 
        static_cast<const std::vector<TemplateItem>*>(templates_ptr);
    
    if (!templates || templates->empty() || target_mask.empty()) {
        return false;
    }
    
    // 为每个模板维护最佳结果（用于调试输出）
    struct TemplateBestResult {
        double best_confidence = 0.0;
        double best_angle_deg = 0.0;
        cv::Mat best_aligned_mask;
        std::mutex mutex;  // 每个模板的互斥锁
    };
    
    std::vector<TemplateBestResult> template_results(templates->size());
    
    // 全局最佳结果（需要互斥锁保护）
    std::mutex global_result_mutex;
    int shared_best_template_idx = -1;
    double shared_best_confidence = 0.0;
    double shared_best_angle_deg = 0.0;
    cv::Mat shared_best_aligned_mask;
    
    // 提前终止标志（用于通过阈值）
    std::atomic<bool> should_early_exit(false);
    
    // 计算初始角度（度）
    // 模板掩膜保持原始角度，阀体在template.standardized_angle位置
    // 目标工件的阀体在target_standardized_angle_rad位置
    // 要让模板掩膜对齐目标工件，需要旋转的角度 = target_standardized_angle_rad - template.standardized_angle
    // 但是我们需要对每个模板分别计算，所以这里先使用目标角度作为初始搜索角度
    // 实际匹配时会在每个模板的角度范围内搜索
    double initial_angle_deg = target_standardized_angle_rad * 180.0 / M_PI;
    
    // 计算搜索角度范围
    double angle_step = brute_force_angle_step_deg_;
    double search_range = 180.0;  // 搜索范围：±180度
    double min_angle = initial_angle_deg - search_range;
    double max_angle = initial_angle_deg + search_range;
    
    // 确定线程数（不超过配置的最大线程数）
    size_t num_templates = templates->size();
    size_t total_threads = static_cast<size_t>(brute_force_max_threads_);
    total_threads = std::max(total_threads, size_t(1));  // 至少1个线程
    
    // 计算每个模板分配的线程数
    // 例如：6个模板，18个线程 -> 每个模板分配 18/6 = 3个线程
    size_t threads_per_template = total_threads / num_templates;
    threads_per_template = std::max(threads_per_template, size_t(1));  // 每个模板至少1个线程
    
    // 计算剩余线程（如果不能整除）
    size_t remaining_threads = total_threads - (threads_per_template * num_templates);
    
    // 计算需要搜索的角度数量
    int num_angles = static_cast<int>((max_angle - min_angle) / angle_step) + 1;
    
    // 输出线程分配信息
    RCLCPP_INFO(node_->get_logger(), "暴力匹配：总共 %zu 个线程，%zu 个模板，每个模板分配 %zu 个线程", 
               total_threads, num_templates, threads_per_template);
    if (remaining_threads > 0) {
        RCLCPP_INFO(node_->get_logger(), "剩余 %zu 个线程将分配给前 %zu 个模板", remaining_threads, remaining_threads);
    }
    RCLCPP_INFO(node_->get_logger(), "每个模板需要搜索 %d 个角度（范围: %.2f度 到 %.2f度，步进: %.2f度）", 
               num_angles, min_angle, max_angle, angle_step);
    
    // 创建线程处理模板匹配
    std::vector<std::thread> threads;
    size_t thread_counter = 0;  // 全局线程计数器
    
    // 遍历所有模板
    for (size_t template_idx = 0; template_idx < num_templates; ++template_idx) {
        const auto& tmpl = (*templates)[template_idx];
        
        // 加载模板掩膜（在主线程中加载，避免多线程重复加载）
        std::filesystem::path template_dir = std::filesystem::path(workpiece_template_dir) / tmpl.id;
        std::filesystem::path template_mask_path = template_dir / "mask.jpg";
        
        if (!std::filesystem::exists(template_mask_path)) {
            continue;
        }
        
        cv::Mat template_mask = cv::imread(template_mask_path.string(), cv::IMREAD_GRAYSCALE);
        if (template_mask.empty()) {
            continue;
        }
        
        // 模板掩膜中心（标准化后的掩膜，工件中心在掩膜中心）
        cv::Point2f template_mask_center(
            template_mask.cols / 2.0f,
            template_mask.rows / 2.0f
        );
        
        // 步骤1: 对齐中心（已完成）
        // 构建与目标图像相同尺寸的空白图像，将模板中心放置在与目标中心对应的位置
        cv::Mat template_mask_aligned = cv::Mat::zeros(target_mask.size(), CV_8UC1);
        cv::Point2f target_center_float(target_center.x, target_center.y);
        
        // 将模板掩膜平移到目标中心位置（对齐中心）
        cv::Mat translate_transform = cv::Mat::eye(2, 3, CV_32F);
        translate_transform.at<float>(0, 2) = target_center_float.x - template_mask_center.x;
        translate_transform.at<float>(1, 2) = target_center_float.y - template_mask_center.y;
        // 使用INTER_NEAREST保持二值掩膜的特性，避免线性插值产生中间灰度值
        cv::warpAffine(template_mask, template_mask_aligned, translate_transform, target_mask.size(),
                      cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
        // 重新二值化，确保掩膜是纯二值的（0或255）
        cv::threshold(template_mask_aligned, template_mask_aligned, 127, 255, cv::THRESH_BINARY);
        
        // 步骤2: 取模板外接圆或工件外接圆大的一方，使用大的那个尺寸裁剪模板和工件区域掩膜
        float template_radius = tmpl.feature.workpiece_radius;
        float target_radius_float = static_cast<float>(target_radius);
        float crop_radius = std::max(template_radius, target_radius_float);
        
        // 计算裁剪区域（以目标中心为中心的正方形）
        int crop_size = static_cast<int>(std::ceil(crop_radius * 2.0f));
        int crop_x = static_cast<int>(std::max(0.0f, target_center.x - crop_radius));
        int crop_y = static_cast<int>(std::max(0.0f, target_center.y - crop_radius));
        int crop_w = std::min(crop_size, target_mask.cols - crop_x);
        int crop_h = std::min(crop_size, target_mask.rows - crop_y);
        
        // 检查裁剪区域是否有效
        if (crop_w <= 0 || crop_h <= 0 || 
            crop_x < 0 || crop_y < 0 ||
            crop_x + crop_w > target_mask.cols ||
            crop_y + crop_h > target_mask.rows) {
            RCLCPP_WARN(node_->get_logger(), "模板 %s: 裁剪区域无效，跳过", tmpl.id.c_str());
            continue;
        }
        
        cv::Rect crop_rect(crop_x, crop_y, crop_w, crop_h);
        cv::Mat target_mask_cropped = target_mask(crop_rect).clone();
        cv::Mat template_mask_cropped = template_mask_aligned(crop_rect).clone();
        
        // 检查裁剪后的掩膜是否有效
        if (target_mask_cropped.empty() || template_mask_cropped.empty() ||
            target_mask_cropped.cols <= 0 || target_mask_cropped.rows <= 0 ||
            template_mask_cropped.cols <= 0 || template_mask_cropped.rows <= 0) {
            RCLCPP_WARN(node_->get_logger(), "模板 %s: 裁剪后的掩膜无效，跳过", tmpl.id.c_str());
            continue;
        }
        
        // 计算目标中心在裁剪区域内的相对位置（相对于裁剪区域的左上角）
        // 这是旋转的中心点，而不是裁剪区域的中心
        cv::Point2f target_center_in_crop(
            target_center.x - crop_x,  // 目标中心在裁剪区域内的x坐标
            target_center.y - crop_y   // 目标中心在裁剪区域内的y坐标
        );
        
        // 保存裁剪后的掩膜到debug文件夹
        if (debug_) {
            std::filesystem::path template_debug_dir = debug_dir / ("template_" + tmpl.id);
            std::filesystem::create_directories(template_debug_dir);
            
            std::filesystem::path target_cropped_path = template_debug_dir / "target_mask_cropped.jpg";
            std::filesystem::path template_cropped_path = template_debug_dir / "template_mask_cropped.jpg";
            cv::imwrite(target_cropped_path.string(), target_mask_cropped);
            cv::imwrite(template_cropped_path.string(), template_mask_cropped);
        }
        
        // 步骤3: 对裁剪后的图像进行缩放
        cv::Size scaled_size(
            static_cast<int>(std::round(target_mask_cropped.cols * brute_force_angle_matching_scale_)),
            static_cast<int>(std::round(target_mask_cropped.rows * brute_force_angle_matching_scale_))
        );
        
        // 确保缩放后的尺寸至少为1
        scaled_size.width = std::max(1, scaled_size.width);
        scaled_size.height = std::max(1, scaled_size.height);
        
        // 计算缩放后的旋转中心（目标中心在缩放后掩膜中的位置）
        cv::Point2f scaled_center(
            target_center_in_crop.x * brute_force_angle_matching_scale_,
            target_center_in_crop.y * brute_force_angle_matching_scale_
        );
        
        cv::Mat template_mask_scaled, target_mask_scaled;
        if (std::abs(brute_force_angle_matching_scale_ - 1.0) < 1e-6) {
            // scale=1.0时，不进行resize（避免插值误差），直接使用裁剪后的掩膜
            template_mask_scaled = template_mask_cropped;
            target_mask_scaled = target_mask_cropped;
            // scaled_center 就是 target_center_in_crop
            scaled_center = target_center_in_crop;
        } else {
            // 缩小模式：将两个掩膜都缩放到相同尺寸
            cv::resize(template_mask_cropped, template_mask_scaled, scaled_size, 0, 0, cv::INTER_NEAREST);
            cv::resize(target_mask_cropped, target_mask_scaled, scaled_size, 0, 0, cv::INTER_NEAREST);
            // 重新二值化，确保掩膜是纯二值的
            cv::threshold(template_mask_scaled, template_mask_scaled, 127, 255, cv::THRESH_BINARY);
            cv::threshold(target_mask_scaled, target_mask_scaled, 127, 255, cv::THRESH_BINARY);
        }
        
        // 检查缩放后的掩膜是否有效
        if (template_mask_scaled.empty() || target_mask_scaled.empty() ||
            template_mask_scaled.cols <= 0 || template_mask_scaled.rows <= 0 ||
            target_mask_scaled.cols <= 0 || target_mask_scaled.rows <= 0) {
            RCLCPP_WARN(node_->get_logger(), "模板 %s: 缩放后的掩膜无效，跳过", tmpl.id.c_str());
            continue;
        }
        
        // 确保旋转中心在掩膜范围内
        scaled_center.x = std::clamp(scaled_center.x, 0.0f, static_cast<float>(template_mask_scaled.cols - 1));
        scaled_center.y = std::clamp(scaled_center.y, 0.0f, static_cast<float>(template_mask_scaled.rows - 1));
        
        // 原始模板掩膜的中心（用于在原始尺寸上重新计算对齐掩膜）
        cv::Point2f original_template_mask_center(
            template_mask.cols / 2.0f,
            template_mask.rows / 2.0f
        );
        
        // 确定当前模板分配的线程数（前remaining_threads个模板多分配1个线程）
        size_t current_template_threads = threads_per_template;
        if (template_idx < remaining_threads) {
            current_template_threads += 1;
        }
        
        // 注意：每个线程处理的角度范围会在下面根据模板的标准化角度单独计算
        
        // 计算当前模板的标准化角度
        double template_standardized_angle_rad = tmpl.feature.standardized_angle;
        double template_standardized_angle_deg = template_standardized_angle_rad * 180.0 / M_PI;
        
        // 计算相对角度：目标角度 - 模板角度
        // 这样可以让模板掩膜对齐目标工件
        double relative_angle_deg = initial_angle_deg - template_standardized_angle_deg;
        
        // 调整搜索范围：以相对角度为中心，±180度范围
        double template_min_angle = relative_angle_deg - search_range;
        double template_max_angle = relative_angle_deg + search_range;
        
        // 计算当前模板需要搜索的角度数量
        int template_num_angles = static_cast<int>((template_max_angle - template_min_angle) / angle_step) + 1;
        
        // 重新计算每个线程处理的角度数量
        size_t template_angles_per_thread = (template_num_angles + current_template_threads - 1) / current_template_threads;
        
        if (debug_) {
            RCLCPP_DEBUG(node_->get_logger(), "模板 %zu (%s): 标准化角度=%.2f度, 相对角度=%.2f度, 搜索范围[%.2f, %.2f]度", 
                        template_idx, tmpl.id.c_str(), template_standardized_angle_deg, 
                        relative_angle_deg, template_min_angle, template_max_angle);
        }
        
        // 为当前模板创建线程处理角度搜索
        for (size_t thread_in_template = 0; thread_in_template < current_template_threads; ++thread_in_template) {
            // 计算当前线程处理的角度范围
            int angle_start_idx = static_cast<int>(thread_in_template * template_angles_per_thread);
            int angle_end_idx = std::min(angle_start_idx + static_cast<int>(template_angles_per_thread), template_num_angles);
            
            if (angle_start_idx >= template_num_angles) {
                break;
            }
            
            // 计算实际的角度范围（度）
            double thread_min_angle = template_min_angle + angle_start_idx * angle_step;
            double thread_max_angle = template_min_angle + (angle_end_idx - 1) * angle_step;
            
            // 创建线程处理分配的角度范围
            size_t current_thread_id = thread_counter++;
            threads.emplace_back([&, template_idx, thread_min_angle, thread_max_angle, angle_step, 
                                 template_mask_scaled, target_mask_scaled, scaled_center,
                                 template_mask, target_mask, target_center, original_template_mask_center,
                                 current_thread_id, thread_in_template]() {
                // 线程局部最佳结果（仅针对当前模板）
                double local_best_confidence = 0.0;
                double local_best_angle_deg = 0.0;
                
                // 处理分配的角度范围
                for (double angle_deg = thread_min_angle; angle_deg <= thread_max_angle; angle_deg += angle_step) {
                    // 检查是否需要提前退出（如果其他线程已经找到满足通过阈值的结果）
                    if (should_early_exit.load()) {
                        break;
                    }
                    
                    // 归一化角度到0-360度范围
                    double normalized_angle = angle_deg;
                    while (normalized_angle < 0.0) normalized_angle += 360.0;
                    while (normalized_angle >= 360.0) normalized_angle -= 360.0;
                    
                    // 步骤4: 让掩膜绕缩放后的图像中心旋转，计算IOU
                    // 创建旋转变换矩阵（以缩放后掩膜中心为旋转中心）
                    if (template_mask_scaled.empty() || template_mask_scaled.cols <= 0 || template_mask_scaled.rows <= 0 ||
                        target_mask_scaled.empty() || target_mask_scaled.cols <= 0 || target_mask_scaled.rows <= 0) {
                        continue;
                    }
                    
                    cv::Mat transform = cv::getRotationMatrix2D(scaled_center, normalized_angle, 1.0);
                    cv::Mat rotated_template;
                    // 使用INTER_NEAREST保持二值掩膜的特性，避免线性插值产生中间灰度值
                    cv::warpAffine(template_mask_scaled, rotated_template, transform, target_mask_scaled.size(),
                                  cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
                    // 重新二值化，确保掩膜是纯二值的（0或255）
                    cv::threshold(rotated_template, rotated_template, 127, 255, cv::THRESH_BINARY);
                    
                    // 计算重合度（IoU）
                    double confidence = calculateMaskIoU(rotated_template, target_mask_scaled);
                    
                    // Debug: 检查掩膜尺寸（仅第一次）
                    if (debug_ && template_idx == 0 && thread_in_template == 0 && 
                        std::abs(angle_deg - thread_min_angle) < angle_step) {
                        RCLCPP_DEBUG(node_->get_logger(), 
                                    "原始模板掩膜尺寸: %dx%d, 原始目标掩膜尺寸: %dx%d",
                                    template_mask.cols, template_mask.rows,
                                    target_mask.cols, target_mask.rows);
                        RCLCPP_DEBUG(node_->get_logger(), 
                                    "缩小后模板掩膜尺寸: %dx%d, 缩小后目标掩膜尺寸: %dx%d, 缩小比例: %.2f",
                                    template_mask_scaled.cols, template_mask_scaled.rows,
                                    target_mask_scaled.cols, target_mask_scaled.rows,
                                    brute_force_angle_matching_scale_);
                    }
                    
                    // 检查是否达到通过阈值（提前终止条件）
                    if (confidence >= brute_force_acceptance_threshold_) {
                        // 设置提前退出标志（原子操作，确保线程安全）
                        bool expected = false;
                        if (should_early_exit.compare_exchange_strong(expected, true)) {
                            // 在原始尺寸上重新计算对齐掩膜（用于返回结果）
                            // 使用归一化后的角度进行旋转（OpenCV的getRotationMatrix2D需要0-360度）
                            cv::Mat transform_full = cv::getRotationMatrix2D(original_template_mask_center, normalized_angle, 1.0);
                            transform_full.at<double>(0, 2) += target_center.x - original_template_mask_center.x;
                            transform_full.at<double>(1, 2) += target_center.y - original_template_mask_center.y;
                            cv::Mat aligned_mask_full;
                            // 使用INTER_NEAREST保持二值掩膜的特性
                            cv::warpAffine(template_mask, aligned_mask_full, transform_full, target_mask.size(),
                                          cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
                            // 重新二值化，确保掩膜是纯二值的（0或255）
                            cv::threshold(aligned_mask_full, aligned_mask_full, 127, 255, cv::THRESH_BINARY);
                            
                            // 当前线程成功设置了标志，立即更新全局结果并退出
                            {
                                std::lock_guard<std::mutex> lock(global_result_mutex);
                                shared_best_confidence = confidence;
                                // 保存原始相对角度（angle_deg），而不是归一化后的角度
                                // 这样在计算dtheta时才能正确表示相对旋转
                                shared_best_angle_deg = angle_deg;
                                shared_best_template_idx = static_cast<int>(template_idx);
                                shared_best_aligned_mask = aligned_mask_full.clone();
                            }
                            RCLCPP_INFO(node_->get_logger(), 
                                       "达到通过阈值 (%.2f >= %.2f)，提前终止匹配", 
                                       confidence, brute_force_acceptance_threshold_);
                            break;
                        }
                        // 如果其他线程已经设置了标志，当前线程只更新结果，然后退出循环
                    }
                    
                    // 更新线程局部最佳匹配
                    if (confidence > local_best_confidence) {
                        local_best_confidence = confidence;
                        // 保存原始相对角度（angle_deg），而不是归一化后的角度
                        // 这样在计算dtheta时才能正确表示相对旋转
                        local_best_angle_deg = angle_deg;
                        // 注意：这里只保存角度，不保存缩小后的掩膜，最终掩膜会在原始尺寸上重新计算
                    }
                }
                
                // 更新模板局部最佳结果和全局最佳结果（需要加锁）
                if (local_best_confidence > 0.0) {
                    // 在原始尺寸上重新计算最佳对齐掩膜
                    // local_best_angle_deg 是原始相对角度，需要归一化到 0-360 度用于旋转
                    double normalized_best_angle = local_best_angle_deg;
                    while (normalized_best_angle < 0.0) normalized_best_angle += 360.0;
                    while (normalized_best_angle >= 360.0) normalized_best_angle -= 360.0;
                    
                    cv::Mat transform_full = cv::getRotationMatrix2D(original_template_mask_center, normalized_best_angle, 1.0);
                    transform_full.at<double>(0, 2) += target_center.x - original_template_mask_center.x;
                    transform_full.at<double>(1, 2) += target_center.y - original_template_mask_center.y;
                    cv::Mat aligned_mask_full;
                    // 使用INTER_NEAREST保持二值掩膜的特性
                    cv::warpAffine(template_mask, aligned_mask_full, transform_full, target_mask.size(),
                                  cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(0));
                    // 重新二值化，确保掩膜是纯二值的（0或255）
                    cv::threshold(aligned_mask_full, aligned_mask_full, 127, 255, cv::THRESH_BINARY);
                    
                    // 总是更新当前模板的最佳结果（用于调试输出）
                    {
                        std::lock_guard<std::mutex> lock(template_results[template_idx].mutex);
                        if (local_best_confidence > template_results[template_idx].best_confidence) {
                            template_results[template_idx].best_confidence = local_best_confidence;
                            // 保存原始相对角度，用于后续计算dtheta
                            template_results[template_idx].best_angle_deg = local_best_angle_deg;
                            template_results[template_idx].best_aligned_mask = aligned_mask_full.clone();
                        }
                    }
                    
                    // 如果还没有提前退出，更新全局最佳结果
                    // 如果已经提前退出，说明其他线程已经找到了满足通过阈值的结果，不需要更新
                    if (!should_early_exit.load()) {
                        std::lock_guard<std::mutex> lock(global_result_mutex);
                        // 再次检查（可能在获取锁的过程中，其他线程已经设置了退出标志）
                        if (!should_early_exit.load() && local_best_confidence > shared_best_confidence) {
                            shared_best_confidence = local_best_confidence;
                            shared_best_angle_deg = local_best_angle_deg;
                            shared_best_template_idx = static_cast<int>(template_idx);
                            shared_best_aligned_mask = aligned_mask_full.clone();
                            
                            // 检查是否达到通过阈值（可能在其他线程中达到）
                            if (shared_best_confidence >= brute_force_acceptance_threshold_) {
                                should_early_exit.store(true);
                                RCLCPP_INFO(node_->get_logger(), 
                                           "达到通过阈值 (%.2f >= %.2f)，设置提前终止标志", 
                                           shared_best_confidence, brute_force_acceptance_threshold_);
                            }
                        }
                    }
                }
            });
        }
    }
    
    // 等待所有线程完成
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    // 打印每个模板的最佳结果（用于调试）
    RCLCPP_INFO(node_->get_logger(), "========== 暴力匹配结果汇总 ==========");
    for (size_t i = 0; i < templates->size(); ++i) {
        const auto& tmpl = (*templates)[i];
        const auto& result = template_results[i];
        
        if (result.best_confidence > 0.0) {
            RCLCPP_INFO(node_->get_logger(), 
                       "姿态ID: %s | 最高重叠度: %.4f | 最佳角度: %.2f度", 
                       tmpl.id.c_str(), result.best_confidence, result.best_angle_deg);
        } else {
            RCLCPP_INFO(node_->get_logger(), 
                       "姿态ID: %s | 最高重叠度: 0.0000 | 最佳角度: N/A (未找到匹配)", 
                       tmpl.id.c_str());
        }
    }
    
    // 输出全局最佳结果
    if (shared_best_template_idx >= 0 && shared_best_template_idx < static_cast<int>(templates->size())) {
        const auto& best_tmpl = (*templates)[shared_best_template_idx];
        RCLCPP_INFO(node_->get_logger(), "========== 全局最佳匹配 ==========");
        RCLCPP_INFO(node_->get_logger(), 
                   "最佳姿态ID: %s | 最高重叠度: %.4f | 最佳角度: %.2f度", 
                   best_tmpl.id.c_str(), shared_best_confidence, shared_best_angle_deg);
    } else {
        RCLCPP_WARN(node_->get_logger(), "未找到任何匹配的模板");
    }
    
    // 检查是否找到有效匹配
    if (shared_best_template_idx < 0 || shared_best_template_idx >= static_cast<int>(templates->size())) {
        RCLCPP_WARN(node_->get_logger(), "未找到任何匹配的模板");
        return false;
    }
    
    // 检查舍弃阈值：如果最佳置信度低于舍弃阈值，则认为匹配失败
    if (shared_best_confidence < brute_force_rejection_threshold_) {
        RCLCPP_WARN(node_->get_logger(), 
                   "最佳匹配置信度 %.4f 低于舍弃阈值 %.4f，判定为匹配失败", 
                   shared_best_confidence, brute_force_rejection_threshold_);
        return false;
    }
    
    // 设置输出结果
    best_template_idx = shared_best_template_idx;
    best_confidence = shared_best_confidence;
    best_angle_deg = shared_best_angle_deg;
    best_aligned_mask = shared_best_aligned_mask;
    
    return true;
}

void ROS2Communication::handleProcessDebugStep(
    const std::shared_ptr<interface::srv::ProcessDebugStep::Request> request,
    std::shared_ptr<interface::srv::ProcessDebugStep::Response> response) {
    
    RCLCPP_INFO(node_->get_logger(), "收到调试步骤处理请求: step_id=%s", request->step_id.c_str());
    
    // 算法实现时将完善此部分
    response->success = false;
    response->error_message = "算法未实现";
}

void ROS2Communication::handleListTemplates(
    const std::shared_ptr<interface::srv::ListTemplates::Request> request,
    std::shared_ptr<interface::srv::ListTemplates::Response> response) {
    
    RCLCPP_INFO(node_->get_logger(), "收到模板列表请求");
    
    // 算法实现时将完善此部分
    response->success = false;
    response->error_message = "算法未实现";
    // ListTemplates响应中没有count字段，只有template_ids等数组
}

void ROS2Communication::handleVisualizeGraspPose(
    const std::shared_ptr<interface::srv::VisualizeGraspPose::Request> request,
    std::shared_ptr<interface::srv::VisualizeGraspPose::Response> response) {
    
    RCLCPP_INFO(node_->get_logger(), "收到可视化抓取姿态请求");
    
    // 算法实现时将完善此部分
    response->success = false;
    response->error_message = "算法未实现";
}

void ROS2Communication::handleStandardizeTemplate(
    const std::shared_ptr<interface::srv::StandardizeTemplate::Request> request,
    std::shared_ptr<interface::srv::StandardizeTemplate::Response> response) {
    
    RCLCPP_INFO(node_->get_logger(), "收到模板标准化请求: workpiece_id=%s", request->workpiece_id.c_str());
    
    try {
        if (!template_standardizer_) {
            response->success = false;
            response->error_message = "模板标准化器未初始化";
            response->processed_count = 0;
            response->skipped_count = 0;
            return;
        }
        
        // 使用节点中保存的debug标志位
        // 调用模板标准化器
        std::vector<StandardizationResult> results = template_standardizer_->standardizeTemplate(
            template_root_, request->workpiece_id, calib_file_, debug_);
        
        // 统计结果
        int processed_count = 0;
        int skipped_count = 0;
        std::vector<std::string> processed_pose_ids;
        std::vector<std::string> skipped_pose_ids;
        
        for (const auto& result : results) {
            if (result.success) {
                processed_count++;
                processed_pose_ids.push_back(result.pose_id);
            } else {
                skipped_count++;
                skipped_pose_ids.push_back(result.pose_id);
                RCLCPP_WARN(node_->get_logger(), "姿态 %s 标准化失败: %s", 
                           result.pose_id.c_str(), result.error_message.c_str());
            }
        }
        
        // 设置响应
        response->success = (processed_count > 0);
        response->processed_count = processed_count;
        response->skipped_count = skipped_count;
        response->processed_pose_ids = processed_pose_ids;
        response->skipped_pose_ids = skipped_pose_ids;
        
        if (processed_count == 0 && skipped_count > 0) {
            response->error_message = "所有姿态标准化失败";
        } else if (skipped_count > 0) {
            response->error_message = "部分姿态标准化失败";
        } else {
            response->error_message = "";
        }
        
        RCLCPP_INFO(node_->get_logger(), "模板标准化完成: 处理 %d 个姿态, 跳过 %d 个姿态", 
                   processed_count, skipped_count);
        
    } catch (const std::exception& e) {
        response->success = false;
        response->error_message = "模板标准化异常: " + std::string(e.what());
        response->processed_count = 0;
        response->skipped_count = 0;
        RCLCPP_ERROR(node_->get_logger(), "%s", response->error_message.c_str());
    }
}

} // namespace visual_pose_estimation

