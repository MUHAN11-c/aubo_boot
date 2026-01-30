// 模板标准化器实现
// 负责将模板图像标准化（裁剪、旋转），并计算标准化后的抓取姿态

#include "visual_pose_estimation/template_standardizer.hpp"
#include "visual_pose_estimation/preprocessor.hpp"
#include "visual_pose_estimation/feature_extractor.hpp"
#include <fstream>
#include <sstream>
#include <regex>
#include <cmath>
#include <jsoncpp/json/json.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace visual_pose_estimation {

// 构造函数：初始化模板标准化器
// @param preprocessor 预处理器实例
// @param feature_extractor 特征提取器实例
TemplateStandardizer::TemplateStandardizer(
    std::shared_ptr<Preprocessor> preprocessor,
    std::shared_ptr<FeatureExtractor> feature_extractor)
    : preprocessor_(preprocessor), feature_extractor_(feature_extractor) {
}

// 标准化模板：处理指定工件的所有姿态模板
// @param template_root 模板根目录
// @param workpiece_id 工件ID
// @param calib_file 标定文件路径
// @param debug 是否启用调试模式
// @return 标准化结果列表
std::vector<StandardizationResult> TemplateStandardizer::standardizeTemplate(
    const std::string& template_root,
    const std::string& workpiece_id,
    const std::string& calib_file,
    bool debug) {
    
    std::vector<StandardizationResult> results;
    
    // 构建工件目录路径
    std::filesystem::path workpiece_dir = std::filesystem::path(template_root) / workpiece_id;
    
    if (!std::filesystem::exists(workpiece_dir)) {
        StandardizationResult error_result;
        error_result.success = false;
        error_result.error_message = "工件文件夹不存在: " + workpiece_dir.string();
        results.push_back(error_result);
        return results;
    }
    
    // 遍历所有pose_*目录
    for (const auto& entry : std::filesystem::directory_iterator(workpiece_dir)) {
        if (!entry.is_directory()) {
            continue;
        }
        
        std::string pose_dir_name = entry.path().filename().string();
        if (pose_dir_name.find("pose_") != 0) {
            continue;
        }
        
        std::string pose_id = pose_dir_name.substr(5);  // 去掉"pose_"前缀
        std::filesystem::path pose_dir = entry.path();
        
        // 标准化单个姿态
        StandardizationResult result = standardizePose(pose_dir, pose_id, calib_file, debug);
        results.push_back(result);
    }
    
    return results;
}

// 标准化单个姿态：处理单个pose目录的模板
// @param pose_dir 姿态目录路径
// @param pose_id 姿态ID
// @param calib_file 标定文件路径
// @param debug 是否启用调试模式
// @return 标准化结果
StandardizationResult TemplateStandardizer::standardizePose(
    const std::filesystem::path& pose_dir,
    const std::string& pose_id,
    const std::string& calib_file,
    bool debug) {
    
    StandardizationResult result;
    result.pose_id = pose_id;
    
    try {
        // 1. 检查必要文件
        std::filesystem::path original_image_path = pose_dir / "original_image.jpg";
        std::filesystem::path grab_position_path = pose_dir / "grab_position.json";
        
        if (!std::filesystem::exists(original_image_path)) {
            result.success = false;
            result.error_message = "缺少original_image.jpg";
            return result;
        }
        
        if (!std::filesystem::exists(grab_position_path)) {
            result.success = false;
            result.error_message = "缺少grab_position.json";
            return result;
        }
        
        // 2. 读取原始图像
        cv::Mat original_image = cv::imread(original_image_path.string(), cv::IMREAD_COLOR);
        if (original_image.empty()) {
            result.success = false;
            result.error_message = "无法读取原始图像";
            return result;
        }
        
        // 创建debug目录（如果启用debug模式）
        std::filesystem::path debug_dir;
        if (debug) {
            debug_dir = pose_dir / "debug";
            std::filesystem::create_directories(debug_dir);
            
            // 保存原始图像
            std::filesystem::path original_debug_path = debug_dir / "00_original_image.jpg";
            cv::imwrite(original_debug_path.string(), original_image);
        }
        
        // 3. 加载手眼标定参数
        // 优先使用传入的calib_file，如果为空则从pose_dir查找
        std::filesystem::path calibration_path;
        if (!calib_file.empty() && std::filesystem::exists(calib_file)) {
            calibration_path = calib_file;
        } else {
            // 回退到从pose_dir查找
            calibration_path = pose_dir.parent_path() / "hand_eye_calibration.xml";
        }
        
        if (!std::filesystem::exists(calibration_path)) {
            result.success = false;
            result.error_message = "缺少hand_eye_calibration.xml";
            return result;
        }
        
        cv::Mat camera_matrix, dist_coeffs, T_E_C;
        if (!loadHandEyeCalibration(calibration_path.string(), camera_matrix, dist_coeffs, T_E_C)) {
            result.success = false;
            result.error_message = "无法加载手眼标定参数";
            return result;
        }
        
        // 4. 预处理和特征提取
        std::vector<cv::Mat> components = preprocessor_->preprocess(original_image);
        if (components.empty()) {
            result.success = false;
            result.error_message = "预处理失败：未找到连通域";
            return result;
        }
        
        // 保存所有连通域（debug模式）
        if (debug) {
            for (size_t i = 0; i < components.size(); ++i) {
                std::filesystem::path component_path = debug_dir / ("01_component_" + std::to_string(i) + ".jpg");
                cv::imwrite(component_path.string(), components[i]);
            }
        }
        
        // 选择最大的连通域（通常是工件）
        cv::Mat largest_component = components[0];
        int max_area = cv::countNonZero(largest_component);
        size_t largest_idx = 0;
        for (size_t i = 0; i < components.size(); ++i) {
            int area = cv::countNonZero(components[i]);
            if (area > max_area) {
                max_area = area;
                largest_component = components[i];
                largest_idx = i;
            }
        }
        
        // 保存选中的最大连通域（debug模式）
        if (debug) {
            std::filesystem::path selected_component_path = debug_dir / "02_selected_largest_component.jpg";
            cv::imwrite(selected_component_path.string(), largest_component);
        }
        
        // 5. 提取特征
        std::vector<cv::Mat> single_component = {largest_component};
        std::vector<ComponentFeature> features = feature_extractor_->extractFeatures(single_component);
        
        if (features.empty()) {
            result.success = false;
            result.error_message = "特征提取失败";
            return result;
        }
        
        const ComponentFeature& feature = features[0];
        
        // 保存特征提取可视化（debug模式）
        if (debug) {
            // 在原始图像上绘制工件外接圆和阀体外接圆
            cv::Mat feature_vis = original_image.clone();
            
            // 绘制工件外接圆（绿色）
            cv::circle(feature_vis, feature.workpiece_center, 
                      static_cast<int>(feature.workpiece_radius), 
                      cv::Scalar(0, 255, 0), 3);
            cv::circle(feature_vis, feature.workpiece_center, 5, cv::Scalar(0, 255, 0), -1);
            
            // 绘制阀体外接圆（蓝色）
            if (feature.valve_radius > 0) {
                cv::circle(feature_vis, feature.valve_center, 
                          static_cast<int>(feature.valve_radius), 
                          cv::Scalar(255, 0, 0), 3);
                cv::circle(feature_vis, feature.valve_center, 5, cv::Scalar(255, 0, 0), -1);
                
                // 绘制两圆心连线（黄色）
                cv::line(feature_vis, feature.workpiece_center, feature.valve_center, 
                        cv::Scalar(0, 255, 255), 2);
            }
            
            std::filesystem::path feature_vis_path = debug_dir / "03_feature_extraction_visualization.jpg";
            cv::imwrite(feature_vis_path.string(), feature_vis);
        }
        
        // 6. 裁剪和旋转图像（生成标准化图像）
        // 计算裁剪区域（以工件外接圆为中心的正方形）
        float radius = feature.workpiece_radius;
        cv::Point2f center = feature.workpiece_center;
        
        int crop_size = static_cast<int>(std::ceil(radius * 2.0f));
        int crop_x = static_cast<int>(std::max(0.0f, center.x - radius));
        int crop_y = static_cast<int>(std::max(0.0f, center.y - radius));
        int crop_w = std::min(crop_size, original_image.cols - crop_x);
        int crop_h = std::min(crop_size, original_image.rows - crop_y);
        
        cv::Rect crop_rect(crop_x, crop_y, crop_w, crop_h);
        cv::Mat cropped_mask = largest_component(crop_rect).clone();
        
        // 使用掩膜抠图工件，背景用白色填充
        cv::Mat cropped_image = cv::Mat(crop_h, crop_w, CV_8UC3, cv::Scalar(255, 255, 255));  // 白色背景（BGR格式）
        cv::Mat cropped_original = original_image(crop_rect).clone();
        cropped_original.copyTo(cropped_image, cropped_mask);  // 使用掩膜复制工件部分
        
        // 保存裁剪后的图像（debug模式）
        if (debug) {
            std::filesystem::path cropped_image_path = debug_dir / "04_cropped_image.jpg";
            std::filesystem::path cropped_mask_path = debug_dir / "04_cropped_mask.jpg";
            cv::imwrite(cropped_image_path.string(), cropped_image);
            cv::imwrite(cropped_mask_path.string(), cropped_mask);
            
            // 在裁剪后的图像上绘制特征
            cv::Mat cropped_vis = cropped_image.clone();
            cv::Point2f cropped_center(center.x - crop_x, center.y - crop_y);
            cv::Point2f cropped_valve_center(feature.valve_center.x - crop_x, feature.valve_center.y - crop_y);
            
            cv::circle(cropped_vis, cropped_center, static_cast<int>(radius), 
                      cv::Scalar(0, 255, 0), 2);
            cv::circle(cropped_vis, cropped_center, 5, cv::Scalar(0, 255, 0), -1);
            
            if (feature.valve_radius > 0) {
                cv::circle(cropped_vis, cropped_valve_center, 
                          static_cast<int>(feature.valve_radius), 
                          cv::Scalar(255, 0, 0), 2);
                cv::circle(cropped_vis, cropped_valve_center, 5, cv::Scalar(255, 0, 0), -1);
                cv::line(cropped_vis, cropped_center, cropped_valve_center, 
                        cv::Scalar(0, 255, 255), 2);
            }
            
            std::filesystem::path cropped_vis_path = debug_dir / "04_cropped_visualization.jpg";
            cv::imwrite(cropped_vis_path.string(), cropped_vis);
        }
        
        // 计算裁剪后图像中的工件中心
        cv::Point2f cropped_center(center.x - crop_x, center.y - crop_y);
        
        // 不再旋转图像，保持原始角度
        // 直接使用裁剪后的图像和掩膜作为标准化结果
        cv::Mat rotated_image = cropped_image.clone();
        cv::Mat rotated_mask = cropped_mask.clone();
        
        // 旋转角度设为0（不旋转）
        double rotation_angle = 0.0;
        
        // 保存标准化后的图像（debug模式）
        if (debug) {
            std::filesystem::path rotated_image_path = debug_dir / "05_standardized_image.jpg";
            std::filesystem::path rotated_mask_path = debug_dir / "05_standardized_mask.jpg";
            cv::imwrite(rotated_image_path.string(), rotated_image);
            cv::imwrite(rotated_mask_path.string(), rotated_mask);
        }
        
        // 7. 读取姿态JSON文件
        cv::Mat T_B_E_grasp;
        if (!loadPoseJSON(grab_position_path.string(), T_B_E_grasp)) {
            result.success = false;
            result.error_message = "无法加载抓取姿态";
            return result;
        }
        
        // 读取准备姿态（如果存在）
        cv::Mat T_B_E_preparation;
        bool has_preparation = false;
        std::filesystem::path prep_pose_path = pose_dir / "preparation_position.json";
        if (std::filesystem::exists(prep_pose_path)) {
            if (loadPoseJSON(prep_pose_path.string(), T_B_E_preparation)) {
                has_preparation = true;
            }
        }
        
        // 读取相机姿态
        cv::Mat T_B_E_camera;
        std::filesystem::path camera_pose_path = pose_dir / "camera_pose.json";
        if (std::filesystem::exists(camera_pose_path)) {
            loadPoseJSON(camera_pose_path.string(), T_B_E_camera);
        } else if (has_preparation) {
            T_B_E_camera = T_B_E_preparation.clone();
        } else {
            T_B_E_camera = T_B_E_grasp.clone();
        }
        
        // 8. 构建特征参数字典
        std::map<std::string, double> feature_params;
        feature_params["workpiece_center_x_original"] = center.x;
        feature_params["workpiece_center_y_original"] = center.y;
        feature_params["workpiece_radius_original"] = radius;
        feature_params["workpiece_area"] = feature.workpiece_area;
        feature_params["valve_center_x"] = feature.valve_center.x;
        feature_params["valve_center_y"] = feature.valve_center.y;
        feature_params["valve_radius"] = feature.valve_radius;
        feature_params["valve_area"] = feature.valve_area;
        // 保存旋转角度（现在为0，因为不旋转）
        feature_params["rotation_angle_deg"] = rotation_angle * 180.0 / M_PI;
        feature_params["rotation_angle_rad"] = rotation_angle;
        feature_params["crop_x"] = crop_x;
        feature_params["crop_y"] = crop_y;
        feature_params["crop_width"] = crop_w;
        feature_params["crop_height"] = crop_h;
        
        // 计算裁剪后的阀体中心（在裁剪后的图像坐标系中，不旋转）
        cv::Point2f valve_center_cropped(feature.valve_center.x - crop_x, feature.valve_center.y - crop_y);
        cv::Point2f final_valve_center = valve_center_cropped;  // 不旋转，直接使用裁剪后的位置
        
        feature_params["final_valve_center_x"] = final_valve_center.x;
        feature_params["final_valve_center_y"] = final_valve_center.y;
        feature_params["final_workpiece_radius"] = radius;
        // 保存原始标准化角度（用于姿态计算）
        feature_params["standardized_angle_rad"] = feature.standardized_angle;
        feature_params["standardized_angle_deg"] = feature.standardized_angle * 180.0 / M_PI;
        
        // 9. 计算标准化姿态
        cv::Mat T_B_E_standardized_grasp = computeStandardizedPose(
            T_B_E_grasp, T_B_E_camera, T_E_C, camera_matrix, feature_params);
        
        cv::Mat T_B_E_standardized_preparation;
        if (has_preparation) {
            T_B_E_standardized_preparation = computeStandardizedPose(
                T_B_E_preparation, T_B_E_camera, T_E_C, camera_matrix, feature_params);
        }
        
        // 在原始图像上绘制抓取姿态可视化（无论是否处于debug模式）
        cv::Mat gripper_vis_image = original_image.clone();
        
        // 计算模板图像相机坐标系下的抓取姿态
        // 参考旧代码的实现：
        // 1. 计算拍摄模板图像时的相机位姿（机器人基座坐标系）
        //    T_B_C_template = T_B_E_camera * T_E_C
        // 2. 计算模板图像相机坐标系到机器人基座坐标系的逆变换
        //    T_C_template_B = inv(T_B_C_template)
        // 3. 计算模板图像相机坐标系下的抓取姿态
        //    T_C_template_E_grasp = T_C_template_B * T_B_E_grasp
        cv::Mat T_B_C_template = T_B_E_camera * T_E_C;  // 拍摄模板图像时的相机位姿（基座坐标系）
        cv::Mat T_C_template_B = T_B_C_template.inv();  // 模板图像相机坐标系到机器人基座坐标系
        cv::Mat T_C_template_E_grasp = T_C_template_B * T_B_E_grasp;  // 模板图像相机坐标系下的抓取姿态
        
        // 绘制手抓
        drawGripper(gripper_vis_image, T_C_template_E_grasp, camera_matrix, dist_coeffs, 50.0, 100.0);
        
        // 无论是否处于debug模式，都要保存gripper可视化图像到模板目录
        std::filesystem::path gripper_vis_template_path = pose_dir / "gripper_visualization.jpg";
        cv::imwrite(gripper_vis_template_path.string(), gripper_vis_image);
        
        // 保存最终可视化图（debug模式）
        if (debug) {
            // 在标准化后的图像上绘制外接圆和阀体圆
            cv::Mat final_vis = rotated_image.clone();
            
            // 工件外接圆圆心（在裁剪后的图像中）
            cv::Point2f final_workpiece_center(cropped_center.x, cropped_center.y);
            cv::circle(final_vis, final_workpiece_center, static_cast<int>(radius), 
                      cv::Scalar(0, 255, 0), 3);
            cv::circle(final_vis, final_workpiece_center, 5, cv::Scalar(0, 255, 0), -1);
            
            // 裁剪后的阀体中心（不旋转）
            cv::Point2f final_valve_center = valve_center_cropped;
            
            if (feature.valve_radius > 0) {
                cv::circle(final_vis, final_valve_center, static_cast<int>(feature.valve_radius), 
                          cv::Scalar(255, 0, 0), 3);
                cv::circle(final_vis, final_valve_center, 5, cv::Scalar(255, 0, 0), -1);
                
                // 绘制两圆心连线
                cv::line(final_vis, final_workpiece_center, final_valve_center, 
                        cv::Scalar(0, 255, 255), 2);
            }
            
            std::filesystem::path final_vis_path = debug_dir / "06_final_standardized_visualization.jpg";
            cv::imwrite(final_vis_path.string(), final_vis);
            
            // Debug模式：也保存到debug目录
            std::filesystem::path gripper_vis_path = debug_dir / "07_gripper_visualization_on_original.jpg";
            cv::imwrite(gripper_vis_path.string(), gripper_vis_image);
        }
        
        // 10. 保存标准化结果
        if (!saveStandardizationResults(
                pose_dir, feature_params,
                T_B_E_grasp, T_B_E_standardized_grasp,
                T_B_E_preparation, T_B_E_standardized_preparation,
                rotated_image, rotated_mask)) {
            result.success = false;
            result.error_message = "保存标准化结果失败";
            return result;
        }
        
        result.success = true;
        return result;
        
    } catch (const std::exception& e) {
        result.success = false;
        result.error_message = "标准化异常: " + std::string(e.what());
        return result;
    }
}

// 加载手眼标定参数：支持YAML和XML格式
// @param calibration_path 标定文件路径
// @param camera_matrix 输出相机内参矩阵
// @param dist_coeffs 输出畸变系数
// @param T_E_C 输出手眼变换矩阵（末端执行器到相机）
// @return 成功返回true，失败返回false
bool TemplateStandardizer::loadHandEyeCalibration(
    const std::string& calibration_path,
    cv::Mat& camera_matrix,
    cv::Mat& dist_coeffs,
    cv::Mat& T_E_C) {
    
    try {
        // 检查文件扩展名，判断是 YAML 还是 XML
        std::filesystem::path path(calibration_path);
        std::string ext = path.extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        
        if (ext == ".yaml" || ext == ".yml") {
            // 解析 YAML 格式
            YAML::Node config = YAML::LoadFile(calibration_path);
            
            // 读取相机内参矩阵
            if (config["camera_matrix"]) {
                const YAML::Node& cam_matrix_node = config["camera_matrix"];
                if (cam_matrix_node.IsSequence() && cam_matrix_node.size() == 3) {
                    camera_matrix = cv::Mat(3, 3, CV_64F);
                    for (int i = 0; i < 3; i++) {
                        if (cam_matrix_node[i].IsSequence() && cam_matrix_node[i].size() == 3) {
                            for (int j = 0; j < 3; j++) {
                                camera_matrix.at<double>(i, j) = cam_matrix_node[i][j].as<double>();
                            }
                        } else {
                            return false;
                        }
                    }
                } else {
                    return false;
                }
            } else {
                return false;
            }
            
            // 读取畸变系数
            if (config["distortion_coefficients"]) {
                const YAML::Node& dist_node = config["distortion_coefficients"];
                if (dist_node.IsSequence()) {
                    dist_coeffs = cv::Mat(dist_node.size(), 1, CV_64F);
                    for (size_t i = 0; i < dist_node.size(); i++) {
                        dist_coeffs.at<double>(i, 0) = dist_node[i].as<double>();
                    }
                } else {
                    return false;
                }
            } else {
                return false;
            }
            
            // 读取手眼标定变换矩阵
            if (config["hand_eye_calibration"] && config["hand_eye_calibration"]["transformation_matrix"]) {
                const YAML::Node& transform_node = config["hand_eye_calibration"]["transformation_matrix"];
                if (transform_node.IsSequence() && transform_node.size() == 4) {
                    T_E_C = cv::Mat(4, 4, CV_64F);
                    for (int i = 0; i < 4; i++) {
                        if (transform_node[i].IsSequence() && transform_node[i].size() == 4) {
                            for (int j = 0; j < 4; j++) {
                                T_E_C.at<double>(i, j) = transform_node[i][j].as<double>();
                            }
                        } else {
                            return false;
                        }
                    }
                } else {
                    return false;
                }
            } else {
                return false;
            }
            
            return true;
            
        } else {
            // 解析 XML 格式（原有逻辑）
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
        }
        
    } catch (const std::exception& e) {
        return false;
    }
}

// 加载姿态JSON文件：从JSON文件读取机器人姿态（位置和四元数）
// @param pose_path 姿态JSON文件路径
// @param T_B_E 输出变换矩阵（基座坐标系到末端执行器）
// @return 成功返回true，失败返回false
bool TemplateStandardizer::loadPoseJSON(
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

// 计算标准化姿态：将原始姿态转换为标准化坐标系下的姿态
// @param T_B_E_pose 原始姿态（基座坐标系）
// @param T_B_E_camera 相机姿态（基座坐标系）
// @param T_E_C 手眼变换矩阵
// @param camera_matrix 相机内参矩阵
// @param feature_params 特征参数字典
// @return 标准化后的姿态（基座坐标系）
cv::Mat TemplateStandardizer::computeStandardizedPose(
    const cv::Mat& T_B_E_pose,
    const cv::Mat& T_B_E_camera,
    const cv::Mat& T_E_C,
    const cv::Mat& camera_matrix,
    const std::map<std::string, double>& feature_params) {
    
    // 计算相机坐标系下的姿态
    cv::Mat T_B_C_camera = T_B_E_camera * T_E_C;
    cv::Mat T_C_camera_B = T_B_C_camera.inv();
    cv::Mat T_C_camera_E_pose = T_C_camera_B * T_B_E_pose;
    
    cv::Mat R_pose_camera = T_C_camera_E_pose(cv::Rect(0, 0, 3, 3));
    cv::Vec3d t_pose_camera(
        T_C_camera_E_pose.at<double>(0, 3),
        T_C_camera_E_pose.at<double>(1, 3),
        T_C_camera_E_pose.at<double>(2, 3)
    );
    
    // 获取旋转角度
    double rotation_angle_rad = feature_params.at("rotation_angle_rad");
    
    // 构建绕Z轴旋转矩阵
    cv::Mat rotation_z_camera = cv::Mat::eye(3, 3, CV_64F);
    rotation_z_camera.at<double>(0, 0) = std::cos(rotation_angle_rad);
    rotation_z_camera.at<double>(0, 1) = -std::sin(rotation_angle_rad);
    rotation_z_camera.at<double>(1, 0) = std::sin(rotation_angle_rad);
    rotation_z_camera.at<double>(1, 1) = std::cos(rotation_angle_rad);
    
    // 计算工件中心在相机坐标系下的位置
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    
    double workpiece_center_x_original = feature_params.at("workpiece_center_x_original");
    double workpiece_center_y_original = feature_params.at("workpiece_center_y_original");
    double workpiece_center_z_camera = t_pose_camera[2];
    
    cv::Vec3d workpiece_center_camera(
        (workpiece_center_x_original - cx) * workpiece_center_z_camera / fx,
        (workpiece_center_y_original - cy) * workpiece_center_z_camera / fy,
        workpiece_center_z_camera
    );
    
    // 计算从工件中心到姿态位置的偏移
    cv::Vec3d offset_from_center = t_pose_camera - workpiece_center_camera;
    double offset_x = offset_from_center[0];
    double offset_y = offset_from_center[1];
    
    // 旋转偏移（注意：由于不再旋转图像，rotation_angle_rad应该是0，所以不需要旋转）
    // 但是为了保持代码的一致性，仍然使用旋转矩阵（即使角度为0）
    double offset_x_rotated = offset_x * std::cos(rotation_angle_rad) - offset_y * std::sin(rotation_angle_rad);
    double offset_y_rotated = offset_x * std::sin(rotation_angle_rad) + offset_y * std::cos(rotation_angle_rad);
    
    // 计算标准化后的位置
    cv::Vec3d t_standardized_camera(
        workpiece_center_camera[0] + offset_x_rotated,
        workpiece_center_camera[1] + offset_y_rotated,
        t_pose_camera[2]
    );
    
    // 计算标准化后的旋转（注意：由于不再旋转，rotation_angle_rad=0，所以R_standardized_camera = R_pose_camera）
    cv::Mat R_standardized_camera = rotation_z_camera * R_pose_camera;
    
    // 构建标准化变换矩阵
    cv::Mat T_C_camera_E_standardized = cv::Mat::eye(4, 4, CV_64F);
    R_standardized_camera.copyTo(T_C_camera_E_standardized(cv::Rect(0, 0, 3, 3)));
    T_C_camera_E_standardized.at<double>(0, 3) = t_standardized_camera[0];
    T_C_camera_E_standardized.at<double>(1, 3) = t_standardized_camera[1];
    T_C_camera_E_standardized.at<double>(2, 3) = t_standardized_camera[2];
    
    // 转换回基座坐标系
    cv::Mat T_B_E_standardized = T_B_C_camera * T_C_camera_E_standardized;
    
    return T_B_E_standardized;
}

// 保存标准化结果：将标准化后的图像、掩码和姿态信息保存到文件
// @param pose_dir 姿态目录路径
// @param feature_params 特征参数字典
// @param T_B_E_grasp 原始抓取姿态
// @param T_B_E_standardized_grasp 标准化抓取姿态
// @param T_B_E_preparation 原始准备姿态（可选）
// @param T_B_E_standardized_preparation 标准化准备姿态（可选）
// @param rotated_image 标准化后的图像
// @param rotated_mask 标准化后的掩码
// @return 成功返回true，失败返回false
bool TemplateStandardizer::saveStandardizationResults(
    const std::filesystem::path& pose_dir,
    const std::map<std::string, double>& feature_params,
    const cv::Mat& T_B_E_grasp,
    const cv::Mat& T_B_E_standardized_grasp,
    const cv::Mat& T_B_E_preparation,
    const cv::Mat& T_B_E_standardized_preparation,
    const cv::Mat& rotated_image,
    const cv::Mat& rotated_mask) {
    
    try {
        // 保存标准化图像和掩膜
        std::filesystem::path image_path = pose_dir / "image.jpg";
        std::filesystem::path mask_path = pose_dir / "mask.jpg";
        cv::imwrite(image_path.string(), rotated_image);
        cv::imwrite(mask_path.string(), rotated_mask);
        
        // 辅助函数：将旋转矩阵转换为四元数
        auto rotationMatrixToQuaternion = [](const cv::Mat& R) -> cv::Vec4d {
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
        };
        
        // 辅助函数：将姿态矩阵转换为JSON
        auto poseToJson = [&](const cv::Mat& T_B_E) -> Json::Value {
            Json::Value pose_json;
            Json::Value position;
            position["x"] = T_B_E.at<double>(0, 3);
            position["y"] = T_B_E.at<double>(1, 3);
            position["z"] = T_B_E.at<double>(2, 3);
            
            cv::Mat R = T_B_E(cv::Rect(0, 0, 3, 3));
            cv::Vec4d quat = rotationMatrixToQuaternion(R);
            
            Json::Value orientation;
            orientation["x"] = quat[0];
            orientation["y"] = quat[1];
            orientation["z"] = quat[2];
            orientation["w"] = quat[3];
            
            Json::Value cartesian_position;
            cartesian_position["position"] = position;
            cartesian_position["orientation"] = orientation;
            
            pose_json["cartesian_position"] = cartesian_position;
            return pose_json;
        };
        
        // 构建template_info.json
        Json::Value template_info;
        
        // 保存特征参数
        Json::Value feature_params_json;
        for (const auto& [key, value] : feature_params) {
            feature_params_json[key] = value;
        }
        template_info["feature_parameters"] = feature_params_json;
        
        // 保存原始和标准化姿态
        template_info["original_grasp_pose"] = poseToJson(T_B_E_grasp);
        template_info["standardized_grasp_pose"] = poseToJson(T_B_E_standardized_grasp);
        
        if (!T_B_E_preparation.empty()) {
            template_info["original_preparation_pose"] = poseToJson(T_B_E_preparation);
            template_info["standardized_preparation_pose"] = poseToJson(T_B_E_standardized_preparation);
        }
        
        template_info["rotation_angle_deg"] = feature_params.at("rotation_angle_deg");
        template_info["rotation_angle_rad"] = feature_params.at("rotation_angle_rad");
        
        // 保存JSON文件
        std::filesystem::path template_info_path = pose_dir / "template_info.json";
        std::ofstream template_info_file(template_info_path.string());
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "  ";
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(template_info, &template_info_file);
        template_info_file.close();
        
        return true;
        
    } catch (const std::exception&) {
        return false;
    }
}

// 绘制夹爪：在图像上绘制夹爪的可视化（用于调试）
// @param image 输入图像（会被修改）
// @param T_C_E 末端执行器在相机坐标系下的变换矩阵
// @param camera_matrix 相机内参矩阵
// @param dist_coeffs 畸变系数
// @param gripper_opening_mm 夹爪开口宽度（毫米）
// @param gripper_length_mm 夹爪长度（毫米）
void TemplateStandardizer::drawGripper(
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

} // namespace visual_pose_estimation


