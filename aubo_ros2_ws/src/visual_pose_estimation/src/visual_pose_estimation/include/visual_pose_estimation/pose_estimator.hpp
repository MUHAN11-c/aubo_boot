#ifndef VISUAL_POSE_ESTIMATION_POSE_ESTIMATOR_HPP
#define VISUAL_POSE_ESTIMATION_POSE_ESTIMATOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include "visual_pose_estimation/feature_extractor.hpp"

namespace visual_pose_estimation {

// 前向声明
class Preprocessor;

/**
 * @brief 模板项数据结构
 */
struct TemplateItem {
    std::string id;                    // 模板ID（如 "pose_1"）
    ComponentFeature feature;          // 模板特征
    cv::Mat T_C_E_grasp;              // 相机坐标系下的抓取姿态
    cv::Mat T_C_E_prep;               // 相机坐标系下的准备姿态（可选）
    std::string crop_path;            // 模板图像路径
    
    TemplateItem() {
        T_C_E_grasp = cv::Mat::eye(4, 4, CV_64F);
        T_C_E_prep = cv::Mat::eye(4, 4, CV_64F);
    }
};

/**
 * @brief 姿态估计结果
 */
struct PoseEstimationResult {
    std::string template_id;
    cv::Mat T_B_E_grasp;              // 基座坐标系下的抓取姿态
    cv::Mat T_B_E_prep;               // 基座坐标系下的准备姿态
    double confidence;                 // 置信度
    cv::Rect bounding_box;            // 边界框
    cv::Vec3d target_center_camera;   // 工件中心在相机坐标系下的位置
    bool has_target_center_camera;
    std::map<std::string, double> meta;
    
    PoseEstimationResult()
        : confidence(0.0)
        , target_center_camera(0.0, 0.0, 0.0)
        , has_target_center_camera(false) {}
};

/**
 * @brief 姿态估计器类
 * 
 * 负责模板匹配和姿态计算，包括：
 * 1. 加载模板库
 * 2. 选择最佳匹配模板
 * 3. 计算2D对齐矩阵
 * 4. 计算最终姿态
 */
class PoseEstimator {
public:
    /**
     * @brief 构造函数
     * @param preprocessor 预处理器指针
     * @param feature_extractor 特征提取器指针
     */
    PoseEstimator(
        std::shared_ptr<Preprocessor> preprocessor,
        std::shared_ptr<FeatureExtractor> feature_extractor);
    
    /**
     * @brief 析构函数
     */
    ~PoseEstimator() = default;
    
    /**
     * @brief 加载模板库
     * @param template_root 模板库根目录
     * @param camera_matrix 相机内参
     * @param T_E_C 末端到相机的变换矩阵
     * @return 是否加载成功
     */
    bool loadTemplateLibrary(
        const std::string& template_root,
        const cv::Mat& camera_matrix,
        const cv::Mat& T_E_C);
    
    /**
     * @brief 选择最佳匹配模板
     * @param target_feature 目标特征
     * @param templates 模板列表
     * @return 最佳匹配模板索引和距离
     */
    std::pair<int, double> selectBestTemplate(
        const ComponentFeature& target_feature,
        const std::vector<TemplateItem>& templates);
    
    /**
     * @brief 计算2D对齐矩阵
     * @param template_feature 模板特征
     * @param target_feature 目标特征
     * @param allow_scale 是否允许尺度变化
     * @return 2D相似变换矩阵
     */
    cv::Mat compute2DAlignment(
        const ComponentFeature& template_feature,
        const ComponentFeature& target_feature,
        bool allow_scale = false);
    
    /**
     * @brief 估计姿态
     * @param target_feature 目标特征
     * @param best_template 最佳匹配模板
     * @param camera_matrix 相机内参
     * @param T_B_C_camera 基座到相机的变换矩阵
     * @param T_E_C 末端到相机的变换矩阵
     * @return 姿态估计结果
     */
    PoseEstimationResult estimatePose(
        const ComponentFeature& target_feature,
        const TemplateItem& best_template,
        const cv::Mat& camera_matrix,
        const cv::Mat& T_B_C_camera,
        const cv::Mat& T_E_C);

private:
    std::shared_ptr<Preprocessor> preprocessor_;
    std::shared_ptr<FeatureExtractor> feature_extractor_;
    
    /**
     * @brief 加载手眼标定参数
     */
    bool loadHandEyeCalibration(
        const std::string& calibration_path,
        cv::Mat& camera_matrix,
        cv::Mat& dist_coeffs,
        cv::Mat& T_E_C);
    
    /**
     * @brief 加载姿态JSON文件
     */
    bool loadPoseJSON(
        const std::string& pose_path,
        cv::Mat& T_B_E);
    
    /**
     * @brief 旋转矩阵转四元数
     */
    cv::Vec4d rotationMatrixToQuaternion(const cv::Mat& R);
    
    /**
     * @brief 四元数转旋转矩阵
     */
    cv::Mat quaternionToRotationMatrix(const cv::Vec4d& quat);
};

} // namespace visual_pose_estimation

#endif // VISUAL_POSE_ESTIMATION_POSE_ESTIMATOR_HPP

