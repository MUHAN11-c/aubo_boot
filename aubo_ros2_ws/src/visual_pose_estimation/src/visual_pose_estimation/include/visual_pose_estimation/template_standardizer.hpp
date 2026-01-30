#ifndef VISUAL_POSE_ESTIMATION_TEMPLATE_STANDARDIZER_HPP
#define VISUAL_POSE_ESTIMATION_TEMPLATE_STANDARDIZER_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <filesystem>
#include <memory>

namespace visual_pose_estimation {

// 前向声明
class Preprocessor;
class FeatureExtractor;

/**
 * @brief 模板标准化结果
 */
struct StandardizationResult {
    std::string pose_id;              // 姿态ID
    bool success;                     // 是否成功
    std::string error_message;        // 错误信息
    
    StandardizationResult() : success(false) {}
};

/**
 * @brief 模板标准化器类
 * 
 * 负责对模板进行标准化处理，包括：
 * 1. 读取原始模板图像
 * 2. 执行特征提取
 * 3. 计算标准化姿态
 * 4. 保存标准化结果
 */
class TemplateStandardizer {
public:
    /**
     * @brief 构造函数
     * @param preprocessor 预处理器指针
     * @param feature_extractor 特征提取器指针
     */
    TemplateStandardizer(
        std::shared_ptr<Preprocessor> preprocessor,
        std::shared_ptr<FeatureExtractor> feature_extractor
    );
    
    /**
     * @brief 析构函数
     */
    ~TemplateStandardizer() = default;
    
    /**
     * @brief 标准化模板
     * @param template_root 模板根目录
     * @param workpiece_id 工件ID
     * @param debug 是否启用调试模式
     * @return 标准化结果列表（每个姿态对应一个结果）
     * 
     * 处理流程：
     * 1. 遍历工件文件夹下的所有pose_*目录
     * 2. 对每个姿态执行标准化：
     *    - 读取original_image.jpg
     *    - 执行预处理和特征提取
     *    - 读取姿态JSON文件
     *    - 计算标准化姿态
     *    - 保存标准化结果
     */
    std::vector<StandardizationResult> standardizeTemplate(
        const std::string& template_root,
        const std::string& workpiece_id,
        const std::string& calib_file = "",
        bool debug = false
    );
    
    /**
     * @brief 标准化单个姿态
     * @param pose_dir 姿态目录路径
     * @param pose_id 姿态ID
     * @param calib_file 手眼标定文件路径（如果为空，则从pose_dir查找）
     * @param debug 是否启用调试模式
     * @return 标准化结果
     */
    StandardizationResult standardizePose(
        const std::filesystem::path& pose_dir,
        const std::string& pose_id,
        const std::string& calib_file = "",
        bool debug = false
    );

private:
    std::shared_ptr<Preprocessor> preprocessor_;
    std::shared_ptr<FeatureExtractor> feature_extractor_;
    
    /**
     * @brief 加载手眼标定参数
     * @param calibration_path 标定文件路径
     * @param camera_matrix 输出相机内参矩阵
     * @param dist_coeffs 输出畸变系数
     * @param T_E_C 输出末端到相机的变换矩阵
     * @return 是否加载成功
     */
    bool loadHandEyeCalibration(
        const std::string& calibration_path,
        cv::Mat& camera_matrix,
        cv::Mat& dist_coeffs,
        cv::Mat& T_E_C
    );
    
    /**
     * @brief 加载姿态JSON文件
     * @param pose_path 姿态文件路径
     * @param T_B_E 输出基座到末端的变换矩阵
     * @return 是否加载成功
     */
    bool loadPoseJSON(
        const std::string& pose_path,
        cv::Mat& T_B_E
    );
    
    /**
     * @brief 计算标准化姿态
     * @param T_B_E_pose 原始姿态矩阵
     * @param T_B_E_camera 相机姿态矩阵
     * @param T_E_C 末端到相机的变换矩阵
     * @param camera_matrix 相机内参
     * @param feature_params 特征参数（包含旋转角度等信息）
     * @return 标准化后的姿态矩阵
     */
    cv::Mat computeStandardizedPose(
        const cv::Mat& T_B_E_pose,
        const cv::Mat& T_B_E_camera,
        const cv::Mat& T_E_C,
        const cv::Mat& camera_matrix,
        const std::map<std::string, double>& feature_params
    );
    
    /**
     * @brief 保存标准化结果
     * @param pose_dir 姿态目录
     * @param feature_params 特征参数
     * @param T_B_E_grasp 原始抓取姿态
     * @param T_B_E_standardized_grasp 标准化抓取姿态
     * @param T_B_E_preparation 原始准备姿态（可选）
     * @param T_B_E_standardized_preparation 标准化准备姿态（可选）
     * @param rotated_image 标准化后的图像
     * @param rotated_mask 标准化后的掩膜
     * @return 是否保存成功
     */
    bool saveStandardizationResults(
        const std::filesystem::path& pose_dir,
        const std::map<std::string, double>& feature_params,
        const cv::Mat& T_B_E_grasp,
        const cv::Mat& T_B_E_standardized_grasp,
        const cv::Mat& T_B_E_preparation,
        const cv::Mat& T_B_E_standardized_preparation,
        const cv::Mat& rotated_image,
        const cv::Mat& rotated_mask
    );
    
    /**
     * @brief 绘制手抓可视化
     * @param image 输入输出图像
     * @param T_C_E 相机坐标系到末端执行器坐标系的变换矩阵
     * @param camera_matrix 相机内参矩阵
     * @param dist_coeffs 畸变系数
     * @param gripper_opening_mm 夹爪开口大小（毫米）
     * @param gripper_length_mm 夹爪长度（毫米）
     */
    void drawGripper(
        cv::Mat& image,
        const cv::Mat& T_C_E,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs,
        double gripper_opening_mm = 50.0,
        double gripper_length_mm = 100.0
    );
};

} // namespace visual_pose_estimation

#endif // VISUAL_POSE_ESTIMATION_TEMPLATE_STANDARDIZER_HPP


