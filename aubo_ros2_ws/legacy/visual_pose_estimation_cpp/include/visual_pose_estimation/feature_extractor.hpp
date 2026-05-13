#ifndef VISUAL_POSE_ESTIMATION_FEATURE_EXTRACTOR_HPP
#define VISUAL_POSE_ESTIMATION_FEATURE_EXTRACTOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>

namespace visual_pose_estimation {

/**
 * @brief 连通域特征结构体
 * 
 * 保存每个连通域提取的特征信息
 */
struct ComponentFeature {
    // 工件整体外接圆特征
    cv::Point2f workpiece_center;      // 外接圆圆心坐标
    float workpiece_radius;            // 外接圆半径（像素）
    double workpiece_area;              // 工件连通域面积（像素²）
    
    // 阀体外接圆特征
    cv::Point2f valve_center;          // 阀体外接圆圆心坐标
    float valve_radius;                // 阀体外接圆半径（像素）
    double valve_area;                 // 阀体面积（像素²）
    
    // 标准化旋转角度
    double standardized_angle;         // 标准化旋转角度（弧度）
    double standardized_angle_deg;   // 标准化旋转角度（度）
    
    // 原始连通域掩码（可选，用于调试）
    cv::Mat component_mask;
    
    ComponentFeature()
        : workpiece_center(0, 0)
        , workpiece_radius(0.0f)
        , workpiece_area(0.0)
        , valve_center(0, 0)
        , valve_radius(0.0f)
        , valve_area(0.0)
        , standardized_angle(0.0)
        , standardized_angle_deg(0.0)
    {}
};

/**
 * @brief 特征提取器类
 * 
 * 负责从连通域中提取标准化特征，包括：
 * 1. 初步筛选干扰连通域
 * 2. 多线程并行处理连通域
 * 3. 提取工件外接圆特征
 * 4. 提取阀体外接圆特征
 * 5. 计算标准化旋转角度
 */
class FeatureExtractor {
public:
    /**
     * @brief 构造函数
     */
    FeatureExtractor();
    
    /**
     * @brief 析构函数
     */
    ~FeatureExtractor() = default;
    
    /**
     * @brief 设置参数
     * @param params 参数字典
     */
    void setParameters(const std::map<std::string, double>& params);
    
    /**
     * @brief 获取参数
     * @return 参数字典
     */
    std::map<std::string, double> getParameters() const;
    
    /**
     * @brief 提取连通域特征（主函数）
     * @param components 连通域二值图像列表
     * @return 特征列表（每个连通域对应一个特征结构体）
     * 
     * 处理流程：
     * 1. 初步筛选干扰连通域（根据面积等条件）
     * 2. 根据筛选后的连通域数量启用相应数量的线程
     * 3. 并行处理每个连通域，提取特征
     * 4. 返回所有连通域的特征列表
     */
    std::vector<ComponentFeature> extractFeatures(const std::vector<cv::Mat>& components);
    
    /**
     * @brief 筛选连通域
     * @param components 输入连通域列表
     * @return 筛选后的连通域列表
     */
    std::vector<cv::Mat> filterComponents(const std::vector<cv::Mat>& components);

private:
    // 参数
    std::map<std::string, double> parameters_;
    
    // 线程同步
    std::mutex result_mutex_;
    
    // 默认参数
    void initializeDefaultParameters();
    
    // 获取参数值（带默认值）
    double getParam(const std::string& key, double fallback) const;
    
    /**
     * @brief 提取单个连通域的特征
     * @param component_mask 连通域二值掩码
     * @return 特征结构体
     */
    ComponentFeature extractSingleComponentFeature(const cv::Mat& component_mask);
    
    /**
     * @brief 提取工件外接圆（大圆）
     * @param mask 连通域掩码
     * @param combine_contours 是否合并多个轮廓
     * @param min_area 最小轮廓面积
     * @return 圆心坐标和半径
     */
    std::pair<cv::Point2f, float> extractWorkpieceCircle(
        const cv::Mat& mask,
        bool combine_contours = true,
        int min_area = 100
    );
    
    /**
     * @brief 提取阀体外接圆（小圆）
     * @param mask 连通域掩码
     * @return 圆心坐标、半径和处理后的掩码
     */
    std::tuple<cv::Point2f, float, cv::Mat> extractValveCircle(const cv::Mat& mask);
    
    /**
     * @brief 计算标准化旋转角度
     * @param workpiece_center 工件外接圆圆心
     * @param valve_center 阀体外接圆圆心
     * @return 旋转角度（弧度）
     */
    double calculateStandardizedAngle(const cv::Point2f& workpiece_center, const cv::Point2f& valve_center);
    
    /**
     * @brief 处理单个连通域的线程函数
     * @param component_mask 连通域掩码
     * @param results 结果容器（需要加锁）
     * @param index 连通域索引
     */
    void processComponentThread(
        const cv::Mat& component_mask,
        std::vector<ComponentFeature>& results,
        size_t index
    );
};

} // namespace visual_pose_estimation

#endif // VISUAL_POSE_ESTIMATION_FEATURE_EXTRACTOR_HPP

