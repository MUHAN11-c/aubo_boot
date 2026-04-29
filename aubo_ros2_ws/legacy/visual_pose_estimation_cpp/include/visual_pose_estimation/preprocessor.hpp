#ifndef VISUAL_POSE_ESTIMATION_PREPROCESSOR_HPP
#define VISUAL_POSE_ESTIMATION_PREPROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>

namespace visual_pose_estimation {

/**
 * @brief 图像预处理类
 * 
 * 负责图像预处理，包括：
 * 1. 图像缩放（根据缩放因子）
 * 2. 背景去除
 * 3. 连通域提取
 * 4. 恢复原始尺寸
 */
class Preprocessor {
public:
    /**
     * @brief 构造函数
     */
    Preprocessor();
    
    /**
     * @brief 析构函数
     */
    ~Preprocessor() = default;
    
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
     * @brief 设置缩放因子
     * @param scale_factor 缩放因子 (0-1)
     */
    void setScaleFactor(double scale_factor);
    
    /**
     * @brief 获取缩放因子
     * @return 缩放因子
     */
    double getScaleFactor() const;
    
    /**
     * @brief 预处理图像：去除背景并提取连通域
     * @param image 输入图像（BGR格式）
     * @return 连通域二值图像列表（已恢复原始尺寸）
     * 
     * 处理流程：
     * 1. 根据缩放因子缩小图像
     * 2. 去除背景
     * 3. 提取连通域
     * 4. 将每个连通域恢复到原始尺寸
     * 5. 返回连通域二值图像列表
     */
    std::vector<cv::Mat> preprocess(const cv::Mat& image);
    
    /**
     * @brief 去除绿色背景
     * @param image 输入图像（BGR格式）
     * @return 非绿色掩码（二值图像，255=前景，0=背景）
     */
    cv::Mat removeGreenBackground(const cv::Mat& image);

private:
    // 参数
    std::map<std::string, double> parameters_;
    double scale_factor_;  // 缩放因子 (0-1)
    
    // 默认参数
    void initializeDefaultParameters();
    
    // 获取参数值（带默认值）
    double getParam(const std::string& key, double fallback) const;
    
    // 提取连通域
    std::vector<cv::Mat> extractConnectedComponents(const cv::Mat& binary_mask);
    
    // 将掩码恢复到原始尺寸
    cv::Mat resizeMaskToOriginal(const cv::Mat& mask, const cv::Size& original_size);

    // 根据面积、长宽比等条件筛选连通域
    std::vector<cv::Mat> filterComponents(const std::vector<cv::Mat>& components);
};

} // namespace visual_pose_estimation

#endif // VISUAL_POSE_ESTIMATION_PREPROCESSOR_HPP

