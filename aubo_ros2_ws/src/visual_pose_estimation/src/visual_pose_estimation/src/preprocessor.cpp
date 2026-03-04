// 图像预处理器实现
// 负责去除绿色背景、提取连通域、筛选有效区域等预处理操作

#include "visual_pose_estimation/preprocessor.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace visual_pose_estimation {

// 构造函数：初始化预处理器
Preprocessor::Preprocessor() : scale_factor_(1.0) {
    initializeDefaultParameters();
}

// 初始化默认参数
void Preprocessor::initializeDefaultParameters() {
    // 背景去除参数
    parameters_["bg_border_ratio"] = 0.08;
    parameters_["bg_hue_margin"] = 12.0;
    parameters_["bg_hue_std_mul"] = 3.0;
    parameters_["bg_sat_margin"] = 25.0;
    parameters_["bg_sat_std_mul"] = 2.0;
    parameters_["bg_val_margin"] = 35.0;
    parameters_["bg_val_std_mul"] = 2.0;
    parameters_["bg_cleanup_kernel"] = 7.0;
    parameters_["bg_enable_classic_hsv"] = 1.0;
    parameters_["bg_lab_threshold"] = 3.5;
    parameters_["bg_median_ksize"] = 5.0;
    parameters_["bg_foreground_close_kernel"] = 9.0;
    parameters_["bg_use_histogram"] = 1.0;
    parameters_["bg_min_noise_area"] = 100.0;
    parameters_["bg_erode_before_dilate"] = 1.0;
    parameters_["bg_border_bg_ratio_threshold"] = 0.7;
    parameters_["bg_component_min_area"] = 1500.0;
    parameters_["bg_component_max_area"] = 15000000.0;
    parameters_["bg_component_min_aspect_ratio"] = 0.2;
    parameters_["bg_component_max_aspect_ratio"] = 5.0;
    parameters_["bg_component_min_width"] = 40.0;
    parameters_["bg_component_min_height"] = 40.0;
    parameters_["bg_component_max_count"] = 3.0;
    
    // 缩放因子（从配置文件读取）
    parameters_["scale_factor"] = 1.0;
}

// 设置参数
// @param params 参数字典
void Preprocessor::setParameters(const std::map<std::string, double>& params) {
    for (const auto& [key, value] : params) {
        parameters_[key] = value;
        if (key == "scale_factor" || key == "preprocess.scale_factor") {
            scale_factor_ = std::clamp(value, 0.1, 1.0);
        }
    }
}

// 获取所有参数
// @return 参数字典
std::map<std::string, double> Preprocessor::getParameters() const {
    return parameters_;
}

// 设置缩放因子
// @param scale_factor 缩放因子（0.1-1.0）
void Preprocessor::setScaleFactor(double scale_factor) {
    scale_factor_ = std::clamp(scale_factor, 0.1, 1.0);
    parameters_["scale_factor"] = scale_factor_;
}

// 获取缩放因子
// @return 缩放因子
double Preprocessor::getScaleFactor() const {
    return scale_factor_;
}

// 获取单个参数值（支持嵌套配置路径）
// @param key 参数键名
// @param fallback 默认值
// @return 参数值或默认值
double Preprocessor::getParam(const std::string& key, double fallback) const {
    // 首先尝试从配置文件读取的嵌套键名（preprocess.background.*）
    std::string config_key = "preprocess.background." + key.substr(3); // 去掉 "bg_" 前缀
    auto it = parameters_.find(config_key);
    if (it != parameters_.end()) {
        return it->second;
    }
    // 如果不存在，尝试使用原始键名（bg_*）
    it = parameters_.find(key);
    if (it != parameters_.end()) {
        return it->second;
    }
    return fallback;
}

// 预处理主函数：对输入图像进行预处理，返回连通域列表
// @param image 输入图像
// @return 连通域列表（已恢复到原始尺寸）
std::vector<cv::Mat> Preprocessor::preprocess(const cv::Mat& image) {
    std::vector<cv::Mat> result;
    
    if (image.empty()) {
        return result;
    }
    
    // 保存原始尺寸
    cv::Size original_size = image.size();
    
    // 步骤1: 根据缩放因子缩小图像
    cv::Mat scaled_image;
    if (scale_factor_ < 1.0) {
        cv::Size scaled_size(
            static_cast<int>(std::round(image.cols * scale_factor_)),
            static_cast<int>(std::round(image.rows * scale_factor_))
        );
        cv::resize(image, scaled_image, scaled_size, 0, 0, cv::INTER_AREA);
    } else {
        scaled_image = image.clone();
    }
    
    // 步骤2: 去除背景
    cv::Mat foreground_mask = removeGreenBackground(scaled_image);
    
    if (foreground_mask.empty()) {
        return result;
    }
    
    // 步骤3: 提取连通域
    std::vector<cv::Mat> components = extractConnectedComponents(foreground_mask);
    std::vector<cv::Mat> filtered_components = filterComponents(components);
    if (!filtered_components.empty()) {
        components = filtered_components;
    }
    
    // 步骤4: 将每个连通域恢复到原始尺寸
    for (const auto& component : components) {
        cv::Mat resized_component = resizeMaskToOriginal(component, original_size);
        if (!resized_component.empty()) {
            result.push_back(resized_component);
        }
    }
    
    return result;
}

// 去除绿色背景：使用HSV和Lab颜色空间进行背景分割
// @param image 输入图像
// @return 前景掩码（非绿色区域为255）
cv::Mat Preprocessor::removeGreenBackground(const cv::Mat& image) {
    if (image.empty()) {
        return cv::Mat();
    }

    auto get_param = [&](const std::string& key, double fallback) -> double {
        return getParam(key, fallback);
    };

    // 转换到多个颜色空间
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsv_channels;
    cv::split(hsv, hsv_channels);

    cv::Mat lab;
    cv::cvtColor(image, lab, cv::COLOR_BGR2Lab);

    // 边缘采样区域比例
    double border_ratio = std::clamp(get_param("bg_border_ratio", 0.15), 0.05, 0.30);
    int margin = static_cast<int>(std::round(border_ratio * std::min(image.cols, image.rows)));
    margin = std::max(10, margin);
    margin = std::min(margin, std::min(image.cols, image.rows) / 3);

    // 创建边缘掩码
    cv::Mat border_mask = cv::Mat::zeros(image.size(), CV_8UC1);
    if (margin > 0) {
        border_mask.rowRange(0, margin).setTo(255);
        border_mask.rowRange(image.rows - margin, image.rows).setTo(255);
        border_mask.colRange(0, margin).setTo(255);
        border_mask.colRange(image.cols - margin, image.cols).setTo(255);
    }

    if (cv::countNonZero(border_mask) < image.rows * image.cols * 0.1) {
        border_mask.setTo(255);
    }

    // 计算边缘区域的统计信息
    cv::Scalar mean_hsv, std_hsv;
    cv::Scalar mean_lab, std_lab;
    
    cv::meanStdDev(hsv, mean_hsv, std_hsv, border_mask);
    cv::meanStdDev(lab, mean_lab, std_lab, border_mask);
    
    // 同时计算整个图像的统计信息
    cv::Scalar mean_hsv_full, std_hsv_full;
    cv::Scalar mean_lab_full, std_lab_full;
    cv::meanStdDev(hsv, mean_hsv_full, std_hsv_full);
    cv::meanStdDev(lab, mean_lab_full, std_lab_full);
    
    // 检查边缘区域是否被工件污染
    double hsv_hue_diff = std::abs(mean_hsv[0] - mean_hsv_full[0]);
    double hsv_sat_diff = std::abs(mean_hsv[1] - mean_hsv_full[1]);
    double hsv_val_diff = std::abs(mean_hsv[2] - mean_hsv_full[2]);
    
    if (hsv_hue_diff > 30.0 || hsv_sat_diff > 50.0 || hsv_val_diff > 60.0) {
        double weight_edge = 0.3;
        double weight_full = 0.7;
        mean_hsv[0] = weight_edge * mean_hsv[0] + weight_full * mean_hsv_full[0];
        mean_hsv[1] = weight_edge * mean_hsv[1] + weight_full * mean_hsv_full[1];
        mean_hsv[2] = weight_edge * mean_hsv[2] + weight_full * mean_hsv_full[2];
        std_hsv[0] = weight_edge * std_hsv[0] + weight_full * std_hsv_full[0];
        std_hsv[1] = weight_edge * std_hsv[1] + weight_full * std_hsv_full[1];
        std_hsv[2] = weight_edge * std_hsv[2] + weight_full * std_hsv_full[2];
        
        mean_lab[0] = weight_edge * mean_lab[0] + weight_full * mean_lab_full[0];
        mean_lab[1] = weight_edge * mean_lab[1] + weight_full * mean_lab_full[1];
        mean_lab[2] = weight_edge * mean_lab[2] + weight_full * mean_lab_full[2];
        std_lab[0] = weight_edge * std_lab[0] + weight_full * std_lab_full[0];
        std_lab[1] = weight_edge * std_lab[1] + weight_full * std_lab_full[1];
        std_lab[2] = weight_edge * std_lab[2] + weight_full * std_lab_full[2];
    }
    
    auto clamp_channel = [](double value, double max_value) {
        return std::max(0.0, std::min(value, max_value));
    };
    mean_hsv[0] = clamp_channel(mean_hsv[0], 180.0);
    mean_hsv[1] = clamp_channel(mean_hsv[1], 255.0);
    mean_hsv[2] = clamp_channel(mean_hsv[2], 255.0);

    // 使用直方图分析找到主要背景色（如果启用）
    cv::Mat bg_mask = cv::Mat::zeros(image.size(), CV_8UC1);
    
    if (get_param("bg_use_histogram", 1.0) > 0.5) {
        cv::Mat border_h;
        hsv_channels[0].copyTo(border_h, border_mask);
        
        int histSize = 180;
        float hRange[] = {0, 180};
        const float* ranges[] = {hRange};
        int channels[] = {0};
        
        cv::Mat hist;
        cv::calcHist(&border_h, 1, channels, border_mask, hist, 1, &histSize, ranges, true, false);
        
        double minVal, maxVal;
        int minIdx[1], maxIdx[1];
        cv::minMaxIdx(hist, &minVal, &maxVal, minIdx, maxIdx);
        
        if (maxIdx[0] >= 0) {
            mean_hsv[0] = maxIdx[0];
        }
        
        cv::Mat border_s, border_v;
        hsv_channels[1].copyTo(border_s, border_mask);
        hsv_channels[2].copyTo(border_v, border_mask);
        
        std::vector<uchar> s_values, v_values;
        for (int i = 0; i < border_s.rows; i++) {
            for (int j = 0; j < border_s.cols; j++) {
                if (border_mask.at<uchar>(i, j) > 0) {
                    s_values.push_back(border_s.at<uchar>(i, j));
                    v_values.push_back(border_v.at<uchar>(i, j));
                }
            }
        }
        
        if (!s_values.empty()) {
            std::sort(s_values.begin(), s_values.end());
            mean_hsv[1] = s_values[s_values.size() / 2];
        }
        if (!v_values.empty()) {
            std::sort(v_values.begin(), v_values.end());
            mean_hsv[2] = v_values[v_values.size() / 2];
        }
    }

    // 计算自适应阈值
    double hue_margin = get_param("bg_hue_margin", 20.0);
    double hue_std_mul = get_param("bg_hue_std_mul", 2.5);
    double sat_margin = get_param("bg_sat_margin", 40.0);
    double sat_std_mul = get_param("bg_sat_std_mul", 2.5);
    double val_margin = get_param("bg_val_margin", 50.0);
    double val_std_mul = get_param("bg_val_std_mul", 2.5);
    
    double hue_threshold = std::clamp(
        hue_margin + hue_std_mul * std_hsv[0],
        15.0,
        60.0
    );
    double sat_threshold = std::clamp(
        sat_margin + sat_std_mul * std_hsv[1],
        25.0,
        120.0
    );
    double val_threshold = std::clamp(
        val_margin + val_std_mul * std_hsv[2],
        30.0,
        140.0
    );

    // HSV空间匹配
    cv::Mat hue_diff;
    cv::absdiff(hsv_channels[0], cv::Scalar(static_cast<uchar>(std::round(mean_hsv[0]))), hue_diff);
    cv::Mat hue_wrap;
    cv::absdiff(hue_diff, cv::Scalar(180), hue_wrap);
    cv::min(hue_diff, hue_wrap, hue_diff);

    cv::Mat sat_diff;
    cv::absdiff(hsv_channels[1], cv::Scalar(static_cast<int>(std::round(mean_hsv[1]))), sat_diff);

    cv::Mat val_diff;
    cv::absdiff(hsv_channels[2], cv::Scalar(static_cast<int>(std::round(mean_hsv[2]))), val_diff);

    cv::Mat hsv_mask = (hue_diff < hue_threshold) & (sat_diff < sat_threshold) & (val_diff < val_threshold);
    hsv_mask.convertTo(bg_mask, CV_8UC1, 255.0);

    // Lab空间匹配
    std::vector<cv::Mat> lab_channels;
    cv::split(lab, lab_channels);

    cv::Mat lab_distance = cv::Mat::zeros(image.size(), CV_32F);
    for (int i = 0; i < 3; ++i) {
        cv::Mat channel_float;
        lab_channels[i].convertTo(channel_float, CV_32F);
        cv::Mat diff;
        cv::absdiff(channel_float, cv::Scalar(static_cast<float>(mean_lab[i])), diff);
        float denom = static_cast<float>(std_lab[i]);
        if (denom < 2.0f) {
            denom = 2.0f;
        }
        cv::divide(diff, denom, diff);
        lab_distance += diff;
    }
    lab_distance /= 3.0f;

    double lab_threshold = std::clamp(get_param("bg_lab_threshold", 4.0), 2.0, 15.0);
    cv::Mat lab_mask;
    cv::compare(lab_distance, lab_threshold, lab_mask, cv::CMP_LT);
    
    cv::bitwise_and(bg_mask, lab_mask, bg_mask);

    // 经典绿色范围检测
    cv::Mat classic_green_mask;
    if (get_param("bg_enable_classic_hsv", 1.0) > 0.5) {
        cv::Scalar lower_green(25, 25, 25);
        cv::Scalar upper_green(95, 255, 255);
        cv::inRange(hsv, lower_green, upper_green, classic_green_mask);
        cv::bitwise_or(bg_mask, classic_green_mask, bg_mask);
    }

    // 形态学清理背景掩码
    int cleanup_kernel_size = static_cast<int>(std::round(get_param("bg_cleanup_kernel", 9.0)));
    int cleanup_dilate_iter = 1;
    if (cleanup_kernel_size % 2 == 0) {
        cleanup_kernel_size += 1;
    }
    if (cleanup_kernel_size >= 3) {
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cleanup_kernel_size, cleanup_kernel_size));
        cv::dilate(bg_mask, bg_mask, kernel, cv::Point(-1, -1), cleanup_dilate_iter);
        cv::erode(bg_mask, bg_mask, kernel, cv::Point(-1, -1), 1);
    }

    // 获取前景掩码
    cv::Mat non_green;
    cv::bitwise_not(bg_mask, non_green);
    
    if (non_green.size() != image.size()) {
        cv::Mat resized_non_green;
        cv::resize(non_green, resized_non_green, image.size());
        non_green = resized_non_green;
    }

    // 去除前景中的小噪声
    double min_noise_area = get_param("bg_min_noise_area", 100.0);
    if (min_noise_area > 0) {
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(non_green, labels, stats, centroids, 8);
        
        cv::Mat cleaned_fg = cv::Mat::zeros(image.size(), CV_8UC1);
        
        for (int i = 1; i < num_labels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area >= static_cast<int>(min_noise_area)) {
                cleaned_fg.setTo(255, labels == i);
            }
        }
        non_green = cleaned_fg;
    }

    // 前景形态学处理
    bool erode_before_dilate = get_param("bg_erode_before_dilate", 0.0) > 0.5;
    if (erode_before_dilate) {
        int fg_erode_kernel = 5;
        cv::Mat fg_erode_k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(fg_erode_kernel, fg_erode_kernel));
        cv::erode(non_green, non_green, fg_erode_k, cv::Point(-1, -1), 1);
    }

    cv::Mat original_fg = non_green.clone();
    
    int fg_close_kernel_size = static_cast<int>(std::round(get_param("bg_foreground_close_kernel", 11.0)));
    int fg_close_iterations = 2;
    
    if (fg_close_kernel_size > 9) {
        fg_close_kernel_size = 9;
    }
    
    if (fg_close_kernel_size % 2 == 0 && fg_close_kernel_size > 0) {
        fg_close_kernel_size += 1;
    }
    if (fg_close_kernel_size >= 3) {
        cv::Mat fg_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(fg_close_kernel_size, fg_close_kernel_size));
        cv::morphologyEx(non_green, non_green, cv::MORPH_CLOSE, fg_kernel, cv::Point(-1, -1), fg_close_iterations);
        cv::bitwise_or(non_green, original_fg, non_green);
    }

    // 中值滤波去除小噪声
    int median_kernel = static_cast<int>(std::round(get_param("bg_median_ksize", 7.0)));
    if (median_kernel > 5) {
        median_kernel = 5;
    }
    if (median_kernel % 2 == 0 && median_kernel > 0) {
        median_kernel += 1;
    }
    if (median_kernel >= 3) {
        cv::medianBlur(non_green, non_green, median_kernel);
    }

    return non_green;
}

// 提取连通域：从二值掩码中提取所有连通域
// @param binary_mask 二值掩码
// @return 连通域列表
std::vector<cv::Mat> Preprocessor::extractConnectedComponents(const cv::Mat& binary_mask) {
    std::vector<cv::Mat> components;
    
    if (binary_mask.empty()) {
        return components;
    }
    
    cv::Mat labels, stats, centroids;
    int num_labels = cv::connectedComponentsWithStats(binary_mask, labels, stats, centroids, 8);
    
    // 提取每个连通域（跳过背景标签0）
    for (int i = 1; i < num_labels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > 0) {
            cv::Mat component = cv::Mat::zeros(binary_mask.size(), CV_8UC1);
            component.setTo(255, labels == i);
            components.push_back(component);
        }
    }
    
    return components;
}

// 将掩码恢复到原始尺寸
// @param mask 缩放后的掩码
// @param original_size 原始尺寸
// @return 恢复后的掩码
cv::Mat Preprocessor::resizeMaskToOriginal(const cv::Mat& mask, const cv::Size& original_size) {
    if (mask.empty() || original_size.width <= 0 || original_size.height <= 0) {
        return cv::Mat();
    }
    
    // 如果已经是原始尺寸，直接返回
    if (mask.size() == original_size) {
        return mask.clone();
    }
    
    // 使用最近邻插值恢复尺寸（保持二值特性）
    cv::Mat resized;
    cv::resize(mask, resized, original_size, 0, 0, cv::INTER_NEAREST);
    
    return resized;
}

// 筛选连通域：根据面积、宽高比、尺寸等条件筛选
// @param components 输入连通域列表
// @return 筛选后的连通域列表
std::vector<cv::Mat> Preprocessor::filterComponents(const std::vector<cv::Mat>& components) {
    std::vector<cv::Mat> filtered;
    if (components.empty()) {
        return filtered;
    }

    double min_area = getParam("bg_component_min_area", 1500.0);
    double max_area = getParam("bg_component_max_area", 15000000.0);
    double min_aspect = getParam("bg_component_min_aspect_ratio", 0.2);
    double max_aspect = getParam("bg_component_max_aspect_ratio", 5.0);
    double min_width = getParam("bg_component_min_width", 40.0);
    double min_height = getParam("bg_component_min_height", 40.0);
    int max_count = static_cast<int>(std::round(getParam("bg_component_max_count", 3.0)));

    struct Candidate {
        cv::Mat mask;
        double area;
    };
    std::vector<Candidate> candidates;

    for (const auto& comp : components) {
        if (comp.empty()) {
            continue;
        }

        double area = cv::countNonZero(comp);
        if (area < min_area || area > max_area) {
            continue;
        }

        cv::Rect bbox = cv::boundingRect(comp);
        if (bbox.width < static_cast<int>(std::round(min_width)) ||
            bbox.height < static_cast<int>(std::round(min_height))) {
            continue;
        }

        double aspect = static_cast<double>(bbox.width) / std::max(1, bbox.height);
        if (aspect < 1.0) {
            aspect = 1.0 / aspect;
        }
        if (aspect < min_aspect || aspect > max_aspect) {
            continue;
        }

        candidates.push_back(Candidate{comp, area});
    }

    if (candidates.empty()) {
        return filtered;
    }

    if (max_count > 0 && static_cast<int>(candidates.size()) > max_count) {
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) {
                      return a.area > b.area;
                  });
        candidates.resize(max_count);
    }

    filtered.reserve(candidates.size());
    for (const auto& c : candidates) {
        filtered.push_back(c.mask);
    }

    return filtered;
}

} // namespace visual_pose_estimation

