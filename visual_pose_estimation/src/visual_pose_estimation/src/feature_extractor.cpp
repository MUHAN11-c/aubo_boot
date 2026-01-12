#include "visual_pose_estimation/feature_extractor.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <thread>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace visual_pose_estimation {

FeatureExtractor::FeatureExtractor() {
    initializeDefaultParameters();
}

void FeatureExtractor::initializeDefaultParameters() {
    // 连通域筛选参数
    parameters_["min_component_area"] = 2000.0;      // 最小连通域面积
    parameters_["max_component_area"] = 10000000.0;  // 最大连通域面积
    
    // 大圆提取参数
    parameters_["big_circle_combine_contours"] = 1.0;  // 是否合并轮廓
    parameters_["big_circle_min_area"] = 100.0;        // 最小轮廓面积
    
    // 小圆（阀体）提取参数
    parameters_["small_circle_erode_kernel"] = 11.0;
    parameters_["small_circle_erode_iterations"] = 5.0;
    parameters_["small_circle_largest_cc"] = 1.0;       // 是否保留最大连通域
    parameters_["small_circle_dilate_kernel"] = 9.0;
    parameters_["small_circle_dilate_iterations"] = 1.0;
    
    // 线程参数
    parameters_["max_threads"] = 8.0;  // 最大线程数
}

void FeatureExtractor::setParameters(const std::map<std::string, double>& params) {
    for (const auto& [key, value] : params) {
        parameters_[key] = value;
    }
}

std::map<std::string, double> FeatureExtractor::getParameters() const {
    return parameters_;
}

double FeatureExtractor::getParam(const std::string& key, double fallback) const {
    // 首先尝试从配置文件读取的嵌套键名（preprocess.feature_extraction.*）
    std::string config_key = "preprocess.feature_extraction." + key;
    auto it = parameters_.find(config_key);
    if (it != parameters_.end()) {
        return it->second;
    }
    
    // 尝试 preprocess.*
    config_key = "preprocess." + key;
    it = parameters_.find(config_key);
    if (it != parameters_.end()) {
        return it->second;
    }
    
    // 如果不存在，尝试使用原始键名
    it = parameters_.find(key);
    if (it != parameters_.end()) {
        return it->second;
    }
    return fallback;
}

std::vector<cv::Mat> FeatureExtractor::filterComponents(const std::vector<cv::Mat>& components) {
    std::vector<cv::Mat> filtered;
    
    double min_area = getParam("feature_extraction.min_component_area", 2000.0);
    double max_area = getParam("feature_extraction.max_component_area", 10000000.0);
    
    for (const auto& component : components) {
        if (component.empty()) {
            continue;
        }
        
        // 计算连通域面积
        int area = cv::countNonZero(component);
        
        // 筛选条件：面积在范围内
        if (area >= static_cast<int>(min_area) && area <= static_cast<int>(max_area)) {
            filtered.push_back(component);
        }
    }
    
    return filtered;
}

std::vector<ComponentFeature> FeatureExtractor::extractFeatures(const std::vector<cv::Mat>& components) {
    std::vector<ComponentFeature> results;
    
    if (components.empty()) {
        return results;
    }
    
    // 步骤1: 初步筛选干扰连通域
    std::vector<cv::Mat> filtered_components = filterComponents(components);
    
    if (filtered_components.empty()) {
        return results;
    }
    
    // 步骤2: 根据连通域数量确定线程数
    size_t num_components = filtered_components.size();
    size_t max_threads = static_cast<size_t>(getParam("feature_extraction.max_threads", 8.0));
    size_t num_threads = std::min(num_components, max_threads);
    num_threads = std::max(num_threads, size_t(1));  // 至少1个线程
    
    // 步骤3: 初始化结果容器
    results.resize(num_components);
    
    // 步骤4: 创建线程并分配任务
    std::vector<std::thread> threads;
    size_t components_per_thread = (num_components + num_threads - 1) / num_threads;
    
    for (size_t t = 0; t < num_threads; ++t) {
        size_t start_idx = t * components_per_thread;
        size_t end_idx = std::min(start_idx + components_per_thread, num_components);
        
        if (start_idx >= num_components) {
            break;
        }
        
        // 创建线程处理分配的连通域
        threads.emplace_back([this, &filtered_components, &results, start_idx, end_idx]() {
            for (size_t i = start_idx; i < end_idx; ++i) {
                processComponentThread(filtered_components[i], results, i);
            }
        });
    }
    
    // 步骤5: 等待所有线程完成
    for (auto& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }
    
    // 步骤6: 移除无效的特征（面积为零的）
    results.erase(
        std::remove_if(results.begin(), results.end(),
            [](const ComponentFeature& feat) {
                return feat.workpiece_area == 0.0;
            }),
        results.end()
    );
    
    return results;
}

void FeatureExtractor::processComponentThread(
    const cv::Mat& component_mask,
    std::vector<ComponentFeature>& results,
    size_t index) {
    
    // 提取特征
    ComponentFeature feature = extractSingleComponentFeature(component_mask);
    
    // 加锁写入结果
    {
        std::lock_guard<std::mutex> lock(result_mutex_);
        if (index < results.size()) {
            results[index] = feature;
        }
    }
}

ComponentFeature FeatureExtractor::extractSingleComponentFeature(const cv::Mat& component_mask) {
    ComponentFeature feature;
    
    if (component_mask.empty()) {
        return feature;
    }
    
    // 保存原始掩码
    feature.component_mask = component_mask.clone();
    
    // 计算工件连通域面积
    feature.workpiece_area = cv::countNonZero(component_mask);
    
    // 提取工件外接圆（大圆）
    bool combine_contours = getParam("big_circle.combine_contours", 1.0) > 0.5;
    int min_area = static_cast<int>(getParam("big_circle.min_area", 100.0));
    auto [workpiece_center, workpiece_radius] = extractWorkpieceCircle(component_mask, combine_contours, min_area);
    
    feature.workpiece_center = workpiece_center;
    feature.workpiece_radius = workpiece_radius;
    
    // 提取阀体外接圆（小圆）
    auto [valve_center, valve_radius, valve_mask] = extractValveCircle(component_mask);
    
    feature.valve_center = valve_center;
    feature.valve_radius = valve_radius;
    feature.valve_area = cv::countNonZero(valve_mask);
    
    // 计算标准化旋转角度
    feature.standardized_angle = calculateStandardizedAngle(workpiece_center, valve_center);
    feature.standardized_angle_deg = feature.standardized_angle * 180.0 / M_PI;
    
    return feature;
}

std::pair<cv::Point2f, float> FeatureExtractor::extractWorkpieceCircle(
    const cv::Mat& mask,
    bool combine_contours,
    int min_area) {
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    if (contours.empty()) {
        return {cv::Point2f(0, 0), 0.0f};
    }
    
    if (combine_contours) {
        // 合并所有满足条件的轮廓
        std::vector<cv::Point> all_points;
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) >= min_area) {
                all_points.insert(all_points.end(), contour.begin(), contour.end());
            }
        }
        
        if (all_points.empty()) {
            // 如果没有满足条件的轮廓，使用最大轮廓
            auto max_it = std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });
            if (max_it != contours.end()) {
                all_points = *max_it;
            } else {
                all_points = contours[0];
            }
        }
        
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(all_points, center, radius);
        return {center, radius};
    } else {
        // 使用最大轮廓
        auto max_contour = *std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });
        
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(max_contour, center, radius);
        return {center, radius};
    }
}

std::tuple<cv::Point2f, float, cv::Mat> FeatureExtractor::extractValveCircle(const cv::Mat& mask) {
    if (mask.empty()) {
        return {cv::Point2f(0, 0), 0.0f, cv::Mat()};
    }
    
    // 获取参数（支持嵌套配置路径）
    int erode_kernel = static_cast<int>(getParam("small_circle.erode_kernel", 11.0));
    int erode_iterations = static_cast<int>(getParam("small_circle.erode_iterations", 5.0));
    bool keep_largest_cc = getParam("small_circle.largest_cc", 1.0) > 0.5;
    int dilate_kernel = static_cast<int>(getParam("small_circle.dilate_kernel", 9.0));
    int dilate_iterations = static_cast<int>(getParam("small_circle.dilate_iterations", 1.0));
    
    // 确保核大小为奇数
    if (erode_kernel % 2 == 0) {
        erode_kernel += 1;
    }
    if (dilate_kernel % 2 == 0) {
        dilate_kernel += 1;
    }
    
    // 步骤1: 腐蚀
    cv::Mat kernel_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erode_kernel, erode_kernel));
    cv::Mat eroded;
    cv::erode(mask, eroded, kernel_erode, cv::Point(-1, -1), erode_iterations);
    
    cv::Mat processed = eroded.clone();
    
    // 步骤2: 保留最大连通域
    if (keep_largest_cc) {
        cv::Mat labels, stats, centroids;
        int num_labels = cv::connectedComponentsWithStats(processed, labels, stats, centroids, 8);
        
        if (num_labels > 1) {
            // 找到最大连通域
            int max_area = 0;
            int max_label = 1;
            for (int i = 1; i < num_labels; ++i) {
                int area = stats.at<int>(i, cv::CC_STAT_AREA);
                if (area > max_area) {
                    max_area = area;
                    max_label = i;
                }
            }
            
            cv::Mat largest_cc = cv::Mat::zeros(processed.size(), CV_8UC1);
            largest_cc.setTo(255, labels == max_label);
            processed = largest_cc;
        }
    }
    
    // 步骤3: 膨胀
    if (dilate_iterations > 0 && dilate_kernel > 0) {
        cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_kernel, dilate_kernel));
        cv::dilate(processed, processed, kernel_dilate, cv::Point(-1, -1), dilate_iterations);
    }
    
    // 步骤4: 提取小圆（使用与大圆相同的方法）
    auto [center, radius] = extractWorkpieceCircle(processed, false, 0);
    
    return {center, radius, processed};
}

double FeatureExtractor::calculateStandardizedAngle(
    const cv::Point2f& workpiece_center,
    const cv::Point2f& valve_center) {
    
    // 计算从工件中心到阀体中心的向量
    double dx = valve_center.x - workpiece_center.x;
    double dy = valve_center.y - workpiece_center.y;
    
    // 使用atan2计算角度（从x轴正方向开始，逆时针为正）
    // 注意：OpenCV的坐标系y轴向下，所以需要调整
    double angle = std::atan2(dy, dx);
    
    return angle;
}

} // namespace visual_pose_estimation

