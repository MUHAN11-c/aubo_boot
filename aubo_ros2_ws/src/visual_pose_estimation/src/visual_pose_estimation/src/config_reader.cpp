// 配置文件读取器实现
// 用于从YAML配置文件中读取参数，支持嵌套结构

#include "visual_pose_estimation/config_reader.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>
#include <yaml-cpp/yaml.h>

namespace visual_pose_estimation {

// 从YAML文件加载配置
// @param config_file 配置文件路径
// @return 成功返回true，失败返回false
bool ConfigReader::loadFromFile(const std::string& config_file) {
    try {
        YAML::Node config = YAML::LoadFile(config_file);
        
        // 递归遍历YAML节点，支持嵌套结构
        std::function<void(const YAML::Node&, const std::string&)> traverse;
        traverse = [&](const YAML::Node& node, const std::string& prefix) {
            if (node.IsMap()) {
                for (auto it = node.begin(); it != node.end(); ++it) {
                    std::string key = it->first.as<std::string>();
                    std::string full_key = prefix.empty() ? key : prefix + "." + key;
                    
                    const YAML::Node& value = it->second;
                    if (value.IsScalar()) {
                        config_map_[full_key] = value.as<std::string>();
                    } else if (value.IsMap() || value.IsSequence()) {
                        traverse(value, full_key);
                    }
                }
            } else if (node.IsSequence()) {
                for (size_t i = 0; i < node.size(); ++i) {
                    std::string full_key = prefix + "[" + std::to_string(i) + "]";
                    traverse(node[i], full_key);
                }
            } else if (node.IsScalar()) {
                config_map_[prefix] = node.as<std::string>();
            }
        };
        
        traverse(config, "");
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

// 获取字符串类型配置值
// @param key 配置键名（支持点号分隔的嵌套键名）
// @param default_value 默认值
// @return 配置值或默认值
std::string ConfigReader::getString(const std::string& key, const std::string& default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        return it->second;
    }
    return default_value;
}

// 获取整数类型配置值
// @param key 配置键名（支持点号分隔的嵌套键名）
// @param default_value 默认值
// @return 配置值或默认值
int ConfigReader::getInt(const std::string& key, int default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        try {
            return std::stoi(it->second);
        } catch (const std::exception&) {
            return default_value;
        }
    }
    return default_value;
}

// 获取浮点数类型配置值
// @param key 配置键名（支持点号分隔的嵌套键名）
// @param default_value 默认值
// @return 配置值或默认值
double ConfigReader::getDouble(const std::string& key, double default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        try {
            return std::stod(it->second);
        } catch (const std::exception&) {
            return default_value;
        }
    }
    return default_value;
}

// 获取布尔类型配置值
// @param key 配置键名（支持点号分隔的嵌套键名）
// @param default_value 默认值
// @return 配置值或默认值（支持"true"/"1"/"yes"/"on"等字符串表示）
bool ConfigReader::getBool(const std::string& key, bool default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        return (value == "true" || value == "1" || value == "yes" || value == "on");
    }
    return default_value;
}

// 检查配置键是否存在
// @param key 配置键名（支持点号分隔的嵌套键名）
// @return 存在返回true，不存在返回false
bool ConfigReader::has(const std::string& key) const {
    return config_map_.find(key) != config_map_.end();
}

} // namespace visual_pose_estimation

