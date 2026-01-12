#include "visual_pose_estimation/config_reader.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>
#include <yaml-cpp/yaml.h>

namespace visual_pose_estimation {

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

std::string ConfigReader::getString(const std::string& key, const std::string& default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        return it->second;
    }
    return default_value;
}

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

bool ConfigReader::getBool(const std::string& key, bool default_value) const {
    auto it = config_map_.find(key);
    if (it != config_map_.end()) {
        std::string value = it->second;
        std::transform(value.begin(), value.end(), value.begin(), ::tolower);
        return (value == "true" || value == "1" || value == "yes" || value == "on");
    }
    return default_value;
}

bool ConfigReader::has(const std::string& key) const {
    return config_map_.find(key) != config_map_.end();
}

} // namespace visual_pose_estimation

