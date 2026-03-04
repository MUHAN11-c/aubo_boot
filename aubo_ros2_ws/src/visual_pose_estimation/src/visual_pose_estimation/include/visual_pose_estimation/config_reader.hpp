#ifndef VISUAL_POSE_ESTIMATION_CONFIG_READER_HPP
#define VISUAL_POSE_ESTIMATION_CONFIG_READER_HPP

#include <string>
#include <map>
#include <memory>

namespace visual_pose_estimation {

/**
 * @brief 配置文件读取类
 * 
 * 负责读取和解析YAML配置文件
 */
class ConfigReader {
public:
    /**
     * @brief 构造函数
     */
    ConfigReader() = default;

    /**
     * @brief 析构函数
     */
    ~ConfigReader() = default;

    /**
     * @brief 从文件加载配置
     * @param config_file 配置文件路径
     * @return 是否加载成功
     */
    bool loadFromFile(const std::string& config_file);

    /**
     * @brief 获取配置值（字符串）
     * @param key 配置键
     * @param default_value 默认值
     * @return 配置值
     */
    std::string getString(const std::string& key, const std::string& default_value = "") const;
    
    /**
     * @brief 获取配置值（整数）
     * @param key 配置键
     * @param default_value 默认值
     * @return 配置值
     */
    int getInt(const std::string& key, int default_value = 0) const;
    
    /**
     * @brief 获取配置值（浮点数）
     * @param key 配置键
     * @param default_value 默认值
     * @return 配置值
     */
    double getDouble(const std::string& key, double default_value = 0.0) const;
    
    /**
     * @brief 获取配置值（布尔值）
     * @param key 配置键
     * @param default_value 默认值
     * @return 配置值
     */
    bool getBool(const std::string& key, bool default_value = false) const;

    /**
     * @brief 检查配置是否存在
     * @param key 配置键
     * @return 是否存在
     */
    bool has(const std::string& key) const;

private:
    std::map<std::string, std::string> config_map_;
};

} // namespace visual_pose_estimation

#endif // VISUAL_POSE_ESTIMATION_CONFIG_READER_HPP

