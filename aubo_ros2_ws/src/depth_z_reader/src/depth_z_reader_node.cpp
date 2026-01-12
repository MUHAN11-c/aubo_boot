#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>

class DepthZReaderNode : public rclcpp::Node
{
public:
    DepthZReaderNode()
        : Node("depth_z_reader_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("depth_image_topic", "/camera/depth/image_raw");
        this->declare_parameter<int>("pixel_x", 350);  // 像素 x 坐标
        this->declare_parameter<int>("pixel_y", 175);  // 像素 y 坐标
        this->declare_parameter<double>("depth_scale", 0.00025);  // 深度缩放因子，默认适用于 scale_unit=0.25 的相机（转换为米）
        this->declare_parameter<bool>("publish_center_depth", true);  // 是否发布中心点深度
        this->declare_parameter<double>("publish_rate", 10.0);  // 发布频率
        this->declare_parameter<bool>("search_valid_depth", true);  // 如果中心点无效，是否搜索附近有效深度值
        this->declare_parameter<int>("search_radius", 50);  // 搜索半径（像素）

        // 获取参数
        depth_topic_ = this->get_parameter("depth_image_topic").as_string();
        pixel_x_ = this->get_parameter("pixel_x").as_int();
        pixel_y_ = this->get_parameter("pixel_y").as_int();
        depth_scale_ = this->get_parameter("depth_scale").as_double();
        bool publish_center = this->get_parameter("publish_center_depth").as_bool();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        search_valid_depth_ = this->get_parameter("search_valid_depth").as_bool();
        search_radius_ = this->get_parameter("search_radius").as_int();

        // 创建深度值发布者
        if (publish_center) {
            center_depth_pub_ = this->create_publisher<std_msgs::msg::Float32>(
                "depth_z/center", 10);
        }

        // 延迟初始化 image_transport 订阅（在构造函数完成后）
        // 使用定时器在节点完全初始化后创建订阅
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (!it_) {
                    // 使用 image_transport 订阅深度图像
                    // 注意：也可以直接使用 create_subscription，但 image_transport 提供更好的兼容性和扩展性
                    // 如果遇到 QoS 不匹配问题，可以改用：
                    // depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                    //     depth_topic_, rclcpp::SensorDataQoS(), 
                    //     std::bind(&DepthZReaderNode::depthImageCallback, this, std::placeholders::_1));
                    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
                    try {
                        depth_image_sub_ = it_->subscribe(
                            depth_topic_,
                            1,
                            std::bind(&DepthZReaderNode::depthImageCallback, this, std::placeholders::_1)
                        );
                        init_timer_->cancel(); // 只执行一次
                        RCLCPP_INFO(this->get_logger(), "✓ 深度图像订阅已初始化: %s", depth_topic_.c_str());
                        RCLCPP_INFO(this->get_logger(), "  等待深度图像数据...");
                        RCLCPP_INFO(this->get_logger(), "  提示: 如果长时间未收到数据，请检查:");
                        RCLCPP_INFO(this->get_logger(), "    1. 相机节点是否正在运行 (ros2 node list)");
                        RCLCPP_INFO(this->get_logger(), "    2. 话题是否有数据发布 (ros2 topic hz %s)", depth_topic_.c_str());
                        RCLCPP_INFO(this->get_logger(), "    3. 相机节点日志中是否有错误信息");
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "订阅深度图像话题失败: %s", e.what());
                        RCLCPP_WARN(this->get_logger(), "请检查话题名称是否正确: %s", depth_topic_.c_str());
                    }
                }
            });

        // 创建定时器定期发布深度值
        if (publish_rate > 0.0) {
            auto timer_period = std::chrono::milliseconds(
                static_cast<int>(1000.0 / publish_rate));
            timer_ = this->create_wall_timer(
                timer_period,
                std::bind(&DepthZReaderNode::timerCallback, this));
        }

        RCLCPP_INFO(this->get_logger(), "深度 z 读取节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅深度图像话题: %s", depth_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "深度缩放因子: %f", depth_scale_);
        if (pixel_x_ < 0 || pixel_y_ < 0) {
            RCLCPP_INFO(this->get_logger(), "将读取图像中心点的深度值");
        } else {
            RCLCPP_INFO(this->get_logger(), "将读取像素 (%d, %d) 的深度值", pixel_x_, pixel_y_);
        }
        if (search_valid_depth_) {
            RCLCPP_INFO(this->get_logger(), "已启用自动搜索有效深度值 (搜索半径: %d 像素)", search_radius_);
        }
    }

private:
    void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        // 调试：确认回调被调用
        RCLCPP_DEBUG(this->get_logger(), "收到深度图像消息 (尺寸: %dx%d, 编码: %s)", 
            msg->width, msg->height, msg->encoding.c_str());
        
        // 标记已收到数据（即使后续处理失败）
        if (!has_depth_data_) {
            has_depth_data_ = true;
            RCLCPP_INFO(this->get_logger(), "✓ 开始接收深度图像数据 (尺寸: %dx%d, 编码: %s)", 
                msg->width, msg->height, msg->encoding.c_str());
        }
        
        try {
            // 将 ROS 图像消息转换为 OpenCV 图像
            cv_bridge::CvImagePtr cv_ptr;
            
            // 检查图像编码格式
            std::string encoding = msg->encoding;
            if (encoding != sensor_msgs::image_encodings::TYPE_16UC1 && 
                encoding != sensor_msgs::image_encodings::MONO16) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "深度图像编码格式为 %s，期望 16UC1 或 MONO16", encoding.c_str());
            }
            
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            
            cv::Mat depth_image = cv_ptr->image;
            
            if (depth_image.empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "接收到空的深度图像");
                return;
            }

            // 确定要读取的像素位置
            int x, y;
            if (pixel_x_ < 0 || pixel_y_ < 0) {
                // 使用图像中心
                x = depth_image.cols / 2;
                y = depth_image.rows / 2;
            } else {
                x = pixel_x_;
                y = pixel_y_;
            }

            // 检查像素坐标是否有效
            if (x < 0 || x >= depth_image.cols || y < 0 || y >= depth_image.rows) {
                RCLCPP_WARN(this->get_logger(), 
                    "像素坐标 (%d, %d) 超出图像范围 (%d x %d)", 
                    x, y, depth_image.cols, depth_image.rows);
                return;
            }

            // 读取深度值（16位无符号整数，通常是毫米）
            uint16_t depth_raw = depth_image.at<uint16_t>(y, x);
            
            // 检查无效深度值（0 或 65535 通常表示无效）
            if (depth_raw == 0 || depth_raw == 65535) {
                // 如果启用搜索，尝试在附近找到有效深度值
                if (search_valid_depth_) {
                    int found_x = x, found_y = y;
                    uint16_t found_depth = searchValidDepth(depth_image, x, y, found_x, found_y);
                    if (found_depth > 0 && found_depth < 65535) {
                        depth_raw = found_depth;
                        x = found_x;
                        y = found_y;
                        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "中心点深度无效，在 (%d, %d) 找到有效深度值", found_x, found_y);
                    } else {
                        // 即使深度值无效，也更新状态（但使用特殊标记）
                        latest_depth_raw_ = depth_raw;
                        latest_depth_z_ = 0.0;
                        return;
                    }
                } else {
                    // 即使深度值无效，也更新状态（但使用特殊标记）
                    latest_depth_raw_ = depth_raw;
                    latest_depth_z_ = 0.0;
                    return;
                }
            }
            
            // 转换为米（根据深度缩放因子）
            double depth_z = static_cast<double>(depth_raw) * depth_scale_;

            // 保存最新的深度值
            latest_depth_z_ = depth_z;
            latest_depth_raw_ = depth_raw;
            latest_pixel_x_ = x;
            latest_pixel_y_ = y;

            // 发布中心点深度值
            if (center_depth_pub_) {
                std_msgs::msg::Float32 depth_msg;
                depth_msg.data = depth_z;
                center_depth_pub_->publish(depth_msg);
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理深度图像时发生异常: %s", e.what());
        }
    }

    void timerCallback()
    {
        if (has_depth_data_) {
            if (latest_depth_raw_ == 0 || latest_depth_raw_ == 65535) {
                int x, y;
                if (pixel_x_ < 0 || pixel_y_ < 0) {
                    x = -1;  // 表示中心
                    y = -1;
                } else {
                    x = pixel_x_;
                    y = pixel_y_;
                }
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "深度 z 值 (像素 %d, %d): 无效 (原始值: %u) - 可能是遮挡或超出测量范围", 
                    x, y, latest_depth_raw_);
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "深度 z 值 (像素 %d, %d): %.4f 米 (原始值: %u)", 
                    latest_pixel_x_, latest_pixel_y_, latest_depth_z_, latest_depth_raw_);
            }
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "尚未接收到深度数据 - 请检查话题 %s 是否有数据发布", depth_topic_.c_str());
        }
    }

    // 在附近搜索有效深度值
    uint16_t searchValidDepth(const cv::Mat& depth_image, int center_x, int center_y, 
                              int& found_x, int& found_y) {
        for (int r = 1; r <= search_radius_; r++) {
            // 从内到外搜索圆形区域
            for (int dy = -r; dy <= r; dy++) {
                for (int dx = -r; dx <= r; dx++) {
                    // 检查是否在圆形范围内
                    if (dx*dx + dy*dy > r*r) continue;
                    
                    int check_x = center_x + dx;
                    int check_y = center_y + dy;
                    
                    // 检查边界
                    if (check_x < 0 || check_x >= depth_image.cols || 
                        check_y < 0 || check_y >= depth_image.rows) {
                        continue;
                    }
                    
                    uint16_t depth = depth_image.at<uint16_t>(check_y, check_x);
                    // 检查是否为有效深度值
                    if (depth > 0 && depth < 65535) {
                        found_x = check_x;
                        found_y = check_y;
                        return depth;
                    }
                }
            }
        }
        return 0;  // 未找到有效深度值
    }

        std::shared_ptr<image_transport::ImageTransport> it_;
        image_transport::Subscriber depth_image_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr center_depth_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    std::string depth_topic_;

    int pixel_x_;
    int pixel_y_;
    double depth_scale_;
    bool search_valid_depth_;
    int search_radius_;
    double latest_depth_z_ = 0.0;
    uint16_t latest_depth_raw_ = 0;
    int latest_pixel_x_ = -1;
    int latest_pixel_y_ = -1;
    bool has_depth_data_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthZReaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

