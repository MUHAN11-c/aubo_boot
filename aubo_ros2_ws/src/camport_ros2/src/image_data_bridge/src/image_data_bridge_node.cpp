#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <percipio_camera_interface/msg/image_data.hpp>
#include <percipio_camera_interface/msg/camera_status.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

class ImageDataBridgeNode : public rclcpp::Node
{
public:
    ImageDataBridgeNode()
        : Node("image_data_bridge_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("input_image_topic", "color/image_raw");
        this->declare_parameter<std::string>("camera_status_topic", "/camera_status");
        this->declare_parameter<std::string>("output_topic", "/image_data");
        this->declare_parameter<std::string>("camera_id", "DA3234363");
        this->declare_parameter<bool>("use_jpeg_encoding", false);
        this->declare_parameter<int>("jpeg_quality", 90);

        // 获取参数
        input_image_topic_ = this->get_parameter("input_image_topic").as_string();
        std::string camera_status_topic = this->get_parameter("camera_status_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();
        camera_id_ = this->get_parameter("camera_id").as_string();
        use_jpeg_encoding_ = this->get_parameter("use_jpeg_encoding").as_bool();
        jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();

        // 创建发布者
        image_data_pub_ = this->create_publisher<percipio_camera_interface::msg::ImageData>(
            output_topic, 10);

        // 创建订阅者 - 订阅相机状态
        camera_status_sub_ = this->create_subscription<percipio_camera_interface::msg::CameraStatus>(
            camera_status_topic, 10,
            std::bind(&ImageDataBridgeNode::cameraStatusCallback, this, std::placeholders::_1));

        // 延迟初始化image_transport订阅（在构造函数完成后）
        // 使用定时器在节点完全初始化后创建订阅
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (!image_sub_) {
                    // 使用image_transport订阅相机图像（与percipio_camera的发布方式匹配）
                    it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
                    image_sub_ = it_->subscribe(
                        input_image_topic_, 10,
                        std::bind(&ImageDataBridgeNode::imageCallback, this, std::placeholders::_1));
                    init_timer_->cancel(); // 只执行一次
                    RCLCPP_INFO(this->get_logger(), "图像订阅已初始化: %s", input_image_topic_.c_str());
                }
            });

        RCLCPP_INFO(this->get_logger(), "图像数据桥接节点已启动");
        RCLCPP_INFO(this->get_logger(), "  输入图像话题: %s", input_image_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  相机状态话题: %s", camera_status_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  输出话题: %s", output_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  相机ID: %s", camera_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "  JPEG编码: %s", use_jpeg_encoding_ ? "启用" : "禁用");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try {
            // 创建ImageData消息
            auto image_data_msg = percipio_camera_interface::msg::ImageData();

            // 设置header
            image_data_msg.header = msg->header;
            image_data_msg.camera_id = camera_id_;

            // 如果启用了JPEG编码，进行压缩
            if (use_jpeg_encoding_ && 
                (msg->encoding == "rgb8" || msg->encoding == "bgr8" || msg->encoding == "bgra8")) {
                
                // 使用cv_bridge转换为OpenCV格式
                cv_bridge::CvImagePtr cv_ptr;
                try {
                    if (msg->encoding == "rgb8") {
                        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
                    } else if (msg->encoding == "bgr8") {
                        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                    } else {
                        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                    }
                } catch (cv_bridge::Exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
                    return;
                }

                // 转换为BGR格式（OpenCV标准）
                cv::Mat bgr_image;
                if (cv_ptr->encoding == "rgb8") {
                    cv::cvtColor(cv_ptr->image, bgr_image, cv::COLOR_RGB2BGR);
                } else {
                    bgr_image = cv_ptr->image;
                }

                // JPEG编码
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(jpeg_quality_);

                std::vector<uchar> jpeg_buffer;
                cv::imencode(".jpg", bgr_image, jpeg_buffer, compression_params);

                // 设置压缩后的图像消息
                image_data_msg.image.header = msg->header;
                image_data_msg.image.height = msg->height;
                image_data_msg.image.width = msg->width;
                image_data_msg.image.encoding = "jpeg";
                image_data_msg.image.is_bigendian = false;
                image_data_msg.image.step = jpeg_buffer.size();
                image_data_msg.image.data = jpeg_buffer;

                RCLCPP_DEBUG(this->get_logger(), 
                    "JPEG压缩: 原始大小=%zu bytes, 压缩后=%zu bytes (质量=%d)",
                    msg->data.size(), jpeg_buffer.size(), jpeg_quality_);
            } else {
                // 直接使用原始图像数据
                image_data_msg.image = *msg;
            }

            // 如果从相机状态获取了相机ID，使用状态中的ID
            if (!current_camera_id_.empty()) {
                image_data_msg.camera_id = current_camera_id_;
            }

            // 发布消息
            image_data_pub_->publish(image_data_msg);

            RCLCPP_DEBUG(this->get_logger(), 
                "发布ImageData: 相机ID=%s, 图像尺寸=%dx%d, 编码=%s",
                image_data_msg.camera_id.c_str(),
                image_data_msg.image.width,
                image_data_msg.image.height,
                image_data_msg.image.encoding.c_str());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理图像时发生异常: %s", e.what());
        }
    }

    void cameraStatusCallback(const percipio_camera_interface::msg::CameraStatus::SharedPtr msg)
    {
        // 更新相机ID（如果状态消息中有）
        if (!msg->camera_id.empty()) {
            current_camera_id_ = msg->camera_id;
            RCLCPP_DEBUG(this->get_logger(), "更新相机ID: %s", current_camera_id_.c_str());
        }
    }

    rclcpp::Publisher<percipio_camera_interface::msg::ImageData>::SharedPtr image_data_pub_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<percipio_camera_interface::msg::CameraStatus>::SharedPtr camera_status_sub_;
    rclcpp::TimerBase::SharedPtr init_timer_;

    std::string input_image_topic_;
    std::string camera_id_;
    std::string current_camera_id_;
    bool use_jpeg_encoding_;
    int jpeg_quality_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageDataBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

