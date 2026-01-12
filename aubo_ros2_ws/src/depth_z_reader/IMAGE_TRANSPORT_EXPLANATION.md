# 为什么使用 image_transport？

## 两种订阅方式的对比

### 方式 1: 使用 image_transport（当前方式）

```cpp
#include <image_transport/image_transport.hpp>

it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
depth_image_sub_ = it_->subscribe(
    depth_topic_,
    1,
    std::bind(&DepthZReaderNode::depthImageCallback, this, std::placeholders::_1)
);
```

**优点：**
- ✅ 与发布端一致（相机节点使用 `image_transport::create_publisher`）
- ✅ 支持图像压缩传输（可选，减少带宽）
- ✅ 更好的 QoS 兼容性
- ✅ 支持多种传输插件（compressed, theora 等）
- ✅ ROS 图像传输的标准方式

**缺点：**
- ❌ 需要额外的依赖（image_transport）
- ❌ 代码稍微复杂一些
- ❌ 需要处理 `shared_from_this()` 的时机问题

### 方式 2: 直接使用 ROS2 订阅（更简单）

```cpp
// 不需要 image_transport 头文件

depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_,
    rclcpp::SensorDataQoS(),  // 或 rclcpp::QoS(10)
    std::bind(&DepthZReaderNode::depthImageCallback, this, std::placeholders::_1)
);
```

**优点：**
- ✅ 代码更简单直接
- ✅ 不需要额外依赖
- ✅ 不需要处理 `shared_from_this()` 问题
- ✅ 更轻量级

**缺点：**
- ❌ 不支持图像压缩传输
- ❌ 需要手动配置 QoS（可能与发布端不匹配）
- ❌ 与使用 `image_transport` 发布的节点可能不完全兼容

## 当前选择 image_transport 的原因

1. **兼容性**：相机节点使用 `image_transport::create_publisher`，使用 `image_transport` 订阅可以确保更好的兼容性
2. **标准实践**：ROS 中图像传输的标准方式是使用 `image_transport`
3. **未来扩展**：如果将来需要压缩传输或其他功能，`image_transport` 更容易扩展

## 如果遇到问题，可以改用直接订阅

如果使用 `image_transport` 遇到 QoS 不匹配或数据接收问题，可以改用直接订阅：

```cpp
// 在构造函数中直接创建订阅（不需要延迟初始化）
depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_topic_,
    rclcpp::SensorDataQoS().keep_last(10),  // 匹配相机节点的 QoS
    std::bind(&DepthZReaderNode::depthImageCallback, this, std::placeholders::_1)
);
```

## 总结

- **简单场景**：直接订阅更简单
- **标准实践**：image_transport 更符合 ROS 规范
- **当前代码**：使用 image_transport 是为了与相机节点保持一致

两种方式都可以工作，选择哪种主要取决于：
- 是否需要图像压缩
- 是否需要与使用 image_transport 的发布端完全兼容
- 代码简洁性 vs 功能扩展性

