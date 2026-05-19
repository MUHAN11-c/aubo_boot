# REP 2007 -- Type Adaptation Feature

- **Status**: Final
- **Type**: Standards Track
- **Source**: https://www.ros.org/reps/rep-2007.html

## 核心概念

允许开发者将 ROS 接口类型与自定义类型之间的转换自动化。

### 使用示例

```cpp
template<>
struct rclcpp::TypeAdapter<std::string, std_msgs::msg::String> {
  using is_specialized = std::true_type;
  using custom_type = std::string;
  using ros_message_type = std_msgs::msg::String;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination) {
    destination.data = source;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination) {
    destination = source.data;
  }
};

// 使用
using MyAdaptedType = TypeAdapter<std::string, std_msgs::msg::String>;
auto pub = node->create_publisher<MyAdaptedType>("topic", 10);
```

### 支持范围
- **Topics**: Publisher/Subscriber 均支持
- **Services**: Client/Service 均支持
- **Actions**: Goal/Feedback/Result 均支持

### 设计决策
- 放置在 rclcpp 而非 rcl（避免类型擦除，利用 C++ 所有权机制）
- 在 publisher/subscriber 实例化时指定（而非 publish 时）
- 支持 convert 和 serialize/deserialize 函数

完整内容见：https://www.ros.org/reps/rep-2007.html
