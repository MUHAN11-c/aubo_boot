# REP 2005 -- ROS 2 Common Packages

- **Status**: Active
- **Type**: Informational
- **Source**: https://www.ros.org/reps/rep-2005.html

## 核心概念

本 REP 列出了 ROS 2 生态系统中**质量可靠、广泛使用、积极维护**的通用包。

### 收录标准
- 开源
- 与 ROS 2 项目直接关联
- 广泛适用
- 有证据证明被社区非平凡使用
- 质量等级 ≥ 3（见 REP 2004）
- 由组织或至少 2 人维护

### 内容分类（Humble+）

**构建系统与工具**: ament, colcon, bloom, rosdep, vcstool

**第三方库**: poco_vendor, pybind11_vendor, etc.

**ROS 接口管线**: rosidl (generators, parsers, typesupport)

**中间件**: rmw_fastrtps, rmw_cyclonedds, rmw_connextdds

**客户端库**: rcl, rclcpp, rclcpp_action, rclcpp_lifecycle, rclpy, rclc (micro-ROS)

**编排**: launch, launch_ros, lifecycle, composition

**特性**:
- Navigation2
- MoveIt2
- rosbag2
- tf2
- robot_state_publisher
- image_transport
- vision_opencv

**工具**: rviz2, rqt, ros2cli, ros2doctor

### 安全漏洞报告
安全漏洞请邮件 security@openrobotics.org

完整内容见：https://www.ros.org/reps/rep-2005.html
