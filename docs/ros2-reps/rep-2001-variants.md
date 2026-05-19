# REP 2001 -- ROS 2 Variants

- **Status**: Active
- **Type**: Informational
- **Source**: https://www.ros.org/reps/rep-2001.html

## 核心内容摘要

ROS 2 提供多个"变体"(variant)，代表不同层次的安装入口。

### 变体层级

| 变体 | 内容 | GUI |
|------|------|-----|
| **ros_core** | 核心通信协议：rclcpp, rclpy, rcl, common_interfaces, rmw | 无 |
| **ros_base** | ros_core + tf2, robot_state_publisher, urdf, kdl_parser, rosbag2 | 无 |
| **desktop** | ros_base + rviz2, rqt, demo_nodes, turtlesim, examples | 有 |
| **perception** | ros_base + vision_opencv, image_pipeline, pcl | 无 |
| **simulation** | ros_base + gazebo bridge | 无 |
| **desktop_full** | desktop + perception + simulation | 有 |

### Humble ros_core 构成
```
ros_core:
  packages: [ament_cmake, ament_cmake_auto, class_loader, common_interfaces,
             launch, launch_ros, launch_testing, pluginlib,
             rcl, rcl_lifecycle, rclcpp, rclcpp_action, rclcpp_lifecycle, rclpy,
             ros2cli_common_extensions, ros2launch,
             rosidl_default_generators, rosidl_default_runtime,
             sros2, sros2_cmake]
  rmw: Fast-RTPS (默认) / CycloneDDS / Connext
```

### 独立变体（跨发行版）
- **ros-build-essential**: cmake, git, python3, python3-setuptools
- **ros-dev-tools**: + bloom, colcon, rosdep, vcstool, wget

完整内容见：https://www.ros.org/reps/rep-2001.html
