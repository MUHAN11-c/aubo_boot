# URDF 目录结构说明（MoveIt2/ROS2 标准布局）

本目录按 ROS2 与 MoveIt2 常见约定组织机器人描述文件。

## 目录结构

```
urdf/
├── robots/                    # 机器人型号相关的基础 URDF（单机型）
│   └── aubo_e5_10_clean.urdf  # E5_10 机械臂本体（仅 links/joints）
├── xacro/
│   └── inc/                   # 主 xacro 与可复用宏（launch 入口在此）
│       ├── aubo_ros2.xacro   # 主描述文件（含基座、末端、ros2_control）
│       ├── aubo_ros2_control.ros2_control.xacro
│       └── end_effector.xacro
└── README.md
```

## 约定说明

- **robots/**：存放某一型号的“裸”URDF（仅机械臂），便于复用与版本管理。
- **xacro/inc/**：主入口与可被 include 的 xacro 宏；launch 通过 `FindPackageShare(aubo_description)/urdf/xacro/inc/aubo_ros2.xacro` 加载。
- 所有 `xacro:include` 使用 `$(find aubo_description)/urdf/...` 包相对路径，保证在 source 工作空间后 xacro 解析正确。

## 参考

- [ROS2 URDF/Xacro 教程](https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)
- MoveIt2 配置包通常将描述入口放在 description 包的 `urdf/` 或 `urdf/inc/` 下。
