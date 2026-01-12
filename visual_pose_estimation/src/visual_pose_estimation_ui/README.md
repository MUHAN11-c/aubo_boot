# Visual Pose Estimation UI

基于Python的Web UI界面，用于姿态估计系统的可视化操作和交互。

## 项目结构

```
visual_pose_estimation_ui/
├── algorithm_server.py          # 主HTTP服务器（算法API）
├── index.html                    # Web UI主页面
├── web_ui/                       # Web UI资源文件
│   └── resources/               # 静态资源（图片、CSS等）
├── scripts/                      # 辅助脚本
├── configs/                      # 配置文件
├── docs/                         # 文档
├── start_algorithm_server.sh     # 启动脚本
├── stop_web_services.sh         # 停止脚本
└── requirements.txt             # Python依赖
```

## 功能特性

- Web界面操作
- 图像采集和显示
- 姿态估计可视化
- 模板管理
- 调试工具
- 实时日志显示

## 安装依赖

```bash
pip3 install -r requirements.txt
```

## 启动服务

```bash
./start_algorithm_server.sh
```

服务将在以下地址启动：
- Web UI: http://localhost:8088
- API服务器: http://localhost:8088

## 停止服务

```bash
./stop_web_services.sh
```

## 依赖说明

本项目依赖以下ROS2包：
- `interface` - ROS2接口定义包
- `cv_bridge` - OpenCV桥接
- `rclpy` - ROS2 Python客户端库

确保这些包已正确安装并编译。

