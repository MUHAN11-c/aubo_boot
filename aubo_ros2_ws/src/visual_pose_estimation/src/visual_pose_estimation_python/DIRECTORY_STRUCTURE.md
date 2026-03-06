# 目录结构标准化说明

本文档说明了 `visual_pose_estimation_python` 包的标准化目录结构。

## 标准化后的目录结构

```
visual_pose_estimation_python/
├── visual_pose_estimation_python/  # Python模块（核心代码）
│   ├── __init__.py
│   ├── main.py                     # 主节点入口
│   ├── config_reader.py            # 配置读取器
│   ├── preprocessor.py             # 深度图预处理器
│   ├── feature_extractor.py        # 特征提取器
│   ├── template_standardizer.py    # 模板标准化器
│   ├── pose_estimator.py           # 姿态估计器
│   ├── ros2_communication.py       # ROS2通信模块
│   └── debug_visualizer.py         # 调试可视化器
│
├── configs/                         # 旧版配置文件（保留兼容）
│   └── trigger_depth_thresholds_camera.json  # 深度阈值配置
│
├── launch/                          # ROS2启动文件
│   └── visual_pose_estimation_python.launch.py
│
├── test/                            # 核心模块测试
│   ├── test_copyright.py           # 版权测试
│   ├── test_flake8.py              # 代码风格测试
│   └── test_pep257.py              # 文档风格测试
│
├── web_ui/                          # Web UI界面
│   ├── index.html                  # Web UI主页面
│   ├── README.md                   # Web UI文档
│   ├── requirements.txt            # Python依赖
│   │
│   ├── configs/                    # 默认配置目录（推荐统一放此处）
│   │   ├── debug_thresholds.json   # 调试阈值配置
│   │   ├── app_config.json         # 模板根路径、拍照姿态等
│   │   ├── config_paths.md         # 配置路径说明
│   │   ├── camera_intrinsics.yaml  # 相机内参（标准名）
│   │   └── hand_eye_calibration.yaml  # 手眼标定（标准名）
│   │
│   ├── scripts/                    # Web UI后端脚本
│   │   ├── http_bridge_server.py  # HTTP桥接服务器
│   │   └── params_manager.py      # 参数管理器
│   │
│   ├── docs/                       # Web UI文档
│   │   ├── DEBUG_IMPLEMENTATION_SUMMARY.md
│   │   ├── DEBUG_USAGE.md
│   │   ├── FIX_APPLIED.md
│   │   ├── PROJECT_SUMMARY.md
│   │   ├── QUICK_REFERENCE.md
│   │   ├── TEST_GUIDE.md
│   │   ├── 使用示例.md
│   │   └── 快速开始.md
│   │
│   ├── test/                       # Web UI测试
│   │   ├── test_color.py          # 颜色测试
│   │   ├── test_debug_api.py      # API测试
│   │   └── test_startup.sh        # 启动测试
│   │
│   ├── resources/                  # Web UI资源文件
│   │   └── logo.png               # Logo图片
│   │
│   ├── start_web_ui.sh            # 启动脚本
│   ├── stop_web_ui.sh             # 停止脚本
│   ├── check_installation.sh      # 安装检查脚本
│   └── start_service_now.sh       # 快速启动脚本
│
├── resource/                       # ROS2资源文件
│   └── visual_pose_estimation_python
│
├── .gitignore                      # Git忽略文件
├── package.xml                     # ROS2包配置
├── setup.py                        # Python包配置
├── setup.cfg                       # 配置文件
├── README.md                       # 主文档
└── DIRECTORY_STRUCTURE.md          # 本文档
```

## 目录说明

### 核心模块 (`visual_pose_estimation_python/`)
包含所有核心Python模块，实现了视觉姿态估计的主要功能。

### 配置文件 (`configs/`)
存储各种配置文件，包括默认配置、手眼标定和深度阈值配置。

### 启动文件 (`launch/`)
ROS2启动文件，用于启动节点。

### 测试文件 (`test/`)
核心模块的单元测试和代码质量测试。

### Web UI (`web_ui/`)
完整的Web界面系统，包含前端、后端、配置、文档和测试。

### 资源文件 (`resource/`)
ROS2包所需的资源文件。

## 标准化操作

已完成的标准化操作：

1. ✅ 创建 `web_ui/test/` 目录，移动测试文件
2. ✅ 整理 `web_ui/docs/` 目录，统一管理文档
3. ✅ 整理 `web_ui/resources/` 目录，统一管理资源文件
4. ✅ 删除备份文件：
   - `web_ui/scripts/http_bridge_server.py.backup`
5. ✅ 创建 `.gitignore` 文件，忽略临时文件和缓存

## 注意事项

- `__pycache__/` 目录已被 `.gitignore` 忽略
- 运行时文件（如 `.web_ui.pid`）已被 `.gitignore` 忽略
- 所有文档已统一整理到相应的 `docs/` 目录
- 测试文件已分类到相应的 `test/` 目录

## 维护建议

1. Web UI相关的文档应放在 `web_ui/docs/` 目录
2. 新的测试文件应根据功能放入相应的 `test/` 目录
3. 配置文件应统一放在相应的 `configs/` 目录
4. 避免在根目录直接放置文件，除非是必需的配置文件
