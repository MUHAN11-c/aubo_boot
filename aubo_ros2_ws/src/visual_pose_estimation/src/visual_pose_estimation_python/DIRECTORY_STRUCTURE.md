# 目录结构标准化说明

本文档说明当前 `visual_pose_estimation_python` 如何按 ROS2 `ament_python` 功能包边界收敛。

## 当前推荐结构

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
├── launch/                          # ROS2启动文件
│   ├── visual_pose_estimation_python.launch.py
│   └── visual_pose_estimation_web.launch.py
│
├── test/                            # 自动化测试
│   ├── test_copyright.py           # 版权测试
│   ├── test_flake8.py              # 代码风格测试
│   ├── test_pep257.py              # 文档风格测试
│   └── test_web_app.py             # Web 回归测试主入口
│
├── web_ui/                          # Web UI界面
│   ├── index.html                  # Web UI主页面
│   ├── README.md                   # Web UI文档
│   ├── requirements.txt            # Python依赖
│   ├── static/                     # FastAPI 根入口静态页
│   ├── assets/                     # 前端静态资源
│   │
│   ├── configs/                    # 默认配置目录
│   │   ├── debug_thresholds.json   # 调试阈值配置
│   │   ├── app_config.json         # Web 运行时配置
│   │   ├── config_paths.md         # 配置路径说明
│   │   ├── camera_intrinsics.yaml  # 相机内参（标准名）
│   │   └── hand_eye_calibration.yaml  # 手眼标定（标准名）
│   │
│   ├── scripts/                    # Web UI辅助脚本
│   │   └── rembg_subprocess.py    # rembg 子进程入口
│   │
│   ├── tools/                      # 手工验证工具
│   │   ├── check_fastapi_startup.sh
│   │   ├── generate_color_channel_demo.py
│   │   └── verify_debug_api.py
│   │
│   ├── docs/                       # Web UI文档
│   │   ├── FASTAPI_DOCS_INDEX.md
│   │   ├── FASTAPI_WEB.md
│   │   ├── FASTAPI_BEGINNER_GUIDE.md
│   │   ├── FASTAPI_MIGRATION_GUIDE.md
│   │   ├── FASTAPI_TESTING_GUIDE.md
│   │   ├── FASTAPI_EXTENSION_GUIDE.md
│   │   ├── FASTAPI_INTERFACE_TEMPLATE.md
│   │   ├── FASTAPI_ARCHITECTURE_DIAGRAMS.md
│   │   ├── FASTAPI_END_TO_END_EXAMPLE.md
│   │   └── 使用示例.md
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

### 配置文件 (`web_ui/configs/`)
存储 Web 默认配置、手眼标定、相机内参与调试阈值。

### 启动文件 (`launch/`)
ROS2启动文件，用于启动节点。

### 测试文件 (`test/`)
存放自动化测试；依赖运行中服务的脚本不再放在这里。

### Web UI (`web_ui/`)
完整的 Web 资源层，包含前端、配置、文档和手工验证工具。

### 资源文件 (`resource/`)
ROS2包所需的资源文件。

## 标准化操作

已完成的标准化操作：

1. ✅ 补齐 `resource/visual_pose_estimation_python`，回归标准 `ament_python` 元数据
2. ✅ 统一 `web_ui/configs`、静态资源、脚本和模板路径解析入口
3. ✅ 新增 `visual_pose_estimation_web.launch.py`，让 Web 服务拥有独立 launch 入口
4. ✅ 保留 `test/test_web_app.py` 作为 Web 自动回归主入口
5. ✅ 将手工验证脚本从 `web_ui/test/` 挪到 `web_ui/tools/`
6. ✅ 创建 `.gitignore`，忽略缓存和运行产物

## 注意事项

- `__pycache__/` 目录已被 `.gitignore` 忽略
- 运行时文件（如 `.web_ui.pid`）已被 `.gitignore` 忽略
- 所有文档已统一整理到 `web_ui/docs/`
- 手工验证脚本统一放到 `web_ui/tools/`

## 维护建议

1. Web UI相关的文档应放在 `web_ui/docs/` 目录
2. 自动化测试放在 `test/`；手工验证脚本放在 `web_ui/tools/`
3. 配置文件应统一放在相应的 `configs/` 目录
4. 避免在根目录直接放置文件，除非是必需的配置文件
