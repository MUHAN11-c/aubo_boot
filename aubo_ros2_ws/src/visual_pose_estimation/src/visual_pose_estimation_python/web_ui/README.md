# Visual Pose Estimation Python - Web UI

基于Python的可视化网页界面，用于 `visual_pose_estimation_python` 模块的交互式操作和调试。

## 📋 目录

- [功能特性](#功能特性)
- [项目结构](#项目结构)
- [依赖要求](#依赖要求)
- [安装](#安装)
- [使用方法](#使用方法)
- [API接口](#api接口)
- [学习文档](#学习文档)
- [测试验证](#测试验证)
- [故障排除](#故障排除)

## ✨ 功能特性

- 🖼️ **图像输入**: 支持图像上传、拖拽和本地文件选择
- 🎯 **姿态估计**: 实时调用ROS2服务进行物体姿态估计
- 📊 **结果可视化**: 直观显示检测结果、置信度和位姿信息
- 📚 **模板管理**: 浏览、选择和管理工件模板
- ⚙️ **模板标准化**: 一键标准化指定工件的所有模板
- 🌐 **Web界面**: 现代化、响应式的用户界面

## 📁 项目结构

```
web_ui/
├── index.html                  # Web UI主页面
├── static/                     # FastAPI 根入口静态页
├── assets/                     # 静态资源
├── scripts/                    # Web UI辅助脚本
│   └── rembg_subprocess.py    # rembg 子进程入口
├── configs/                    # 配置文件目录
├── docs/                       # 文档目录
├── resources/                  # 兼容保留资源目录
├── start_web_ui.sh            # 启动脚本
├── stop_web_ui.sh             # 停止脚本
├── requirements.txt           # Python依赖
└── README.md                  # 本文档
```

## 📦 依赖要求

### ROS2环境
- ROS2 (Humble/Foxy或更高版本)
- `rclpy` - ROS2 Python客户端库
- `cv_bridge` - OpenCV与ROS2图像消息转换

### Python包
- Python 3.8+
- OpenCV (cv2) >= 4.5.0
- NumPy >= 1.19.0
- PyYAML >= 5.3.0

### ROS2服务
Web UI需要以下ROS2服务运行：
- `/estimate_pose` - 姿态估计服务
- `/list_templates` - 列出模板服务
- `/standardize_template` - 模板标准化服务

## 🚀 安装

### 1. 安装Python依赖

```bash
cd /path/to/visual_pose_estimation_python/web_ui
pip3 install -r requirements.txt
```

### 2. 确保ROS2环境已配置

```bash
source /opt/ros/humble/setup.bash  # 或你的ROS2发行版
source /path/to/your/workspace/install/setup.bash
```

### 3. 启动visual_pose_estimation_python节点

在另一个终端中启动主节点：

```bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
```

## 🎮 使用方法

### 启动Web UI

```bash
cd /path/to/visual_pose_estimation_python/web_ui
./start_web_ui.sh
```

启动成功后，会显示：

```
========================================
✓ Web UI 启动成功！
========================================

访问地址:
  http://127.0.0.1:8088/
  http://127.0.0.1:8088/legacy-ui/index.html
```

### 访问界面

在浏览器中打开 `http://127.0.0.1:8088/`

### 使用流程

1. **上传图像**
   - 点击上传区域选择文件
   - 或直接拖拽图像到上传区域

2. **输入工件ID**
   - 在"工件ID"输入框中输入目标工件的ID
   - 例如: `workpiece_01`

3. **执行姿态估计**
   - 点击"估计姿态"按钮
   - 等待处理完成
   - 查看右侧的检测结果

4. **管理模板**
   - 滚动到"模板管理"区域
   - 点击"刷新模板列表"查看所有模板
   - 选择模板可自动填充工件ID
   - 点击"标准化模板"对选定工件进行标准化处理

### 停止服务

```bash
cd /path/to/visual_pose_estimation_python/web_ui
./stop_web_ui.sh
```

或在运行start_web_ui.sh的终端按 `Ctrl+C`

## 🔌 API接口

Web UI 现在由 FastAPI 提供接口，当前页面默认会跳转到兼容的旧版 UI。

### 1. 姿态估计

**端点**: `POST /api/estimate_pose`

**请求体**:
```json
{
  "depth_image": "data:image/png;base64,...",
  "color_image": "data:image/jpeg;base64,...",
  "object_id": "workpiece_01"
}
```

**响应**:
```json
{
  "success": true,
  "success_num": 2,
  "confidence": [0.95, 0.92],
  "vis_image": "data:image/jpeg;base64,...",
  "pose_images": ["data:image/jpeg;base64,..."],
  "positions": [{"x": 0.1, "y": 0.2, "z": 0.3}],
  "grab_positions": [...]
}
```

### 2. 列出模板

**端点**: `POST /api/list_templates`

**请求体**:
```json
{
  "templates_dir": ""
}
```

**响应**:
```json
{
  "success": true,
  "workpiece_ids": ["workpiece_01", "workpiece_02"],
  "count": 5,
  "templates": [...]
}
```

### 3. 标准化模板

**端点**: `POST /api/standardize_template`

**请求体**:
```json
{
  "workpiece_id": "workpiece_01"
}
```

**响应**:
```json
{
  "success": true,
  "processed_count": 3,
  "skipped_count": 1,
  "processed_pose_ids": ["1", "2", "3"],
  "skipped_pose_ids": ["4"]
}
```

### 4. 服务状态

**端点**: `GET /status`

**响应**:
```json
{
  "status": "online",
  "port": 8088,
  "service": "visual_pose_estimation_fastapi_web",
  "timestamp": "2026-01-20 12:00:00"
}
```

## 📚 学习文档

如果你是第一次接触当前 Web 架构，建议按下面顺序阅读：

1. `docs/FASTAPI_DOCS_INDEX.md`
   - 所有 FastAPI 学习资料的总入口和推荐学习路线
2. `docs/FASTAPI_WEB.md`
   - 先看整体说明、启动方式和当前目录职责
3. `docs/FASTAPI_BEGINNER_GUIDE.md`
   - 从零理解 FastAPI 在本项目里的角色、层次划分和调用链
4. `docs/FASTAPI_MIGRATION_GUIDE.md`
   - 看这次迁移替换为什么这样做、完整迁移过程和收益
5. `docs/FASTAPI_TESTING_GUIDE.md`
   - 看如何编写测试验证代码、如何隔离 ROS2 依赖、如何验证接口行为
6. `docs/FASTAPI_EXTENSION_GUIDE.md`
   - 看后续如何继续扩展功能
7. `docs/FASTAPI_INTERFACE_TEMPLATE.md`
   - 看新增接口、Service、bridge 和测试模板
8. `docs/FASTAPI_END_TO_END_EXAMPLE.md`
   - 看一遍完整的“从 0 到新增一个接口”的端到端示例

## ✅ 测试验证

当前 Web 层的核心测试文件是：

- `test/test_web_app.py`

运行方式：

```bash
cd <workspace>/src/visual_pose_estimation/src/visual_pose_estimation_python
pytest test/test_web_app.py
```

如果只想验证某一类功能，例如：

```bash
pytest test/test_web_app.py -k debug
pytest test/test_web_app.py -k websocket
pytest test/test_web_app.py -k template
```

测试设计思路：

- 使用真实 `create_app()` 创建 FastAPI 应用
- 使用 `DummyRosBridge` / `DummyNode` 替换真实 ROS2 依赖
- 使用临时目录替代真实模板目录
- 验证路由、业务结果、文件副作用、WebSocket 行为

更多说明请看：

- `docs/FASTAPI_TESTING_GUIDE.md`

## 🔧 配置

### 修改端口

修改 `start_web_ui.sh` 中的 `HTTP_PORT`，或使用：

```bash
ros2 run visual_pose_estimation_python visual_pose_estimation_web --host 127.0.0.1 --port 8088
```

### 模板目录

模板目录解析优先级如下：

1. `template_root` launch 参数
2. 环境变量 `VPE_TEMPLATE_ROOT`
3. `web_ui/configs/app_config.json` 中的 `template_root`
4. 工作空间中的 `visual_pose_estimation/templates`

## ⚠️ 故障排除

### 1. 端口被占用

**问题**: 启动时提示端口8088已被占用

**解决**:
```bash
# 方法1: 使用stop脚本
./stop_web_ui.sh

# 方法2: 手动查找并终止进程
lsof -ti:8088 | xargs kill -9
```

### 2. ROS2服务未运行

**问题**: Web UI显示"服务未运行"

**解决**:
```bash
# 检查服务是否运行
ros2 service list | grep estimate_pose

# 如果没有，启动主节点
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
```

### 3. 图像上传失败

**问题**: 图像上传后无法显示

**解决**:
- 检查图像格式（支持 JPG, PNG）
- 检查图像大小（建议小于10MB）
- 在浏览器控制台查看错误信息

### 4. 姿态估计超时

**问题**: 姿态估计长时间无响应

**解决**:
- 检查ROS2节点是否正常运行
- 查看终端日志输出
- 确认模板已正确配置
- 尝试使用较小的图像

### 5. 模板列表为空

**问题**: 点击"刷新模板列表"后没有显示模板

**解决**:
- 检查模板目录是否存在
- 确认模板文件格式正确
- 查看ROS2节点日志

## 📝 日志查看

启动脚本会在终端显示详细日志：

```bash
# 查看Web UI服务日志
# 直接在启动终端查看

# 查看ROS2节点日志
ros2 node info /algorithm_http_server_node
```

## 🔗 相关链接

- [visual_pose_estimation_python 文档](../README.md)
- [ROS2 官方文档](https://docs.ros.org/)

## 📄 许可证

本项目遵循与 `visual_pose_estimation_python` 相同的许可证。

## 👥 维护者

如有问题，请联系项目维护者或提交Issue。

---

**最后更新**: 2026-01-20
