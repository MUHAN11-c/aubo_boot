# 快速参考卡

## 🚀 一键启动

```bash
cd /home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui
./start_web_ui.sh
```

访问: http://localhost:8089

## 🛑 一键停止

```bash
./stop_web_ui.sh
```

## 📋 常用命令

### 启动ROS2节点

```bash
source /opt/ros/humble/setup.bash
cd /home/mu/IVG/aubo_ros2_ws
source install/setup.bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
```

### 检查服务状态

```bash
# 检查ROS2服务
ros2 service list | grep estimate_pose

# 检查端口占用
lsof -i:8089

# 检查Web服务状态
curl http://localhost:8089/status
```

### 清理端口

```bash
# 方法1
./stop_web_ui.sh

# 方法2
lsof -ti:8089 | xargs kill -9
```

## 🎯 基本操作流程

```
1. 启动ROS2节点
   ↓
2. 启动Web UI (./start_web_ui.sh)
   ↓
3. 打开浏览器 (http://localhost:8089)
   ↓
4. 上传图像
   ↓
5. 输入工件ID
   ↓
6. 点击"估计姿态"
   ↓
7. 查看结果
```

## 📊 API端点

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/estimate_pose` | POST | 姿态估计 |
| `/api/list_templates` | POST | 列出模板 |
| `/api/standardize_template` | POST | 标准化模板 |
| `/api/upload_image` | POST | 上传图像 |
| `/status` | GET | 服务状态 |

## 🔧 常见问题速查

| 问题 | 解决方法 |
|------|----------|
| 端口被占用 | `./stop_web_ui.sh` |
| ROS2服务未运行 | 启动主节点 |
| 图像上传失败 | 检查格式(JPG/PNG) |
| 检测无结果 | 检查工件ID和模板 |
| 界面无响应 | 刷新浏览器 |

## 📁 目录结构速查

```
web_ui/
├── index.html              # Web界面
├── start_web_ui.sh        # 启动脚本
├── stop_web_ui.sh         # 停止脚本
├── scripts/
│   └── http_bridge_server.py  # 服务器
└── docs/                  # 文档
    ├── 快速开始.md
    └── 使用示例.md
```

## 🎨 界面快捷键

- **拖拽上传**: 拖动图像到上传区
- **点击上传**: 点击上传区选择文件
- **Ctrl+C**: 停止服务（在终端）
- **F5**: 刷新页面
- **Ctrl+Shift+I**: 打开开发者工具

## 📞 获取帮助

1. 查看 [README.md](README.md)
2. 查看 [快速开始.md](docs/快速开始.md)
3. 查看 [使用示例.md](docs/使用示例.md)
4. 查看 [PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)

## 🔑 关键配置

| 配置项 | 默认值 | 修改位置 |
|--------|--------|----------|
| HTTP端口 | 8089 | `http_bridge_server.py` |
| ROS2节点名 | http_bridge_node_python | `http_bridge_server.py` |
| 模板目录 | 从ROS2配置读取 | 主节点配置文件 |

## 💡 最佳实践

✅ 先启动ROS2节点，再启动Web UI  
✅ 使用清晰的图像（光照均匀）  
✅ 定期标准化模板  
✅ 记录常用的工件ID  
✅ 保持模板库整洁  

## ⚠️ 注意事项

❗ 图像大小建议 < 5MB  
❗ 单用户使用，避免并发  
❗ 定期清理临时文件  
❗ 生产环境需要额外配置安全性  

---

**快速联系**: 查看主文档获取支持信息  
**版本**: 1.0.0  
**更新**: 2026-01-20
