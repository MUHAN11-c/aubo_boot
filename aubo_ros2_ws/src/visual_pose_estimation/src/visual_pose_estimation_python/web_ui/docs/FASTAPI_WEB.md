# FastAPI Web 说明

当前 Web 服务已迁移到 `visual_pose_estimation_python/web/` 下的 FastAPI 架构。

## 推荐配套阅读

- `FASTAPI_DOCS_INDEX.md`
  - 所有 FastAPI 文档的总入口和推荐学习顺序
- `FASTAPI_BEGINNER_GUIDE.md`
  - 面向零基础，解释 FastAPI 在本项目中的基础概念与代码对应关系
- `FASTAPI_MIGRATION_GUIDE.md`
  - 解释本项目从旧 Web 方案迁移到 FastAPI 的完整过程、原因与收益
- `FASTAPI_TESTING_GUIDE.md`
  - 解释本项目如何编写测试验证代码、如何隔离 ROS2 依赖、如何验证接口行为
- `FASTAPI_EXTENSION_GUIDE.md`
  - 解释后续如何新增接口、如何接 ROS2 能力、如何按现有架构继续开发
- `FASTAPI_INTERFACE_TEMPLATE.md`
  - 提供新增接口、Service、ROS2 bridge、Dummy 和测试的可复用模板
- `FASTAPI_ARCHITECTURE_DIAGRAMS.md`
  - 提供当前架构图、时序图、迁移前后对照图、测试分层图
- `FASTAPI_END_TO_END_EXAMPLE.md`
  - 从 0 演示“新增一个完整接口”的端到端开发和测试流程

## 启动方式

推荐使用以下任一方式启动：

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py host:=127.0.0.1 port:=8088
```

或：

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run visual_pose_estimation_python visual_pose_estimation_web --host 127.0.0.1 --port 8088
```

## 访问地址

- FastAPI 根入口：`http://127.0.0.1:8088/`
- 兼容旧版 UI：`http://127.0.0.1:8088/legacy-ui/index.html`
- 服务状态：`http://127.0.0.1:8088/status`
- 健康检查：`http://127.0.0.1:8088/health`
- WebSocket：`ws://127.0.0.1:8088/ws`

## 当前后端结构

- `visual_pose_estimation_python/web/app.py`
  - FastAPI 应用入口
- `visual_pose_estimation_python/web/routers/`
  - 路由层
- `visual_pose_estimation_python/web/services/native_api.py`
  - 原生 Web 业务逻辑
- `visual_pose_estimation_python/web/ros_bridge/manager.py`
  - ROS2 bridge 生命周期管理
- `visual_pose_estimation_python/web/ros_bridge/node_runtime.py`
  - ROS2Node 运行时实现
- `visual_pose_estimation_python/web/runtime_support.py`
  - 模板目录、配置、姿态归一化、rembg 运行时支持
- `visual_pose_estimation_python/web/params_manager.py`
  - Debug 阈值参数管理

## 当前状态

- 旧 `web_ui/scripts/http_bridge_server.py` 已移除
- 旧 `web_ui/scripts/params_manager.py` 已移除
- PLC 路由已移除
- `LegacyHttpBridgeService` 已移除
- FastAPI 已成为唯一的 Web 后端入口
