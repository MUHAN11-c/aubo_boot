# 函数调用检查报告

这份报告原本针对旧的 `http_bridge_server.py` 单体实现整理，现已不再适用。

## 当前有效的 Web 调用主链

1. `visual_pose_estimation_python/web/app.py`
   - 创建 FastAPI 应用
   - 注册 `system`、`camera`、`pose`、`templates`、`robot`、`grasp`、`debug` 路由

2. `visual_pose_estimation_python/web/routers/*.py`
   - 暴露 `/api/*` 与 `/ws`
   - 通过依赖注入调用 `NativeWebService`

3. `visual_pose_estimation_python/web/services/native_api.py`
   - 负责模板、机器人、抓取、debug 等核心 Web 业务逻辑
   - 调用 `RosBridgeManager.node`

4. `visual_pose_estimation_python/web/ros_bridge/manager.py`
   - 管理 ROS2 bridge 生命周期
   - 启动 `visual_pose_estimation_python/web/ros_bridge/node_runtime.py` 中的 `ROS2Node`

5. `visual_pose_estimation_python/web/ros_bridge/node_runtime.py`
   - 封装相机触发、姿态估计、模板标准化、机器人控制、夹爪切换、循环抓取控制等 ROS2 交互

6. `visual_pose_estimation_python/ros2_communication.py`
   - 提供底层 ROS2 服务实现
   - 调用 `pose_estimator.py`、`template_standardizer.py`、`preprocessor.py`、`feature_extractor.py`

## 说明

- 旧 `web_ui/scripts/http_bridge_server.py` 已移除
- 旧 `web_ui/scripts/params_manager.py` 已移除
- 当前 Web 入口应以 FastAPI 与 `visual_pose_estimation_python/web/*` 目录为准
