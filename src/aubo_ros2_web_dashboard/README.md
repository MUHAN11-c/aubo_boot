# aubo_ros2_web_dashboard

IVG 演示栈 Web 控制台（2026-09-30 自 `~/aubo_boot` 移植，Jazzy 适配）。
FastAPI 网关（**零 ROS 依赖纯代理**，端口 **8095**）+ 8 个 MPA 面板
（门户 / 机械臂手动控制 / 视觉抓取 / 咖啡拉花 / 调试 / 监控 / 日志 / 设置），
经 rosbridge(9090) / foxglove_bridge(8765) / web_video_server(8089) 与 ROS 图交互。

**不进 peach `harvest_system` / lifecycle；8090 归 `peach_observability`，本包不改不动。**

## 目的

演示栈（拉花 / 换刀 / 视觉抓取循环）的人机控制面。真机使用=操作员起 real
演示栈（授权语义与原系统一致）；本面板**不是急停**（ISO 13850 急停在柜/示教器）。

## 与 aubo_boot 原版的差异（移植适配）

| 项 | aubo_boot | 本仓 |
|---|---|---|
| 网关端口 | 8090 | **8095**（8090 归 peach_observability） |
| web_video 配置段/访问器 | install 侧有、源缺失（源树漂移） | 补齐（config.py + defaults.yaml） |
| robotwebtools 前端库 | 符号链接独立仓 | 解引用内嵌 web/public/js/robotwebtools |
| 机械臂状态 | ivg RobotStatus @ /robot_status | aubo_msgs/RobotStatus @ /aubo_io_controller/robot_status（JS 端 mapRobotStatus 映射字段） |
| 末端位姿显示 | RobotStatus.cartesian_position 内嵌 | 轮询 `/get_current_state`（vision/latte 面板） |
| FK/IK | /aubo_driver/get_fk(ik) ivg 类型 | `/aubo/get_fk(ik)` aubo_msgs（字段同名同序） |
| IO 写 | /aubo_driver/set_io ivg SetRobotIO | `/aubo_io_controller/set_io` aubo_msgs/SetIO（fun=1 板载 DO） |
| upstream_proxy 尾部空 router 重绑 | 有（疑似死代码） | 删除 |

已知降级：调试面板"工具电压"按钮调 `/aubo/set_tool_voltage`（本仓无对应服务，
点击报服务不存在，属预期降级）。

## 公有 API

- 入口 `ivg_fastapi_static_gateway`（console script）/ `python3 -m
  aubo_ros2_web_dashboard.fastapi_static_gateway`
- HTTP：`/health`、`/api/v1/runtime`、`/api/v1/settings`（POST）、
  `/api/v1/tool-geometries`、`/api/ivg/robot-mesh/{pkg}/{path}`、
  `/api/ivg/proxy/web-video/*`、静态面板 `/`
- WebSocket 代理：`/ws/rosbridge` → 127.0.0.1:9090；`/ws/foxglove` → 127.0.0.1:8765
- 配置：`config/defaults.yaml`（端口/上游/前端话题服务默认值，单一来源）

## 例子

```bash
# 依赖 move_group/演示栈服务已起；foxglove/web_video 由 start_ivg_demo.sh 起
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
# 浏览器打开 http://<host>:8095/
```

## 如何 build / 测

```bash
colcon build --packages-select aubo_ros2_web_dashboard
colcon test --packages-select aubo_ros2_web_dashboard && colcon test-result --verbose
```

Python 依赖（fastapi/uvicorn/websockets/httpx）钉在仓库根 `requirements.txt`，
装进 `aubo_py3.12` venv；rosbridge/foxglove/web_video_server 走 Jazzy apt
（`scripts/env_bootstrap.sh` 已列）。

## 安全边界

- 面板可下发真机运动与 IO——**操作员起 real 栈=授权**；勿在无人监督下开 real
- 本面板不是急停；急停回路在柜/示教器
- 与 peach harvest_system 勿同机同域并行（URDF 换装会改 RSP TF 树）

## 许可

Apache-2.0
