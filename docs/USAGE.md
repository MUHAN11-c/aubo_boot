# IVG 使用文档

> **面向**: 操作者 / 用户 | **最后更新**: 2026-05-15
>
> 本文档说明如何启动和使用 IVG 智能机器人操作系统。无需了解内部实现细节。

---

## 一、系统概览

IVG (Intelligent Vision Grasp) 是一套智能机器人操作系统，由以下部分组成：

```
相机 (Percipio FM830)  ──→  GraspNet (AI 抓取位姿预测)
                       ──→  VPE (视觉位姿估计)
AUBO 机械臂             ──→  MoveIt 2 (运动规划)
                       ──→  工具快换 (气动/电动夹爪)
Web Dashboard (:8090)   ──→  浏览器操作入口
```

整个系统通过 **`start_aubo_new_driver.sh`** 一键启动，浏览器打开 `http://<IP>:8090` 即可操作喵~

---

## 二、启动系统

### 2.1 一键启动

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
./start_aubo_new_driver.sh
```

脚本会自动：
1. 编译工作空间（可跳过: `SKIP_BUILD=1`）
2. 检测机械臂是否在线 → 自动选择真机/仿真模式
3. 按依赖顺序启动全部 16 个组件
4. 打印所有 Web 界面访问 URL

### 2.2 启动选项（环境变量）

```bash
# 跳过编译（代码未修改时加快启动）
SKIP_BUILD=1 ./start_aubo_new_driver.sh

# 跳过 rosbag 录制
SKIP_ROSBAG=1 ./start_aubo_new_driver.sh

# 跳过 RViz2
SKIP_RVIZ=1 ./start_aubo_new_driver.sh

# 自定义机械臂 IP
AUBO_IP=192.168.1.100 ./start_aubo_new_driver.sh

# 自定义 Web 端口
WEB_DASH_PORT=9000 ./start_aubo_new_driver.sh
```

### 2.3 全量环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `AUBO_IP` | `169.254.10.98` | 机械臂控制器 IP |
| `WEB_HOST` | `127.0.0.1` | VPE Web 绑定地址 |
| `WEB_PORT` | `8088` | VPE Web 端口 |
| `WEB_DASH_HOST` | `0.0.0.0` | Dashboard 网关绑定 |
| `WEB_DASH_PORT` | `8090` | Dashboard 网关端口 |
| `ROSBRIDGE_PORT` | `9090` | rosbridge WebSocket 端口 |
| `WEB_VIDEO_PORT` | `8089` | 相机 MJPEG 视频端口 |
| `HAND_EYE_PORT` | `8070` | 手眼标定 Web 端口 |
| `IVG_ROSBAG_DIR` | `rosbags/ivg_session` | 数据录制目录 |
| `SKIP_BUILD` | `0` | 跳过 colcon build |
| `SKIP_ROSBAG` | `0` | 跳过 rosbag 录制 |
| `SKIP_RVIZ` | `0` | 跳过 RViz2 |

### 2.4 停止系统

在启动脚本的终端按 `Ctrl+C`，所有组件自动终止。

如果 `Ctrl+C` 无效（偶发），手动清理：

```bash
pkill -f rosbridge_websocket
pkill -f aubo_driver
pkill -f move_group
pkill -f web_video_server
pkill -f 'uvicorn.*8090'
pkill -f 'ros2 bag'
```

---

## 三、Web 界面入口

启动完成后输出类似：

```
──────── 本机浏览器 ────────
手眼标定:     http://127.0.0.1:8070/
VPE FastAPI:  http://127.0.0.1:8088/
IVG 门户:     http://127.0.0.1:8090/
视觉抓取:     http://127.0.0.1:8090/vision_grasp_panel.html
咖啡拉花:     http://127.0.0.1:8090/coffee_latte_panel.html
```

| URL | 功能 |
|-----|------|
| `:8090/` | **IVG 门户** — 导航入口，所有功能的起点 |
| `:8090/vision_grasp_panel.html` | **视觉抓取面板** — 相机画面 + 抓取控制 + 关节曲线 + 末端位姿 |
| `:8090/coffee_latte_panel.html` | **咖啡拉花面板** — DO/DI 控制 + 工序流程 |
| `:8090/tf_monitor_panel.html` | **TF 监控面板** — 3D 机械臂模型 + 坐标系可视化 |
| `:8090/settings_panel.html` | **设置面板** — 话题名/服务名配置 |
| `:8070/` | **手眼标定** — 相机-机械臂标定工具 |
| `:8088/` | **VPE Web** — 视觉位姿估计模板管理 |

---

## 四、日常操作流程

### 4.1 视觉抓取（工件模式）

1. 将工件放置在相机视野内
2. 打开浏览器 `:8090/vision_grasp_panel.html`
3. 确认连接状态为「已连接」
4. 确认相机画面正常（左栏 3D 模型 + 右栏相机图）
5. 选择「工件（视觉估计）」模式
6. 输入目标工件编号（与现场 object_id 一致）
7. 点击 **「执行单次抓取」**

系统自动完成：VPE 位姿估计 → MoveIt 规划 → 机械臂接近 → 夹爪闭合 → 抬升 → 放置

### 4.2 视觉抓取（AI 大模型模式）

1. 选择「AI大模型抓取」模式
2. 点击 **「开始采集」** 启动 GraspNet 捕获
3. 观察投影视图中 AI 预测的抓取位姿（黄色夹爪图标）
4. 点击 **「循环抓取开」** 开始连续抓取
5. 点击 **「循环抓取关」** 停止

### 4.3 工具快换

1. 在视觉抓取面板右侧找到「末端夹爪快换」区域
2. 当前工具指示灯标识正在使用的工具
3. 点击目标工具按钮（如「夹爪2 φ60」）
4. 等待机械臂自动执行：脱离当前工具 → 移动到新工具 dock 位 → 锁紧新工具 → 回安全位
5. 状态栏「当前」更新为新工具

### 4.4 咖啡拉花

1. 打开 `:8090/coffee_latte_panel.html`
2. 使用右侧「工序开关」控制 DO2（打花）/ DO4（咖啡）
3. 底部 DI 反馈灯显示设备状态（咖啡反馈 / 打花反馈 / 警告）

---

## 五、状态监控

### 5.1 底部状态栏

所有页面底部显示 5 个指示灯：

| 指示灯 | 含义 | 状态 |
|--------|------|------|
| **在线** | 机械臂是否在通信 | 在线/离线 |
| **使能** | 电机是否使能 | 已使能/未使能 |
| **运动** | 是否正在运动 | 运动中/静止 |
| **规划** | 运动规划状态 | 空闲/规划中/执行中/错误 |
| **模式** | 驱动模式 | 真实/仿真 |

### 5.2 关节曲线

视觉抓取面板底部显示 6 个关节的实时角度曲线。X 轴为时间，Y 轴为弧度值。图例自动标注关节名。

### 5.3 末端位姿

右栏显示末端执行器的实时位姿：XYZ 位置（米）、RPY 欧拉角（度）、四元数（xyzw）。

---

## 六、rosbag 数据录制

系统启动时自动录制全部话题到 `rosbags/ivg_session/`。

### 手动回放

```bash
cd /home/mu/aubo_boot/aubo_ros2_ws
source install/setup.bash

# 列出录制的话题
ros2 bag info rosbags/ivg_session/

# 回放（循环）
ros2 bag play rosbags/ivg_session/ --loop
```

### 禁用录制

```bash
SKIP_ROSBAG=1 ./start_aubo_new_driver.sh
```

---

## 七、常见问题

详细排查指南见 [DEPLOYMENT.md §13](../DEPLOYMENT.md#13-常见问题排查)。以下为日常高频问题速查：

| 问题 | 解决 |
|------|------|
| 启动脚本卡在 `[1]` | 机械臂未开机或 IP 不可达。检查 `ping 169.254.10.98`，或重启进入仿真模式 |
| Web 页面打不开 | `curl http://127.0.0.1:8090/health` 应返回 `{"status":"ok"}` |
| 相机画面黑屏 | 相机掉线。检查 USB 连接，`ros2 topic hz /camera/color/image_raw` |
| 抓取失败 | 目标超出工作空间或碰撞。检查 `ros2 service call /get_planning_scene` |
| 连接状态「已断开」 | rosbridge 崩溃或网络中断。等待自动重连（最多 12 次），或刷新页面 |

---

*最后更新: 2026-05-29*
