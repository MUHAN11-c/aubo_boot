# aubo_ros2_web_dashboard

ROS 2 **ament_python** 包：托管 **灵视 IVG** 现场 Web 门户（门户首页、视觉抓取、咖啡拉花）与 **ROS 控制台**（话题 / 服务 / 参数 / 图 / 2D / 3D）。浏览器通过 **[roslibjs](https://github.com/RobotWebTools/roslibjs)** 经 **WebSocket** 连接本机 **[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)**（`rosbridge_server` + `rosapi` 等），**不**使用已停更的 Node.js **ros2-web-bridge** 作运行时桥。

**Foxglove 路线（Vue3 + FastAPI + foxglove_bridge）** 见同工作空间包 **`aubo_ros2_foxglove_dashboard`**：控制面板走 FastAPI，Foxglove Studio **直连** `foxglove_bridge`（默认 WS `8765`），与本包端口与协议独立。

---

## 目录总览

| 路径 | 作用 |
|------|------|
| `package.xml` | ROS 包清单：依赖 **`rosbridge_suite`**（含 `rosbridge_server`、`rosapi`）、`tf2_web_republisher` 等。 |
| `setup.py` / `setup.cfg` | 安装 Python 包、`launch/`、`web/public/` 到 `share/aubo_ros2_web_dashboard/`。 |
| `resource/aubo_ros2_web_dashboard` | ament 索引用空标记文件。 |
| `aubo_ros2_web_dashboard/` | Python 模块：`threaded_static_server` 等。 |
| `launch/` | `ros2 launch` 描述文件，见下表。 |
| `web/public/` | **HTTP 文档根**：门户、视觉抓取、控制台等静态资源（`threaded_static_server` 的 `--directory`）。 |

---

## `launch/`

| 文件 | 作用与实现要点 |
|------|----------------|
| `web_dashboard.launch.py` | 启动 **[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)**：`rosbridge_server` 的 WebSocket launch（含 **rosapi**）、**tf2_web_republisher**、静态 HTTP 服务托管 `web/public`。参数：`web_host`、`web_port`（默认 8090）、`rosbridge_port`（默认 9090）。浏览器 **直连** `ws://主机:rosbridge_port`。 |

---

## `web/public/`（静态前端 / 文档根）

源码位于包内 `web/public/`；`colcon` 安装后为 `share/aubo_ros2_web_dashboard/web/public/`。launch 中 **`threaded_static_server`** 将该目录作为 HTTP 根目录。**业务脚本**（`js/vision_grasp_panel.js`、`js/ros_console.js`、`js/coffee_latte_panel.js`、`js/ivg_site_nav.js` 及门户内联脚本）为 **ES2015+**（`let`/`const`、箭头函数、模板字符串等），由浏览器直接执行，**无 Babel/打包**；需现代 Chromium / Firefox / Edge。

| 文件 | 作用与实现要点 |
|------|----------------|
| `index.html` | **灵视 IVG 门户首页**：功能介绍与快速入口；链到视觉抓取、咖啡拉花、控制台；视觉位姿（默认 `http://hostname:8088/`）与手眼（`:8080`）由脚本按当前 `hostname` 拼接，控制台与面板链接继承 `location.search`（如 `?rosbridge_port=`）。 |
| `vision_grasp_panel.html` | **视觉抓取面板**：相机图、关节曲线、状态区、抓取与 GraspNet 控制等。`body` 含 `ivg-single-screen`（单屏视口布局）。详见下文 **「视觉抓取面板」**。 |
| `topics_lab.html` | **ROS 控制台**：顶栏、全站导航、IVG 快捷条、多标签主区（话题、服务、动作、参数、关系图、2D、3D）；加载 `ros_console.js`。 |
| `js/ros_console.js` | **控制台核心逻辑**：`ROSLIB.Ros` 直连 `ws://主机:rosbridge_port`、rosapi、订阅/服务/图/2D/3D；IVG 预设话题与服务。 |
| `js/vendor/*` | 第三方库：`roslib`、`eventemitter2`、`easeljs`、`ros2d`、`three`、`OrbitControls`；`fetch_vendor.sh` / `README.txt` 说明获取方式。 |
| `coffee_latte_panel.html` | **咖啡拉花面板**（演示）：`body.ivg-single-screen`；布局与视觉页一致，当前 Canvas 动画 + 日志，无 rosbridge。 |
| `css/vision_grasp_panel.css` | 视觉抓取页主样式；单屏下相机与操作区 **左右分栏**（`.layout-camera-controls`），预览区 **4:3**、`object-fit: contain`，变量如 `--cam-split-max-w` 控制左侧最大宽度。 |
| `css/coffee_latte_panel.css` | `@import vision_grasp_panel.css` 后追加拉花顶栏状态、画布与单屏下差异样式。 |
| `css/ivg_site_nav.css` | 全站主导航（门户 / 视觉抓取 / 咖啡拉花 / ROS 控制台）。 |
| `js/ivg_site_nav.js` | 导航链接附加 `location.search`（继承 `rosbridge_port` 等），并按 `data-ivg-page` 高亮。 |
| `css/home.css` | `index.html` 门户样式。 |
| `css/topics_lab.css` | `topics_lab.html` 布局与组件样式。 |
| `css/demo.css` / `component.css` | 上游 RobotWebTools / demo 遗留样式（其它旧页面若引用）。 |
| `assets/` | **图片与图标**：`assets/images/`、`assets/icons/`；说明见 `assets/README.md`。 |
| `README.md` | 上游 **ros2-web-bridge** 会议 demo 原始说明（硬件与 Node 桥接流程）；与本仓库 IVG 集成以本文件与 `launch` 为准。 |

---

## 视觉抓取面板（`vision_grasp_panel.html`）

面向 IVG 现场：在浏览器中查看相机图、机器人 `RobotStatus`（末端位置、四元数、RPY、关节）、VPE 字符串状态、GraspNet 发布的 `PoseArray` 摘要，以及 6 关节趋势曲线；并通过 rosbridge 调用 `demo_driver` / `graspnet_ros2` 相关服务。

### 布局（`ivg-single-screen`）

- `body.ivg-single-screen`：锁定 **整页视口高度**（`100dvh`），主区用 CSS Grid 分配 **工具栏 → 相机+操作 → 曲线与状态**，避免整页纵向滚动条；底部状态 `pre` 可在区域内滚动（滚动条弱化）。
- **`.layout-camera-controls`**（HTML 包裹相机区块与控制区块）：宽屏 **两列**——左 **预览**（最大宽度见 `:root` 中 **`--cam-split-max-w`**，默认约 800px；列宽还与 **`--cam-split-col-pct`** 取 `min`），预览盒 **`aspect-ratio: 4/3`**，画布 **`object-fit: contain`**，避免 640×480 被压扁；右 **抓取与工具控制**（内容多时可纵向滚动）。窄屏（约 ≤920px）改为上下堆叠。
- 若从源码中去掉 `body` 上的 `ivg-single-screen`，则退化为常规纵向排版（默认发布 HTML 带单屏 class）。

### 连接

- **直连** `ws://当前页主机:9090`（可用 `?rosbridge_port=` 覆盖）。彩色图等可走 `web_video_server` MJPEG，端口可在顶栏或 `?web_video_port=` 配置。

### 话题设置

- 主界面 **「话题设置」** 打开模态框，编辑五个订阅话题。
- **保存并重连**：写入浏览器 `localStorage` 键 `ivg_vision_grasp_topics_v1` 并立即重连；下次进入页面自动恢复。
- **恢复默认**：填回内置默认话题名并清除上述存储（仍须点「保存并重连」才会按默认值重连，或直接点顶栏「重连」若输入框已是默认）。
- **Esc** 或点击遮罩、关闭按钮可关闭模态（未点保存则不上盘存储，输入框内容保留）。

### 订阅话题（默认见下表；与 `vision_grasp_panel.js` 中 `TOPIC_DEFAULTS` 一致）

| 用途 | 默认话题 | 消息类型 |
|------|-----------|----------|
| 彩色图 | `/camera/color/image_raw` | `sensor_msgs/msg/Image`（rgb8/rgba8/bgr8/mono8 等） |
| 末端与关节（文本 + 曲线） | `/aubo_driver/robot_status` | `demo_interface/msg/RobotStatus` |
| 关节曲线（与上并行） | `/joint_states` | `sensor_msgs/msg/JointState`（取 `position` 前 6 维） |
| VPE 状态 | `/system_status` | `std_msgs/msg/String` |
| 抓取位姿列表 | `/grasp_poses_base` | `geometry_msgs/msg/PoseArray` |

**关节曲线说明**：`RobotStatus` 与 `JointState` 两路若同时高频发布，会交替写入同一条历史缓冲，曲线仅表示趋势，不保证关节顺序与机械臂轴一一对应（未解析 `JointState.name`）。

### 服务调用

| 服务 | 类型 | 说明 |
|------|------|------|
| `/execute_single_grasp` | `demo_interface/srv/ExecuteGraspPose` | `object_id` + `use_visual_estimation` |
| `/loop_grasp_control` | `std_srvs/srv/SetBool` | `demo_driver` 后端循环线程（内部固定先视觉估计再抓取，**不能**通过此服务切换非视觉） |
| `/graspnet_capture_control` | `std_srvs/srv/SetBool` | GraspNet 采集开关 |
| `/publish_grasps_worker_loop_control` | `std_srvs/srv/SetBool` | GraspNet worker 循环 |
| `/run_gripper_swap` | `demo_interface/srv/RunGripperSwap` | 快换；面板按钮使用 `direction=gripper2` |

### 抓取策略（页面单选：工件 / GraspNet）

- **工件（视觉）**：单次与循环均使用 `use_visual_estimation=true`（单次直接调服务；循环为 `loop_grasp_control=true`，与后端实现一致）。
- **GraspNet（非视觉）**：单次调用 `use_visual_estimation=false`。循环时因后端 `loop_grasp_control` 线程写死视觉路径，面板会先 `loop_grasp_control=false`，再在**浏览器内**按固定间隔重复调用 `/execute_single_grasp(false)`（间隔见 `vision_grasp_panel.js` 中 `clientGraspLoop.pauseMs`）。
- **停止**：停止浏览器循环并 `loop_grasp_control=false`；WebSocket 断开时也会停止浏览器循环。
- **「单次抓取（非视觉）」**（GraspNet 区块）：始终 `use_visual_estimation=false`，不受单选影响，便于点云流程下明确触发非视觉单次。

### 相关文件

- `web/public/css/vision_grasp_panel.css`
- `web/public/js/vision_grasp_panel.js`（文件头为开发与运维用的完整约定表）

---

## 咖啡拉花面板（`coffee_latte_panel.html`）

- **当前**：`body.ivg-single-screen` 与视觉页相同的单屏 + **左右分栏**（预览在左、杯位与动作在右）；`css/coffee_latte_panel.css` 通过 `@import` 复用 `vision_grasp_panel.css`；`js/coffee_latte_panel.js` 绘制杯口与奶泡心形动画并写活动日志，**无 rosbridge**。
- **后续**：可参照 `vision_grasp_panel.js` 接入 rosbridge 与话题/服务；导航 `data-ivg-page="latte"` 已与 `ivg_site_nav.js` 对齐。

---

## 数据流简图

```
浏览器 --WebSocket--> rosbridge_suite（rosbridge_server + rosapi）--> ROS 2
```

---

## 相关命令备忘

```bash
# 编译安装包（在工作空间根）
colcon build --packages-select aubo_ros2_web_dashboard --symlink-install
source install/setup.bash

# 静态 + rosbridge（默认 HTTP 8090、WS 9090）
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py

# 可选参数示例
ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py web_host:=0.0.0.0 web_port:=8090 rosbridge_port:=9090
```

### 启动后浏览器入口（`web_dashboard.launch.py`）

假设静态站为 `http://<主机>:<web_port>`（默认 `8090`），rosbridge 为 `ws://<主机>:<rosbridge_port>`（默认 `9090`）。若 rosbridge 非 9090，请在各页面 URL 后加 **`?rosbridge_port=<端口>`**（与 `ivg_site_nav.js`、控制台一致）。

| 页面 | 路径 |
|------|------|
| 灵视 IVG 门户 | `/index.html` |
| 视觉抓取面板 | `/vision_grasp_panel.html` |
| 咖啡拉花面板 | `/coffee_latte_panel.html` |
| ROS 控制台 | `/topics_lab.html` |

### 工作空间一键脚本（IVG 全栈示例）

工作空间根目录下的 **`start_IVG_graspnet_points_fastapi_web_dashboard.sh`**（与本包同级，属现场集成脚本）在 **Terminator** 中拉起机械臂、感知、GraspNet、FastAPI、**本 launch（直连）**、rosbag 等；结束时会在终端打印上述静态页链接（含本机 / 局域网 IP）。环境变量示例：`WEB_DASH_HOST`、`WEB_DASH_PORT`、`ROSBRIDGE_PORT`、`WEB_HOST`、`WEB_PORT`（FastAPI）等，见脚本内注释。

---

## 与 `.vscode/` 说明

编辑器本地配置（如 `c_cpp_properties.json`），不参与 ROS 运行时；可随个人环境调整，一般无需随包分发说明。
