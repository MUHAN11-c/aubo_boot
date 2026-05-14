# IVG Web Dashboard — 零基础本地部署构建学习文档

> **适用对象**: 零基础，想在本地从零搭建整个系统
> **目标**: 从一台干净的 Ubuntu 22.04 开始，到浏览器打开 IVG 门户

---

## 第 1 章：你需要准备什么

### 硬件要求

| 项目 | 最低要求 | 说明 |
|------|---------|------|
| 操作系统 | Ubuntu 22.04 LTS | 必须是 Linux（ROS 2 只支持 Linux） |
| 内存 | 8 GB | 编译 C++ 代码需要 |
| 磁盘 | 20 GB 可用 | 含所有依赖 |
| 网络 | 能访问 GitHub / PyPI / npm | 需要下载依赖 |

> **零基础提示**: 如果你是 Windows/Mac 用户，需要先安装虚拟机（如 VirtualBox）或 WSL2，在里面装 Ubuntu 22.04 喵~

### 你不需要的东西

- **不需要机械臂硬件** — 仿真模式可以在没有机械臂的情况下运行
- **不需要 GPU** — GraspNet AI 推理需要 GPU，但 Web Dashboard 前端不需要
- **不需要摄像头** — 仿真模式下相机画面为空，不影响页面功能

---

## 第 2 章：环境搭建（15 步）

### 第 1 步：更新系统

```bash
sudo apt update && sudo apt upgrade -y
```

### 第 2 步：安装基础工具

```bash
sudo apt install -y \
  build-essential cmake git curl wget \
  terminator \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep
```

> **termintor** 是什么：一个终端模拟器，可以开多个标签页。启动脚本用它给每个组件开一个标签页，方便查看日志喵~

### 第 3 步：安装 ROS 2 Humble

```bash
# 添加 ROS 2 仓库
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
```

### 第 4 步：安装 ROS 2 相关工具

```bash
sudo apt install -y \
  ros-humble-rosbridge-server \
  ros-humble-tf2-web-republisher \
  ros-humble-web-video-server \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers
```

### 第 5 步：安装 Node.js（Vue 3 前端需要）

```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
# 验证
node --version   # 应显示 v20.x 或更高
npm --version    # 应显示 10.x 或更高
```

> **Node.js 是什么**: JavaScript 的运行环境。你写 Vue 3 代码需要 Node.js 来编译打包喵~

### 第 6 步：安装 Python 依赖

```bash
pip install fastapi uvicorn httpx websockets pyyaml
```

> **每个包的作用**:
> - `fastapi` — Web 框架
> - `uvicorn` — 运行 FastAPI 的服务器
> - `httpx` — 异步 HTTP 客户端（代理转发用）
> - `websockets` — WebSocket 协议支持
> - `pyyaml` — 解析 YAML 配置文件

### 第 7 步：克隆项目

```bash
cd ~
git clone https://github.com/your-org/IVG2.0.git
cd IVG2.0/aubo_ros2_ws
```

### 第 8 步：编译 ROS 2 工作空间（首次）

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ivg_interfaces
source install/setup.bash
colcon build
```

> **首次编译**需要 10-30 分钟（取决于机器性能），之后增量编译只需几秒喵~

### 第 9 步：构建 Vue 3 前端

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web
npm install     # 下载前端依赖（首次需要 1-2 分钟）
npm run build   # 打包前端 → web/dist/
```

> **npm install 做什么**: 读取 `package.json`，下载 Vue 3、Element Plus、Tailwind CSS 等 ~165 个依赖包喵~

### 第 10 步：验证构建产物

```bash
ls /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/dist/
# 应该看到: index.html, assets/ 目录
```

---

## 第 3 章：启动系统

### 方式 A：独立启动 Web 网关（想只测试前端）

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -m aubo_ros2_web_dashboard.fastapi_static_gateway \
  8090 \
  -d /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web/dist

# 浏览器打开 http://localhost:8090
```

> 这种模式下前端可以显示，但因为没有 rosbridge，连接状态会显示"未连接"，ROS 数据不会更新喵~

### 方式 B：完整启动（含 rosbridge，需要完整 ROS 2 环境）

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch aubo_ros2_web_dashboard web_dashboard.launch.py
```

> 这会启动 rosbridge + tf2_web_republisher + web_video_server + FastAPI 网关。然后浏览器打开 `http://localhost:8090` 就能看到完整功能喵~

### 方式 C：在 ROS 2 大系统里启动

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
./start_aubo_new_driver.sh
```

> 这会启动完整的 IVG 系统（机械臂 + 相机 + GraspNet + Web 等 16 个组件）喵~

---

## 第 4 章：开发工作流（改代码 → 看效果）

### 后端 Python 代码修改

```bash
# 修改 Python 代码后
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select aubo_ros2_web_dashboard
# 重启网关进程
```

### 前端 Vue 3 代码修改（推荐方式）

```bash
# 开发模式 — 热更新，保存代码浏览器自动刷新
cd /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web
npm run dev
# 浏览器打开 http://localhost:5173
```

> **热更新**：Vite 开发服务器监视你的代码，改了保存后浏览器立即更新，不需要手动 F5 喵~

### 前端修改后的生产构建

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_web_dashboard/web
npm run build      # 重新打包 → web/dist/
```

---

## 第 5 章：常见问题排查

### Q1: `npm install` 报错 "EACCES: permission denied"

```bash
# 不要用 sudo npm install。先修复 npm 权限：
mkdir ~/.npm-global
npm config set prefix '~/.npm-global'
echo 'export PATH=~/.npm-global/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
```

### Q2: `colcon build` 报错 "package not found"

```bash
# 确保 source 了 ROS 2 环境
source /opt/ros/humble/setup.bash
# 首次编译需要先编译 ivg_interfaces
colcon build --packages-select ivg_interfaces
source install/setup.bash
```

### Q3: 浏览器打开显示空白页

```bash
# 检查 Vite 构建产物是否存在
ls web/dist/index.html
# 如果不存在: cd web && npm run build
# 检查网关是否启动
curl http://localhost:8090/health
```

### Q4: Vue 开发服务器能打开页面，但连接状态显示"未连接"

这是正常的——Vite 开发服务器 (5173 端口) 不含 rosbridge 代理。需要同时启动 FastAPI 网关 (8090 端口) 来提供 rosbridge 代理喵~

### Q5: 构建产物太大怎么办

```bash
# Vite 已经自动压缩。gzip 后仅 ~75KB JS + ~51KB CSS
# Element Plus 已按需引入 (unplugin-vue-components)
# 如果还想减小，检查是否有未使用的 Element Plus 组件被引入
```

---

## 第 6 章：依赖关系图

```
你的电脑需要安装:
│
├── ROS 2 Humble (系统级)
│   ├── rosbridge_server     → WebSocket 桥
│   ├── tf2_web_republisher  → TF 坐标发布
│   └── web_video_server     → 相机视频流
│
├── Python 3.10 (系统自带)
│   ├── fastapi       → Web 框架
│   ├── uvicorn       → ASGI 服务器
│   ├── httpx         → HTTP 客户端
│   ├── websockets    → WebSocket
│   ├── pyyaml        → 配置解析
│   └── ament_index   → ROS 2 包查找
│
└── Node.js 22+ (前端打包)
    ├── vue 3.5       → UI 框架
    ├── vite 6        → 构建工具
    ├── element-plus  → UI 组件库
    ├── tailwindcss 4 → 原子 CSS
    ├── @vueuse/core  → 工具函数库
    ├── roslib 1.x    → ROS WebSocket 客户端
    ├── pinia 2       → 状态管理
    ├── vue-router 4  → 页面路由
    └── typescript 5  → 类型检查
```

---

## 第 7 章：复刻检查清单

如果你想在另一台机器上完整复刻本项目，按此清单逐项执行：

```
□ Ubuntu 22.04 已安装
□ ROS 2 Humble 已安装 (第3步)
□ Node.js 22+ 已安装 (第5步)
□ Python 依赖已安装 (第6步)
□ 项目已克隆 (第7步)
□ ROS 2 工作空间已编译 (第8步)
□ Vue 3 前端已构建 (第9步)
□ 浏览器可以打开 http://localhost:8090
□ 网关 /health 返回 {"status":"ok"}
□ 前端 6 个页面都能正常切换
```

---

*最后更新: 2026-05-16*
