# 安装与独立运行

`ros_web_link` 是独立 ROS 2 包：网关 + 浏览器 JS 客户端 + 监控台 + 调用示例。
本文只写**最小可运行**路径；demo（TB3/Panda）见 [api/launch.md](api/launch.md)。

## 1. 依赖边界

### core（必须）

| 类型 | 组件 | 说明 |
|---|---|---|
| ROS | `rclpy`、`rcl_interfaces`、常用 msg/srv/action 包 | `package.xml` 已声明 |
| ROS | `ivg_foxglove_bridge` | Foxglove WebSocket v1 服务端；sibling 包，见下 |
| Python | `fastapi`、`uvicorn`、`websockets`、`httpx`、`PyYAML` | 钉版本于仓库根 `requirements.txt`，装进 venv |
| 浏览器 | 现代 ES Modules + WebSocket | 无构建步骤 |

`ivg_foxglove_bridge` 是**独立 sibling ROS 包**（对应官方 `foxglove_bridge` 的角色）。
本包不 vendored 它；`monitor.launch.py start_bridge:=false` 可复用任意 Foxglove v1
兼容桥，因此网关/JS/console 本身不与某个桥实现绑定。

### optional（demo / 测试）

- 官方 examples：`turtlebot3_simulations`、`turtlebot3_navigation2`、
  `ros2_examples`、`moveit_resources_panda_*`、`nav2_bringup`、`image_transport`；
- 测试：`pytest`、`websockets`；JS 单测另有 `vitest`（`npm install`）。
- UI 组合包（如 `aubo_ros2_web_dashboard`）不是 core 依赖。

## 2. 构建

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
# 1) bridge（sibling）
colcon build --packages-select ivg_foxglove_bridge
# 2) 本包
colcon build --packages-select ros_web_link
source install/setup.bash
```

Python 网关依赖（推荐工作区 venv）：

```bash
python3 -m venv aubo_py3.12
./aubo_py3.12/bin/pip install -r requirements.txt
```

`monitor.launch.py` 会自动探测 `<workspace>/aubo_py3.12/bin/python`；也可用
`python:=...` 或环境变量 `RWL_PYTHON` 覆盖。`transport.launch.xml` 使用 PATH 中的
`python3`，请确保它指向已装依赖的解释器。

## 3. 最小验证

```bash
# 自包含：桥 + 网关 + comm_fixture
ros2 launch ros_web_link transport.launch.xml fixture:=true

# 另一终端
curl -s --noproxy '*' http://127.0.0.1:8095/health
python3 examples/python/01_subscribe.py --count 2
python3 examples/python/02_publish_and_service.py
python3 examples/python/03_params_and_graph.py

# 浏览器
xdg-open http://127.0.0.1:8095/
```

独立监控（先启动你自己的仿真/真机）：

```bash
ros2 launch ros_web_link monitor.launch.py
# 默认桥 18766 / 网关 8095；复用外部桥：
ros2 launch ros_web_link monitor.launch.py start_bridge:=false bridge_port:=8765
```

## 4. 安全与部署

- 网关默认绑定 `0.0.0.0:8095`，**无鉴权**。生产环境请置于反向代理/TLS 后，
  或在 `monitor.launch.py` 用 `bind:=127.0.0.1`。
- 桥默认由 `ivg_foxglove_bridge` 的 launch 绑定；不要把未鉴权的桥暴露到公网。
- UI 的“交互锁”只是页面写出约束，不是安全联锁；硬件急停不经 ROS。
- 机器人 mesh/视频代理只读；`robot-mesh` 有路径穿越防护。

## 5. 测试

```bash
# Python 单元（venv；免 source 时补 ROS env 前缀）
PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages \
LD_LIBRARY_PATH=/opt/ros/jazzy/lib ROS_DISTRO=jazzy \
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test \
  -k "api_surface or examples or docs_links or gateway or js_units" -q

# JS 单测
cd src/ros_web_link && npm install && node_modules/.bin/vitest run

# 完整官方仿真 E2E（需 /dev/shm、/dev/dri）
src/ros_web_link/scripts/run_full_sim_e2e.sh
```

详细测试矩阵见 [testing.md](testing.md)。
