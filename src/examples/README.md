# examples — 官方示例子工作空间

上游官方示例统一收纳在 `src/examples/`，与 `src/peach/`、`src/ivg/`、`src/peach_sim/`
同级。分组只是目录收纳：包名、话题名、colcon 递归发现与 `--packages-select` 均不变。

| 目录 | 上游 | 包 |
|------|------|----|
| `ros2_examples/` | [ros2/examples](https://github.com/ros2/examples) `jazzy` | `examples_rclcpp_*` / `examples_rclpy_*` |
| `turtlebot3_simulations/` | [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations) `jazzy` | `turtlebot3_gazebo` / `turtlebot3_fake_node` / `turtlebot3_simulations` |
| `turtlebot3_msgs/` | [ROBOTIS-GIT/turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs) `jazzy` | `turtlebot3_msgs` |
| `turtlebot3_navigation2`（symlink） | [ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3) `jazzy` 的 `turtlebot3_navigation2` | 官方地图 + nav2 参数，供 `ros_web_link` 仿真 E2E |
| `panda_description/` | [moveit/moveit_resources](https://github.com/moveit/moveit_resources) `ros2` | `moveit_resources_panda_description` |
| `panda_moveit_config/` | 同上 | `moveit_resources_panda_moveit_config` |

用途：`src/ros_web_link` 的官方数据面 E2E（`sim_demo.launch.py`、`panda_moveit_web.launch.py`、
`official_demo.launch.py`）与 MoveIt2 Panda 示例。本地自写项目不参与该验证面。

## 恢复克隆（本机/新克隆）

`ros2_examples`、`turtlebot3_msgs`、`turtlebot3_simulations` 为带 `.git` 的裸克隆，按
`.gitignore` 不入库；`turtlebot3_navigation2` 是构建用符号链接。缺失时按下面恢复：

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
git clone -b jazzy https://github.com/ros2/examples.git src/examples/ros2_examples
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git src/examples/turtlebot3_msgs
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git src/examples/turtlebot3_simulations
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git external/turtlebot3
ln -sfn ../../external/turtlebot3/turtlebot3_navigation2 src/examples/turtlebot3_navigation2
```

`external/navigation2`（Nav2 基线克隆）与 `external/turtlebot3` 仍留在顶层 `external/`：
前者只作源码基线、不参与构建，后者除 `turtlebot3_navigation2` 外也不进本工作区构建面。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# 官方仿真数据面 / Panda MoveIt E2E（需先完成上面的克隆）
./aubo_py3.12/bin/python -m pytest src/ros_web_link/test -k 'sim or moveit or official' -v
```

只动官方示例时可用 `colcon build --packages-select <pkg...>`；包内构建产物与工作区
`build/`、`install/`、`log/` 均不入库。
