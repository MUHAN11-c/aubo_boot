# serial_force_sim

5 点力 **CoP 闭环对齐仿真**（mock 机械臂）。与真机驱动 **`serial_force` 分开**：
本包不碰串口、不进 `harvest_system`，只消费 `/force/points` 契约，用假力源替代
`serial_force_node` 把「5 点力 → CoP → 姿态校正」整条链路跑通并做统计。

- 详细说明（模型、控制律、指标、20 组结果、限界）：**[doc/force_align_sim.md](doc/force_align_sim.md)**
- 统计表 / 逐组 CSV / 原始 JSONL / 三联图：`doc/force_align_sim_*`
- 真机驱动与点位来源：[../serial_force/README.md](../serial_force/README.md)、
  [../serial_force/doc/force_pad_positions.md](../serial_force/doc/force_pad_positions.md)

## 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select serial_force_sim
source install/setup.bash
```

## 入口

| 命令 | 作用 |
|---|---|
| `ros2 run serial_force_sim fake_force` | 虚拟平面 + 5 点弹簧力源（替代串口驱动） |
| `ros2 run serial_force_sim cop_web` | 5 点 / CoP 实时网页（默认 8100） |
| `ros2 run serial_force_sim cop_control` | 单次闭环：CoP → IK → JTC |
| `ros2 run serial_force_sim cop_trials` | 多场景批量试验，写 JSONL |
| `ros2 run serial_force_sim cop_analyze` | JSONL → CSV / 统计表 / 图 |

## 快速开始

```bash
# 前置：mock 臂 + MoveIt
ros2 launch aubo_e5_bringup bringup.launch.py \
  hardware_mode:=mock moveit_enabled:=true \
  camera_enabled:=false extrinsics_enabled:=false \
  tool_profile:=bite_shear_v1

# 1) 力源 + 网页
ros2 run serial_force_sim fake_force --ros-args -p tilt_deg:=4.0
ros2 run serial_force_sim cop_web --port 8100

# 2) 单次闭环
ros2 run serial_force_sim cop_control

# 3) 批量 + 统计（默认写 $ROS_HOME/serial_force_sim/）
ros2 run serial_force_sim cop_trials --tilt 1,2,3,4,5 --yaw 0,90 --repeats 2
ros2 run serial_force_sim cop_analyze \
  $ROS_HOME/serial_force_sim/force_align_sim_trials.jsonl
```

参数：`tilt_deg`（倾角）、`tilt_yaw_deg`（方位）、`preload_mm`（预载）、
`k_n_per_mm`（单点刚度）；控制参数 `--center-mm 1.2`、`--max-rot-deg 5`、
`--max-steps 12`。

## 边界

- 静力线性弹簧模型，无摩擦/迟滞/噪声/臂动力学；只用于趋势与门限验证。
- 同一时刻只起一个 `fake_force`；真机用 `serial_force_node`，不要再起 `fake_force`。
- 真机驱动、串口协议、udev、`harvest_system` 都不在本包，改它们去 `serial_force`。
