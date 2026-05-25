# aubo_direct_api_node 测试命令文档

本文档用于测试 `aubo_direct_api_node`（**完全不走 TCP2CAN 跟随**，直接调用机械臂原生 SDK API）。

## 1. 编译与启动

```bash
cd ~/aubo_boot/aubo_ros2_ws
colcon build --packages-select aubo_driver_ros2
source install/setup.bash
```

启动节点（按实际控制器 IP 修改）：

```bash
ros2 run aubo_driver_ros2 aubo_direct_api_node --ros-args \
  -p server_host:=169.254.10.98 \
  -p server_port:=8899 \
  -p username:=aubo \
  -p password:=‘123456’ \
  -p auto_startup:=true
```

## 2. 基础状态检查

查看状态话题：

```bash
ros2 topic echo /aubo_driver/direct/robot_status --once
```

获取当前状态服务：

```bash
ros2 service call /aubo_driver/direct/get_current_state demo_interface/srv/GetCurrentState "{}"
```

## 3. 上电/下电

上电（启动机械臂）：

```bash
ros2 service call /aubo_driver/direct/set_robot_enable demo_interface/srv/SetRobotEnable "{enable: true}"
```

下电（关机）：

```bash
ros2 service call /aubo_driver/direct/set_robot_enable demo_interface/srv/SetRobotEnable "{enable: false}"
```

## 4. 设置速度因子

```bash
ros2 service call /aubo_driver/direct/set_speed_factor demo_interface/srv/SetSpeedFactor "{velocity_factor: 0.3}"
```

> 取值建议：`0.01 ~ 1.0`

## 5. 关节运动测试

### 5.1 服务方式（阻塞）

以下示例是 6 轴关节目标（弧度）：

```bash
ros2 service call /aubo_driver/direct/set_robot_pose demo_interface/srv/SetRobotPose \
"{target_pose: [0.0, -0.6, 1.2, 0.0, 1.57, 0.0], use_joints: true, is_radian: true, velocity: 0.2}"
```

### 5.2 话题方式（非阻塞）

```bash
ros2 topic pub --once /aubo_driver/direct_joint_cmd std_msgs/msg/Float32MultiArray \
"{data: [0.0, -0.5, 1.0, 0.0, 1.57, 0.0]}"
```

## 6. 笛卡尔运动测试

### 6.1 set_robot_pose（XYZ + RPY）

`use_joints=false` 时，`target_pose=[x,y,z,roll,pitch,yaw]`：

```bash
ros2 service call /aubo_driver/direct/set_robot_pose demo_interface/srv/SetRobotPose \
"{target_pose: [0.35, 0.10, 0.25, 3.14, 0.0, 1.57], use_joints: false, is_radian: true, velocity: 0.2}"
```

### 6.2 movel（直线）

```bash
ros2 service call /aubo_driver/direct/movel demo_interface/srv/Movel \
"{target_pose: {position: {x: 0.35, y: 0.10, z: 0.25}, orientation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}, velocity_factor: 0.2, acceleration_factor: 0.2, move_axis: ''}"
```

沿单轴增量移动（例如沿 x 正向移动 5cm）：

```bash
ros2 service call /aubo_driver/direct/movel demo_interface/srv/Movel \
"{target_pose: {position: {x: 0.25, y: 0.0, z: 0.0}, orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}}, velocity_factor: 0.95, acceleration_factor: 0.45, move_axis: 'x'}"
```

### 6.3 话题方式（PoseStamped）

```bash
ros2 topic pub --once /aubo_driver/direct_cartesian_cmd geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'base_link'}, pose: {position: {x: 0.32, y: 0.08, z: 0.28}, orientation: {w: 0.0, x: 1.0, y: 0.0, z: 0.0}}}"
```

## 7. FK / IK 测试

FK：

```bash
ros2 service call /aubo_driver/direct/get_fk aubo_msgs/srv/GetFK \
"{joint: [0.0, -0.5, 1.0, 0.0, 1.57, 0.0]}"
```

IK（`ref_joint` 建议给当前关节）：

```bash
ros2 service call /aubo_driver/direct/get_ik aubo_msgs/srv/GetIK \
"{ref_joint: [0.0, -0.5, 1.0, 0.0, 1.57, 0.0], pos: [0.35, 0.10, 0.25], ori: [0.0, 1.0, 0.0, 0.0]}"
```

## 8. 运动控制（暂停/继续/停止）

暂停：

```bash
ros2 service call /aubo_driver/direct/pause std_srvs/srv/Trigger "{}"
```

继续：

```bash
ros2 service call /aubo_driver/direct/resume std_srvs/srv/Trigger "{}"
```

停止（普通停止）：

```bash
ros2 service call /aubo_driver/direct/stop std_srvs/srv/Trigger "{}"
```

快速停止：

```bash
ros2 service call /aubo_driver/direct/fast_stop std_srvs/srv/Trigger "{}"
```

## 9. IO 测试

设置数字输出：

```bash
ros2 service call /aubo_driver/direct/set_io demo_interface/srv/SetRobotIO \
"{io_type: 'digital_output', io_index: 0, value: 1.0}"
```

读取数字输出：

```bash
ros2 service call /aubo_driver/direct/read_io demo_interface/srv/ReadRobotIO \
"{io_type: 'digital_output', io_index: 0}"
```

设置模拟输出：

```bash
ros2 service call /aubo_driver/direct/set_io demo_interface/srv/SetRobotIO \
"{io_type: 'analog_output', io_index: 0, value: 2.5}"
```

工具端数字 IO：

```bash
ros2 service call /aubo_driver/direct/set_io demo_interface/srv/SetRobotIO \
"{io_type: 'tool_io', io_index: 0, value: 1.0}"
```

## 10. 常见问题

- 若提示登录失败，先 `ping` 控制器 IP，并确认端口 `8899` 可达。
- 若控制器已被其他程序登录占用，需先释放原连接。
- 真机测试建议先把 `velocity_factor` 调低（如 `0.1~0.2`）再验证轨迹。
- 本节点为直调 API 方式，不需要也不会调用 `robotServiceEnterTcp2CanbusMode()`。
