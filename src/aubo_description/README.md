# aubo_description

AUBO E5 工作单元 URDF / xacro / mesh。采摘栈通过 `robot_state_publisher` 消费展开后的 `robot_description`。

## 关键文件

- `urdf/aubo_e5.urdf.xacro`：整机描述
- `urdf/aubo_e5.ros2_control.xacro`：**只读**。按 `hardware_mode` 选 mock / sim / real 插件并填 `robot_ip`

## 关节顺序（权威）

`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`

本包不含控制器与运动逻辑。改几何会同时影响 MoveIt、手眼 TF 和碰撞。
