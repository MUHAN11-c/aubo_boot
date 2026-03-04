# AUBO MoveIt + ROS 2 Control 故障排除快速指南

本文档提供常见问题的快速诊断和解决方案。

---

## 问题诊断流程图

```
启动失败？
├─ PackageNotFoundError
│  └─→ 检查包是否构建和source
├─ SubstitutionFailure
│  └─→ 检查启动参数和文件路径
└─ 运行时错误
   ├─ 控制器未启动
   │  └─→ 检查ros2_controllers.yaml格式
   ├─ Action服务器未找到
   │  └─→ 检查moveit_controllers.yaml中的action_ns
   └─ 机器人模型不显示
      └─→ 检查URDF/SRDF和TF
```

---

## 快速检查清单

在深入调试之前，先运行这些命令进行快速检查：

```bash
# 1. 检查环境
echo $ROS_DISTRO
source ~/aubo_ros2_ws/install/setup.bash

# 2. 检查包
ros2 pkg list | grep aubo

# 3. 检查关键文件是否存在
ros2 pkg prefix aubo_description
ros2 pkg prefix aubo_moveit_config
ls $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro

# 4. 测试XACRO解析
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5 > /tmp/test_urdf.urdf
check_urdf /tmp/test_urdf.urdf  # 如果安装了urdfdom工具
```

---

## 常见错误代码及解决方案

### ERR001: PackageNotFoundError

**错误信息：**
```
PackageNotFoundError: "package 'aubo_description' not found"
```

**快速诊断：**
```bash
# 检查包是否存在
ros2 pkg list | grep aubo_description

# 如果不存在
cd ~/aubo_ros2_ws
colcon build --packages-select aubo_description
source install/setup.bash
```

**解决方案：**
- 确保包已构建：`colcon build --packages-select <package_name>`
- 确保已source工作空间：`source install/setup.bash`
- 检查package.xml中的包名是否正确

---

### ERR002: XACRO解析失败

**错误信息：**
```
SubstitutionFailure('executed command failed. Command: xacro ...')
```

**快速诊断：**
```bash
# 手动测试XACRO
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5

# 检查文件是否存在
find ~/aubo_ros2_ws -name "aubo_ros2.xacro"
```

**解决方案：**
- 检查XACRO文件路径是否正确
- 检查XACRO语法（括号、引号等）
- 检查依赖的XACRO文件是否存在
- 检查参数是否正确传递（如 `aubo_type`）

---

### ERR003: 控制器启动失败

**错误信息：**
```
[ERROR] [spawner_joint_state_broadcaster]: process has died [pid XXX, exit code 1]
```

**快速诊断：**
```bash
# 检查controller_manager节点
ros2 node list | grep controller_manager

# 检查控制器配置
ros2 param list /controller_manager

# 检查服务
ros2 service list | grep controller_manager
```

**解决方案：**

1. **检查ros2_controllers.yaml格式：**
```yaml
# 确保格式正确
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

2. **增加超时时间：**
```python
cmd=["ros2 run controller_manager spawner.py joint_state_broadcaster --controller-manager-timeout 20"]
```

3. **检查关节名称是否匹配：**
```bash
# 从URDF提取关节名称
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5 | grep -oP 'name="\K[^"]*(?=".*type="revolute")'

# 与ros2_controllers.yaml中的关节列表对比
grep -A 10 "joints:" config/ros2_controllers.yaml
```

---

### ERR004: MoveIt找不到Action服务器

**错误信息：**
```
[WARN] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: 
Waiting for joint_trajectory_controller/follow_joint_trajectory to come up
```

**快速诊断：**
```bash
# 检查Action服务器
ros2 action list | grep follow_joint_trajectory

# 检查控制器状态
ros2 control list_controllers

# 检查MoveIt配置
ros2 param get /move_group moveit_simple_controller_manager
```

**解决方案：**

1. **修正action_ns配置（常见错误）：**
```yaml
# moveit_controllers.yaml
joint_trajectory_controller:
  action_ns: follow_joint_trajectory  # 正确：只有action名称
  # action_ns: joint_trajectory_controller/follow_joint_trajectory  # 错误！
```

2. **等待Action服务器启动（可能需要几秒钟）**

3. **检查控制器是否处于active状态：**
```bash
ros2 control list_controllers
# 应该显示：joint_trajectory_controller[active]
```

---

### ERR005: 机器人模型不显示

**错误信息：**
- RViz中看不到机器人模型
- TF错误："Frame does not exist"

**快速诊断：**
```bash
# 检查TF树
ros2 run tf2_ros tf2_echo base_link ee_link

# 检查joint_states话题
ros2 topic echo /joint_states --once

# 检查robot_state_publisher节点
ros2 node info /robot_state_publisher
```

**解决方案：**

1. **确保joint_state_broadcaster运行：**
```bash
ros2 control list_controllers | grep joint_state_broadcaster
```

2. **检查joint_states话题有数据：**
```bash
ros2 topic hz /joint_states
```

3. **检查URDF是否正确加载：**
```bash
ros2 param get /robot_state_publisher robot_description | head -20
```

---

### ERR006: SRDF错误

**错误信息：**
```
Error: Semantic description is not specified for the same robot as the URDF
Error: Link 'ee_link' declared as part of a chain... is not known to the URDF
```

**快速诊断：**
```bash
# 检查URDF中的机器人名称
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo.xacro aubo_type:=e5 | grep -i "robot name"

# 检查SRDF中的机器人名称
grep -i "robot name" config/aubo_i5.srdf
```

**解决方案：**

1. **统一机器人名称：**
```xml
<!-- URDF中 -->
<robot name="aubo_e5" xmlns:xacro="http://ros.org/wiki/xacro">

<!-- SRDF中 -->
<robot name="aubo_e5">
```

2. **添加缺失的链接（如ee_link）：**
在 `aubo.xacro` 中添加末端执行器链接（参考主文档）

---

### ERR007: 参数类型错误

**错误信息：**
```
TypeError('Unexpected type for parameter value None')
```

**快速诊断：**
```bash
# 检查YAML语法
python3 -c "import yaml; print(yaml.safe_load(open('config/ros2_controllers.yaml')))"
```

**解决方案：**

1. **检查空参数：**
```yaml
joint_state_broadcaster:
  ros__parameters: {}  # 使用 {} 而不是 null
```

2. **确保所有必需参数都有值**

---

## 调试技巧

### 1. 启用详细日志

在启动文件中添加日志级别参数：

```python
Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_description, ros2_controllers_path],
    output="screen",
    arguments=['--ros-args', '--log-level', 'debug'],  # 添加这行
)
```

### 2. 逐步启动

不要一次启动所有节点，逐步启动以定位问题：

```bash
# 1. 只启动controller_manager
ros2 run controller_manager ros2_control_node \
    --ros-args \
    -p robot_description:="$(xacro ...)" \
    -p ros2_controllers_config:="$(cat config/ros2_controllers.yaml)"

# 2. 在另一个终端启动spawner
ros2 run controller_manager spawner.py joint_state_broadcaster

# 3. 检查是否成功，再启动下一个
ros2 run controller_manager spawner.py joint_trajectory_controller
```

### 3. 检查进程状态

```bash
# 检查进程是否在运行
ps aux | grep ros2_control_node
ps aux | grep move_group

# 检查进程退出代码
echo $?  # 在进程结束后立即检查
```

### 4. 查看日志文件

```bash
# ROS 2日志位置
ls ~/.ros/log/

# 查看最新的日志
ls -lt ~/.ros/log/latest/ | head -10
```

---

## 验证成功标志

配置成功后，您应该看到以下标志：

### ✅ 节点运行正常

```bash
$ ros2 node list
/controller_manager
/move_group
/robot_state_publisher
/rviz2
```

### ✅ 控制器处于active状态

```bash
$ ros2 control list_controllers
joint_state_broadcaster[active]
joint_trajectory_controller[active]
```

### ✅ Action服务器可用

```bash
$ ros2 action list
/joint_trajectory_controller/follow_joint_trajectory
/move_group/execute_trajectory
/move_group/move_action
```

### ✅ 关节状态发布正常

```bash
$ ros2 topic hz /joint_states
average rate: 50.000
```

### ✅ TF变换正常

```bash
$ ros2 run tf2_ros tf2_echo base_link ee_link
[INFO] [tf2_echo]: Waiting for transform base_link -> ee_link:
At time 1234.567
- Translation: [x, y, z]
- Rotation: in Quaternion [x, y, z, w]
```

### ✅ RViz中功能正常

- 机器人模型正确显示
- 可以规划轨迹
- 可以执行轨迹
- 机器人模型在执行时移动

---

## 获取帮助

如果问题仍未解决：

1. **检查完整日志：**
```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py use_ros2_control:=true use_fake_hardware:=true 2>&1 | tee launch.log
```

2. **收集系统信息：**
```bash
# ROS版本
echo $ROS_DISTRO

# 已安装的包
ros2 pkg list | grep -E "(moveit|control)" | sort

# 工作空间结构
tree -L 3 ~/aubo_ros2_ws/src/aubo_ros2_driver/
```

3. **参考官方文档：**
- [MoveIt 2 Troubleshooting](https://moveit.picknik.ai/main/doc/how_to_guides/how_to_guides.html)
- [ROS 2 Control Issues](https://github.com/ros-controls/ros2_control/issues)

---

**文档版本**: 1.0  
**最后更新**: 2024-12-18

