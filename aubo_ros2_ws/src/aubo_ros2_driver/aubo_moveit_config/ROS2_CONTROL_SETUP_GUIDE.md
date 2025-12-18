# AUBO MoveIt配置与ROS 2 Control集成教程

## 目录
1. [概述](#概述)
2. [前置条件](#前置条件)
3. [文件结构说明](#文件结构说明)
4. [从零开始的配置步骤](#从零开始的配置步骤)
5. [常见问题及解决方案](#常见问题及解决方案)
6. [验证配置是否成功](#验证配置是否成功)
7. [参考资源](#参考资源)

---

## 概述

本文档详细说明了如何为AUBO机器人配置MoveIt 2和ROS 2 Control，实现仿真控制。配置完成后，可以通过MoveIt在RViz中进行运动规划，并通过ROS 2 Control执行轨迹。

### 主要组件

- **MoveIt 2**: 运动规划框架
- **ROS 2 Control**: 机器人控制框架
- **Fake Hardware Interface**: 仿真硬件接口（用于测试）

---

## 前置条件

### 系统要求

- **ROS 2发行版**: Foxy Fitzroy 或更高版本
- **操作系统**: Ubuntu 20.04 (Foxy) 或 Ubuntu 22.04 (Humble)

### 必需的ROS 2包

确保已安装以下包：

```bash
# 核心MoveIt包
sudo apt install ros-foxy-moveit ros-foxy-moveit-core ros-foxy-moveit-ros-planning-interface

# ROS 2 Control包
sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers \
     ros-foxy-controller-manager ros-foxy-joint-trajectory-controller \
     ros-foxy-joint-state-broadcaster

# 其他依赖
sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher \
     ros-foxy-joint-state-publisher-gui ros-foxy-robot-state-publisher \
     ros-foxy-rviz2
```

### 工作空间结构

确保您的工作空间结构如下：

```
aubo_ros2_ws/
├── src/
│   └── aubo_ros2_driver/
│       ├── aubo_description/          # 机器人描述包
│       │   ├── urdf/
│       │   │   └── xacro/
│       │   │       └── inc/
│       │   │           ├── aubo.xacro
│       │   │           ├── aubo_ros2.xacro
│       │   │           └── aubo_ros2_control.ros2_control.xacro
│       │   └── package.xml
│       └── aubo_moveit_config/        # MoveIt配置包
│           ├── config/
│           │   ├── ros2_controllers.yaml
│           │   ├── moveit_controllers.yaml
│           │   └── ...
│           ├── launch/
│           │   └── aubo_moveit.launch.py
│           └── package.xml
└── install/
```

---

## 文件结构说明

### 关键配置文件

#### 1. `aubo_moveit_config/config/ros2_controllers.yaml`

ROS 2 Control控制器配置文件，定义控制器管理器和各个控制器的参数。

**关键配置项：**

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # 控制器更新频率（Hz）

    # 控制器类型定义
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

# 控制器详细参数
joint_trajectory_controller:
  ros__parameters:
    command_interfaces: [position]          # 命令接口类型
    state_interfaces: [position, velocity]  # 状态接口类型
    joints:                                 # 控制的关节列表
      - shoulder_joint
      - upperArm_joint
      - foreArm_joint
      - wrist1_joint
      - wrist2_joint
      - wrist3_joint

joint_state_broadcaster:
  ros__parameters: {}  # 无需额外参数
```

#### 2. `aubo_moveit_config/config/moveit_controllers.yaml`

MoveIt控制器管理器配置文件，定义MoveIt如何与ROS 2 Control交互。

**关键配置项：**

```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

controller_names:
  - joint_trajectory_controller

joint_trajectory_controller:
  action_ns: follow_joint_trajectory  # Action命名空间（重要！）
  type: FollowJointTrajectory
  default: true
  joints:
    - shoulder_joint
    - upperArm_joint
    - foreArm_joint
    - wrist1_joint
    - wrist2_joint
    - wrist3_joint
```

**注意：** `action_ns` 必须设置为 `follow_joint_trajectory`，不能包含控制器名称前缀。

#### 3. `aubo_description/urdf/xacro/inc/aubo_ros2.xacro`

包含ROS 2 Control硬件接口的URDF宏定义。

#### 4. `aubo_moveit_config/launch/aubo_moveit.launch.py`

主启动文件，根据 `use_ros2_control` 参数选择使用ROS 2 Control或传统的joint_state_publisher。

---

## 从零开始的配置步骤

### 步骤1: 创建工作空间并获取代码

```bash
# 创建工作空间
mkdir -p ~/aubo_ros2_ws/src
cd ~/aubo_ros2_ws/src

# 获取aubo_ros2_driver代码（假设已存在）
# git clone <your-repo-url> aubo_ros2_driver
```

### 步骤2: 创建/检查aubo_description包

确保 `aubo_description` 包存在且包含必要的URDF/XACRO文件：

```bash
cd ~/aubo_ros2_ws/src/aubo_ros2_driver

# 检查包结构
ls -la aubo_description/
ls -la aubo_description/urdf/xacro/inc/
```

**必需文件：**
- `aubo_description/package.xml`
- `aubo_description/CMakeLists.txt`
- `aubo_description/urdf/xacro/inc/aubo.xacro` (基础URDF)
- `aubo_description/urdf/xacro/inc/aubo_ros2.xacro` (ROS 2 Control版本)
- `aubo_description/urdf/xacro/inc/aubo_ros2_control.ros2_control.xacro` (硬件接口定义)

### 步骤3: 配置ros2_controllers.yaml

在 `aubo_moveit_config/config/ros2_controllers.yaml` 中配置控制器：

```bash
cd ~/aubo_ros2_ws/src/aubo_ros2_driver/aubo_moveit_config/config
```

创建或编辑 `ros2_controllers.yaml`，内容如[文件结构说明](#文件结构说明)中所示。

**关键点：**
- `controller_manager.ros__parameters` 下定义控制器类型
- 控制器详细参数在顶层定义（如 `joint_trajectory_controller:`）
- 关节名称必须与URDF中的关节名称完全匹配

### 步骤4: 配置moveit_controllers.yaml

在 `aubo_moveit_config/config/moveit_controllers.yaml` 中配置MoveIt控制器：

```bash
# 编辑文件
nano aubo_moveit_config/config/moveit_controllers.yaml
```

**关键配置：**
- `action_ns: follow_joint_trajectory` （不要包含控制器名称前缀）
- `type: FollowJointTrajectory`
- `joints` 列表必须与 `ros2_controllers.yaml` 中的关节列表一致

### 步骤5: 配置启动文件

编辑 `aubo_moveit_config/launch/aubo_moveit.launch.py`，确保包含ROS 2 Control支持：

**关键代码段：**

```python
if use_ros2_control_val == "true":
    # ROS2 Control 控制器管理器节点
    ros2_controllers_path = os.path.join(
        get_package_share_directory("aubo_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )
    
    # 启动控制器
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py joint_state_broadcaster --controller-manager-timeout 20"],
                shell=True,
                output="screen",
            )
        ]
    )
    
    trajectory_controller_spawner = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py joint_trajectory_controller --controller-manager-timeout 20"],
                shell=True,
                output="screen",
            )
        ]
    )
```

### 步骤6: 构建工作空间

```bash
cd ~/aubo_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select aubo_description aubo_moveit_config
source install/setup.bash
```

### 步骤7: 启动配置

```bash
# 启动MoveIt with ROS 2 Control
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
    use_ros2_control:=true \
    use_fake_hardware:=true \
    aubo_type:=e5
```

---

## 常见问题及解决方案

### 问题1: PackageNotFoundError - "package 'aubo_description' not found"

**症状：**
```
[ERROR] [launch]: Caught exception in launch: PackageNotFoundError("package 'aubo_description' not found")
```

**原因：**
- `aubo_description` 包不存在或未正确构建
- 工作空间未source

**解决方案：**

1. 检查包是否存在：
```bash
ros2 pkg list | grep aubo_description
```

2. 如果不存在，创建包：
```bash
cd ~/aubo_ros2_ws/src/aubo_ros2_driver
mkdir -p aubo_description
# 创建package.xml和CMakeLists.txt
```

3. 重新构建并source：
```bash
cd ~/aubo_ros2_ws
colcon build --packages-select aubo_description
source install/setup.bash
```

---

### 问题2: SubstitutionFailure - "launch configuration 'aubo_type' does not exist"

**症状：**
```
[ERROR] [launch]: Caught exception in launch: SubstitutionFailure("launch configuration 'aubo_type' does not exist")
```

**原因：**
启动文件中缺少 `aubo_type` 参数声明。

**解决方案：**

在 `generate_launch_description()` 函数中添加参数声明：

```python
declared_arguments.append(
    DeclareLaunchArgument(
        "aubo_type",
        default_value="e5",
        description="Type of AUBO robot (e5, i5, etc.)",
    )
)
```

---

### 问题3: XACRO解析错误 - "executed command failed"

**症状：**
```
[ERROR] [launch]: Caught exception in launch: SubstitutionFailure('executed command failed...')
```

**原因：**
- XACRO文件路径不正确
- XACRO文件语法错误
- 依赖的XACRO文件缺失

**解决方案：**

1. 检查XACRO文件是否存在：
```bash
find ~/aubo_ros2_ws -name "aubo_ros2.xacro"
```

2. 手动测试XACRO解析：
```bash
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5
```

3. 检查XACRO语法错误：
```bash
# 查找语法错误
grep -n "xacro:include\|xacro:macro" path/to/file.xacro
```

---

### 问题4: SRDF错误 - "Semantic description is not specified for the same robot"

**症状：**
```
[ERROR]: Semantic description is not specified for the same robot as the URDF
```

**原因：**
SRDF文件中的机器人名称与URDF中的机器人名称不匹配。

**解决方案：**

1. 检查URDF中的机器人名称：
```bash
grep -i "robot name" $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo.xacro
```

2. 检查SRDF中的机器人名称：
```bash
grep -i "robot name" aubo_moveit_config/config/aubo_i5.srdf
```

3. 修改SRDF使名称匹配（通常改为 `aubo_e5` 或与URDF一致）

---

### 问题5: Link错误 - "Link 'ee_link' declared as part of a chain... is not known to the URDF"

**症状：**
```
[ERROR]: Link 'ee_link' declared as part of a chain in group 'manipulator' is not known to the URDF
```

**原因：**
SRDF中引用了URDF中不存在的末端执行器链接。

**解决方案：**

在 `aubo.xacro` 中添加末端执行器链接：

```xml
<link name="ee_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="0.01" />
    <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001" />
  </inertial>
  <visual>
    <geometry>
      <box size="0.01 0.01 0.01" />
    </geometry>
    <material name="Grey" />
  </visual>
  <collision>
    <geometry>
      <box size="0.01 0.01 0.01" />
    </geometry>
  </collision>
</link>

<joint name="ee_fixed_joint" type="fixed">
  <parent link="wrist3_Link" />
  <child link="ee_link" />
  <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
</joint>
```

---

### 问题6: 控制器启动失败 - "Waiting for /controller_manager services"

**症状：**
```
[INFO] [spawner_joint_state_broadcaster]: Waiting for /controller_manager services
[ERROR] [spawner_joint_state_broadcaster]: process has died [pid XXX, exit code 1]
```

**原因：**
- `ros2_control_node` 未正确启动
- 超时时间太短
- 配置文件格式错误

**解决方案：**

1. 检查controller_manager节点是否启动：
```bash
ros2 node list | grep controller_manager
```

2. 检查控制器管理器服务：
```bash
ros2 service list | grep controller_manager
```

3. 增加spawner超时时间（在launch文件中）：
```python
cmd=["ros2 run controller_manager spawner.py joint_state_broadcaster --controller-manager-timeout 20"]
```

4. 检查 `ros2_controllers.yaml` 格式是否正确（参考[文件结构说明](#文件结构说明)）

---

### 问题7: MoveIt等待Action服务器 - "Waiting for ... follow_joint_trajectory to come up"

**症状：**
```
[WARN] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: 
Waiting for joint_trajectory_controller/follow_joint_trajectory to come up
```

**原因：**
- `action_ns` 配置错误（常见：包含了控制器名称前缀）
- 控制器未成功启动
- Action服务器名称不匹配

**解决方案：**

1. 检查 `moveit_controllers.yaml` 中的 `action_ns`：
```yaml
joint_trajectory_controller:
  action_ns: follow_joint_trajectory  # 正确：只有action名称
  # action_ns: joint_trajectory_controller/follow_joint_trajectory  # 错误：包含控制器名称
```

2. 检查Action服务器是否存在：
```bash
ros2 action list | grep follow_joint_trajectory
```

3. 检查控制器状态：
```bash
ros2 control list_controllers
```

4. 等待一段时间，Action服务器可能需要几秒钟启动（这是正常的）

---

### 问题8: TypeError - "Unexpected type for parameter value None"

**症状：**
```
[ERROR]: TypeError('Unexpected type for parameter value None')
```

**原因：**
YAML配置文件中某些参数值为 `None` 或格式不正确。

**解决方案：**

1. 检查YAML文件语法：
```bash
python3 -c "import yaml; yaml.safe_load(open('config/ros2_controllers.yaml'))"
```

2. 确保空参数使用 `{}` 而不是 `null`：
```yaml
joint_state_broadcaster:
  ros__parameters: {}  # 正确
  # ros__parameters: null  # 错误
```

3. 检查所有必需的参数是否都有值

---

## 验证配置是否成功

### 检查清单

按照以下步骤验证配置是否成功：

#### 1. 检查包是否安装

```bash
source ~/aubo_ros2_ws/install/setup.bash
ros2 pkg list | grep aubo
```

**预期输出：**
```
aubo_description
aubo_moveit_config
```

---

#### 2. 检查节点是否启动

启动配置后，检查关键节点：

```bash
ros2 node list
```

**预期输出包含：**
```
/controller_manager
/move_group
/robot_state_publisher
/rviz2
```

---

#### 3. 检查控制器状态

```bash
ros2 control list_controllers
```

**预期输出：**
```
joint_state_broadcaster[active]
joint_trajectory_controller[active]
```

如果控制器状态为 `inactive` 或 `unconfigured`，说明配置有问题。

---

#### 4. 检查Action服务器

```bash
ros2 action list
```

**预期输出包含：**
```
/joint_trajectory_controller/follow_joint_trajectory
/move_group/execute_trajectory
/move_group/move_action
```

---

#### 5. 检查关节状态话题

```bash
ros2 topic echo /joint_states --once
```

**预期输出：**
```
header:
  stamp:
    sec: ...
    nanosec: ...
  frame_id: ''
name:
- 'shoulder_joint'
- 'upperArm_joint'
- 'foreArm_joint'
- 'wrist1_joint'
- 'wrist2_joint'
- 'wrist3_joint'
position: [...]
velocity: [...]
effort: []
```

如果话题为空或没有数据，检查 `joint_state_broadcaster` 是否正常运行。

---

#### 6. 检查TF变换

```bash
ros2 run tf2_ros tf2_echo base_link ee_link
```

**预期输出：**
```
At time ...
- Translation: [x, y, z]
- Rotation: in Quaternion [x, y, z, w]
```

如果出现 "Frame does not exist" 错误，检查URDF和robot_state_publisher配置。

---

#### 7. 在RViz中验证

启动配置后，在RViz中应该看到：

- ✅ 机器人模型正确显示
- ✅ 可以在Motion Planning插件中规划轨迹
- ✅ 可以执行轨迹（Execute按钮）
- ✅ 机器人模型在规划/执行时移动

**在RViz中测试：**

1. 启动配置：
```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py use_ros2_control:=true use_fake_hardware:=true
```

2. 在RViz中：
   - 使用Motion Planning插件
   - 设置目标姿态（使用Interactive Marker）
   - 点击"Plan"按钮规划路径
   - 点击"Execute"按钮执行轨迹
   - 观察机器人模型是否移动

---

### 调试命令汇总

```bash
# 1. 检查包
ros2 pkg list | grep aubo

# 2. 检查节点
ros2 node list
ros2 node info /controller_manager
ros2 node info /move_group

# 3. 检查控制器
ros2 control list_controllers
ros2 control list_controller_types

# 4. 检查话题
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /joint_trajectory_controller/joint_trajectory_controller/commands

# 5. 检查Action
ros2 action list
ros2 action info /joint_trajectory_controller/follow_joint_trajectory

# 6. 检查服务
ros2 service list | grep controller_manager

# 7. 检查TF
ros2 run tf2_ros tf2_echo base_link ee_link
ros2 run tf2_tools view_frames

# 8. 检查参数
ros2 param list /controller_manager
ros2 param get /controller_manager update_rate
```

---

## 参考资源

### 官方文档

- [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html)
- [ROS 2 Control Documentation](https://control.ros.org/)
- [ROS 2 Control Hardware Interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)

### 示例代码

- [moveit2_tutorials](https://github.com/ros-planning/moveit2_tutorials)
- [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)

### 配置文件参考

- `/opt/ros/foxy/share/moveit_resources_panda_moveit_config/config/panda_ros_controllers.yaml`
- `/opt/ros/foxy/share/moveit_resources_panda_moveit_config/config/panda_controllers.yaml`

---

## 附录

### A. 完整的ros2_controllers.yaml模板

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_joint
      - upperArm_joint
      - foreArm_joint
      - wrist1_joint
      - wrist2_joint
      - wrist3_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    state_publish_rate: 50.0
    action_monitor_rate: 20.0

    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.6
      shoulder_joint:
        trajectory: 0.1
        goal: 0.1
      # ... 其他关节的约束

joint_state_broadcaster:
  ros__parameters: {}
```

### B. 完整的moveit_controllers.yaml模板

```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.01

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

controller_names:
  - joint_trajectory_controller

joint_trajectory_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - shoulder_joint
    - upperArm_joint
    - foreArm_joint
    - wrist1_joint
    - wrist2_joint
    - wrist3_joint
```

### C. 常见启动命令

```bash
# 使用ROS 2 Control（仿真）
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
    use_ros2_control:=true \
    use_fake_hardware:=true \
    aubo_type:=e5

# 使用传统joint_state_publisher（仅可视化）
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
    use_ros2_control:=false \
    aubo_type:=e5

# 指定RViz配置
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
    use_ros2_control:=true \
    use_fake_hardware:=true \
    rviz_config:=config/moveit.rviz
```

---

**文档版本**: 1.0  
**最后更新**: 2024-12-18  
**维护者**: AUBO Robotics Team

