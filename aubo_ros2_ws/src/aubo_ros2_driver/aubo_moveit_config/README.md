# aubo_moveit_config

AUBO 机械臂 **MoveIt2** 配置包：SRDF、控制器、规划器、RViz 配置与各类 launch（含与 `demo_driver` 联动）。

## 本文档说明

以下内容按主题合并了包内原独立 Markdown。**`doc/`** 目录下的排障文档不并入本文，见文末「其他文档」。

## 目录

- [故障排除](#故障排除)
- [ROS 2 Control 与 MoveIt 集成](#ros-2-control-与-moveit-集成)
- [RViz Motion Planning 插件与配置映射](#rviz-motion-planning-插件与配置映射)
- [工作空间限制脚本（limit_workspace.py）](#工作空间限制脚本limit_workspacepy)



---

## 故障排除

本文档提供常见问题的快速诊断和解决方案。

---

### 问题诊断流程图

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

### 快速检查清单

在深入调试之前，先运行这些命令进行快速检查：

```bash
## 1. 检查环境
echo $ROS_DISTRO
source /home/mu/IVG2.0/aubo_ros2_ws/install/setup.bash

## 2. 检查包
ros2 pkg list | grep aubo

## 3. 检查关键文件是否存在
ros2 pkg prefix aubo_description
ros2 pkg prefix aubo_moveit_config
ls $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro

## 4. 测试XACRO解析
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5 > /tmp/test_urdf.urdf
check_urdf /tmp/test_urdf.urdf  # 如果安装了urdfdom工具
```

---

### 常见错误代码及解决方案

#### ERR001: PackageNotFoundError

**错误信息：**
```
PackageNotFoundError: "package 'aubo_description' not found"
```

**快速诊断：**
```bash
## 检查包是否存在
ros2 pkg list | grep aubo_description

## 如果不存在
cd /home/mu/IVG2.0/aubo_ros2_ws
colcon build --packages-select aubo_description
source install/setup.bash
```

**解决方案：**
- 确保包已构建：`colcon build --packages-select <package_name>`
- 确保已source工作空间：`source install/setup.bash`
- 检查package.xml中的包名是否正确

---

#### ERR002: XACRO解析失败

**错误信息：**
```
SubstitutionFailure('executed command failed. Command: xacro ...')
```

**快速诊断：**
```bash
## 手动测试XACRO
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5

## 检查文件是否存在
find /home/mu/IVG2.0/aubo_ros2_ws -name "aubo_ros2.xacro"
```

**解决方案：**
- 检查XACRO文件路径是否正确
- 检查XACRO语法（括号、引号等）
- 检查依赖的XACRO文件是否存在
- 检查参数是否正确传递（如 `aubo_type`）

---

#### ERR003: 控制器启动失败

**错误信息：**
```
[ERROR] [spawner_joint_state_broadcaster]: process has died [pid XXX, exit code 1]
```

**快速诊断：**
```bash
## 检查controller_manager节点
ros2 node list | grep controller_manager

## 检查控制器配置
ros2 param list /controller_manager

## 检查服务
ros2 service list | grep controller_manager
```

**解决方案：**

1. **检查ros2_controllers.yaml格式：**
```yaml
## 确保格式正确
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
## 从URDF提取关节名称
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5 | grep -oP 'name="\K[^"]*(?=".*type="revolute")'

## 与ros2_controllers.yaml中的关节列表对比
grep -A 10 "joints:" config/ros2_controllers.yaml
```

---

#### ERR004: MoveIt找不到Action服务器

**错误信息：**
```
[WARN] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: 
Waiting for joint_trajectory_controller/follow_joint_trajectory to come up
```

**快速诊断：**
```bash
## 检查Action服务器
ros2 action list | grep follow_joint_trajectory

## 检查控制器状态
ros2 control list_controllers

## 检查MoveIt配置
ros2 param get /move_group moveit_simple_controller_manager
```

**解决方案：**

1. **修正action_ns配置（常见错误）：**
```yaml
## moveit_controllers.yaml
joint_trajectory_controller:
  action_ns: follow_joint_trajectory  # 正确：只有action名称
  # action_ns: joint_trajectory_controller/follow_joint_trajectory  # 错误！
```

2. **等待Action服务器启动（可能需要几秒钟）**

3. **检查控制器是否处于active状态：**
```bash
ros2 control list_controllers
## 应该显示：joint_trajectory_controller[active]
```

---

#### ERR005: 机器人模型不显示

**错误信息：**
- RViz中看不到机器人模型
- TF错误："Frame does not exist"

**快速诊断：**
```bash
## 检查TF树
ros2 run tf2_ros tf2_echo base_link ee_link

## 检查joint_states话题
ros2 topic echo /joint_states --once

## 检查robot_state_publisher节点
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

#### ERR006: SRDF错误

**错误信息：**
```
Error: Semantic description is not specified for the same robot as the URDF
Error: Link 'ee_link' declared as part of a chain... is not known to the URDF
```

**快速诊断：**
```bash
## 检查URDF中的机器人名称
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo.xacro aubo_type:=e5 | grep -i "robot name"

## 检查SRDF中的机器人名称
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

#### ERR007: 参数类型错误

**错误信息：**
```
TypeError('Unexpected type for parameter value None')
```

**快速诊断：**
```bash
## 检查YAML语法
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

### 调试技巧

#### 1. 启用详细日志

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

#### 2. 逐步启动

不要一次启动所有节点，逐步启动以定位问题：

```bash
## 1. 只启动controller_manager
ros2 run controller_manager ros2_control_node \
    --ros-args \
    -p robot_description:="$(xacro ...)" \
    -p ros2_controllers_config:="$(cat config/ros2_controllers.yaml)"

## 2. 在另一个终端启动spawner
ros2 run controller_manager spawner.py joint_state_broadcaster

## 3. 检查是否成功，再启动下一个
ros2 run controller_manager spawner.py joint_trajectory_controller
```

#### 3. 检查进程状态

```bash
## 检查进程是否在运行
ps aux | grep ros2_control_node
ps aux | grep move_group

## 检查进程退出代码
echo $?  # 在进程结束后立即检查
```

#### 4. 查看日志文件

```bash
## ROS 2日志位置
ls ~/.ros/log/

## 查看最新的日志
ls -lt ~/.ros/log/latest/ | head -10
```

---

### 验证成功标志

配置成功后，您应该看到以下标志：

#### ✅ 节点运行正常

```bash
$ ros2 node list
/controller_manager
/move_group
/robot_state_publisher
/rviz2
```

#### ✅ 控制器处于active状态

```bash
$ ros2 control list_controllers
joint_state_broadcaster[active]
joint_trajectory_controller[active]
```

#### ✅ Action服务器可用

```bash
$ ros2 action list
/joint_trajectory_controller/follow_joint_trajectory
/move_group/execute_trajectory
/move_group/move_action
```

#### ✅ 关节状态发布正常

```bash
$ ros2 topic hz /joint_states
average rate: 50.000
```

#### ✅ TF变换正常

```bash
$ ros2 run tf2_ros tf2_echo base_link ee_link
[INFO] [tf2_echo]: Waiting for transform base_link -> ee_link:
At time 1234.567
- Translation: [x, y, z]
- Rotation: in Quaternion [x, y, z, w]
```

#### ✅ RViz中功能正常

- 机器人模型正确显示
- 可以规划轨迹
- 可以执行轨迹
- 机器人模型在执行时移动

---

### 获取帮助

如果问题仍未解决：

1. **检查完整日志：**
```bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py use_ros2_control:=true use_fake_hardware:=true 2>&1 | tee launch.log
```

2. **收集系统信息：**
```bash
## ROS版本
echo $ROS_DISTRO

## 已安装的包
ros2 pkg list | grep -E "(moveit|control)" | sort

## 工作空间结构
tree -L 3 /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/
```

3. **参考官方文档：**
- [MoveIt 2 Troubleshooting](https://moveit.picknik.ai/main/doc/how_to_guides/how_to_guides.html)
- [ROS 2 Control Issues](https://github.com/ros-controls/ros2_control/issues)

---

**文档版本**: 1.0  
**最后更新**: 2024-12-18



---

## ROS 2 Control 与 MoveIt 集成

### 目录
1. [概述](#概述)
2. [前置条件](#前置条件)
3. [文件结构说明](#文件结构说明)
4. [从零开始的配置步骤](#从零开始的配置步骤)
5. [常见问题及解决方案](#常见问题及解决方案)
6. [验证配置是否成功](#验证配置是否成功)
7. [参考资源](#参考资源)

---

### 概述

本文档详细说明了如何为AUBO机器人配置MoveIt 2和ROS 2 Control，实现仿真控制。配置完成后，可以通过MoveIt在RViz中进行运动规划，并通过ROS 2 Control执行轨迹。

#### 主要组件

- **MoveIt 2**: 运动规划框架
- **ROS 2 Control**: 机器人控制框架
- **Fake Hardware Interface**: 仿真硬件接口（用于测试）

---

### 前置条件

#### 系统要求

- **ROS 2发行版**: Foxy Fitzroy 或更高版本
- **操作系统**: Ubuntu 20.04 (Foxy) 或 Ubuntu 22.04 (Humble)

#### 必需的ROS 2包

确保已安装以下包：

```bash
## 核心MoveIt包
sudo apt install ros-foxy-moveit ros-foxy-moveit-core ros-foxy-moveit-ros-planning-interface

## ROS 2 Control包
sudo apt install ros-foxy-ros2-control ros-foxy-ros2-controllers \
     ros-foxy-controller-manager ros-foxy-joint-trajectory-controller \
     ros-foxy-joint-state-broadcaster

## 其他依赖
sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher \
     ros-foxy-joint-state-publisher-gui ros-foxy-robot-state-publisher \
     ros-foxy-rviz2
```

#### 工作空间结构

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

### 文件结构说明

#### 关键配置文件

##### 1. `aubo_moveit_config/config/ros2_controllers.yaml`

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

## 控制器详细参数
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

##### 2. `aubo_moveit_config/config/moveit_controllers.yaml`

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

##### 3. `aubo_description/urdf/xacro/inc/aubo_ros2.xacro`

包含ROS 2 Control硬件接口的URDF宏定义。

##### 4. `aubo_moveit_config/launch/aubo_moveit.launch.py`

主启动文件，根据 `use_ros2_control` 参数选择使用ROS 2 Control或传统的joint_state_publisher。

---

### 从零开始的配置步骤

#### 步骤1: 创建工作空间并获取代码

```bash
## 创建工作空间
mkdir -p /home/mu/IVG2.0/aubo_ros2_ws/src
cd /home/mu/IVG2.0/aubo_ros2_ws/src

## 获取aubo_ros2_driver代码（假设已存在）
## git clone <your-repo-url> aubo_ros2_driver
```

#### 步骤2: 创建/检查aubo_description包

确保 `aubo_description` 包存在且包含必要的URDF/XACRO文件：

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver

## 检查包结构
ls -la aubo_description/
ls -la aubo_description/urdf/xacro/inc/
```

**必需文件：**
- `aubo_description/package.xml`
- `aubo_description/CMakeLists.txt`
- `aubo_description/urdf/xacro/inc/aubo.xacro` (基础URDF)
- `aubo_description/urdf/xacro/inc/aubo_ros2.xacro` (ROS 2 Control版本)
- `aubo_description/urdf/xacro/inc/aubo_ros2_control.ros2_control.xacro` (硬件接口定义)

#### 步骤3: 配置ros2_controllers.yaml

在 `aubo_moveit_config/config/ros2_controllers.yaml` 中配置控制器：

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/aubo_moveit_config/config
```

创建或编辑 `ros2_controllers.yaml`，内容如[文件结构说明](#文件结构说明)中所示。

**关键点：**
- `controller_manager.ros__parameters` 下定义控制器类型
- 控制器详细参数在顶层定义（如 `joint_trajectory_controller:`）
- 关节名称必须与URDF中的关节名称完全匹配

#### 步骤4: 配置moveit_controllers.yaml

在 `aubo_moveit_config/config/moveit_controllers.yaml` 中配置MoveIt控制器：

```bash
## 编辑文件
nano aubo_moveit_config/config/moveit_controllers.yaml
```

**关键配置：**
- `action_ns: follow_joint_trajectory` （不要包含控制器名称前缀）
- `type: FollowJointTrajectory`
- `joints` 列表必须与 `ros2_controllers.yaml` 中的关节列表一致

#### 步骤5: 配置启动文件

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

#### 步骤6: 构建工作空间

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select aubo_description aubo_moveit_config
source install/setup.bash
```

#### 步骤7: 启动配置

```bash
## 启动MoveIt with ROS 2 Control
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
    use_ros2_control:=true \
    use_fake_hardware:=true \
    aubo_type:=e5
```

---

### 常见问题及解决方案

#### 问题1: PackageNotFoundError - "package 'aubo_description' not found"

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
cd /home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver
mkdir -p aubo_description
## 创建package.xml和CMakeLists.txt
```

3. 重新构建并source：
```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
colcon build --packages-select aubo_description
source install/setup.bash
```

---

#### 问题2: SubstitutionFailure - "launch configuration 'aubo_type' does not exist"

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

#### 问题3: XACRO解析错误 - "executed command failed"

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
find /home/mu/IVG2.0/aubo_ros2_ws -name "aubo_ros2.xacro"
```

2. 手动测试XACRO解析：
```bash
xacro $(ros2 pkg prefix aubo_description)/share/aubo_description/urdf/xacro/inc/aubo_ros2.xacro aubo_type:=e5
```

3. 检查XACRO语法错误：
```bash
## 查找语法错误
grep -n "xacro:include\|xacro:macro" path/to/file.xacro
```

---

#### 问题4: SRDF错误 - "Semantic description is not specified for the same robot"

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

#### 问题5: Link错误 - "Link 'ee_link' declared as part of a chain... is not known to the URDF"

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

#### 问题6: 控制器启动失败 - "Waiting for /controller_manager services"

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

#### 问题7: MoveIt等待Action服务器 - "Waiting for ... follow_joint_trajectory to come up"

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

#### 问题8: TypeError - "Unexpected type for parameter value None"

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

### 验证配置是否成功

#### 检查清单

按照以下步骤验证配置是否成功：

##### 1. 检查包是否安装

```bash
source /home/mu/IVG2.0/aubo_ros2_ws/install/setup.bash
ros2 pkg list | grep aubo
```

**预期输出：**
```
aubo_description
aubo_moveit_config
```

---

##### 2. 检查节点是否启动

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

##### 3. 检查控制器状态

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

##### 4. 检查Action服务器

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

##### 5. 检查关节状态话题

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

##### 6. 检查TF变换

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

##### 7. 在RViz中验证

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

#### 调试命令汇总

```bash
## 1. 检查包
ros2 pkg list | grep aubo

## 2. 检查节点
ros2 node list
ros2 node info /controller_manager
ros2 node info /move_group

## 3. 检查控制器
ros2 control list_controllers
ros2 control list_controller_types

## 4. 检查话题
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /joint_trajectory_controller/joint_trajectory_controller/commands

## 5. 检查Action
ros2 action list
ros2 action info /joint_trajectory_controller/follow_joint_trajectory

## 6. 检查服务
ros2 service list | grep controller_manager

## 7. 检查TF
ros2 run tf2_ros tf2_echo base_link ee_link
ros2 run tf2_tools view_frames

## 8. 检查参数
ros2 param list /controller_manager
ros2 param get /controller_manager update_rate
```

---

### 参考资源

#### 官方文档

- [MoveIt 2 Documentation](https://moveit.picknik.ai/main/index.html)
- [ROS 2 Control Documentation](https://control.ros.org/)
- [ROS 2 Control Hardware Interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)

#### 示例代码

- [moveit2_tutorials](https://github.com/ros-planning/moveit2_tutorials)
- [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)

#### 配置文件参考

- `/opt/ros/foxy/share/moveit_resources_panda_moveit_config/config/panda_ros_controllers.yaml`
- `/opt/ros/foxy/share/moveit_resources_panda_moveit_config/config/panda_controllers.yaml`

---

### 附录

#### A. 完整的ros2_controllers.yaml模板

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

#### B. 完整的moveit_controllers.yaml模板

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

#### C. 常见启动命令

```bash
## 使用ROS 2 Control（仿真）
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
    use_ros2_control:=true \
    use_fake_hardware:=true \
    aubo_type:=e5

## 使用传统joint_state_publisher（仅可视化）
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
    use_ros2_control:=false \
    aubo_type:=e5

## 指定RViz配置
ros2 launch aubo_moveit_config aubo_moveit.launch.py \
    use_ros2_control:=true \
    use_fake_hardware:=true \
    rviz_config:=config/moveit.rviz
```

---

**文档版本**: 1.0  
**最后更新**: 2024-12-18  
**维护者**: AUBO Robotics Team



---

## RViz Motion Planning 插件与配置映射

本文档详细说明MoveIt配置文件如何与RViz2中Motion Planning插件的各个设置项对应。

---

### 目录

1. [概述](#概述)
2. [Motion Planning插件界面结构](#motion-planning插件界面结构)
3. [配置文件映射关系](#配置文件映射关系)
4. [详细配置项对应表](#详细配置项对应表)
5. [配置修改示例](#配置修改示例)

---

### 概述

RViz2的Motion Planning插件通过ROS 2话题和参数服务器与MoveIt的move_group节点通信。插件界面上显示的设置和选项主要从以下配置文件读取：

- **SRDF文件** (`aubo_i5.srdf`) - 规划组、关节状态、碰撞检测设置
- **moveit_controllers.yaml** - 控制器配置
- **ompl_planning.yaml** - 规划器配置
- **kinematics.yaml** - 运动学求解器配置
- **joint_limits.yaml** - 关节限制配置
- **启动文件参数** - 话题名称、命名空间等

---

### Motion Planning插件界面结构

RViz2 Motion Planning插件主要分为以下几个部分：

1. **Motion Planning面板** - 左侧控制面板
   - Context（上下文）- 规划组选择
   - Planning（规划）- 规划参数设置
   - Commands（命令）- 执行控制
   - Current State（当前状态）- 机器人状态
   - Goal State（目标状态）- 目标设置

2. **Displays面板** - 右侧显示设置
   - Planned Path（规划路径）- 路径可视化
   - Scene Robot（场景机器人）- 机器人显示
   - Planning Request（规划请求）- 规划请求可视化

---

### 配置文件映射关系

#### 1. 规划组（Planning Groups）选择

**RViz界面位置：** Context → Planning Group 下拉菜单

**配置文件：** `config/aubo_e5.srdf`

**对应代码：**
```xml
<group name="manipulator">
    <chain base_link="base_link" tip_link="tool_tcp"/>
</group>
<group name="endeffector">
    <link name="tool_tcp"/>
</group>
```

**说明：**
- SRDF文件中定义的`<group>`标签会在Motion Planning插件的"Planning Group"下拉菜单中显示
- 用户可以选择不同的规划组进行运动规划
- 每个规划组定义了哪些关节参与规划

#### ACM (Allowed Collision Matrix) — 末端夹爪工具碰撞豁免

工具通过 `kuaihuan_Link` 挂载在 `wrist3_Link` 下。末端固定链 (`wrist3_Link → camera_Link → kuaihuan_Link → 工具 Link`) 与 `tool_tcp` 之间均属固定运动学关系，不应触发碰撞检测：

```xml
<!-- 5 种末端工具 × 4 个末端固定链 link = 20 条 -->
<disable_collisions link1="kuaihuan_Link" link2="gripper0_Link"  reason="Adjacent"/>
<disable_collisions link1="kuaihuan_Link" link2="gripper1_Link"  reason="Adjacent"/>
... (共 20 条，见 aubo_e5.srdf:69-89)
```

| link1 | 5 个工具 link2 | 理由 |
|-------|---------------|------|
| `kuaihuan_Link` | gripper0/1/2/1coffeecup/1milkcup_Link | Adjacent（直接父子） |
| `camera_Link` | 同上 | Adjacent（隔一层） |
| `wrist3_Link` | 同上 | Never（隔两层） |
| `tool_tcp` | 同上 | Never（同父分叉） |

> 缺乏这些豁免会导致 MoveIt 将工具 mesh 与末端 link 的几何重叠误判为碰撞，规划失败喵~

---

#### 2. 规划器（Planner）选择

**RViz界面位置：** Planning → Planner 下拉菜单

**配置文件：** `config/ompl_planning.yaml`

**对应代码：**
```yaml
manipulator:
  default_planner_config: RRTConnect  # 默认规划器
  planner_configs:                    # 可用规划器列表
    - AnytimePathShortening
    - SBL
    - EST
    - RRTConnect
    - RRTstar
    - PRM
    # ... 更多规划器
```

**说明：**
- `planner_configs` 列表中的规划器会出现在RViz的"Planner"下拉菜单中
- `default_planner_config` 设置默认选中的规划器
- 每个规划组可以有不同的规划器配置

---

#### 3. 规划时间（Planning Time）

**RViz界面位置：** Planning → Planning Time 输入框

**配置文件：** 通过ROS参数动态设置，或通过RViz插件内部参数

**启动文件参数：** `aubo_moveit.launch.py` 中的 `ompl_planning_pipeline_config`

**对应代码：**
```python
## 在启动文件中，可以设置默认规划时间
ompl_planning_pipeline_config = {
    "move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "planning_time": 5.0,  # 默认规划时间（秒）
        # ...
    }
}
```

**说明：**
- 在RViz中可以手动输入规划时间（秒）
- 该值会被发送到move_group节点进行规划请求
- 默认值通常为5.0秒

---

#### 4. 规划尝试次数（Planning Attempts）

**RViz界面位置：** Planning → Planning Attempts 输入框

**配置文件：** 通过ROS参数或RViz插件内部参数

**说明：**
- 指定规划算法尝试多少次来找到有效路径
- 默认值通常为10次
- 在RViz中可以手动修改

---

#### 5. 速度缩放因子（Velocity Scaling Factor）

**RViz界面位置：** Planning → Velocity Scaling Factor 滑块

**配置文件：** `config/joint_limits.yaml`

**对应代码：**
```yaml
default_velocity_scaling_factor: 1.0  # 默认速度缩放因子（0.0-1.0）
default_acceleration_scaling_factor: 0.1  # 默认加速度缩放因子
```

**说明：**
- 在RViz中通过滑块调整（0.0-1.0）
- 该值影响轨迹执行的速度
- `joint_limits.yaml`中的`default_velocity_scaling_factor`设置默认值

---

#### 6. 加速度缩放因子（Acceleration Scaling Factor）

**RViz界面位置：** Planning → Acceleration Scaling Factor 输入框

**配置文件：** `config/joint_limits.yaml`

**对应代码：**
```yaml
default_acceleration_scaling_factor: 0.1  # 默认加速度缩放因子
```

**说明：**
- 在RViz中可以手动输入
- 影响轨迹执行的加速度
- 通常设置为较小的值（如0.1）以确保平滑运动

---

#### 7. 控制器选择和执行

**RViz界面位置：** Commands → Execute 按钮

**配置文件：** `config/moveit_controllers.yaml`

**对应代码：**
```yaml
controller_names:
  - joint_trajectory_controller  # 可用的控制器列表

joint_trajectory_controller:
  action_ns: follow_joint_trajectory  # Action命名空间
  type: FollowJointTrajectory
  default: true  # 默认控制器
  joints:
    - shoulder_joint
    - upperArm_joint
    - foreArm_joint
    - wrist1_joint
    - wrist2_joint
    - wrist3_joint
```

**说明：**
- `controller_names` 列表定义了可用的控制器
- `default: true` 标记的控制器会被默认使用
- MoveIt通过Action接口（`/joint_trajectory_controller/follow_joint_trajectory`）发送轨迹
- Action服务器由ROS 2 Control的`joint_trajectory_controller`提供

---

#### 8. 机器人描述（Robot Description）

**RViz界面位置：** Displays → MotionPlanning → Robot Description 参数

**配置文件：** 启动文件中的`robot_description`参数

**对应代码：**
```python
## aubo_moveit.launch.py
robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]),
    " ",
    PathJoinSubstitution([
        FindPackageShare(support_package), 
        "urdf/xacro/inc/", 
        robot_xacro_file_to_use
    ]),
    " aubo_type:=", aubo_type,
])
robot_description = {"robot_description": robot_description_content}

## 传递给RViz节点
rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    parameters=[
        robot_description,  # ← 这里
        robot_description_semantic,
        # ...
    ],
)
```

**RViz配置：**
```yaml
## config/moveit.rviz
Robot Description: robot_description  # ROS参数名称
```

**说明：**
- RViz从ROS参数服务器读取`robot_description`参数
- 该参数包含完整的URDF/XACRO生成的机器人描述
- 用于在RViz中渲染机器人模型

---

#### 9. 规划场景话题（Planning Scene Topic）

**RViz界面位置：** Displays → MotionPlanning → Planning Scene Topic 参数

**配置文件：** 启动文件中的`planning_scene_monitor_parameters`

**对应代码：**
```python
## aubo_moveit.launch.py
planning_scene_monitor_parameters = {
    "planning_scene_monitor_options": {
        "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
        # ...
    }
}
```

**RViz配置：**
```yaml
## config/moveit.rviz
Planning Scene Topic: monitored_planning_scene  # 话题名称（相对于move_group命名空间）
```

**说明：**
- RViz订阅该话题来获取规划场景信息
- 用于显示碰撞对象、环境障碍物等
- 默认话题：`/move_group/monitored_planning_scene`

---

#### 10. 规划路径话题（Planned Path Topic）

**RViz界面位置：** Displays → MotionPlanning → Planned Path → Trajectory Topic

**配置文件：** RViz配置文件

**对应代码：**
```yaml
## config/moveit.rviz
Planned Path:
  Trajectory Topic: display_planned_path  # 话题名称
```

**说明：**
- move_group发布规划好的轨迹到这个话题
- RViz订阅该话题来可视化规划的路径
- 默认话题：`/display_planned_path`

---

#### 11. 运动学求解器配置

**RViz界面位置：** 隐式使用（用于IK求解）

**配置文件：** `config/kinematics.yaml`

**对应代码：**
```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

**说明：**
- 当用户在RViz中拖动交互式标记（Interactive Marker）设置目标位姿时
- MoveIt使用运动学求解器进行逆运动学（IK）计算
- `kinematics_solver_search_resolution`影响IK求解精度
- `kinematics_solver_timeout`设置求解超时时间

---

#### 12. 关节限制

**RViz界面位置：** 隐式使用（在规划时检查）

**配置文件：** `config/joint_limits.yaml`

**对应代码：**
```yaml
joint_limits:
  shoulder_joint:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: false
    max_acceleration: 3.15
  # ... 其他关节
```

**说明：**
- MoveIt在规划时检查关节限制
- 如果目标位姿超出关节限制，规划会失败
- `max_velocity`和`max_acceleration`限制轨迹执行速度

---

#### 13. 轨迹执行参数

**RViz界面位置：** 隐式使用（在执行时应用）

**配置文件：** 启动文件中的`trajectory_execution`参数

**对应代码：**
```python
## aubo_moveit.launch.py
trajectory_execution = {
    "moveit_manage_controllers": False,
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
}
```

**或者：**

```yaml
## config/moveit_controllers.yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2  # 允许的执行时间缩放
  allowed_goal_duration_margin: 0.5        # 目标时间容差
  allowed_start_tolerance: 0.01            # 起始状态容差
```

**说明：**
- `allowed_execution_duration_scaling`: 允许轨迹执行时间比计划时间长多少倍
- `allowed_goal_duration_margin`: 允许的目标时间误差（秒）
- `allowed_start_tolerance`: 允许的起始位置误差（弧度）

---

#### 14. Move Group命名空间

**RViz界面位置：** Displays → MotionPlanning → Move Group Namespace

**配置文件：** RViz配置文件或启动参数

**对应代码：**
```yaml
## config/view_robot.rviz
Move Group Namespace: ""  # 空字符串表示使用默认命名空间
```

**说明：**
- 如果move_group节点运行在非默认命名空间（如`/robot1/move_group`）
- 需要在此处指定命名空间（如`robot1`）
- 空字符串表示使用默认命名空间（`/move_group`）

---

### 详细配置项对应表

| RViz界面位置 | 配置文件/参数 | 配置项路径/键名 | 说明 |
|-------------|--------------|----------------|------|
| **Context → Planning Group** | `aubo_i5.srdf` | `<group name="...">` | 规划组列表 |
| **Planning → Planner** | `ompl_planning.yaml` | `manipulator.planner_configs` | 可用规划器列表 |
| **Planning → Planner (默认)** | `ompl_planning.yaml` | `manipulator.default_planner_config` | 默认规划器 |
| **Planning → Planning Time** | 启动参数/RViz内部 | 动态参数 | 规划时间（秒） |
| **Planning → Planning Attempts** | RViz内部 | 动态参数 | 规划尝试次数 |
| **Planning → Velocity Scaling** | `joint_limits.yaml` | `default_velocity_scaling_factor` | 速度缩放因子 |
| **Planning → Acceleration Scaling** | `joint_limits.yaml` | `default_acceleration_scaling_factor` | 加速度缩放因子 |
| **Commands → Execute** | `moveit_controllers.yaml` | `controller_names`, `default` | 控制器选择 |
| **Displays → Robot Description** | 启动参数 | `robot_description` | URDF描述 |
| **Displays → Planning Scene Topic** | 启动参数 | `monitored_planning_scene_topic` | 规划场景话题 |
| **Displays → Trajectory Topic** | RViz配置 | `display_planned_path` | 规划路径话题 |
| **IK求解（交互式标记）** | `kinematics.yaml` | `kinematics_solver` | 运动学求解器 |
| **关节限制检查** | `joint_limits.yaml` | `joint_limits.*` | 关节限制 |
| **轨迹执行参数** | `moveit_controllers.yaml` | `trajectory_execution.*` | 执行容差 |

---

### 配置修改示例

#### 示例1: 添加新的规划器

**需求：** 在RViz的规划器下拉菜单中添加新的规划器

**步骤：**

1. 编辑 `config/ompl_planning.yaml`：
```yaml
manipulator:
  planner_configs:
    - AnytimePathShortening
    - SBL
    - EST
    - RRTConnect
    - RRTstar
    - PRM
    - NewPlanner  # ← 添加新规划器（需要先在planner_configs中定义）
```

2. 在文件顶部定义规划器配置：
```yaml
planner_configs:
  NewPlanner:
    type: geometric::RRTConnect  # 规划器类型
    range: 0.0
    # ... 其他参数
```

3. 重新构建并启动：
```bash
colcon build --packages-select aubo_moveit_config
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py
```

4. 在RViz的"Planner"下拉菜单中即可看到新规划器

---

#### 示例2: 修改默认速度缩放因子

**需求：** 降低默认运动速度，使机器人运动更平滑

**步骤：**

1. 编辑 `config/joint_limits.yaml`：
```yaml
default_velocity_scaling_factor: 0.5  # 从1.0改为0.5（降低到50%）
default_acceleration_scaling_factor: 0.05  # 从0.1改为0.05
```

2. 重新构建并启动，RViz中的速度缩放滑块默认值会变为0.5

---

#### 示例3: 更改规划场景话题

**需求：** 使用自定义的规划场景话题

**步骤：**

1. 修改启动文件 `launch/aubo_moveit.launch.py`：
```python
planning_scene_monitor_parameters = {
    "planning_scene_monitor_options": {
        "monitored_planning_scene_topic": "/custom/planning_scene",  # 修改话题名称
        # ...
    }
}
```

2. 或者在RViz中手动修改：
   - Displays → MotionPlanning → Planning Scene Topic
   - 输入：`custom/planning_scene`

---

#### 示例4: 添加新的规划组

**需求：** 添加一个新的规划组（例如只控制前3个关节）

**步骤：**

1. 编辑 `config/aubo_i5.srdf`：
```xml
<group name="arm_base">
    <chain base_link="base_link" tip_link="foreArm_Link"/>
</group>
```

2. 在 `config/ompl_planning.yaml` 中添加该组的规划器配置：
```yaml
arm_base:
  default_planner_config: RRTConnect
  planner_configs:
    - RRTConnect
    - RRTstar
    - PRM
```

3. 在 `config/kinematics.yaml` 中添加运动学配置（如果需要）：
```yaml
arm_base:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

4. 重新构建并启动，在RViz的"Planning Group"下拉菜单中即可看到新组

---

#### 示例5: 配置多个控制器

**需求：** 支持多个控制器（例如主机械臂和末端执行器）

**步骤：**

1. 编辑 `config/moveit_controllers.yaml`：
```yaml
controller_names:
  - arm_controller
  - gripper_controller  # 添加新控制器

arm_controller:
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

gripper_controller:
  action_ns: gripper_action
  type: GripperCommand
  default: false
  joints:
    - gripper_joint
```

2. 在RViz中，MoveIt会根据规划的轨迹自动选择相应的控制器

---

### 实际工作流程

当您在RViz中操作时，配置的使用流程如下：

1. **选择Planning Group** → 从SRDF读取group定义，确定哪些关节参与规划
2. **选择Planner** → 从ompl_planning.yaml读取规划器配置
3. **设置目标位姿** → 使用kinematics.yaml中的运动学求解器计算IK
4. **点击Plan** → move_group使用选定的规划器进行规划，考虑joint_limits.yaml中的限制
5. **点击Execute** → MoveIt通过moveit_controllers.yaml中配置的Action接口发送轨迹
6. **轨迹执行** → ROS 2 Control的joint_trajectory_controller接收并执行轨迹

---

### 关键话题和Action接口

#### 输入话题（RViz → MoveIt）

- `/move_group/goal` - 规划请求
- `/move_group/query` - 查询请求
- `/move_group/execute` - 执行请求

#### 输出话题（MoveIt → RViz）

- `/display_planned_path` - 规划的路径可视化
- `/move_group/monitored_planning_scene` - 规划场景更新
- `/move_group/display_contacts` - 碰撞接触点

#### Action接口（MoveIt ↔ 控制器）

- `/joint_trajectory_controller/follow_joint_trajectory` - 轨迹执行Action
- `/move_group/move_action` - MoveIt移动Action
- `/move_group/execute_trajectory` - 轨迹执行Action

---

### 总结

RViz2 Motion Planning插件的配置主要来源于：

1. **SRDF文件** → 规划组定义、组状态、碰撞检测
2. **OMPL规划配置文件** → 规划器选择、规划器参数
3. **关节限制配置文件** → 速度和加速度限制、默认缩放因子
4. **控制器配置文件** → 轨迹执行、Action接口
5. **启动文件参数** → 话题和命名空间、机器人描述
6. **RViz配置文件** → 可视化设置、显示选项

理解这些对应关系有助于：
- 快速定位配置问题
- 根据需求调整配置
- 优化运动规划性能
- 自定义Motion Planning插件行为
- 调试规划和执行问题

---

**文档版本**: 1.0  
**最后更新**: 2024-12-18  
**维护者**: AUBO Robotics Team



---

## 工作空间限制脚本（limit_workspace.py）

### 功能说明

`limit_workspace.py` 脚本通过添加边界墙（碰撞对象）来限制 MoveIt2 的工作空间范围。机械臂将无法规划超出这些边界的路径。

### 快速开始

#### 1. 启动 MoveIt2

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source install/setup.bash
ros2 launch aubo_moveit_config aubo_moveit.launch.py
```

#### 2. 运行工作空间限制脚本

在另一个终端中：

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source install/setup.bash
python3 src/aubo_ros2_driver/aubo_moveit_config/scripts/limit_workspace.py
```

**注意**：脚本会自动使用同目录下的 `workspace_limits.yaml` 配置文件（如果存在）。如果该文件不存在，则使用默认值。

#### 3. 在 RViz2 中查看

- 打开 RViz2（如果还没有打开）
- 在 Motion Planning 插件的 "Scene Geometry" 部分应该能看到边界墙
- 边界墙会显示为半透明的盒子

### 自定义工作空间边界

有三种方式可以自定义工作空间边界（按优先级从高到低）：

#### 方法1：命令行参数（最灵活，推荐）

```bash
## 使用命令行参数指定边界
python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8

## 只修改部分参数（其他使用默认值）
python3 limit_workspace.py --x-min -0.6 --x-max 0.6

## 修改墙厚度
python3 limit_workspace.py --wall-thickness 0.1
```

**所有命令行参数：**
- `--x-min`: X轴最小边界（米）
- `--x-max`: X轴最大边界（米）
- `--y-min`: Y轴最小边界（米）
- `--y-max`: Y轴最大边界（米）
- `--z-min`: Z轴最小边界（米）
- `--z-max`: Z轴最大边界（米）
- `--wall-thickness`: 边界墙厚度（米，默认0.05）
- `--config` / `-c`: 指定配置文件路径
- `--remove` / `-r`: 移除所有边界墙

#### 方法2：YAML配置文件（便于保存和重复使用）

1. 编辑配置文件 `workspace_limits.yaml`：

```yaml
## X轴边界（米）
x_min: -0.5
x_max: 0.5

## Y轴边界（米）
y_min: -0.3
y_max: 0.3

## Z轴边界（米）
z_min: 0.1
z_max: 0.8

## 边界墙厚度（米）
wall_thickness: 0.05

## 启用/禁用边界墙（可选）
enabled_walls:
  left: true      # 左墙（X轴负方向）
  right: true     # 右墙（X轴正方向）
  front: true     # 前墙（Y轴正方向）
  back: true      # 后墙（Y轴负方向）
  top: true       # 顶墙（Z轴正方向）
  bottom: false   # 底墙（Z轴负方向）
```

2. 使用配置文件运行：

```bash
## 方式1：直接运行（自动使用 workspace_limits.yaml）
python3 limit_workspace.py

## 方式2：指定配置文件路径
python3 limit_workspace.py --config workspace_limits.yaml

## 方式3：使用其他配置文件
python3 limit_workspace.py --config my_custom_config.yaml
```

**默认行为**：如果不指定 `--config` 参数，脚本会自动尝试加载同目录下的 `workspace_limits.yaml` 文件。

**控制边界墙显示：**

在YAML配置文件中，可以通过 `enabled_walls` 选项控制哪些边界墙需要启用：

```yaml
## 示例：只启用左右两面墙（限制X轴方向）
enabled_walls:
  left: true
  right: true
  front: false
  back: false
  top: false
  bottom: false

## 示例：只启用前后两面墙（限制Y轴方向）
enabled_walls:
  left: false
  right: false
  front: true
  back: true
  top: false
  bottom: false

## 示例：只启用顶墙（限制Z轴上方）
enabled_walls:
  left: false
  right: false
  front: false
  back: false
  top: true
  bottom: false
```

#### 方法3：修改代码中的默认值

编辑 `limit_workspace.py` 文件中的默认值（不推荐，除非需要永久修改默认值）。

### 移除边界墙

使用命令行参数移除：

```bash
python3 limit_workspace.py --remove
```

或者使用简写：

```bash
python3 limit_workspace.py -r
```

### 工作原理

脚本可以创建最多 6 面边界墙（根据配置启用）：
1. **左墙**：限制 X 轴负方向
2. **右墙**：限制 X 轴正方向
3. **前墙**：限制 Y 轴正方向
4. **后墙**：限制 Y 轴负方向
5. **顶墙**：限制 Z 轴正方向
6. **底墙**：限制 Z 轴负方向（默认禁用）

这些墙作为碰撞对象添加到 MoveIt2 的规划场景中，MoveIt2 在规划路径时会自动避开这些碰撞对象。

**默认配置**：默认启用前5面墙（左、右、前、后、顶），底墙默认禁用。

### 注意事项

1. **坐标系**：脚本使用 "world" 坐标系，确保与你的机器人配置一致
2. **启动顺序**：必须在 MoveIt2 启动后再运行此脚本
3. **墙的厚度**：建议使用 0.05 米，太薄可能影响碰撞检测，太厚会占用过多空间
4. **性能**：添加边界墙后，规划时间可能会略微增加

### 故障排除

#### 边界墙没有显示

1. 检查 MoveIt2 是否正在运行：`ros2 node list | grep move_group`
2. 检查话题是否存在：`ros2 topic list | grep planning_scene`
3. 在 RViz2 中检查 "Scene Geometry" 是否启用

#### 规划仍然超出边界

1. 检查坐标系是否正确（应该是 "world"）
2. 确保边界墙的位置和尺寸正确
3. 尝试增加墙的厚度

#### 脚本运行错误

1. 确保已 source ROS2 环境：`source install/setup.bash`
2. 确保已安装必要的 ROS2 包：`moveit_msgs`, `shape_msgs`, `geometry_msgs`
3. 检查 Python 版本：`python3 --version`（需要 Python 3.6+）

### 使用示例

#### 示例1：使用命令行参数限制小工作空间

```bash
## 限制工作空间为：X:[-0.5, 0.5], Y:[-0.3, 0.3], Z:[0.1, 0.8]
python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8
```

#### 示例2：使用配置文件

```bash
## 1. 编辑 workspace_limits.yaml
## 2. 运行脚本
python3 limit_workspace.py --config workspace_limits.yaml
```

#### 示例3：只修改部分参数

```bash
## 只修改X轴边界，其他使用默认值
python3 limit_workspace.py --x-min -0.6 --x-max 0.6
```

#### 示例4：查看帮助信息

```bash
python3 limit_workspace.py --help
```

#### 示例5：移除边界墙

```bash
python3 limit_workspace.py --remove
```

#### 示例6：使用配置文件控制边界墙显示

创建配置文件 `my_config.yaml`：

```yaml
x_min: -0.6
x_max: 0.6
y_min: -0.4
y_max: 0.4
z_min: 0.0
z_max: 1.0
wall_thickness: 0.05

## 只启用左右两面墙
enabled_walls:
  left: true
  right: true
  front: false
  back: false
  top: false
  bottom: false
```

运行：

```bash
python3 limit_workspace.py --config my_config.yaml
```

### 与其他限制方法对比

| 方法 | 优点 | 缺点 |
|------|------|------|
| 关节限制（joint_limits.yaml） | 简单、全局生效 | 只能限制关节角度 |
| 路径约束（代码中） | 精确控制姿态 | 需要修改代码 |
| **碰撞对象（本脚本）** | **可视化、灵活** | **需要额外脚本** |


### 快速命令摘录（原 scripts/QUICK_START.md）

### 三种配置方式

#### 1. 命令行参数（最方便）

```bash
## 完整配置
python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8

## 只修改部分参数
python3 limit_workspace.py --x-min -0.6 --x-max 0.6

## 查看帮助
python3 limit_workspace.py --help
```

#### 2. YAML配置文件（推荐用于保存配置）

```bash
## 方式1：直接运行（自动使用 workspace_limits.yaml）
python3 limit_workspace.py

## 方式2：指定配置文件路径
python3 limit_workspace.py --config workspace_limits.yaml
```

**默认行为**：脚本会自动使用同目录下的 `workspace_limits.yaml` 配置文件（如果存在）。

**控制边界墙显示**（在YAML配置文件中）：

```yaml
## 只启用左右两面墙
enabled_walls:
  left: true
  right: true
  front: false
  back: false
  top: false
  bottom: false
```

#### 3. 默认值（无需配置）

```bash
python3 limit_workspace.py
```

### 移除边界墙

```bash
python3 limit_workspace.py --remove
```

### 配置优先级

**命令行参数 > 配置文件 > 默认值**

例如：
- 配置文件设置 `x_max: 0.8`
- 命令行参数 `--x-max 0.6`
- 最终结果：`x_max = 0.6`（命令行参数优先）

### 常用示例

```bash
## 小工作空间
python3 limit_workspace.py --x-min -0.5 --x-max 0.5 --y-min -0.3 --y-max 0.3 --z-min 0.1 --z-max 0.8

## 大工作空间
python3 limit_workspace.py --x-min -1.0 --x-max 1.0 --y-min -1.0 --y-max 1.0 --z-min 0.0 --z-max 1.5

## 只限制X和Y轴
python3 limit_workspace.py --x-min -0.6 --x-max 0.6 --y-min -0.6 --y-max 0.6
```

### 控制边界墙显示

在YAML配置文件中，可以控制哪些边界墙需要启用：

```yaml
## 示例1：只启用左右两面墙（限制X轴方向）
enabled_walls:
  left: true
  right: true
  front: false
  back: false
  top: false
  bottom: false

## 示例2：只启用前后两面墙（限制Y轴方向）
enabled_walls:
  left: false
  right: false
  front: true
  back: true
  top: false
  bottom: false

## 示例3：只启用顶墙（限制Z轴上方）
enabled_walls:
  left: false
  right: false
  front: false
  back: false
  top: true
  bottom: false
```


---

## 其他文档（未并入）

- [`doc/TIMEOUT_ROOT_CAUSE.md`](doc/TIMEOUT_ROOT_CAUSE.md)

