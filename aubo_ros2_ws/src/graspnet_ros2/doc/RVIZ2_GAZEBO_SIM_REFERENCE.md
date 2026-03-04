# 机械臂 RViz2 + Gazebo 联合仿真参考

本文档整理 [ros2_moveit2_ur5e_grasp](https://github.com/Nackustb/ros2_moveit2_ur5e_grasp) 及 ROS2 常规做法，说明如何实现 **RViz2 与 Gazebo 联合仿真**。

---

## 1. 参考项目：如何启动 Gazebo 和显示模型

参考项目在 **ur5e_gripper_sim_control.launch.py**（`ur5e_gripper_moveit_config` 包）中实现，流程如下。

### 1.1 启动 Gazebo

用 **gazebo_ros** 官方的 `gazebo.launch.py`，只传 **world**（他们用自定义 `gazebo/sim_env.world`），不传 gui（默认 true，会起 gzserver + gzclient）：

```python
gazebo_world_file = os.path.join(
    get_package_share_directory('ur5e_gripper_moveit_config'), 'gazebo', 'sim_env.world'
)
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
    ),
    launch_arguments={"world": gazebo_world_file}.items(),
)
```

### 1.2 显示模型（Spawn 到 Gazebo）

用 **gazebo_ros** 的 **spawn_entity.py** 作为 Node 启动，从 **topic** 读 URDF（`-topic robot_description`），不写临时文件：

```python
gazebo_spawn_robot = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    name="spawn_ur",
    arguments=["-entity", "ur5e_gripper", "-topic", "robot_description"],
    output="screen",
)
```

### 1.3 启动顺序（保证模型能显示）

1. **robot_state_publisher**：用 xacro 生成 `robot_description` 并发布到 `/robot_description`，且带 **use_sim_time: True**。
2. **Gazebo**：`IncludeLaunchDescription(gazebo.launch.py)` 起空世界（gzserver 会加载 **GazeboRosFactory**，才提供 `/spawn_entity` 服务）。
3. **spawn_entity.py**：订阅 `/robot_description`，调用 `/spawn_entity` 把机器人生成到 Gazebo 里。若 spawn 与 Gazebo 同时启动，可能报错 “Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?”——因 gzserver 尚未就绪；本仓库用 **TimerAction 延迟 5 秒** 再执行 spawn。
4. **controller spawner**：joint_state_broadcaster、ur5e_arm_controller、gripper_controller 等（URDF 里配了 gazebo_ros2_control，controller_manager 由 Gazebo 插件提供）。
5. **RViz2**：通过 `RegisterEventHandler(OnProcessExit(joint_state_broadcaster_spawner, on_exit=[rviz_node]))` 在 joint_state_broadcaster 起来后再启动，保证场景就绪。

因此：**先有 robot_state_publisher 发布 robot_description，再起 Gazebo，再起 spawn_entity（-topic robot_description）**，模型就能在 Gazebo 里显示。

---

## 2. 参考项目结构（simulation.launch 层级）

- **simulation.launch.py**：一次拉起仿真 + MoveIt + 视觉等  
  - 包含 `ur5e_gripper_sim_control.launch.py`（Gazebo + 机器人 + 可选 RViz）
  - 包含 `ur5e_gripper_moveit.launch.py`（MoveIt2，`use_sim_time:=true`）
  - 其余为视觉、OctoMap 等

- **ur5e_gripper_sim_control.launch.py** 要点小结：
  1. Gazebo：`IncludeLaunchDescription(gazebo_ros/launch/gazebo.launch.py)`，只传 `world`。
  2. **robot_state_publisher** 使用 **use_sim_time: True**。
  3. Spawn：**Node(spawn_entity.py, arguments=[..., "-topic", "robot_description"])**。
  4. RViz2 在 joint_state_broadcaster 的 OnProcessExit 后再启。
  5. controller_manager 由 Gazebo 的 gazebo_ros2_control 提供。

---

## 3. 通用要点总结

| 项目         | 说明 |
|--------------|------|
| **Gazebo 启动** | 使用 `IncludeLaunchDescription(gazebo_ros/launch/gazebo.launch.py)`，可传 `world`、`gui:=true/false`。内部拆成 gzserver + gzclient，减少单进程卡死/崩溃。 |
| **use_sim_time** | 联合仿真时：robot_state_publisher、move_group、rviz2 等建议 `use_sim_time:=true`，时间与 Gazebo 的 `/clock` 一致。若仅要 Gazebo 里“看模型”且不动仿真，也可不设。 |
| **Spawn 方式** | 推荐 `-topic robot_description`，由 robot_state_publisher 提供；或 `-file /path/to.urdf`（xacro 生成后写入文件）。 |
| **RViz2**    | 与 Gazebo 同时开：同一 `robot_description` + `/joint_states`，RViz2 用 MotionPlanning 等插件做规划/可视化，Gazebo 做物理/碰撞（若用 gazebo_ros2_control）。 |
| **控制器**   | **真仿真**：URDF 里配 `gazebo_ros2_control`，controller_manager 连 Gazebo；**仅可视化**：本机 ros2_control（mock）发 `/joint_states`，Gazebo 里只 spawn 模型不接控制。 |

---

## 4. 本仓库（graspnet_ros2）当前做法

- **Gazebo**：`use_gazebo:=true` 时通过 **IncludeLaunchDescription(gazebo_ros/launch/gazebo.launch.py)** 启动，与参考一致只传 **world**；**Gazebo 与 spawn 放在节点列表最后**。
- **Spawn**：与参考一致使用 **Node(spawn_entity.py, arguments=[..., "-topic", "robot_description"])**，不再写临时文件；**robot_state_publisher 先于 Gazebo 与 spawn 启动**，保证 `/robot_description` 已有数据。
- **use_sim_time**：启用 Gazebo 时为 **True**（与参考一致），供 robot_state_publisher、move_group、rviz 等与 `/clock` 同步。
- **RViz2**：与 MoveIt、graspnet_demo 一起由同一 launch 启动，共用 `robot_description` 与 `/joint_states`。

---

## 5. 参考链接

- 参考项目：<https://github.com/Nackustb/ros2_moveit2_ur5e_grasp>
- ROS2 Gazebo 与 MoveIt2：<https://robotics.stackexchange.com/questions/104062/how-to-simulate-in-gazebo-moveit2-planning-executing>
- MoveIt2 仿真配置：<https://automaticaddison.com/how-to-configure-moveit-2-for-a-simulated-robot-arm/>
