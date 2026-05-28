# ROS 2 Ecosystem Packages

## MoveIt 2

Official docs: https://moveit.picknik.ai/humble/
Source: https://github.com/moveit/moveit2/tree/humble

### MoveGroupInterface (C++)

The primary API for motion planning.

```cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Setup with MultiThreadedExecutor to prevent CurrentStateMonitor timeout
auto node = std::make_shared<rclcpp::Node>("my_node");
auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
auto move_group = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
auto planning_scene = moveit::planning_interface::PlanningSceneInterface(node);

// Basic configuration
move_group.setMaxVelocityScalingFactor(0.5);      // 50% max vel. Range (0,1]
move_group.setMaxAccelerationScalingFactor(0.5);  // 50% max accel
move_group.setPlanningTime(5.0);                  // seconds
move_group.setNumPlanningAttempts(10);
move_group.setPoseReferenceFrame("base_link");
move_group.setEndEffectorLink("tool_tcp");
move_group.allowReplanning(true);  // may cause trajectory mutation mid-motion
```

### Plan + Execute
```cpp
// Named target (from SRDF)
move_group.setNamedTarget("home");
move_group.move();  // blocking: needs async spinner

// Joint space target
std::vector<double> joints = {0.0, -0.5, 1.2, 0.0, 1.5, 0.0};
move_group.setJointValueTarget(joints);
move_group.move();

// Pose target (IK)
geometry_msgs::msg::Pose target;
target.position.x = 0.5;
target.orientation.w = 1.0;
move_group.setPoseTarget(target);
move_group.move();

// Plan only (no execute)
moveit::planning_interface::MoveGroupInterface::Plan plan;
if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    move_group.execute(plan);
}
```

### Cartesian Path
```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;
// ... populate waypoints ...

moveit_msgs::msg::RobotTrajectory trajectory;
const double jump_threshold = 0.0;   // 0.0 = disable jump detection
const double eef_step = 0.01;        // 1cm between points

double fraction = move_group.computeCartesianPath(
    waypoints, eef_step, jump_threshold, trajectory);

// CRITICAL: fraction is 0.0–1.0, NOT success/fail
if (fraction >= 0.95) {
    move_group.execute(trajectory);
} else if (fraction >= 0.50) {
    // Retry without collision checking
    // Must use lower-level API — MoveGroupInterface doesn't expose this
} else {
    RCLCPP_ERROR(logger, "Cartesian path failed: %.2f fraction", fraction);
}
```

### Collision Objects
```cpp
// Add collision object
moveit_msgs::msg::CollisionObject obj;
obj.id = "table";
obj.header.frame_id = "base_link";
obj.primitives.resize(1);
obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
obj.primitives[0].dimensions = {1.0, 1.0, 0.05};  // x,y,z
obj.primitive_poses.resize(1);
obj.primitive_poses[0].position.z = -0.025;  // half thickness below
obj.operation = obj.ADD;
planning_scene.applyCollisionObject(obj);

// Attach object to robot
moveit_msgs::msg::AttachedCollisionObject aco;
aco.object.id = "gripper";
aco.link_name = "tool_tcp";
aco.touch_links = {"tool_tcp", "wrist3"};  // disable coll checking for these
aco.object.operation = aco.object.ADD;
planning_scene.applyAttachedCollisionObject(aco);
```

### Planning Scene ROS API
```cpp
// Monitor current scene via topic
// Sub to /monitored_planning_scene for async updates

// Service: get full scene
// ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene
```

### SRDF (Semantic Robot Description Format)
```xml
<!-- Disable collision checking between link pairs -->
<disable_collisions link1="link_a" link2="link_b" reason="Adjacent" />
<!-- Always adjacent links are disabled by default -->

<!-- Define groups -->
<group name="manipulator">
    <chain base_link="base_link" tip_link="tool_tcp" />
</group>

<!-- Define end effectors (for grasp generation) -->
<end_effector name="gripper" parent_link="tool_tcp" group="gripper" />
```

### CurrentStateMonitor timeout
```
Symptom: "Failed to fetch current robot state" after 1s

Root cause: SingleThreadedExecutor — MoveGroupInterface callbacks
(jointStateCallback) are blocked by long-running operations.

Fix: Use MultiThreadedExecutor (4+ threads) + ReentrantCallbackGroup
for the node hosting MoveGroupInterface.
```
Reference: https://github.com/moveit/moveit2/issues/2645

### Key Pitfalls
- `move()` is **blocking** — requires async spinner (MultiThreadedExecutor)
- `computeCartesianPath()` returns **fraction**, not bool
- `jump_threshold = 0.0` disables jump detection (use for cartesian)
- `allowReplanning(true)` can mutate trajectory mid-motion
- `setMaxVelocityScalingFactor(0.0)` is invalid — range is (0, 1]

---

## ros2_control

Official docs: https://control.ros.org/humble/
Source: https://github.com/ros-controls/ros2_control

### Architecture
```
┌──────────────┐     ┌─────────────────────┐     ┌──────────────────┐
│ Controller   │ ←→  │ Controller Manager  │ ←→  │ Hardware         │
│ (ros2_control│     │ (manages lifecycle, │     │ Interface        │
│  interface)  │     │  resource claims)   │     │ (talks to robot) │
└──────────────┘     └─────────────────────┘     └──────────────────┘
```

### URDF Hardware Tags
```xml
<ros2_control name="MyRobot" type="system">
    <hardware>
        <plugin>my_hardware/MyHardwareInterface</plugin>
    </hardware>
    <joint name="joint1">
        <command_interface name="position" />
        <command_interface name="velocity" />
        <state_interface name="position" />
        <state_interface name="velocity" />
    </joint>
</ros2_control>
```

### Controller YAML configuration
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
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    joints:
      - joint1
      - joint2
      # ...
```

### CLI Commands
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control load_controller joint_trajectory_controller
ros2 control configure_controller joint_trajectory_controller
ros2 control switch_controllers --activate jtc --deactivate jsb
ros2 control set_controller_state jtc configure
```

### Hardware Interface Types
| Type | Use Case |
|------|----------|
| `system` | Single-component robot (most common) |
| `sensor` | Read-only sensor device |
| `actuator` | Single actuator (per-joint) |

### Command Interface Types
- `position` — in radians or meters
- `velocity` — in rad/s or m/s
- `effort` — in Nm or N
- `acceleration` — in rad/s² or m/s²

---

## Navigation 2 (Nav2)

Official docs: https://docs.nav2.org/
Source: https://github.com/ros-navigation/navigation2

### Core Components
```
Map Server  →  AMCL (localization)  →  Planner Server  →  Controller Server
   (static    (particle filter,        (global path,      (local path,
    map)       TF→/map→/odom)           A*/Smac/etc.)      DWB/MPPI/etc.)
                                          │
                                    Behavior Tree
                                    (recovery, etc.)
```

### Behavior Tree
```xml
<root>
  <BehaviorTree ID="Navigate">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="1" name="ComputePathToPose">
          <ComputePathToPose goal="{goal}" path="{path}" />
          <ClearEntireCostmap name="ClearGlobalCostmap" />
        </RecoveryNode>
      </RateController>
      <RecoveryNode number_of_retries="1" name="FollowPath">
        <FollowPath path="{path}" />
        <ClearEntireCostmap name="ClearLocalCostmap" />
      </RecoveryNode>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

### Planners
| Planner | Best For |
|---------|----------|
| NavFn | Classic global planner |
| Smac Planner Hybrid-A* | Ackermann/non-holonomic |
| Smac Planner 2D | Holonomic, fast |
| Theta* | Line-of-sight optimal |

### Controllers
| Controller | Best For |
|-----------|----------|
| DWB | Differential drive, flexible |
| MPPI | Model-predictive, dynamic obstacles |
| Regulated Pure Pursuit | Ackermann, smooth |
| Rotation Shim | Adds rotate-in-place before DWB |

### Launch Example
```python
Node(package="nav2_map_server", executable="map_server",
     parameters=["map.yaml"]),
Node(package="nav2_amcl", executable="amcl",
     parameters=["amcl_params.yaml"]),
Node(package="nav2_planner", executable="planner_server",
     parameters=["planner_params.yaml"]),
Node(package="nav2_controller", executable="controller_server",
     parameters=["controller_params.yaml"]),
Node(package="nav2_bt_navigator", executable="bt_navigator",
     parameters=["bt_params.yaml"]),
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        FindPackageShare("nav2_bringup"), "/launch/navigation_launch.py"]))
```

### nav2 Simple Commander (Python API)
```python
from nav2_simple_commander.robot_navigator import BasicNavigator

nav = BasicNavigator()
nav.setInitialPose(initial_pose)
nav.goToPose(goal_pose)

while not nav.isTaskComplete():
    feedback = nav.getFeedback()
    # ...
result = nav.getResult()
```

---

## Gazebo (Ignition)

### Spawn robot from URDF
```python
from launch_ros.actions import Node

Node(package="gazebo_ros", executable="spawn_entity.py",
     arguments=["-entity", "my_robot", "-topic", "robot_description",
                "-x", "0.0", "-y", "0.0", "-z", "0.1"])
```

### ros2_control + Gazebo bridge
```xml
<!-- In URDF: -->
<gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
        <ros>
            <namespace>/my_robot</namespace>
            <argument>~/robot_description</argument>
        </ros>
    </plugin>
</gazebo>
```

### Common Gazebo plugins
```xml
<!-- Differential drive -->
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
</plugin>

<!-- Camera sensor -->
<plugin name="camera" filename="libgazebo_ros_camera.so">
    <image_topic_name>/camera/image_raw</image_topic_name>
</plugin>

<!-- IMU sensor -->
<plugin name="imu" filename="libgazebo_ros_imu_sensor.so">
    <topic_name>/imu</topic_name>
</plugin>
```

---

## rviz2

### Launch with config
```python
Node(package="rviz2", executable="rviz2",
     arguments=["-d", PathJoinSubstitution([
         FindPackageShare("pkg"), "rviz", "config.rviz"])])
```

### Common Display Types
| Display | Topic / Source |
|---------|---------------|
| RobotModel | `/robot_description` parameter |
| TF | `/tf`, `/tf_static` |
| MarkerArray | `/visualization_marker_array` |
| PointCloud2 | `/points`, `/depth/points` |
| Image | `/camera/image_raw` |
| LaserScan | `/scan` |
| Path | `/plan` |
| Odometry | `/odom` |
| Map | `/map` |

---

## rosbag2

```bash
ros2 bag record /topic1 /topic2 -o my_bag
ros2 bag play my_bag
ros2 bag info my_bag
ros2 bag record -a                    # record all topics
ros2 bag record -e ".*image.*"        # regex filter
ros2 bag play my_bag --rate 0.5       # half speed
ros2 bag play my_bag --start-offset 10.0  # skip first 10s
```

### C++ Recorder API
```cpp
#include <rosbag2_cpp/writer.hpp>
// See rosbag2_cpp API for programmatic recording
```
