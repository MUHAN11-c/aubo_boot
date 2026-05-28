---
name: ros2-development
description: Comprehensive ROS 2 (Humble) development reference — topics, services, parameters, actions, plugins, launch, lifecycle nodes, executors, QoS, tf2. Covers ecosystem packages MoveIt 2, ros2_control, nav2, and gazebo. Use when writing, reviewing, or debugging ROS 2 C++/Python code, designing node architecture, fixing build issues, or answering "how do I..." questions about any ROS 2 concept.
---

# ROS 2 Development

Comprehensive ROS 2 Humble development reference. All concepts covered
in both **C++ and Python**, following the progressive structure of
[fishros/ros2bookcode](https://github.com/fishros/ros2bookcode).

Official docs: https://docs.ros.org/en/humble/

## Learning Path (beginner → advanced)

| Level | Topic | Code Example |
|-------|-------|-------------|
| 1 | Hello World node | [EXAMPLES.md#1-hello-world](EXAMPLES.md) |
| 2 | OOP node + modern C++ | [EXAMPLES.md#2-oop-node](EXAMPLES.md) |
| 3 | Topic pub/sub | [EXAMPLES.md#3-topic](EXAMPLES.md) |
| 4 | Custom interfaces (.msg/.srv/.action) | [EXAMPLES.md#4-interfaces](EXAMPLES.md) |
| 5 | Service server/client | [EXAMPLES.md#5-service](EXAMPLES.md) |
| 6 | Parameters + launch files | [EXAMPLES.md#6-parameters](EXAMPLES.md) |
| 7 | TF2 broadcaster/listener | [EXAMPLES.md#7-tf2](EXAMPLES.md) |
| 8 | Action server/client | [EXAMPLES.md#8-action](EXAMPLES.md) |
| 9 | Lifecycle nodes | [EXAMPLES.md#9-lifecycle](EXAMPLES.md) |
| 10 | Advanced (QoS/executor/compose/intra-process) | [EXAMPLES.md#10-advanced](EXAMPLES.md) |

## Workflow

On every ROS 2 request:

1. **Task type**: [NEW] write code / [REVIEW] check code / [DEBUG] diagnose
   issue / [LEARN] explain concept
2. **User level**: beginner → progressive examples + explanations. experienced
   → patterns + production code. unclear → ask briefly.
3. **Load reference**: step-by-step tutorial → [EXAMPLES.md](EXAMPLES.md) |
   core API → [REFERENCE-core.md](REFERENCE-core.md) |
   MoveIt/control/nav2 → [REFERENCE-ecosystem.md](REFERENCE-ecosystem.md) |
   patterns/debugging → [REFERENCE-patterns.md](REFERENCE-patterns.md) |
   system design/architecture → [REFERENCE-architecture.md](REFERENCE-architecture.md)

## Quick Reference

### Topics
```cpp
auto pub = node->create_publisher<Msg>("topic", 10);
auto sub = node->create_subscription<Msg>("topic", 10, callback);
```
```python
self.pub = self.create_publisher(Msg, "topic", 10)
self.sub = self.create_subscription(Msg, "topic", callback, 10)
```

### Services — always async
```cpp
auto client = node->create_client<Srv>("service");
auto future = client->async_send_request(req);
// NEVER future.wait_for() inside a MutuallyExclusive callback → deadlock
```

### Parameters
```cpp
if (!node->has_parameter("p")) node->declare_parameter("p", 0);
node->get_parameter("p", val);
node->set_parameter(rclcpp::Parameter("p", 42));
```

### Actions
```cpp
auto server = rclcpp_action::create_server<Act>(
  node, "action", goal_cb, cancel_cb, accept_cb);
auto goal_future = client->async_send_goal(goal, options);
```

### Launch
```python
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(package="pkg", executable="node", parameters=[{"p": 42}])])
```

## Key Rules

1. **Never sync-wait** inside MutuallyExclusive callback → deadlock.
   See [REFERENCE-core.md#callback-groups](REFERENCE-core.md).
2. **Never `shared_from_this()`** in constructor → `bad_weak_ptr`.
   See [REFERENCE-patterns.md](REFERENCE-patterns.md).
3. **LifecycleNode services** only work in Active state.
4. **`robot_state_publisher`** uses `/parameter_events`, not `/robot_description`
   topic. Use `set_parameters()` not `publish()`.
5. **`computeCartesianPath` returns fraction** (0–1), not bool.

## Reference Files

- [EXAMPLES.md](EXAMPLES.md) — progressive tutorials, C++ & Python side-by-side
- [REFERENCE-core.md](REFERENCE-core.md) — topics, services, params, actions,
  plugins, launch, lifecycle, executors, callback groups, QoS, tf2
- [REFERENCE-ecosystem.md](REFERENCE-ecosystem.md) — MoveIt 2, ros2_control,
  nav2, Gazebo, rviz2, rosbag2
- [REFERENCE-patterns.md](REFERENCE-patterns.md) — design patterns,
  anti-patterns, debugging workflow, common errors, package templates
- [REFERENCE-architecture.md](REFERENCE-architecture.md) — software architecture
  design patterns from Autoware, Nav2, micro-ROS, ros2_control, MoveIt2 et al.

## External Resources

- ROS 2 Humble docs: https://docs.ros.org/en/humble/
- ROS 2 design docs: https://design.ros2.org/
- MoveIt 2 docs: https://moveit.picknik.ai/humble/
- ros2_control docs: https://control.ros.org/humble/
- nav2 docs: https://docs.nav2.org/
- Progressive examples (C++ & Python): https://github.com/fishros/ros2bookcode
- rclcpp source: https://github.com/ros2/rclcpp/tree/humble
- MoveIt 2 source: https://github.com/moveit/moveit2/tree/humble
