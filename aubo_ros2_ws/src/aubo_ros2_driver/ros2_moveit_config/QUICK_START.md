# 快速开始指南

## 一键启动脚本

创建以下脚本可以简化启动过程：

### start_ros1.sh (ROS 1 端)

```bash
#!/bin/bash
# 启动 ROS 1 端所有组件

source /opt/ros/noetic/setup.bash
source /home/mu/IVG/aubo_ws/devel/setup.bash

# 启动 roscore
gnome-terminal -- bash -c "roscore; exec bash" &

sleep 2

# 启动 MoveIt 和机器人驱动
ROBOT_IP=${1:-127.0.0.1}
gnome-terminal -- bash -c "roslaunch aubo_e5_moveit_config moveit_planning_execution.launch robot_ip:=$ROBOT_IP; exec bash" &

sleep 3

# 启动 demo_driver 服务
gnome-terminal -- bash -c "rosrun demo_driver execute_trajectory_server_node; exec bash" &
gnome-terminal -- bash -c "rosrun demo_driver robot_status_publisher_node; exec bash" &

echo "ROS 1 端已启动"
echo "请等待所有节点启动完成后再启动 ROS 2 端"
```

### start_ros2.sh (ROS 2 端)

```bash
#!/bin/bash
# 启动 ROS 2 端所有组件

source /opt/ros/foxy/setup.bash
source /home/mu/ros2_ws/install/setup.bash

ROBOT_IP=${1:-127.0.0.1}

ros2 launch ros2_moveit_config moveit_bridge_control.launch.py robot_ip:=$ROBOT_IP
```

## 使用步骤

1. **终端 1 - 启动 ROS 1 端**：
   ```bash
   chmod +x start_ros1.sh
   ./start_ros1.sh <机器人IP>
   ```

2. **终端 2 - 启动 ROS 2 端**：
   ```bash
   chmod +x start_ros2.sh
   ./start_ros2.sh <机器人IP>
   ```

3. **在 RViz2 中操作**：
   - 等待 RViz2 启动完成
   - 在 Motion Planning 插件中设置目标位姿
   - 点击 "Plan" 规划路径
   - 点击 "Execute" 执行轨迹

## 验证桥接

```bash
# 检查服务是否可访问
ros2 service list | grep execute_trajectory

# 检查话题是否桥接
ros2 topic list | grep joint_states
ros2 topic echo /joint_states

# 检查状态话题
ros2 topic echo /robot_status
```

