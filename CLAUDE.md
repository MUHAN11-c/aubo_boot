# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## ROS 2 参数隔离（重要！）

**ROS 2 没有全局参数服务器**（与 ROS 1 `rosparam` 完全不同）。每个节点各自维护独立的参数副本。
**正确做法**：用 `AsyncParametersClient::set_parameters()` 设置其他节点参数。


## 重要注意事项

1. **所有回复必须使用中文，且每句末尾都要加上"喵~"**，无论回答、解释、代码注释还是错误信息，一律遵守此规则喵~

2. **回复要有修改的理论依据最好先阅读源码和所用库的官方源码以及文档，详细方案经过确认后再执行修改代码，每次修改完代码后都需要更新相关的文档喵~**

3. **严格禁止根据经验和注释猜测判断，必须查看所在流程的源码和所用库的源码及官方文档分析，要有十足的理论依据再判断和修改喵~**
   - 不允许凭"我觉得"、"通常来说"、"应该是"等经验推断下结论喵~
   - 不允许仅凭代码注释就断定行为，注释可能过时或误导，必须以实际源码实现为准喵~
   - 分析任何问题时，必须追溯到相关流程的实际源码（本地源码 + 依赖库源码），不可停留在表面日志或报错信息上喵~
   - 修改前必须确认理解了完整调用链：入口 → 中间层 → 底层库/API 的实际行为喵~
   - 对于 ROS 2 / MoveIt / AUBO SDK 等关键依赖，优先查阅官方源码仓库（如 github.com/ros2/、github.com/ros-planning/moveit2）而非二手博客喵~

4. **AUBO SDK 是同步阻塞的**（TCP 通信，单次调用 2-225ms），不能放在 ROS 2 实时控制循环中直接调用。新框架将 SDK 调用隔离在独立线程喵~

5. **RIB 必须在同一条 TCP 连接上读写**：发送轨迹点用 `conn_control_`，RIB 诊断也必须在 `conn_control_` 上查询，不同连接会导致 RIB 值不更新喵~

6. **IO 引脚语义不一致**：`ExecuteGraspPoseWorker`（`true=打开`）与 `PublishGraspsClientWorker`/`ABWorker`（`true=闭合`）的夹爪 IO 语义相反，源于不同工位的电气接线差异。IO 引脚映射见 `aubo_ros2_driver/README.md` 第 3 节喵~

7. **GraspNet Z 轴 180° 翻转**：所有 GraspNet 消费者通过 `applyGraspZFlip180()` 自动修正预测位姿的 Z 轴喵~

8. **MoveIt CurrentStateMonitor 回调竞争**：长耗时服务需放入独立的 callback group，否则会阻塞 `CurrentStateMonitor` 导致 `Failed to fetch current robot state` 喵~

9. **`shared_from_this()` 不能在 Node 构造函数中调用**：ROS 2 的 `enable_shared_from_this` 在构造阶段未就绪，会抛出 `std::bad_weak_ptr`。延迟到回调（如订阅回调、服务回调）中首次使用时再调用喵~

10. **`AsyncParametersClient` 不可在构造函数中创建**：同理，需要 `shared_from_this()`。改为在首次 `onToolStatus()` 回调中按需创建喵~

11. **工具切换前必须清除碰撞模型**：`gripper_swap_worker.changeToTool()` 入口处调用 `publishToolStatus(false)`，让 `scene_attach_worker` 立即脱离 PlanningScene 附着体 + 更新 URDF 为无工具版本，避免切换运动过程中附着的夹爪碰撞体与机械臂自身发生自碰撞导致轨迹规划失败喵~

12. **`tools.yaml` 的 `attach_offset` 与 `aubo_e5.urdf.xacro` 的 `gripper_link` origin 必须严格对齐**：位姿不一致会导致 Web/RViz2 中模型位置与物理安装位置出现偏差喵~

## 调试日志读取

用户反馈问题/报错时，优先通过以下命令收集运行时信息辅助分析，而不是猜测原因喵~

### 0. ROS 2 日志文件（最优先）

```bash
# 最新日志目录（按时间排序，最新的在最下面）
ls -lt ~/.ros/log/

# 读取最新日志目录下所有文件的报错内容
grep -rn "Error\|ERROR\|FATAL\|CRITICAL\|Exception\|Traceback" ~/.ros/log/$(ls -t ~/.ros/log/ | head -1)/

# 查看特定节点的日志
cat ~/.ros/log/$(ls -t ~/.ros/log/ | head -1)/<node_name>*.log

# 查看 launch 文件的完整控制台输出（launch 前缀日志）
cat ~/.ros/log/$(ls -t ~/.ros/log/ | head -1)/launch.log
```

### 1. 系统存活检查

```bash
# 当前运行的节点列表（对照 start_aubo_new_driver.sh 的 16 步确认哪些挂了）
ros2 node list

# 生命周期节点状态（Dashboard 等 LifecycleNode）
ros2 lifecycle list
ros2 lifecycle get /aubo_dashboard
```

### 2. 分组件诊断命令

按 `start_aubo_new_driver.sh` 的启动顺序，各组件的关键检查命令：

| 步骤 | 组件 | 诊断命令 |
|------|------|---------|
| [1/16] | 新框架机械臂 | `ros2 topic echo /joint_states --once`、`ros2 action list`、`ros2 service list \| grep aubo` |
| [3/16] | Percipio 相机 | `ros2 topic hz /camera/color/image_raw`、`ros2 topic echo /camera/color/camera_info --once` |
| [7/16] | VPE 位姿估计 | `ros2 service list \| grep vpe`、检查 FastAPI: `curl -s http://127.0.0.1:8088/health` |
| [8/16] | GraspNet | `ros2 topic hz /graspnet/grasps`、`ros2 topic echo /graspnet/grasps --once` |
| [10/16] | 工具快换 | `ros2 topic echo /tool_changer_status --once`、`ros2 service call /get_current_tool tool_changer_interface/srv/GetCurrentTool` |
| [12/16] | Web Dashboard | `curl -s http://127.0.0.1:8090/api/ivg/health`、`ros2 param get /robot_state_publisher robot_description \| head -c 200` |
| [13/16] | rosbag 录制 | `ros2 topic list \| wc -l`（确认话题数量） |

### 3. rosbag 录制数据

启动脚本会自动录制 rosbag 到 `aubo_ros2_ws/rosbags/ivg_session/`，可用于回溯问题发生时的完整数据流喵~

```bash
# 查看录制数据信息
ros2 bag info aubo_ros2_ws/rosbags/ivg_session/

# 列出录制的话题
ros2 bag info aubo_ros2_ws/rosbags/ivg_session/ | grep -A100 "Topic information"

# 回放特定话题（按时间范围过滤）
ros2 bag play aubo_ros2_ws/rosbags/ivg_session/ --topics /joint_states /tool_changer_status

# 导出话题数据到文本分析
ros2 bag play aubo_ros2_ws/rosbags/ivg_session/ --topics /joint_states 2>&1 | head -50
```

**注意**：脚本在启动 [13/16] 时会 `rm -rf` 旧的 rosbag 目录，所以每次启动后只能看到**当次运行**的录制数据喵~

### 5. 关键话题速查

```bash
# 机械臂状态
ros2 topic echo /joint_states --once          # 关节角度
ros2 topic echo /aubo_robot_status --once     # 机械臂运行状态

# 抓取相关
ros2 topic echo /graspnet/grasps --once       # GraspNet 预测位姿
ros2 topic echo /tool_changer_status --once   # 当前工具状态

# TF 树完整性（检查是否有断链）
ros2 run tf2_tools view_frames                # 生成 frames.pdf
ros2 run tf2_echo <parent> <child>            # 检查特定变换

# ROS 2 图连通性
ros2 topic info /joint_states                 # 查看发布者/订阅者数量
```

### 6. 常见报错速查表

| 报错关键词 | 可能原因 | 优先检查 |
|-----------|---------|---------|
| `Failed to fetch current robot state` | CurrentStateMonitor 回调被阻塞 | `ros2 node info /move_group` 查看 callback group |
| `bad_weak_ptr` | 构造函数中调用了 `shared_from_this()` | 检查节点构造函数代码 |
| `Could not find parameter robot_description` | 参数未设置到该节点 | `ros2 param get <node> robot_description` |
| `Connection refused` / `TCP` | 机械臂不可达或 SDK 连接失败 | `ping 169.254.10.98`、检查网线 |
| `Controller XXX failed to activate` | ros2_control 控制器配置错误 | `ros2 control list_controllers` |
| `RIB` 相关 | TCP 连接混用或队列溢出 | 检查 `conn_control_` 连接一致性 |
| `planning failed` / `No valid plan` | 碰撞检测失败或目标不可达 | `ros2 service call /get_planning_scene` 检查碰撞场景 |
| `transform not found` | TF 树断链 | `ros2 run tf2_tools view_frames` |
