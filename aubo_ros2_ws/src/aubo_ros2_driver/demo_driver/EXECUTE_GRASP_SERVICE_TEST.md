# ExecuteGraspPoseWorker 服务测试指南

## 构建

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select demo_interface demo_driver
source install/setup.bash
```

## 启动节点

```bash
ros2 launch demo_driver execute_grasp_pose_worker.launch.py
```

## 测试服务

### 1. 单次抓取（使用参数常量）

```bash
ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose "{object_id: 'test_object', use_visual_estimation: false}"
```

### 2. 单次抓取（使用视觉估计）

确保 `/estimate_pose` 服务正在运行，然后：

```bash
ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose "{object_id: 'default', use_visual_estimation: true}"
```

### 3. 启动循环抓取

```bash
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: true}"
```

### 4. 停止循环抓取

```bash
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: false}"
```

## 参数说明

节点参数统一使用 **`egp_` 前缀**（与 `main` 中 `automatically_declare_parameters_from_overrides` 配合，避免重复声明）。可通过 launch 或 `ros2 run ... --ros-args -p ...` 覆盖，例如：

- `egp_object_id`: 循环抓取默认工件 ID，默认 `default`
- `egp_grasp_position`: 抓取位置 [x, y, z]
- `egp_grasp_orientation`: 抓取姿态四元数 [qx, qy, qz, qw]
- 其余见 `execute_grasp_pose_worker.cpp` 顶部参数表

## Web 工作流程「自动抓取」

视觉 Web UI（`http_bridge_server` + `app.js`）中 **「自动抓取」** 按钮会调用 **`/api/execute_single_grasp`**，即 `execute_grasp_pose_worker` 的 **`ExecuteGraspPose`**（`use_visual_estimation: true` 时 worker 内会再次请求 `/estimate_pose`，再执行 `runOneCycle()`）。请保证已启动 `execute_grasp_pose_worker` 节点，且桥接请求体可带 **`timeout_sec`**（默认 300，最大 900）。

## 前端集成

前端可以通过 ROS2 桥接（如 rosbridge_suite）调用这些服务：

### JavaScript 示例（使用 roslibjs）

```javascript
// 单次抓取
const executeSingleGraspService = new ROSLIB.Service({
  ros: ros,
  name: '/execute_single_grasp',
  serviceType: 'demo_interface/srv/ExecuteGraspPose'
});

const request = new ROSLIB.ServiceRequest({
  object_id: 'default',
  use_visual_estimation: true
});

executeSingleGraspService.callService(request, function(result) {
  console.log('抓取结果:', result.success, result.message);
  console.log('最终位置:', result.final_position);
});

// 循环抓取控制
const loopGraspControlService = new ROSLIB.Service({
  ros: ros,
  name: '/loop_grasp_control',
  serviceType: 'std_srvs/srv/SetBool'
});

// 启动循环
loopGraspControlService.callService({data: true}, function(result) {
  console.log('循环抓取已启动:', result.success, result.message);
});

// 停止循环
loopGraspControlService.callService({data: false}, function(result) {
  console.log('循环抓取已停止:', result.success, result.message);
});
```

## 日志监控

```bash
ros2 run rqt_console rqt_console
```

关键日志标签：
- `[estimatePoseFromVision]` - 视觉估计调用
- `[handleExecuteSingleGrasp]` - 单次抓取服务处理
- `[handleLoopGraspControl]` - 循环抓取控制
- `[loopGraspThread]` - 循环抓取线程执行

## 故障排查

1. **服务不可用**：
   - 检查节点是否正常启动：`ros2 node list`
   - 检查服务是否注册：`ros2 service list | grep grasp`

2. **视觉估计失败**：
   - 确认 `/estimate_pose` 服务正在运行：`ros2 service list | grep estimate_pose`
   - 检查视觉节点日志

3. **循环抓取无法启动**：
   - 检查是否已有循环在运行（会返回 "循环抓取已在运行中"）
   - 查看节点日志了解详细错误信息
