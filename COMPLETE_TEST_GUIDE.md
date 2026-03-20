# 服务驱动抓取系统 - 完整测试指南

## 系统架构

该系统已从原来的前端直接控制机械臂的方式，升级为通过 ROS2 服务触发的架构：

```
前端 (app.js)
    ↓ HTTP API
Python 桥接服务器 (http_bridge_server.py)
    ↓ ROS2 服务调用
execute_grasp_pose_worker (C++ 节点)
    ↓ 直接控制
机械臂 + MoveIt2
```

## 修改的文件

### 后端（C++）
1. `/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_interface/srv/ExecuteGraspPose.srv` - **新建**
2. `/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_interface/CMakeLists.txt` - 添加新服务
3. `/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/include/demo_driver/execute_grasp_pose_worker.h` - 添加服务支持
4. `/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/src/execute_grasp_pose_worker.cpp` - 实现服务回调
5. `/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/CMakeLists.txt` - 添加依赖
6. `/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/demo_driver/package.xml` - 添加依赖

### 后端（Python）
1. `/home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/scripts/http_bridge_server.py` - 添加 API 端点和 ROS2 服务客户端

### 前端
1. `/home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/scripts/app.js` - 添加服务调用函数

## 构建步骤

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws

# 1. Source ROS2 环境
source /opt/ros/humble/setup.bash

# 2. 构建 demo_interface（新服务定义）
colcon build --packages-select demo_interface
source install/setup.bash

# 3. 构建 demo_driver（C++ 抓取节点）
colcon build --packages-select demo_driver
source install/setup.bash
```

## 启动测试

### 终端 1: 启动机械臂相关节点
```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source install/setup.bash

# 启动机械臂和 MoveIt（假设你已有launch文件）
ros2 launch [你的launch文件]
```

### 终端 2: 启动 execute_grasp_pose_worker 节点
```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source install/setup.bash

# 启动抓取服务节点
ros2 launch demo_driver execute_grasp_pose_worker.launch.py
```

你应该看到类似输出：
```
[INFO] [执行抓取位姿 Worker]: ExecuteGraspPoseWorker 初始化完成
[INFO] [执行抓取位姿 Worker]: 服务已创建: /execute_single_grasp, /loop_grasp_control
[INFO] [执行抓取位姿 Worker]: ExecuteGraspPoseWorker 等待服务调用...
```

### 终端 3: 启动视觉估计节点（如果需要）
```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source install/setup.bash

# 启动视觉估计服务
ros2 run visual_pose_estimation_python main
```

### 终端 4: 启动 HTTP 桥接服务器
```bash
cd /home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/scripts
source /home/mu/IVG2.0/aubo_ros2_ws/install/setup.bash

# 启动 Web 服务器
python3 http_bridge_server.py
```

你应该看到：
```
🎯 视觉姿态估计算法Web UI服务器启动成功!
🌐 服务地址: http://localhost:8088/
📱 Web界面: http://localhost:8088/index.html
```

### 终端 5: 打开浏览器访问前端
```bash
# 在浏览器中打开
http://localhost:8088/index.html
```

## 测试步骤

### 方式 1: 通过前端 UI 测试

1. **打开浏览器**: 访问 `http://localhost:8088/index.html`

2. **测试单次抓取（使用视觉估计）**:
   - 在工作流程界面中选择工件 ID
   - 点击"采集图像"
   - 点击"姿态估计"
   - 点击"自动抓取"
   - **注意**: 前端现在会调用 `/api/execute_single_grasp` API

3. **测试循环抓取**:
   - 选择工件 ID
   - 点击"循环抓取"按钮
   - 系统会自动循环：采集图像 → 估计 → 抓取
   - 再次点击按钮停止循环

### 方式 2: 通过命令行测试 ROS2 服务

#### 测试单次抓取（使用参数常量）
```bash
ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose "{object_id: 'test', use_visual_estimation: false}"
```

#### 测试单次抓取（使用视觉估计）
```bash
ros2 service call /execute_single_grasp demo_interface/srv/ExecuteGraspPose "{object_id: 'default', use_visual_estimation: true}"
```

#### 启动循环抓取
```bash
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: true}"
```

#### 停止循环抓取
```bash
ros2 service call /loop_grasp_control std_srvs/srv/SetBool "{data: false}"
```

### 方式 3: 通过 HTTP API 测试

#### 测试单次抓取
```bash
curl -X POST http://localhost:8088/api/execute_single_grasp \
  -H "Content-Type: application/json" \
  -d '{"object_id": "default", "use_visual_estimation": true}'
```

#### 启动循环抓取
```bash
curl -X POST http://localhost:8088/api/loop_grasp_control \
  -H "Content-Type: application/json" \
  -d '{"data": true}'
```

#### 停止循环抓取
```bash
curl -X POST http://localhost:8088/api/loop_grasp_control \
  -H "Content-Type: application/json" \
  -d '{"data": false}'
```

## 日志监控

### 监控 execute_grasp_pose_worker 节点日志
```bash
ros2 topic echo /rosout | grep execute_grasp_pose_worker
```

或使用 rqt_console:
```bash
ros2 run rqt_console rqt_console
```

关键日志标签：
- `[estimatePoseFromVision]` - 视觉估计调用
- `[handleExecuteSingleGrasp]` - 单次抓取服务处理
- `[handleLoopGraspControl]` - 循环抓取控制
- `[loopGraspThread]` - 循环抓取线程执行
- `[runOneCycle]` - 抓取周期执行
- `[runGraspApproach]` - 笛卡尔路径规划

## 故障排查

### 1. 服务不可用
**问题**: 调用服务时返回 "服务不可用"

**检查**:
```bash
# 检查节点是否运行
ros2 node list | grep execute_grasp_pose_worker

# 检查服务是否注册
ros2 service list | grep grasp

# 应该看到:
# /execute_single_grasp
# /loop_grasp_control
```

**解决**: 确保 `execute_grasp_pose_worker_node` 正在运行

### 2. 视觉估计失败
**问题**: `use_visual_estimation=true` 时失败

**检查**:
```bash
# 检查视觉估计服务
ros2 service list | grep estimate_pose
```

**解决**: 确保 `visual_pose_estimation` 节点正在运行

### 3. HTTP API 调用失败
**问题**: 前端无法调用 API

**检查**:
- 确保 `http_bridge_server.py` 正在运行（端口 8088）
- 检查浏览器控制台的错误信息
- 检查 CORS 设置

### 4. 编译错误
**问题**: `colcon build` 失败

**常见原因**:
- 未 source ROS2 环境：`source /opt/ros/humble/setup.bash`
- 未 source 工作空间：`source install/setup.bash`
- 缺少依赖包：检查 `package.xml`

## 新旧对比

### 旧架构（前端直接控制）
```
前端 → HTTP API → Python → 多个 ROS2 服务调用（move_to_pose, set_io等）→ 机械臂
```
**缺点**: 
- 前端逻辑复杂
- 前端需要管理所有抓取步骤
- 难以在纯 ROS2 环境中复用

### 新架构（服务驱动）
```
前端 → HTTP API → Python → /execute_single_grasp → execute_grasp_pose_worker → 机械臂
```
**优点**:
- 前端简化为触发器
- 抓取逻辑封装在 C++ 节点中
- 可以直接用 `ros2 service call` 测试
- 便于集成到其他 ROS2 系统

## 参数配置

可以通过 launch 文件或命令行参数覆盖：

```bash
ros2 launch demo_driver execute_grasp_pose_worker.launch.py \
  object_id:=my_workpiece \
  grasp_z_offset:=0.02 \
  height_above:=0.15
```

主要参数：
- `object_id`: 工件ID（默认: "default"）
- `grasp_position`: 抓取位置 [x, y, z]
- `grasp_orientation`: 抓取姿态四元数 [qx, qy, qz, qw]
- `grasp_z_offset`: 夹爪补偿 (m，默认: 0.01)
- `height_above`: 安全高度 (m，默认: 0.1)
- `lift_offset`: 抬起高度 (m，默认: 0.2)

## 下一步开发

如果需要进一步扩展功能：

1. **添加状态反馈**: 实时发布抓取进度到 ROS2 topic
2. **添加错误恢复**: 抓取失败后的自动恢复策略
3. **添加碰撞检测**: 集成 MoveIt 的碰撞检测
4. **添加力控**: 使用力矩传感器进行力控抓取
5. **添加多工件支持**: 同时管理多个工件的抓取任务

## 总结

✅ 后端 C++ 服务节点已实现
✅ Python HTTP 桥接服务器已添加 API 端点  
✅ 前端已添加服务调用函数
✅ 编译成功（无错误）
✅ 架构从前端控制转为服务驱动

现在可以开始测试了！
