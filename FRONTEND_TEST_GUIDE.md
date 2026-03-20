# 前端测试指南 - Execute Grasp Pose Worker

## 🚀 启动系统

### 1. 运行启动脚本（已更新）
```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
./start_IVG.sh
```

启动脚本现在会启动 **10 个节点**（已添加 execute_grasp_pose_worker）：
1. ROS2 MoveIt (Pure ROS2)
2. [已注释] MoveIt Workspace Limit
3. Robot Driver
4. Percipio Camera
5. Camera Control
6. Image Data Bridge
7. Hand Eye Calibration
8. Visual Pose Estimation
9. **Execute Grasp Worker** ⭐ 新增
10. HTTP Bridge Server

### 2. 等待启动完成
看到以下提示后，系统就绪：
```
✓ 所有节点已启动完成！
手眼标定Web界面地址: http://localhost:8080
视觉姿态估计Web界面地址: http://localhost:8088/index.html
```

### 3. 验证服务可用
打开新终端验证：
```bash
source /home/mu/IVG2.0/aubo_ros2_ws/install/setup.bash

# 检查服务是否注册
ros2 service list | grep grasp

# 应该看到：
# /execute_single_grasp
# /loop_grasp_control
```

---

## 🌐 前端测试步骤

### 步骤 1: 打开前端界面

在浏览器中打开：
```
http://localhost:8088/index.html
```

### 步骤 2: 界面布局

前端应该有以下功能区：
- **工作流程区域**: 
  - 工件 ID 选择
  - 采集图像按钮
  - 姿态估计按钮
  - **自动抓取按钮** ⭐ 这个会调用我们的服务
  - **循环抓取按钮** ⭐ 这个会控制循环

### 步骤 3: 测试单次抓取（不使用视觉估计）

**目的**: 测试使用参数常量的抓取

**步骤**:
1. 在前端界面找到工件 ID 输入框
2. 输入 `test` 作为工件 ID
3. **暂时跳过**"采集图像"和"姿态估计"
4. 直接点击"自动抓取"按钮

**预期行为**:
- 前端调用 `/api/execute_single_grasp` API
- Python 服务器转发到 ROS2 服务
- Execute Grasp Worker 使用参数常量执行抓取
- 前端显示成功消息

**观察日志**:
在 terminator 的 "Execute Grasp Worker" 标签页中，你会看到：
```
╔════════════════════════════════════════════════════════════╗
║        [单次抓取服务] 收到新的抓取请求                    ║
╚════════════════════════════════════════════════════════════╝
[handleExecuteSingleGrasp] 请求参数：
[handleExecuteSingleGrasp]   object_id: test
[handleExecuteSingleGrasp]   use_visual_estimation: false（使用参数常量）
...
✓ 抓取周期成功完成 (8/8 步骤)
```

### 步骤 4: 测试单次抓取（使用视觉估计）

**目的**: 测试完整的视觉估计+抓取流程

**步骤**:
1. 确保工件在相机视野内
2. 选择正确的工件 ID（例如 "default"）
3. 点击"采集图像"按钮
4. 点击"姿态估计"按钮
5. 点击"自动抓取"按钮

**预期行为**:
- 前端首先采集图像
- 然后调用姿态估计
- 最后调用 `/api/execute_single_grasp` (use_visual_estimation=true)
- Execute Grasp Worker 使用视觉估计结果执行抓取

**观察日志**:
```
========================================
[estimatePoseFromVision] 开始调用视觉位姿估计服务
========================================
...
[estimatePoseFromVision] ✓ 视觉估计成功，已更新抓取位姿
...
✓ 抓取周期成功完成 (8/8 步骤)
```

### 步骤 5: 测试循环抓取

**目的**: 测试连续自动抓取功能

**步骤**:
1. 选择工件 ID
2. 点击"循环抓取"按钮（按钮文字应该变成"停止循环"）
3. 观察系统自动循环执行：采集图像 → 估计 → 抓取
4. 再次点击"停止循环"按钮停止

**预期行为**:
- 点击后立即启动循环线程
- 系统自动重复：视觉估计 → 抓取
- 每次循环之间等待 1 秒
- 点击停止后，当前循环完成后退出

**观察日志**:
```
╔════════════════════════════════════════════════════════════╗
║           [循环抓取线程] 已启动                           ║
╚════════════════════════════════════════════════════════════╝
...
╔════════════════════════════════════════════════════════════╗
║           循环抓取 - 第   1 次循环                        ║
╚════════════════════════════════════════════════════════════╝
...
╔════════════════════════════════════════════════════════════╗
║      [循环抓取线程] 已退出 (共完成  12 次循环)           ║
╚════════════════════════════════════════════════════════════╝
```

---

## 🔍 监控和调试

### 查看实时日志

**方式 1: Terminator 标签页**
- 直接在 terminator 的 "Execute Grasp Worker" 标签页查看
- 这是最直观的方式

**方式 2: rqt_console**
打开新终端：
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_console rqt_console
```
过滤 "execute_grasp_pose_worker" 查看日志

**方式 3: 命令行实时查看**
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /rosout | grep execute_grasp_pose_worker
```

### 前端控制台

按 `F12` 打开浏览器开发者工具，查看：
- **Console**: JavaScript 日志和错误
- **Network**: API 调用和响应
- 搜索 "execute_single_grasp" 或 "loop_grasp_control"

---

## ⚠️ 常见问题

### 问题 1: 前端点击"自动抓取"无响应

**检查**:
1. 浏览器控制台是否有错误
2. 检查服务是否启动：
   ```bash
   ros2 service list | grep grasp
   ```
3. 检查 HTTP 服务器日志（terminator 的 "HTTP Bridge Server" 标签）

**解决**: 
- 确保所有节点都已启动
- 重启 HTTP 桥接服务器

### 问题 2: 视觉估计失败

**症状**: 日志显示 "视觉估计未检测到目标"

**检查**:
1. 工件是否在相机视野内
2. 光照是否充足
3. 模板是否已正确设置

### 问题 3: 抓取路径规划失败

**症状**: 日志显示 "第X段失败"

**检查**:
1. 关节限位
2. 目标位置是否可达
3. 速度/加速度参数是否过大

### 问题 4: 循环无法启动

**症状**: 点击循环抓取按钮无效

**检查**:
1. 是否已经有循环在运行
2. 查看前端响应消息
3. 查看服务日志

---

## 📊 测试检查清单

使用此清单验证功能：

- [ ] 启动脚本成功启动所有 10 个节点
- [ ] 前端界面可以访问 (http://localhost:8088)
- [ ] 服务已注册 (`ros2 service list | grep grasp`)
- [ ] 单次抓取（参数常量）可以执行
- [ ] 单次抓取（视觉估计）可以执行
- [ ] 循环抓取可以启动
- [ ] 循环抓取可以停止
- [ ] 日志清晰显示每个步骤
- [ ] 前端能正确显示结果

---

## 🎯 快速测试流程

最简单的测试流程（不需要相机）：

1. **启动系统**: `./start_IVG.sh`
2. **等待 30 秒**让所有节点启动
3. **打开浏览器**: http://localhost:8088
4. **点击"自动抓取"**（跳过采集和估计）
5. **观察日志**在 terminator 的 "Execute Grasp Worker" 标签
6. **看到成功**: "✓ 抓取周期成功完成 (8/8 步骤)"

---

## 💡 高级功能

### 通过浏览器控制台直接调用

打开浏览器控制台（F12），可以直接调用服务：

**单次抓取**:
```javascript
fetch('http://localhost:8088/api/execute_single_grasp', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({
    object_id: 'test',
    use_visual_estimation: false
  })
}).then(r => r.json()).then(console.log)
```

**启动循环**:
```javascript
fetch('http://localhost:8088/api/loop_grasp_control', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({data: true})
}).then(r => r.json()).then(console.log)
```

**停止循环**:
```javascript
fetch('http://localhost:8088/api/loop_grasp_control', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({data: false})
}).then(r => r.json()).then(console.log)
```

---

## 📞 需要帮助？

如果遇到问题：
1. 检查所有 terminator 标签页的日志
2. 查看 `/home/mu/IVG2.0/DETAILED_LOGGING_GUIDE.md`
3. 运行 `/home/mu/IVG2.0/test_logs.sh` 进行诊断测试

享受测试！🎉
