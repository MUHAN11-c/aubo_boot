# Execute Grasp Pose Worker - 详细日志说明

## 日志级别说明

系统使用 ROS2 标准日志级别：
- **INFO** (绿色): 正常操作信息，进度更新
- **WARN** (黄色): 警告信息，非致命错误
- **ERROR** (红色): 错误信息，导致操作失败

## 日志输出结构

### 1. 节点启动日志

```
[INFO] [execute_grasp_pose_worker]: ExecuteGraspPoseWorker 初始化完成：
    抓取位置=[0.412, 0.184, 0.255], Z轴旋转=1.5708 rad (90.00°)
[INFO] [execute_grasp_pose_worker]: 服务已创建: /execute_single_grasp, /loop_grasp_control
[INFO] [execute_grasp_pose_worker]: ExecuteGraspPoseWorker 等待服务调用...
```

**说明**: 节点成功初始化，显示默认参数，等待服务调用

---

### 2. 单次抓取服务日志

#### 2.1 服务请求开始
```
╔════════════════════════════════════════════════════════════╗
║        [单次抓取服务] 收到新的抓取请求                    ║
╚════════════════════════════════════════════════════════════╝
[handleExecuteSingleGrasp] 请求参数：
[handleExecuteSingleGrasp]   object_id: default
[handleExecuteSingleGrasp]   use_visual_estimation: true（使用视觉估计）
```

#### 2.2 视觉估计阶段（如果启用）
```
========================================
[estimatePoseFromVision] 开始调用视觉位姿估计服务
[estimatePoseFromVision]   object_id: default
========================================
[estimatePoseFromVision] 等待服务 /estimate_pose 可用...
[estimatePoseFromVision] ✓ 服务 /estimate_pose 已连接
[estimatePoseFromVision] 发送服务请求...
[estimatePoseFromVision] 等待服务响应（最多30秒）...
[estimatePoseFromVision] ✓ 收到服务响应
[estimatePoseFromVision] 响应分析：
[estimatePoseFromVision]   success_num: 1
[estimatePoseFromVision]   grab_position 数量: 1
[estimatePoseFromVision] 检测到目标，提取第1个抓取位姿：
[estimatePoseFromVision]   原始位置: (0.412000, 0.184000, 0.255000)
[estimatePoseFromVision]   原始四元数: (qx=0.707107, qy=0.707107, qz=0.000000, qw=0.000000)
[estimatePoseFromVision] ✓ 视觉估计成功，已更新抓取位姿：
[estimatePoseFromVision]   位置: [0.412, 0.184, 0.255] m
[estimatePoseFromVision]   Z轴旋转: 1.5708 rad (90.00°)
[estimatePoseFromVision]   Z轴旋转变化: 0.0000 rad (0.00°) -> 1.5708 rad (90.00°)
========================================
```

**关键点**:
- ✓ 表示成功
- ⚠ 表示警告
- ❌ 表示错误
- 显示原始检测结果和提取的Z轴旋转

#### 2.3 抓取周期执行
```
[handleExecuteSingleGrasp] 步骤 2/2: 开始执行抓取周期...
────────────────────────────────────────────────────────────

┌────────────────────────────────────────────────────────────┐
│                   开始抓取周期 (runOneCycle)              │
└────────────────────────────────────────────────────────────┘

► 步骤 0/8: 回安全位
✓ 步骤 0 完成

► 步骤 1/8: 构建抓取位姿
  抓取位姿 Position: [0.411769, 0.183644, 0.255341]
  抓取位姿 Orientation (qx,qy,qz,qw): [0.000000, 0.000000, 0.707099, 0.706803]
  Z轴旋转: 1.5700 rad (89.95°)
✓ 步骤 1 完成

► 步骤 2/8: gripper_tip 变换为 end_effector 目标
  目标位姿 Position: [0.411769, 0.183644, 0.265341]
  目标位姿 Orientation (qx,qy,qz,qw): [0.000000, 0.000000, 0.707099, 0.706803]
✓ 步骤 2 完成

► 步骤 3/8: 抓取接近 (多段笛卡尔路径)
[runGraspApproach] 获取当前位姿...
[runGraspApproach] 当前位姿 Position: [0.600000, -0.150000, 0.450000]
[runGraspApproach] 当前位姿 Orientation (qx,qy,qz,qw): [0.706803, 0.707410, 0.000126, -0.000515]
[runGraspApproach] 规划并执行5段笛卡尔路径...
[runGraspApproach] 第1段: X移动 (从0.600 -> 0.412)
  ✓ 规划成功，100.0% (3/3 点)
  ✓ 执行成功
[runGraspApproach] 第2段: Y移动 (从-0.150 -> 0.184)
  ✓ 规划成功，100.0% (5/5 点)
  ✓ 执行成功
[runGraspApproach] 第3段: 上升到安全高度
  ✓ 规划成功，100.0% (4/4 点)
  ✓ 执行成功
[runGraspApproach] 第4段: 旋转到抓取姿态 (相对Z轴旋转)
  当前姿态: (qx=0.706803, qy=0.707410, qz=0.000126, qw=-0.000515)
  相对Z轴旋转: 1.5700 rad (89.95°)
  目标姿态: (qx=0.500000, qy=0.500000, qz=0.500000, qw=0.500000)
  ✓ 规划成功，100.0% (2/2 点)
  ✓ 执行成功
[runGraspApproach] 第5段: 下降到抓取点
  ✓ 规划成功，100.0% (4/4 点)
  ✓ 执行成功
[runGraspApproach] 抓取接近完成（所有5段执行成功）
✓ 步骤 3 完成

► 步骤 4/8: 闭夹爪 (IO=7, 状态=false)
✓ 步骤 4 完成

► 步骤 5/8: 抬起 (z=0.20 m)
✓ 步骤 5 完成

► 步骤 6/8: 移动到放置位 (place_mode=home_offset)
  放置模式：home_offset (回安全位 + 偏移)
  执行偏移：y=-0.20, x=-0.2, z=-0.15
✓ 步骤 6 完成

► 步骤 7/8: 开夹爪 (IO=7, 状态=true)
✓ 步骤 7 完成

► 步骤 8/8: 回安全位
✓ 步骤 8 完成

┌────────────────────────────────────────────────────────────┐
│             ✓ 抓取周期成功完成 (8/8 步骤)                │
└────────────────────────────────────────────────────────────┘

────────────────────────────────────────────────────────────
[handleExecuteSingleGrasp] ✓ 单次抓取完成
[handleExecuteSingleGrasp] 最终抓取位姿：
[handleExecuteSingleGrasp]   位置: (0.412, 0.184, 0.255) m
[handleExecuteSingleGrasp]   姿态: (qx=0.000, qy=0.000, qz=0.707, qw=0.707)
╚════════════════════════════════════════════════════════════╝
```

**关键信息**:
- 每一步都有明确的开始和完成标记
- 显示位置和姿态的详细数值
- 笛卡尔路径分5段执行，每段显示规划和执行结果
- 显示百分比和点数（100% = 全部路径可达）

---

### 3. 循环抓取服务日志

#### 3.1 启动循环
```
╔════════════════════════════════════════════════════════════╗
║        [循环抓取控制] 收到启动请求                          ║
╚════════════════════════════════════════════════════════════╝
[handleLoopGraspControl] 正在启动循环抓取线程...
[handleLoopGraspControl] ✓ 循环抓取已启动
╚════════════════════════════════════════════════════════════╝

╔════════════════════════════════════════════════════════════╗
║           [循环抓取线程] 已启动                           ║
╚════════════════════════════════════════════════════════════╝
[loopGraspThread] 循环抓取配置：
[loopGraspThread]   object_id: default
[loopGraspThread]   模式: 视觉估计 + 自动抓取
[loopGraspThread]   停止条件: 收到停止信号或ROS节点关闭
```

#### 3.2 循环执行
```
╔════════════════════════════════════════════════════════════╗
║           循环抓取 - 第   1 次循环                        ║
╚════════════════════════════════════════════════════════════╝
[loopGraspThread] 步骤 1/2: 调用视觉估计...
[estimatePoseFromVision] ...（详细视觉估计日志）...
[loopGraspThread] ✓ 视觉估计成功
[loopGraspThread] 步骤 2/2: 执行抓取周期...
────────────────────────────────────────────────────────────
...（完整的 runOneCycle 日志）...
────────────────────────────────────────────────────────────
[loopGraspThread] ✓ 循环第 1 次完成，等待1秒后继续...
╚════════════════════════════════════════════════════════════╝
```

#### 3.3 停止循环
```
╔════════════════════════════════════════════════════════════╗
║        [循环抓取控制] 收到停止请求                          ║
╚════════════════════════════════════════════════════════════╝
[handleLoopGraspControl] 正在停止循环抓取线程...
[handleLoopGraspControl] ✓ 循环抓取已停止
╚════════════════════════════════════════════════════════════╝

[loopGraspThread] 收到停止信号
╔════════════════════════════════════════════════════════════╗
║      [循环抓取线程] 已退出 (共完成  12 次循环)           ║
╚════════════════════════════════════════════════════════════╝
```

---

### 4. 错误和警告日志

#### 4.1 视觉估计未检测到目标
```
[estimatePoseFromVision] ⚠ 视觉估计未检测到目标
[estimatePoseFromVision]   可能原因：
[estimatePoseFromVision]   1. 视野中没有目标工件
[estimatePoseFromVision]   2. 光照条件不佳
[estimatePoseFromVision]   3. 模板匹配失败
```

#### 4.2 视觉服务不可用
```
[estimatePoseFromVision] ❌ 视觉估计服务 /estimate_pose 不可用（超时5秒）
[estimatePoseFromVision] 请确保 visual_pose_estimation 节点正在运行
```

#### 4.3 抓取步骤失败
```
✗ 步骤 3 失败：抓取接近失败
[runGraspApproach] ❌ 第2段失败: Y移动
  规划失败：IK求解失败或路径不可达
```

#### 4.4 服务重复调用
```
[handleLoopGraspControl] ⚠ 循环抓取已在运行中
```

---

## 日志监控方法

### 方法 1: 实时查看所有日志
```bash
ros2 run rqt_console rqt_console
```

### 方法 2: 终端实时显示
```bash
ros2 topic echo /rosout | grep execute_grasp_pose_worker
```

### 方法 3: 保存到文件
```bash
ros2 launch demo_driver execute_grasp_pose_worker.launch.py 2>&1 | tee grasp_worker.log
```

### 方法 4: 只看特定标签
```bash
# 只看视觉估计
ros2 topic echo /rosout | grep estimatePoseFromVision

# 只看服务回调
ros2 topic echo /rosout | grep "handleExecuteSingleGrasp\|handleLoopGraspControl"

# 只看错误
ros2 topic echo /rosout | grep "ERROR\|✗\|❌"
```

---

## 日志符号说明

| 符号 | 含义 | 使用场景 |
|------|------|----------|
| ✓ | 成功 | 步骤完成、操作成功 |
| ✗ | 失败 | 步骤失败 |
| ► | 进行中 | 步骤开始 |
| ⚠ | 警告 | 非致命问题 |
| ❌ | 错误 | 致命错误 |
| ═ | 主分隔线 | 主要区块边界 |
| ─ | 次分隔线 | 次级区块边界 |
| │ | 竖线 | 框图边界 |
| ┌┐└┘ | 框角 | 框图四角 |
| ╔╗╚╝ | 粗框角 | 重要框图四角 |
| ║ | 粗竖线 | 重要框图边界 |

---

## 日志级别调整

如果需要更详细的日志，可以在启动时设置：

```bash
ros2 launch demo_driver execute_grasp_pose_worker.launch.py \
  --log-level execute_grasp_pose_worker:=DEBUG
```

级别选项：
- `DEBUG`: 最详细（包含所有调试信息）
- `INFO`: 正常信息（默认）
- `WARN`: 只显示警告和错误
- `ERROR`: 只显示错误
- `FATAL`: 只显示致命错误

---

## 常见日志模式

### 正常单次抓取
1. 收到服务请求
2. 视觉估计成功（如启用）
3. 8个步骤全部成功
4. 返回成功响应

### 正常循环抓取
1. 启动循环线程
2. 重复：视觉估计 → 8步抓取 → 等待1秒
3. 收到停止信号
4. 线程退出并显示总次数

### 视觉估计失败重试
1. 视觉估计失败（未检测到目标）
2. 等待2秒
3. 重新尝试

### 抓取失败重试
1. 某个步骤失败（如笛卡尔路径规划）
2. 等待2秒
3. 重新开始整个周期

---

## 性能指标（从日志中提取）

- **视觉估计响应时间**: `等待服务响应` 到 `收到服务响应` 的时间
- **单次抓取总时间**: `开始抓取周期` 到 `抓取周期成功完成` 的时间
- **循环抓取频率**: 可从循环次数和总时间计算
- **成功率**: 成功次数 / 总尝试次数

---

## 故障诊断清单

根据日志信息诊断问题：

✓ **视觉服务不可用**
- 检查：是否看到 "服务 /estimate_pose 不可用"
- 解决：启动 visual_pose_estimation 节点

✓ **视觉检测失败**
- 检查：是否看到 "未检测到目标"
- 解决：调整光照、工件位置、模板

✓ **笛卡尔路径规划失败**
- 检查：是否看到 "规划失败" 或百分比 < 100%
- 解决：调整速度、加速度参数，检查关节限位

✓ **夹爪控制失败**
- 检查：是否看到 "闭夹爪失败" 或 "开夹爪失败"
- 解决：检查 IO 连接和配置

✓ **循环无法启动**
- 检查：是否看到 "已在运行中"
- 解决：先停止当前循环

---

## 日志示例文件

完整的日志示例已保存在：
- 单次抓取: `/home/mu/IVG2.0/logs/single_grasp_example.log`
- 循环抓取: `/home/mu/IVG2.0/logs/loop_grasp_example.log`
- 错误情况: `/home/mu/IVG2.0/logs/error_cases.log`
