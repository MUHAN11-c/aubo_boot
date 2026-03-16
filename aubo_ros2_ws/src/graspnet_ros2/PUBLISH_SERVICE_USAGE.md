# GraspNet 发布服务使用说明

## 概述

GraspNet Demo 节点现在支持两种工作模式：

1. **自动模式**（`auto_run=true`）：节点启动后自动计算并发布抓取
2. **手动模式**（`auto_run=false`，默认）：通过服务手动触发发布

手动模式解决了 RViz2 启动慢导致看不到可视化的问题。

---

## 工作模式

### 手动模式（推荐）

**启动节点：**
```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=false
```

节点会：
- ✓ 加载模型和数据
- ✓ 等待服务调用
- ✗ 不会自动计算和发布

**启动 RViz2：**
```bash
rviz2 -d <your_rviz_config.rviz>
```

**手动触发发布：**

有三种方法：

#### 方法1：使用 Python 客户端（推荐）
```bash
ros2 run graspnet_ros2 publish_grasps_client
```

#### 方法2：使用便捷脚本
```bash
cd /home/mu/IVG/aubo_ros2_ws/src/graspnet_ros2/scripts
./publish_grasps.sh
```

#### 方法3：使用 ROS2 命令行
```bash
ros2 service call /publish_grasps std_srvs/srv/Trigger
```

---

### 自动模式

**启动节点：**
```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=true
```

节点会：
- ✓ 加载模型和数据
- ✓ 1秒后自动计算并发布抓取
- ⚠️ 如果 RViz2 启动慢，可能看不到可视化

---

## 服务说明

### 服务名称
`/publish_grasps`

### 服务类型
`std_srvs/srv/Trigger`

### 请求参数
无（空请求）

### 响应参数
- `success` (bool): 是否成功
- `message` (string): 详细信息

### 行为
1. **首次调用**：计算抓取 → 发布 MarkerArray 和点云
2. **后续调用**：直接发布缓存的结果（不重新计算）

---

## 使用场景

### 场景1：RViz2 延迟启动
```bash
# 终端1：启动节点（手动模式）
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=false

# 终端2：启动 RViz2（慢慢启动）
rviz2

# 终端3：RViz2 就绪后手动发布
ros2 run graspnet_ros2 publish_grasps_client
```

### 场景2：反复调试可视化
```bash
# 启动节点（只计算一次）
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=false

# 多次调用发布（不重新计算，立即发布）
ros2 run graspnet_ros2 publish_grasps_client
# ... 修改 RViz 设置 ...
ros2 run graspnet_ros2 publish_grasps_client  # 再次发布
```

### 场景3：静态场景多次查看
机械臂移动后，MarkerArray 会随之移动（因为在 camera_frame）。
使用手动模式可以：
1. 拍照后计算抓取
2. 移动机械臂
3. 重新发布查看（抓取位置保持在拍照时的世界坐标）

---

## Launch 文件参数

在 launch 文件中添加：

```python
# 添加 auto_run 参数声明
launch.actions.DeclareLaunchArgument(
    'auto_run',
    default_value='false',
    description='是否自动运行（false时需要手动调用服务）'
),

# 在节点参数中添加
parameters=[{
    'auto_run': launch.substitutions.LaunchConfiguration('auto_run'),
    # ... 其他参数 ...
}]
```

---

## 调试

### 查看服务是否可用
```bash
ros2 service list | grep publish_grasps
```

### 查看节点日志
```bash
ros2 node list
ros2 node info /graspnet_demo_node
```

### 手动测试服务
```bash
ros2 service call /publish_grasps std_srvs/srv/Trigger
```

---

## 常见问题

### Q1: 服务调用后没有显示？
**A:** 检查：
- RViz2 是否订阅了正确的话题（`/grasp_markers`, `/graspnet_pointcloud`）
- TF 树是否正确（`ros2 run tf2_tools view_frames`）
- 坐标系设置（Fixed Frame 应该是 `world` 或 `base_link`）

### Q2: 多次调用会重新计算吗？
**A:** 不会。首次调用时计算，后续调用直接发布缓存结果，速度很快。

### Q3: 如何强制重新计算？
**A:** 重启节点。

### Q4: 可以同时使用自动和手动模式吗？
**A:** 可以。`auto_run=true` 时，节点会自动发布一次，但服务仍然可用。

---

## 性能

| 操作 | 耗时 |
|------|------|
| 首次计算 | ~2-5秒（取决于GPU） |
| 后续发布 | <100ms |
| 服务响应 | 立即 |

---

## 总结

✅ **推荐工作流程：**

1. 启动节点（手动模式）：`auto_run:=false`
2. 慢慢启动 RViz2 并配置
3. 准备好后调用：`ros2 run graspnet_ros2 publish_grasps_client`
4. 查看结果，必要时重复步骤3

这种方式既节省资源，又保证可视化可靠。
