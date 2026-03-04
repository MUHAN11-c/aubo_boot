# GraspNet 服务模式快速开始

## 1. 编译安装

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select graspnet_ros2
source install/setup.bash
```

## 2. 启动节点（手动模式）

```bash
# 终端1：启动 GraspNet 节点
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=false
```

**节点日志应该显示：**
```
[INFO] [graspnet_demo_node]: GraspNet Demo 节点已启动
[INFO] [graspnet_demo_node]: 自动运行: False
[INFO] [graspnet_demo_node]: 发布服务: publish_grasps
[INFO] [graspnet_demo_node]: 手动模式：请调用 /publish_grasps 服务来计算并发布抓取
```

## 3. 启动 RViz2

```bash
# 终端2：启动 RViz2（慢慢启动，不用着急）
rviz2
```

在 RViz2 中配置：
- Fixed Frame: `world` 或 `base_link`
- 添加 MarkerArray 显示：Topic = `/grasp_markers`
- 添加 PointCloud2 显示：Topic = `/graspnet_pointcloud`

## 4. 手动触发发布

RViz2 准备好后，运行：

```bash
# 终端3：触发发布
ros2 run graspnet_ros2 publish_grasps_client
```

**应该看到：**
```
[INFO] [publish_grasps_client]: 等待 /publish_grasps 服务...
[INFO] [publish_grasps_client]: 服务已连接
[INFO] [publish_grasps_client]: 发送发布请求...
[INFO] [publish_grasps_client]: ✓ 成功: 成功发布 1 个抓取和点云
```

**节点日志：**
```
[INFO] [graspnet_demo_node]: 收到发布请求，开始计算抓取...
[INFO] [graspnet_demo_node]: 开始计算抓取...
[INFO] [graspnet_demo_node]: 抓取计算完成，共 1 个抓取
[INFO] [graspnet_demo_node]: 已发布 1 个抓取和 TF 变换
[INFO] [graspnet_demo_node]: 已发布点云: XXXXX 个点
```

## 5. 测试便捷脚本

```bash
# 使用 shell 脚本
cd /home/mu/IVG/aubo_ros2_ws/src/graspnet_ros2/scripts
./publish_grasps.sh
```

## 6. 使用 ROS2 命令行

```bash
# 查看可用服务
ros2 service list | grep publish

# 直接调用服务
ros2 service call /publish_grasps std_srvs/srv/Trigger
```

## 自动模式（旧方式）

如果需要自动运行：

```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=true
```

节点会在启动1秒后自动计算并发布。

---

## 常见问题

### Q: 服务不可用？
```bash
# 检查节点是否运行
ros2 node list

# 检查服务
ros2 service list
```

### Q: RViz 看不到 Marker？
- 检查 Fixed Frame 设置
- 检查 MarkerArray Topic 是否正确
- 重新调用服务发布

### Q: 想重新计算抓取？
重启节点。首次调用服务会计算，后续调用直接发布缓存结果。

---

## 完整工作流程示例

```bash
# 1. 启动节点（手动模式）
ros2 launch graspnet_ros2 graspnet_demo.launch.py auto_run:=false use_open3d:=false

# 2. 等待节点加载完成...

# 3. 启动 RViz2 并配置

# 4. 触发发布
ros2 run graspnet_ros2 publish_grasps_client

# 5. 查看结果

# 6. 需要时重新发布（不重新计算）
ros2 run graspnet_ros2 publish_grasps_client
```

---

## 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `auto_run` | `false` | 是否自动运行 |
| `use_open3d` | `true` | 是否打开 Open3D 窗口 |
| `max_grasps_num` | `1` | 发布的抓取数量 |
| `data_dir` | 自动检测 | 输入数据目录 |

使用示例：
```bash
ros2 launch graspnet_ros2 graspnet_demo.launch.py \
    auto_run:=false \
    use_open3d:=false \
    max_grasps_num:=5
```
