# demo.py 与 11.py 详细对比

## 一、总体定位

| 文件 | 定位 | 运行方式 |
|------|------|----------|
| **demo.py** | 离线演示脚本 | 直接运行 `python demo.py`，从文件读取数据，Open3D 可视化 |
| **11.py** | ROS 节点（ROS1） | `rosrun` 启动，订阅 PointCloud2 话题，发布 Pose + MarkerArray |

---

## 二、依赖与导入

### demo.py
```python
import os, sys, numpy, open3d, scipy.io, PIL.Image
import torch
from graspnetAPI import GraspGroup
from models.graspnet import GraspNet, pred_decode
from utils.collision_detector import ModelFreeCollisionDetector
from utils.data_utils import CameraInfo, create_point_cloud_from_depth_image
```
- **无 ROS 依赖**：纯 Python 脚本
- **需要图像文件**：color.png, depth.png, workspace_mask.png, meta.mat

### 11.py
```python
import rospy, ros_numpy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Vector3
from utils.grasp import Grasp, GraspGroup
from models.graspnet import GraspNet, pred_decode
from utils.collision_detector import ModelFreeCollisionDetector
```
- **ROS1 依赖**：rospy, ros_numpy, sensor_msgs 等
- **需要 ROS 话题**：订阅 PointCloud2，发布 Pose + MarkerArray

---

## 三、参数配置

### demo.py（硬编码默认值）
```python
checkpoint_path = os.path.join(ROOT_DIR, 'logs', 'log_kn', 'checkpoint-rs.tar')
data_dir = os.path.join(ROOT_DIR, 'doc', 'pose_1')
num_point = 20000
num_view = 300
collision_thresh = 0.01
voxel_size = 0.01
gpu = 0
```
- **固定参数**：代码中直接定义，无法运行时修改

### 11.py（ROS 参数服务器）
```python
self.model_path = rospy.get_param("~model_path", "../models/checkpoint-rs.tar")
self.max_point_num = rospy.get_param("~max_point_num", 20000)
self.collision_thresh = rospy.get_param("~collision_thresh", 0.01)
self.voxel_size = rospy.get_param("~voxel_size", 0.01)
self.max_grasps_num = rospy.get_param("~max_grasps_num", 20)
self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "pointcloud")
self.grasp_topic = rospy.get_param("~grasp_topic", "grasps")
self.marker_topic = rospy.get_param("~marker_topic", "marker")
```
- **可配置**：通过 ROS 参数服务器或 launch 文件动态设置

---

## 四、数据输入处理

### demo.py：`get_and_process_data(data_dir)`

| 步骤 | 操作 | 说明 |
|------|------|------|
| 1 | 读取 `color.png`, `depth.png`, `workspace_mask.png`, `meta.mat` | 从文件系统加载 |
| 2 | 深度图反投影：`create_point_cloud_from_depth_image(depth, camera)` | 用相机内参将深度图转为有序点云 (H, W, 3) |
| 3 | 有效点筛选：`mask = (workspace_mask & (depth > 0))` | 只保留工作空间内且深度有效的点 |
| 4 | 采样到固定点数：无放回/有放回采样到 `num_point` | 确保网络输入固定长度 |
| 5 | 组装：采样点送网络，全部有效点用于碰撞检测 | 两套点云（采样 vs 全量） |

**特点**：
- 需要**相机内参**（meta.mat）做反投影
- 需要**工作空间掩码**筛选有效区域
- 输出**有序点云**（与图像像素对应）

### 11.py：`get_points(data: PointCloud2)`

| 步骤 | 操作 | 说明 |
|------|------|------|
| 1 | `ros_numpy.numpify(data)` | 将 ROS PointCloud2 转为 numpy 字典 |
| 2 | 提取 x, y, z：`pc_np = concatenate([pc["x"], pc["y"], pc["z"]], axis=1)` | 直接取点云坐标，形状 (N, 3) |
| 3 | 采样：若 `len(pc_np) > max_point_num` 则无放回采样，否则用全部点 | **不补足**，点数不足时直接用全部点 |
| 4 | 转 tensor：`torch.from_numpy(...).to(device)` | (1, N, 3) tensor |
| 5 | 组装：采样点送网络，**完整点云**用于碰撞检测 | 两套点云（采样 vs 全量） |

**特点**：
- **不需要相机内参**：点云已由其他节点提供
- **不需要工作空间掩码**：假设输入点云已过滤
- 输入是**无序点云**（点列表，无图像对应关系）
- **点数不足时不补足**：demo.py 会补足，11.py 直接用全部点

---

## 五、模型加载

### demo.py：`get_net()`
```python
net = GraspNet(input_feature_dim=0, num_view=num_view, ...)
net.to(DEVICE)
checkpoint = torch.load(checkpoint_path, weights_only=True)
net.load_state_dict(checkpoint['model_state_dict'])
net.eval()
```
- 每次调用都创建新模型（函数式）
- 有文件存在性检查

### 11.py：`__init__()`
```python
self.net = GraspNet(input_feature_dim=0, num_view=300, ...)
self.net.to(self.device)
checkpoint = torch.load(self.model_path)
self.net.load_state_dict(checkpoint["model_state_dict"])
self.net.eval()
```
- 模型作为类成员（对象式）
- 无文件存在性检查

---

## 六、抓取预测

### demo.py：`get_grasps(net, end_points)`
```python
with torch.no_grad():
    end_points = net(end_points)
    grasp_preds = pred_decode(end_points)
gg_array = grasp_preds[0].detach().cpu().numpy()
gg = GraspGroup(gg_array)
return gg
```
- **完全相同**：都调用 `net()` → `pred_decode()` → `GraspGroup()`

### 11.py：`get_grasps(self, end_points)`
```python
with torch.no_grad():
    end_points = self.net(end_points)
    grasp_preds = pred_decode(end_points)
gg_array = grasp_preds[0].detach().cpu().numpy()
gg = GraspGroup(gg_array)
return gg
```
- **完全相同**：逻辑一致

---

## 七、碰撞检测

### demo.py：`collision_detection(gg, cloud)`
```python
mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=voxel_size)
collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=collision_thresh)
gg = gg[~collision_mask]
return gg
```
- **完全相同**：都使用 `ModelFreeCollisionDetector`
- `cloud` 是 `np.array(cloud.points)`（Open3D 点云转 numpy）

### 11.py：`collision_detection(self, gg, cloud)`
```python
mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=self.voxel_size)
collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
gg = gg[~collision_mask]
rospy.loginfo(f"碰撞检测后剩余{len(gg)}个")
return gg
```
- **基本相同**：11.py 多了日志输出

---

## 八、后处理与输出

### demo.py：`vis_grasps(gg, cloud)`
```python
gg = gg.nms()
gg.sort_by_score()
gg = gg[:1]  # 只保留最高分的一条
grippers = gg.to_open3d_geometry_list()
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(...)  # ROS2 相机坐标系
o3d.visualization.draw_geometries([cloud, *grippers, frame])
```
- **输出**：Open3D 窗口可视化
- **只显示 1 条**：最高分抓取
- **坐标系**：ROS2 相机坐标系（x 右、y 下、z 前）

### 11.py：`get_top_n(gg, n)` + `to_pose_msg(gg)` + `vis_grasps(poses, f, qualities)`
```python
gg = gg.nms()
gg.sort_by_score()
gg = gg[:n]  # 保留前 n 条（默认 20）
# 坐标系转换：GraspNet → 机器人坐标系
rot_mat_z = Rotation.from_euler("z", 90, degrees=True).as_matrix()
rot_mat_y = Rotation.from_euler("y", 90, degrees=True).as_matrix()
rot_mat = rot_mat_y @ rot_mat_z
pose_mat[:3, :3] = rot_mat @ grasp.rotation_matrix
pose_mat = forward_pose(pose_mat, grasp.depth)  # 前移 depth
pose = matrix_to_pose_msg(pose_mat)
# 发布到 ROS 话题
self.grasp_pub.publish(pose)
self.marker_pub.publish(MarkerArray(...))
```
- **输出**：ROS Pose 消息 + RViz MarkerArray
- **显示多条**：默认前 20 条（可配置）
- **坐标系转换**：GraspNet 坐标系 → 机器人坐标系（沿 x 轴开合、沿 z 轴接近）
- **前移 depth**：将抓取中心前移到指尖位置

---

## 九、坐标系转换（关键差异）

### demo.py
- **无坐标系转换**：抓取位姿直接使用 GraspNet 输出的坐标系
- **可视化坐标系**：ROS2 相机坐标系（仅用于辅助观察）

### 11.py
- **有坐标系转换**：`to_pose_msg()` 中将 GraspNet 坐标系转换为机器人坐标系
  - 绕 z 轴旋转 90°：`[[0,-1,0],[1,0,0],[0,0,1]]`
  - 绕 y 轴旋转 90°：`[[0,0,1],[0,1,0],[-1,0,0]]`
  - 组合：`rot_mat_y @ rot_mat_z`
- **前移 depth**：`forward_pose(pose_mat, grasp.depth)` 将抓取中心沿 z 轴前移 `depth` 距离到指尖
- **用途**：转换后的位姿可直接用于机器人控制

---

## 十、主流程对比

### demo.py：`demo(data_dir)`
```python
net = get_net()                    # 创建模型
end_points, cloud = get_and_process_data(data_dir)  # 从文件读取并处理
gg = get_grasps(net, end_points)   # 预测
if collision_thresh > 0:
    gg = collision_detection(gg, np.array(cloud.points))  # 碰撞检测
vis_grasps(gg, cloud)              # Open3D 可视化
```
- **一次性执行**：运行一次处理一个数据目录
- **同步流程**：顺序执行，完成后退出

### 11.py：`predict_grasps(data: PointCloud2)`
```python
end_points, points_o3d = self.get_points(data)      # 从 ROS 话题接收
gg = self.get_grasps(end_points)                    # 预测
if self.collision_thresh > 0:
    gg = self.collision_detection(gg, np.array(points_o3d))  # 碰撞检测
gg = self.get_top_n(gg, self.max_grasps_num)        # NMS + 排序 + 取前 n
grasps, q = self.to_pose_msg(gg)                    # 坐标系转换
self.vis_grasps(grasps, data.header.frame_id, q)    # RViz 可视化
self.pub_grasp(grasps)                              # 发布 Pose
```
- **持续运行**：ROS 节点，循环处理话题消息
- **异步回调**：每次收到点云消息触发一次预测

---

## 十一、关键差异总结表

| 维度 | demo.py | 11.py |
|------|---------|-------|
| **输入** | 文件（color/depth/mask/meta） | ROS PointCloud2 话题 |
| **点云来源** | 深度图反投影 | 直接接收点云 |
| **需要相机内参** | ✅ 是（meta.mat） | ❌ 否 |
| **需要工作空间掩码** | ✅ 是 | ❌ 否 |
| **点数不足处理** | 有放回补足到 num_point | 直接用全部点（不补足） |
| **坐标系转换** | ❌ 无 | ✅ 有（GraspNet → 机器人系） |
| **前移 depth** | ❌ 无 | ✅ 有（到指尖位置） |
| **输出数量** | 1 条（最高分） | n 条（默认 20，可配置） |
| **输出方式** | Open3D 窗口 | ROS Pose + MarkerArray |
| **参数配置** | 硬编码 | ROS 参数服务器 |
| **运行方式** | 一次性脚本 | ROS 节点（持续运行） |

---

## 十二、使用场景

### demo.py 适用场景
- ✅ 离线测试与调试
- ✅ 验证数据预处理流程
- ✅ 快速可视化抓取结果
- ✅ 不需要 ROS 环境

### 11.py 适用场景
- ✅ 实时抓取预测（ROS 系统集成）
- ✅ 与机器人控制系统对接
- ✅ 需要坐标系转换（GraspNet → 机器人）
- ✅ 需要发布多条候选抓取

---

## 十三、代码结构对比

### demo.py（函数式）
```
get_net()                    # 创建模型
get_and_process_data()      # 数据处理
get_grasps()                 # 预测
collision_detection()        # 碰撞检测
vis_grasps()                 # 可视化
demo()                       # 主流程
```

### 11.py（面向对象）
```
class GNBServer:
    __init__()               # 初始化（加载模型、订阅/发布）
    get_points()             # 数据处理
    get_grasps()             # 预测
    collision_detection()    # 碰撞检测
    get_top_n()              # NMS + 排序
    to_pose_msg()            # 坐标系转换
    vis_grasps()             # RViz 可视化
    pub_grasp()              # 发布 Pose
    predict_grasps()         # 回调函数（主流程）
```

---

## 十四、建议

1. **数据输入**：11.py 假设输入点云已过滤，若需要工作空间筛选，应在发布 PointCloud2 前处理
2. **点数不足**：11.py 不补足点数，若输入点云过少可能影响预测质量
3. **坐标系**：11.py 的坐标系转换针对特定机器人，若机器人坐标系不同需调整 `to_pose_msg()` 中的旋转矩阵
4. **可视化**：demo.py 用 Open3D，11.py 用 RViz，两者可互补使用
