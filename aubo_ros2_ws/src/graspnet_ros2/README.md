# GraspNet ROS2 功能包

6 自由度抓取位姿预测（ROS2 Humble）。含安装、使用、服务、TF、坐标系与测试说明。

**目录**：[功能](#功能) | [安装与编译](#安装与编译) | [快速开始](#快速开始) | [参数与话题](#参数与话题) | [数据格式](#数据格式) | [publish_grasps 服务](#publish_grasps-服务) | [抓取 TF](#抓取位姿-tf) | [坐标系转换](#坐标系转换) | [测试](#测试) | [IDE 导入](#ide-导入解析)

---

## 功能

- **graspnet_demo_node**：从 `data_dir` 读取 color/depth/mask/meta（或 camera_meta.yaml），预测抓取并发布 MarkerArray、点云与 TF；提供 **`/graspnet_demo_node/publish_grasps`** 服务（手动/自动 `auto_run`）。
- **graspnet_node**：订阅 PointCloud2，实时预测（需 ros_numpy）。
- **image_saver**：订阅相机并保存图像。
- **publish_grasps_client**：命令行调用 publish_grasps。

---

## 安装与编译

```bash
# 依赖（ROS2 Humble）
sudo apt-get install ros-humble-rclpy ros-humble-sensor-msgs \
  ros-humble-visualization-msgs ros-humble-geometry-msgs ros-humble-tf2-ros
pip install torch open3d scipy Pillow numpy

# 编译
cd /path/to/aubo_ros2_ws
colcon build --packages-select graspnet_ros2
source install/setup.bash
```

- **路径**：`graspnet-baseline` 随包安装到 `share/graspnet_ros2/graspnet-baseline/`；开发时从源码找，也可设 `GRASPNET_BASELINE_DIR`。
- **验证**：`ls install/graspnet_ros2/share/graspnet_ros2/graspnet-baseline/`；`ros2 run graspnet_ros2 graspnet_demo_node --ros-args -p data_dir:=/path/to/data`。
- **问题**：找不到 baseline → 重编或设 `GRASPNET_BASELINE_DIR`；导入错误 → 确保 source；模型未找到 → 检查 `model_path` 或传参。

---

## 快速开始

```bash
# 启动（含机械臂、手眼 TF、graspnet_demo_node、可选 Gazebo/RViz2）
ros2 launch graspnet_ros2 graspnet_demo.launch.py
# 指定数据目录
ros2 launch graspnet_ros2 graspnet_demo.launch.py data_dir:=/path/to/data
```

**触发发布**（默认 `auto_run:=false`，需手动触发）：

```bash
ros2 run graspnet_ros2 publish_grasps_client
# 或
ros2 service call /graspnet_demo_node/publish_grasps std_srvs/srv/Trigger
# 或脚本：scripts/publish_grasps.sh
```

- 首次调用：读数据 → 预测 → 发布 MarkerArray、点云、TF（camera_frame → grasp_pose_0,1,...）。后续调用仅重发缓存。
- 自动模式：`auto_run:=true`，启动约 1 秒后自动发布一次。

**RViz2**：Fixed Frame `world`/`base_link`；Topic `/grasp_markers`、`/graspnet_pointcloud`；添加 TF。

**其他**：`ros2 run graspnet_ros2 graspnet_node`（实时点云）；`ros2 run graspnet_ros2 image_saver`（保存图像）。

---

## 参数与话题

### graspnet_demo_node 主要参数

| 参数 | 默认 | 说明 |
|------|------|------|
| model_path | …/checkpoint-rs.tar | 模型权重 |
| data_dir | …/doc/pose_1 | 数据目录 |
| max_grasps_num | 20 | 发布抓取数 |
| frame_id | camera_frame | 抓取/TF 坐标系 |
| hand_eye_yaml_path | （手眼包） | 手眼标定 |
| camera_meta_yaml | 空→config/camera_meta.yaml | 相机内参 |
| use_open3d | true | Open3D 窗口 |
| auto_run | false | 启动后自动发布一次 |
| factor_depth | 4000.0 | 深度缩放（0.25mm 用 4000） |

其余：num_point(20000)、num_view(300)、collision_thresh、voxel_size、gpu、marker_topic、pointcloud_topic、trigger_service、*_image_topic、camera_info_topic 等见 launch 或 `ros2 param list`。

### 话题与服务

- **订阅**（graspnet_node）：`/pointcloud` (PointCloud2)。
- **发布**（graspnet_demo_node）：`/grasp_markers`(MarkerArray)、`/graspnet_pointcloud`(点云)、TF `frame_id`→`grasp_pose_X`。
- **服务**：`/graspnet_demo_node/publish_grasps` (std_srvs/srv/Trigger)。

---

## 数据格式

`data_dir` 需含：`color.png`、`depth.png`（16 位）、`workspace_mask.png`；内参用 `meta.mat`（intrinsic_matrix、factor_depth）或 `config/camera_meta.yaml`。

---

## publish_grasps 服务

- **接口**：Trigger，无请求；响应 success/message。首次调用计算并发布，后续仅重发缓存。
- **推荐**：手动模式先起节点再起 RViz2，就绪后调用；调试时可多次调用重发（不重算）。调用无显示 → 查 RViz 话题、TF、Fixed Frame。强制重算 → 重启节点。
- **性能**：首次 ~2–5 s，后续发布 <100 ms。

---

## 抓取位姿 TF

- **结构**：`camera_frame` → `grasp_pose_0`, `grasp_pose_1`, …；链：world → base_link → … → wrist3_Link → camera_frame → grasp_pose_X。
- **查看**：`ros2 run tf2_ros tf2_echo base_link grasp_pose_0`；`ros2 run tf2_tools view_frames`；RViz2 Add→TF。
- **代码**：`tf2_ros.Buffer` + `TransformListener`，`lookup_transform('base_link','grasp_pose_0', rclpy.time.Time())`。MoveIt 可将目标 frame_id 设为 grasp_pose_0 或先查 TF 再设 pose。
- **约定**：camera_frame 光心 X 右 Y 下 Z 前；grasp_pose_X 已转为 ROS 末端（X=width,Y=height,Z=approach），见下节。
- **注意**：TF 随“计算并发布”写入；重发需再调 publish_grasps。编号 0 最佳。Frame 不存在 → 先调 publish_grasps、查节点与 tf2_monitor；旧数据 → 用 `rclpy.time.Time()`、查 buffer 时长。

---

## 坐标系转换

- **像素→相机**：Z = depth/factor_depth（米）；X=(u-cx)*Z/fx，Y=(v-cy)*Z/fy。内参来自 camera_meta.yaml 或 meta.mat。本项目“世界”= 相机系 (camera_frame)。
- **GraspNet→ROS**：GraspNet 列为 approach/width/height；ROS 末端为 X=width、Y=height、Z=approach。转换在 `graspnet_demo_node.py` 的 `publish_grasp_tf()`、`create_grasp_markers()` 中已实现：

```python
R_ros = np.column_stack([R_graspnet[:,1], R_graspnet[:,2], R_graspnet[:,0]])  # X,Y,Z
```

- **验证**：RViz2 中 grasp_pose_0 红 X=width、绿 Y=height、蓝 Z=approach，夹爪沿 Z 伸出；机械臂通过 TF 得到的是转换后姿态。

---

## 测试

- **内容**：路径查找、模型加载、数据读取、可选抓取预测。
- **运行**：`ros2 launch graspnet_ros2 test.launch.py`；含预测 `test_prediction:=true`；指定路径 `model_path:=/path data_dir:=/path`；`ros2 run graspnet_ros2 graspnet_test_node`。参数：test_model_load、test_data_read、test_prediction、model_path、data_dir、use_rviz、rviz_config。
- **问题**：无 baseline → 编译+source 或 GRASPNET_BASELINE_DIR；ImportError models.graspnet → 路径与 baseline 结构、重编。退出码 0/1 可做 CI。

**坐标转换验证**：启动 demo → RViz 看 TF/Marker 方向（X/Y/Z 同上）→ `tf2_echo base_link grasp_pose_0` → 跑 publish_grasps_client 看规划与执行。失败时查手眼 `tf2_echo wrist3_Link camera_frame`、TF 链 view_frames、运动参数与碰撞。

---

## 目录结构

```
graspnet_ros2/
├── graspnet_ros2/          # 模块：utils, image_saver, graspnet_node, graspnet_demo_node, publish_grasps_client, ...
├── graspnet-baseline/
├── launch/                 # graspnet_demo.launch.py, test.launch.py
├── config/                 # camera_meta.yaml, *.rviz
├── scripts/                # publish_grasps.sh
├── package.xml, setup.py
```

**依赖**：ROS2(rclpy,sensor_msgs,visualization_msgs,geometry_msgs,tf2_ros)、PyTorch、Open3D、NumPy、SciPy、Pillow、graspnet-baseline。

---

## IDE 导入解析

Pyright 报「无法解析导入」时：graspnet-baseline 与 rclpy 等需加入 `extraPaths`（运行靠 sys.path 注入，静态分析不执行）。

**pyrightconfig.json**（工作区根）：

```json
{
  "extraPaths": [
    "aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ]
}
```

工作区为 aubo_ros2_ws 时第一项改为 `"src/graspnet_ros2/graspnet-baseline"`。或用 `.vscode/settings.json` 的 `python.analysis.extraPaths`（填绝对路径）。仍报错 → 重载窗口、确认工作区与 pyrightconfig 一致、`find /opt/ros/humble -name rclpy -type d` 把 rclpy 所在目录加入、核对 Python 版本。

---

## 许可证

MIT License
