# interface

**视觉位姿估计、手眼与图像桥接**等上层功能共用的 **ROS 2 消息与服务定义**（`ament_cmake`）。源码路径：

`aubo_ros2_ws/src/visual_pose_estimation/src/interface/`

依赖本包的主要功能包包括 **`visual_pose_estimation`**、**`visual_pose_estimation_python`**、**`hand_eye_calibration`**、**`image_data_bridge`** 等（以各包 `package.xml` 为准）。

---

## 消息（`msg/`）

| 文件 | 说明 |
|------|------|
| `ImageData.msg` | 带相机 ID 等元数据的图像载荷（如 JPEG/原始像素），供手眼与桥接节点使用 |
| `CartesianPosition.msg` | 笛卡尔位姿相关数据结构 |

---

## 服务（`srv/`）

| 文件 | 说明 |
|------|------|
| `EstimatePose.srv` | 视觉位姿估计 |
| `ListTemplates.srv` | 列出可用模板 |
| `StandardizeTemplate.srv` | 模板标准化 |
| `UpdateParams.srv` | 更新运行参数 |
| `ProcessDebugStep.srv` | 调试步骤处理 |
| `VisualizeGraspPose.srv` | 抓取位姿可视化相关 |

---

## 构建

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select interface
source install/setup.bash
```

字段与语义以各 `.msg` / `.srv` 内注释为准。
