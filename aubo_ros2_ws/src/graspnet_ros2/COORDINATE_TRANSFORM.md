# GraspNet 到 ROS2 坐标系转换说明

## 一、像素 / 相机 / “世界”坐标系关系

在 GraspNet 与本项目中，涉及三类坐标系：**像素坐标系**、**相机坐标系**、以及 GraspNet 输出所在的**参考系（即相机坐标系，在 ROS2 中称为 `camera_frame`）**。下面说明三者的定义与换算。

### 1. 像素坐标系（Image / Pixel Frame）

- **定义**：图像平面上的 2D 坐标，用于表示“第几行、第几列像素”。
- **约定**：
  - **u（列）**：水平方向，从 0 到 `width-1`，对应代码里的 `xmap`。
  - **v（行）**：竖直方向，从 0 到 `height-1`，对应代码里的 `ymap`。
- **单位**：像素（无量纲整数索引）。
- **深度**：每个像素 (u, v) 对应一个深度值 `depth(u,v)`，通常为毫米或与 `factor_depth` 同单位的原始值；换算成物理深度（米）为 `Z = depth / factor_depth`。

### 2. 相机坐标系（Camera / Optical Frame）

- **定义**：以相机光心为原点、光轴为 Z 轴右手系，即 **OpenCV/计算机视觉常用约定**。
- **轴方向**：
  - **X 轴**：沿图像水平向右（u 增大的方向）。
  - **Y 轴**：沿图像竖直向下（v 增大的方向）。
  - **Z 轴**：沿光轴指向场景（前向），即深度增加的方向。
- **单位**：米（本项目 `data_utils` 中点云单位）；GraspNet API 中部分函数使用毫米，需注意注释。
- **与像素的关系**：由**针孔模型 + 内参**给出：

```
内参矩阵 K (3×3):
    [ fx  0  cx ]
K = [  0 fy  cy ]
    [  0  0   1 ]

像素 (u, v) + 物理深度 Z（米）→ 相机系 3D 点 (X, Y, Z)：
  X = (u - cx) * Z / fx
  Y = (v - cy) * Z / fy
  Z = depth_value / factor_depth   （深度图像素值 → 米）
```

- **代码位置**：
  - `graspnet-baseline/utils/data_utils.py`：`create_point_cloud_from_depth_image()`（点云单位：米）。
  - `graspnet-baseline/graspnetAPI/graspnetAPI/utils/utils.py`：`framexy_depth_2_xyz()`、`batch_framexy_depth_2_xyz()`（注释中深度单位：毫米）。
- **内参来源**：`config/camera_meta.yaml` 的 `intrinsic_matrix` 与 `factor_depth`（替代原 `meta.mat`）。

### 3. “世界”坐标系在本项目中的含义（= 相机坐标系 = camera_frame）

- 在 **GraspNet 模型** 与 **plot_gripper_pro_max(center, R, ...)** 中：
  - 输入点云由“深度图 + 内参”反投影得到，**全部在相机坐标系下**。
  - 模型预测的抓取位姿（平移 `center`、旋转 `R`）也是**相对于该相机系**的。
- 因此，文档或注释里说的 **“世界坐标系”** 在本 pipeline 中即 **相机坐标系**；在 ROS2 里该参考系被命名为 **`frame_id`**，默认是 **`camera_frame`**。
- **plot_gripper_pro_max** 的 `center`：**(3,) ndarray，夹爪中心在“世界”系下的位置**，在这里就是 **在相机坐标系 (camera_frame) 下的 (x, y, z)**，单位与点云一致（通常为米）。

### 4. 三者关系小结

```
像素 (u, v) + 深度图 depth(u,v)
    ↓  Z = depth / factor_depth
    ↓  X = (u - cx)*Z/fx,  Y = (v - cy)*Z/fy
相机系 3D 点 (X, Y, Z)  [camera_frame]
    ↓  GraspNet 预测抓取
抓取位姿 (center, R) 仍在相机系 [即“世界”/camera_frame]
    ↓  TF 与 ROS2 中
camera_frame → grasp_pose_0, grasp_pose_1, ...（并可通过手眼标定转到机械臂 base/末端）
```

- **像素 → 相机**：仅用内参和深度，无需外参。
- **相机 = 本项目的“世界”参考系**：GraspNet 的 center/R 都基于此系。
- **相机 → 机械臂基座/末端**：由手眼标定和 TF 树（如 `ee_frame_id` → `camera_frame`）完成，不在“像素/相机/世界”这段公式里体现。

---

## 二、问题背景（GraspNet 与 ROS2 姿态轴定义不同）

GraspNet-baseline 和 ROS2 对抓取位姿的坐标系定义不同，导致需要进行坐标转换。

## 坐标系定义对比

### GraspNet 坐标系
```
rotation_matrix 列向量定义：
- col0 (X轴): approach 方向（手指伸出/接近物体的方向）
- col1 (Y轴): width 方向（手指张开方向）
- col2 (Z轴): height 方向（垂直于抓取平面）
```

### ROS2 标准末端执行器坐标系
```
标准定义：
- X轴: width 方向
- Y轴: height 方向
- Z轴: approach 方向（工具接近方向，最重要！）
```

## 坐标转换公式

```python
# GraspNet -> ROS2 转换
R_graspnet = grasp.rotation_matrix  # (3x3)
R_ros = np.column_stack([
    R_graspnet[:, 1],  # ROS X轴 = GraspNet Y轴 (width)
    R_graspnet[:, 2],  # ROS Y轴 = GraspNet Z轴 (height)
    R_graspnet[:, 0]   # ROS Z轴 = GraspNet X轴 (approach) ⬅️ 关键！
])
```

## 代码实现位置

### 1. TF 发布（已修改）
**文件**: `graspnet_ros2/graspnet_demo_node.py`  
**函数**: `publish_grasp_tf()`  
**行号**: ~739-789

在发布 TF 变换时应用坐标转换，确保 `grasp_pose_0` 等坐标系符合 ROS2 标准。

### 2. RViz Marker 可视化（已有转换）
**文件**: `graspnet_ros2/graspnet_demo_node.py`  
**函数**: `create_grasp_markers()`  
**行号**: ~897-970

Marker 创建时使用相同的坐标转换，确保可视化与 TF 一致。

```python
# Marker 的圆柱体默认沿 Z 轴，所以需要把 approach 映射到 Z 轴
left_pose[:3, :3] = np.column_stack([R[:, 1], R[:, 2], R[:, 0]])
```

## 验证方法

### 在 RViz2 中检查
1. 查看 TF 坐标轴颜色：
   - 🔴 红色 (X轴): 应指向手指张开方向
   - 🟢 绿色 (Y轴): 应垂直于抓取平面
   - 🔵 蓝色 (Z轴): 应指向手指伸出方向（approach）

2. 确认 Marker 和 TF 坐标轴方向一致

### 执行抓取测试
```bash
# 1. 启动系统
ros2 launch graspnet_ros2 graspnet_demo.launch.py use_open3d:=true

# 2. 触发抓取并控制机械臂
ros2 run graspnet_ros2 publish_grasps_client
```

## 重要提示

⚠️ **所有使用抓取位姿的地方都必须使用转换后的坐标系**

- ✅ TF 发布：已转换
- ✅ RViz Marker：已转换
- ✅ 机械臂控制：通过 TF 自动获得正确姿态

## 参考资料

1. GraspNet-baseline 源码：`graspnetAPI/utils/rotation.py` - `batch_viewpoint_params_to_matrix()`
2. GraspNet-baseline 源码：`graspnetAPI/utils/utils.py` - `plot_gripper_pro_max()`
3. ROS REP-103: Standard Units of Measure and Coordinate Conventions
4. 工业机器人 ISO 9787 标准：工具坐标系定义

## 更新历史

- 2026-02-10: 修改 `publish_grasp_tf()` 函数，添加坐标系转换
- 原因: Marker 显示正确但机械臂姿态错误，需要统一坐标系
