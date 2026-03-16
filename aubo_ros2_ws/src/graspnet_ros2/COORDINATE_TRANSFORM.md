# GraspNet 到 ROS2 坐标系转换说明

## 问题背景

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
