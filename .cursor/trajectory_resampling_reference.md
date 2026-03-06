# 轨迹重采样参考：UR等机械臂驱动的设计方法

## 关键发现

### 1. ROS Industrial UniformSampleFilter 方法

**位置**: `industrial_trajectory_filters/src/uniform_sample_filter.cpp`

**核心策略**: **插值重采样**（Interpolation Resampling），而非点选择（Point Thinning）

**关键特点**:
- 使用固定的 `sample_duration`（默认 0.050 秒）
- 从时间 0 开始，每隔 `sample_duration` 插值一个新点
- 使用 KDL 的 `VelocityProfile_Spline` 进行插值
- 保证位置、速度、加速度的连续性

**算法流程**:
```cpp
double interpolated_time = 0.0;
while (interpolated_time < duration_in) {
    // 找到包含 interpolated_time 的两个原始点
    // 使用 spline 插值计算新点的位置、速度、加速度
    trajectory_out.points.push_back(interp_pt);
    interpolated_time += sample_duration_;
}
```

### 2. MoveIt 时间参数化方法

**位置**: `moveit_core/trajectory_processing/src/iterative_time_parameterization.cpp`

**关键算法**:
- Iterative Parabolic Time Parameterization（默认）
- Iterative Spline Parameterization
- Time-optimal Trajectory Generation (TOTG)

**配置参数**:
- `resample_dt`: 均匀时间间隔
- `path_tolerance`: 路径偏差容忍度
- `min_angle_change`: 最小角度变化阈值

### 3. Universal Robots 驱动设计

**关键特点**:
- **Passthrough Controllers**: 将完整轨迹传递给机器人硬件，让机器人控制器进行插补
- **ScaledJointTrajectoryController**: 处理执行速度缩放
- **Speed Scaling State Broadcaster**: 发布当前执行速度

## 当前问题分析

### 当前实现（点选择方法）的局限性：

1. **无法保证均匀时间间隔**: 只能选择现有点，无法生成新点
2. **首尾 segment 难以优化**: 如果原始轨迹的首尾 segment 本身就很大，无法通过选择点来改善
3. **可能丢失轨迹信息**: 跳过某些点可能导致轨迹不够平滑

### 插值重采样方法的优势：

1. **保证均匀时间间隔**: 通过插值生成新点，可以精确控制时间间隔
2. **保证轨迹平滑性**: 使用 spline 插值，保证位置、速度、加速度的连续性
3. **不丢失轨迹信息**: 通过插值生成新点，保留原始轨迹的形状

## 改进建议

### 方案1: 实现插值重采样（推荐）

参考 `UniformSampleFilter` 的实现，使用插值重采样：

```python
def resample_trajectory_uniform_interval(traj, target_interval_sec):
    """
    使用插值重采样，使 segment_T 均匀对齐目标间隔。
    策略：从时间0开始，每隔 target_interval_sec 插值一个新点。
    """
    if target_interval_sec <= 0 or len(traj.points) < 2:
        return traj
    
    out = JointTrajectory()
    out.joint_names = list(traj.joint_names)
    out.points = []
    
    duration = traj.points[-1].time_from_start.to_sec()
    interpolated_time = 0.0
    index_in = 0
    
    # 始终保留起点
    out.points.append(traj.points[0])
    
    while interpolated_time < duration:
        interpolated_time += target_interval_sec
        
        # 找到包含 interpolated_time 的两个原始点
        while (index_in + 1 < len(traj.points) and 
               interpolated_time > traj.points[index_in + 1].time_from_start.to_sec()):
            index_in += 1
        
        if index_in + 1 >= len(traj.points):
            break
        
        # 插值新点
        p1 = traj.points[index_in]
        p2 = traj.points[index_in + 1]
        interp_pt = interpolate_point(p1, p2, interpolated_time)
        out.points.append(interp_pt)
    
    # 保留终点
    out.points.append(traj.points[-1])
    
    return out
```

### 方案2: 改进当前的点选择方法

如果必须使用点选择方法，可以改进算法：

1. **更智能的首尾处理**: 如果首尾 segment 过大，考虑插入中间点或调整时间
2. **更精确的点选择**: 使用更复杂的评分函数，考虑 segment 时间的均匀性
3. **后处理优化**: 对选择后的轨迹进行时间调整，使 segment 时间更均匀

## 参考资源

1. **ROS Industrial UniformSampleFilter**: 
   - `industrial_trajectory_filters/src/uniform_sample_filter.cpp`
   - 默认 `sample_duration = 0.050` 秒

2. **MoveIt 时间参数化**:
   - `moveit_core/trajectory_processing/src/iterative_time_parameterization.cpp`
   - 支持 `resample_dt` 参数

3. **Universal Robots 驱动**:
   - `Universal_Robots_ROS_passthrough_controllers`: 传递完整轨迹给硬件
   - `Universal_Robots_ROS_controllers_cartesian`: 笛卡尔轨迹控制器

## 下一步行动

1. 实现插值重采样函数（方案1）
2. 测试插值重采样效果
3. 如果效果不理想，考虑使用 KDL 的 VelocityProfile_Spline 进行更精确的插值
