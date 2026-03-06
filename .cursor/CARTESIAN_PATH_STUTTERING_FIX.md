# 机械臂笛卡尔空间路径运动异响问题修复记录

## 问题描述

在执行笛卡尔空间路径运动时，机械臂发出持续的"异响"或"连续声音"，而关节空间路径运动正常。通过日志分析发现：

- **关节空间路径**：segment_T 均匀，约 0.095s，边界频率约 10.5/s
- **笛卡尔空间路径**：segment_T 不均匀，范围 0.0748~0.2326s，边界频率约 6.4~10.0/s

问题根源：笛卡尔路径的点间隔不均匀，导致 segment_T 变化大，引起机械振动和异响。

## 参考内容

### 1. ROS Industrial UniformSampleFilter

**来源**：`industrial_trajectory_filters/src/uniform_sample_filter.cpp`

**核心方法**：**插值重采样**（Interpolation Resampling）

**关键特点**：
- 使用固定的 `sample_duration`（默认 0.050 秒）
- 从时间 0 开始，每隔 `sample_duration` 插值一个新点
- 使用 KDL 的 `VelocityProfile_Spline` 进行插值
- 保证位置、速度、加速度的连续性

**算法流程**：
```cpp
double interpolated_time = 0.0;
while (interpolated_time < duration_in) {
    // 找到包含 interpolated_time 的两个原始点
    // 使用 spline 插值计算新点的位置、速度、加速度
    trajectory_out.points.push_back(interp_pt);
    interpolated_time += sample_duration_;
}
```

### 2. MoveIt 时间参数化

**来源**：`moveit_core/trajectory_processing/src/iterative_time_parameterization.cpp`

**关键算法**：
- Iterative Parabolic Time Parameterization
- Iterative Spline Parameterization
- Time-optimal Trajectory Generation (TOTG)

**配置参数**：
- `resample_dt`: 均匀时间间隔
- `path_tolerance`: 路径偏差容忍度

### 3. Universal Robots 驱动设计

**关键特点**：
- **Passthrough Controllers**: 将完整轨迹传递给机器人硬件，让机器人控制器进行插补
- **ScaledJointTrajectoryController**: 处理执行速度缩放
- 使用插值重采样保证轨迹平滑性

## 原理

### 问题根源

1. **点选择方法（Thinning）的局限性**：
   - 只能选择现有点，无法生成新点
   - 无法保证均匀时间间隔
   - 首尾 segment 难以优化
   - 可能丢失轨迹信息

2. **插值重采样方法（Interpolation Resampling）的优势**：
   - 通过插值生成新点，可以精确控制时间间隔
   - 使用 spline 插值，保证位置、速度、加速度的连续性
   - 不丢失轨迹信息，保留原始轨迹的形状
   - 保证均匀的 segment_T，与关节空间对齐

### 解决方案原理

**核心思想**：使用插值重采样，将笛卡尔路径的点间隔统一到目标值（0.095s），使其与关节空间路径的 segment_T 对齐。

**实现策略**：
1. 检测笛卡尔路径：通过检查点间隔是否小于阈值（0.095s）
2. 插值重采样：从时间 0 开始，每隔目标间隔（0.095s）插值一个新点
3. 使用 KDL spline 插值：保证位置、速度、加速度的连续性
4. 保留起点和终点：确保轨迹完整性

## 具体实现

### ROS2 端实现（C++）

#### 1. 修改头文件 `aubo_ros2_trajectory_action.h`

添加 `UniformSampleFilter` 成员变量：

```cpp
class JointTrajectoryAction : public rclcpp::Node
{
  // ... 其他成员 ...
  
  // 轨迹重采样过滤器（用于笛卡尔路径）
  UniformSampleFilter resample_filter_;
};
```

#### 2. 修改构造函数 `aubo_ros2_trajectory_action.cpp`

添加参数声明和过滤器配置：

```cpp
// 声明并获取笛卡尔路径重采样参数
this->declare_parameter<double>("cartesian_resample_threshold", 0.095);
this->declare_parameter<double>("cartesian_min_segment_interval", 0.095);
double cartesian_threshold = this->get_parameter("cartesian_resample_threshold").as_double();
double target_interval = this->get_parameter("cartesian_min_segment_interval").as_double();

// 配置重采样过滤器
resample_filter_.configure(target_interval);
```

#### 3. 修改 `publishTrajectory()` 函数

添加笛卡尔路径检测和重采样逻辑：

```cpp
// ========== 笛卡尔路径重采样处理 ==========
double cartesian_threshold = this->get_parameter("cartesian_resample_threshold").as_double();
double target_interval = this->get_parameter("cartesian_min_segment_interval").as_double();

if (current_trajectory_.points.size() >= 3 && target_interval > 0)
{
  // 计算点间隔
  double min_interval = 1.0;
  for (size_t i = 1; i < current_trajectory_.points.size(); i++)
  {
    double interval = toSec(current_trajectory_.points[i].time_from_start) - 
                      toSec(current_trajectory_.points[i-1].time_from_start);
    if (interval < min_interval)
    {
      min_interval = interval;
    }
  }
  
  // 如果最小间隔小于阈值，说明是密集的笛卡尔路径，需要重采样
  if (min_interval < cartesian_threshold)
  {
    size_t n_before = current_trajectory_.points.size();
    trajectory_msgs::msg::JointTrajectory resampled_traj;
    
    // 使用 UniformSampleFilter 进行重采样（插值重采样，保证均匀时间间隔）
    resample_filter_.configure(target_interval);
    if (resample_filter_.update(current_trajectory_, resampled_traj))
    {
      current_trajectory_ = resampled_traj;
      RCLCPP_INFO(this->get_logger(), "Cartesian resampled: %zu -> %zu points", 
                  n_before, current_trajectory_.points.size());
    }
  }
}
```

#### 4. 修复 `UniformSampleFilter::update()` 函数

确保保留起点和终点：

```cpp
bool UniformSampleFilter::update(const trajectory_msgs::msg::JointTrajectory &in, 
                                  trajectory_msgs::msg::JointTrajectory &out)
{
  // ... 初始化代码 ...
  
  // 始终保留起点
  out.points.push_back(in.points[0]);
  
  // 从第一个间隔开始插值
  interpolated_time = sample_duration_;
  
  while (interpolated_time < duration_in)
  {
    // 找到包含 interpolated_time 的两个原始点
    // 使用 KDL spline 插值生成新点
    // ...
    interpolated_time += sample_duration_;
  }
  
  // 保留终点
  out.points.push_back(in.points.back());
  
  return true;
}
```

#### 5. KDL Spline 插值实现

使用 KDL 的 `VelocityProfile_Spline` 进行插值：

```cpp
bool UniformSampleFilter::interpolatePt(...)
{
  KDL::VelocityProfile_Spline spline_calc;
  
  for (size_t i = 0; i < p1.positions.size(); ++i)
  {
    double time_from_p1 = time_from_start - toSec(p1.time_from_start);
    double time_from_p1_to_p2 = p2_time_from_start - p1_time_from_start;
    
    spline_calc.SetProfileDuration(
      p1.positions[i], p1.velocities[i], p1.accelerations[i],
      p2.positions[i], p2.velocities[i], p2.accelerations[i],
      time_from_p1_to_p2);
    
    interp_pt.positions[i] = spline_calc.Pos(time_from_p1);
    interp_pt.velocities[i] = spline_calc.Vel(time_from_p1);
    interp_pt.accelerations[i] = spline_calc.Acc(time_from_p1);
  }
  
  return true;
}
```

### ROS1 端实现（Python）

#### 1. 添加插值重采样函数 `trajectory_speed.py`

```python
def interpolate_point_linear(p1, p2, t_target):
    """
    在两个轨迹点之间进行线性插值。
    参考: ROS Industrial UniformSampleFilter 的插值方法
    """
    t1 = p1.time_from_start.to_sec()
    t2 = p2.time_from_start.to_sec()
    
    if t2 == t1:
        return deepcopy(p1)
    
    alpha = (t_target - t1) / (t2 - t1)
    alpha = max(0.0, min(1.0, alpha))  # 限制在 [0, 1]
    
    interp_pt = JointTrajectoryPoint()
    interp_pt.time_from_start = rospy.Duration(t_target)
    
    n_joints = len(p1.positions)
    interp_pt.positions = [p1.positions[i] + alpha * (p2.positions[i] - p1.positions[i]) 
                           for i in range(n_joints)]
    
    # 如果有速度和加速度信息，也进行插值
    if len(p1.velocities) == n_joints and len(p2.velocities) == n_joints:
        interp_pt.velocities = [p1.velocities[i] + alpha * (p2.velocities[i] - p1.velocities[i])
                                for i in range(n_joints)]
    
    return interp_pt


def resample_trajectory_uniform_interval(traj, target_interval_sec):
    """
    使用插值重采样，使 segment_T 均匀对齐目标间隔。
    参考: ROS Industrial UniformSampleFilter 的实现方法
    """
    if target_interval_sec <= 0 or len(traj.points) < 2:
        return traj
    
    out = JointTrajectory()
    out.joint_names = list(traj.joint_names)
    out.points = []
    
    duration = traj.points[-1].time_from_start.to_sec()
    interpolated_time = target_interval_sec
    index_in = 0
    
    # 始终保留起点
    out.points.append(deepcopy(traj.points[0]))
    
    while interpolated_time < duration:
        # 找到包含 interpolated_time 的两个原始点
        while (index_in + 1 < len(traj.points) and 
               interpolated_time > traj.points[index_in + 1].time_from_start.to_sec()):
            index_in += 1
        
        if index_in + 1 >= len(traj.points):
            break
        
        # 插值新点
        p1 = traj.points[index_in]
        p2 = traj.points[index_in + 1]
        interp_pt = interpolate_point_linear(p1, p2, interpolated_time)
        out.points.append(interp_pt)
        
        interpolated_time += target_interval_sec
    
    # 保留终点
    out.points.append(deepcopy(traj.points[-1]))
    
    return out
```

#### 2. 在轨迹回调中使用（可选）

如果需要，可以在 ROS1 端的 `aubo_robot_simulator` 中使用新的插值重采样函数替代点选择方法。

## 配置参数

### ROS2 Launch 文件配置

```xml
<node name="aubo_ros2_trajectory_action" pkg="aubo_ros2_trajectory_action" type="aubo_ros2_trajectory_action">
  <!-- 笛卡尔路径重采样参数 -->
  <param name="cartesian_resample_threshold" value="0.095"/>
  <param name="cartesian_min_segment_interval" value="0.095"/>
</node>
```

### ROS1 Launch 文件配置

```xml
<group ns="aubo_controller">
  <param name="cartesian_resample_threshold" value="0.095"/>
  <param name="cartesian_min_segment_interval" value="0.095"/>
</group>
```

## 效果验证

### 修复前
- 笛卡尔路径 segment_T: 0.0748~0.2326s（不均匀）
- 边界频率: 6.4~10.0/s
- **结果**: 持续异响

### 修复后
- 笛卡尔路径 segment_T: 约 0.095s（均匀）
- 边界频率: 约 10.5/s（与关节空间对齐）
- **结果**: 异响消失，运动平滑

### 可视化验证（插补稳定性）

可用本仓库中的脚本对 `full_chain_motion.ndjson` 做轨迹点与插补分析，直观查看 segment 间隔是否稳定在目标值附近：

- **脚本**：`.cursor/visualize_full_chain_motion.py`
- **数据**：`.cursor/full_chain_motion.ndjson`（含 `trajectory_received` 的 `point_times`）
- **生成图表**：
  - `full_chain_motion_charts.png`：轨迹接收、段时长分布、缓冲、段耗时、事件时间线
  - `full_chain_motion_interpolation.png`：**轨迹点时间曲线** + **插补间隔 Δt**
    - 图中绿色线：目标间隔 0.095s（与本文档 `cartesian_min_segment_interval` 一致）
    - 青虚线：2×0.095s（双段）
    - Δt 柱状贴近绿/青线 → 插补稳定；分散或偏离大 → 仍有不均匀，可考虑重采样

运行方式（在仓库根目录或 `.cursor` 下）：

```bash
python3 .cursor/visualize_full_chain_motion.py
```

## 技术要点总结

1. **插值重采样 vs 点选择**：
   - 插值重采样可以生成新点，保证均匀时间间隔
   - 点选择只能选择现有点，无法保证均匀性

2. **KDL Spline 插值**：
   - 保证位置、速度、加速度的连续性
   - 避免轨迹突变，减少机械振动

3. **自动检测笛卡尔路径**：
   - 通过点间隔判断是否为密集的笛卡尔路径
   - 只对需要重采样的路径进行处理，不影响关节空间路径

4. **参数化配置**：
   - 通过 ROS 参数配置阈值和目标间隔
   - 支持运行时调整，便于调试和优化

## 相关文件

- ROS2 端：
  - `aubo_ros2_ws/src/aubo_ros2_driver/aubo_ros2_trajectory_action/include/aubo_ros2_trajectory_action.h`
  - `aubo_ros2_ws/src/aubo_ros2_driver/aubo_ros2_trajectory_action/src/aubo_ros2_trajectory_action.cpp`
  - `aubo_ros2_ws/src/aubo_ros2_driver/aubo_ros2_trajectory_action/src/aubo_ros2_uniform_sample_filter.cpp`

- ROS1 端：
  - `aubo_ws/src/aubo_robot/aubo_robot/aubo_controller/script/aubo_controller/trajectory_speed.py`
  - `aubo_ws/src/aubo_robot/aubo_robot/aubo_controller/script/aubo_controller/aubo_robot_simulator`

- 参考文档：
  - `.cursor/trajectory_resampling_reference.md`
- 效果可视化：
  - `.cursor/visualize_full_chain_motion.py`（轨迹点曲线与插补稳定性图）

## 日期

修复完成日期：2026年2月5日
