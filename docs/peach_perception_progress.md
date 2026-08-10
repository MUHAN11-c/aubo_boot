# 桃子感知与连续 TSDF 重建进度台账

> 最后更新：2026-08-10。当前采用两个功能包：
> peach_pose_ros2 + peach_reconstruction_ros2。

## 当前结论

| 模块 | 职责 | 状态 |
|---|---|---|
| peach_pose_ros2 | YOLO/SAM、单帧几何、初始目标、target_id 记忆 | 软件验证完成 |
| peach_reconstruction_ros2 | 精确时间 FK、有界 ICP、在线 Open3D TSDF、网格/几何精化 | 软件验证完成，待真机 |
| peach_pose_fusion_ros2 | 原独立一致性状态机 | 已移除，必要质量门归入重建包 |

机械臂驱动栈固定只读，重建包只订阅相机、感知和 TF，不修改或绕过驱动接口。
真机由操作者通过硬件或示教器手动上电，任何软件流程不得调用
/aubo_dashboard/startup。

## 已完成

### 感知

- 修复深度尺度、32FC1 深度、TF 变换和 travel_end 变换问题。
- 将算法核、ROS 转换、TF、可视化、点云和离线验证按职责拆分。
- 目标身份在 base_link 中做空间记忆，短时消失后 target_id 保持稳定。
- 参数集中到 frozen dataclass，并由测试保证 YAML 与代码默认值一致。

### 重建

- RGB/Depth/CameraInfo 按消息头时间戳同步。
- 每帧只查询 depth.header.stamp 对应的 base_link←camera TF；失败跳帧，
  不再使用 latest TF 回退。
- Robot FK + 手眼外参提供 T_fk；Open3D 两尺度点到平面 ICP 只估计相对
  T_fk 的小修正，默认边界 10 mm/3°。
- ICP 合格采用 T_refined；ICP 不合格但 FK 预对齐合格时保留 FK；
  二者均差则拒帧。
- 0.8 FPS 下每个唯一同步帧均进入质量门，capture.auto_min_interval_s=0，
  capture.require_robot_static=false；小于 2 mm 且 1° 的近重复视角只跳过
  TSDF 积分，相机接收不中断，也不要求停止后采集。
- 合格帧到达即积分 Open3D ScalableTSDFVolume；finalize 不再重复批量积分，
  只提取局部点云、带法向三角网格和圆柱/球几何结果。
- remove_last_frame 会重建空体积并重放剩余帧，避免已删除帧残留在 TSDF。
- session 保存 RGB、深度、内参、T_fk、T_used、ICP fitness/RMSE/修正量、
  tsdf_cloud.ply、tsdf_mesh.ply 和完整参数快照。

### 软件验证

- Open3D 0.19.0 可用，CUDA 后端可用；当前 0.8 FPS 局部重建继续使用稳定的
  ScalableTSDFVolume，不引入额外 GPU 框架。
- 有界 ICP 单测覆盖模型预热、小误差修正和大修正不接受。
- TSDF 测试覆盖外参方向、深度字节序、彩色点云、局部裁剪和几何精化。
- peach_pose_fusion_ros2 已从源码、入口、配置、测试和现行文档中移除。

## 真机验证待办

1. 启动前检查旧 ROS、相机容器和重建节点进程。
2. 操作者现场确认急停、限位、碰撞等级和低速模式，并手动硬件上电。
3. RViz2 速度缩放设为 0.01，执行围绕目标的连续平滑慢速路径。
4. 连续获取 12～24 个合格帧，观察 diagnostics：
   - tf_failures 应接近 0；
   - 不允许任何 stale/latest TF 帧进入 TSDF；
   - ICP fitness、RMSE和修正量应稳定且不触边界；
   - tsdf.integrated_frames 应和合格帧数一致。
5. finalize 后检查 tsdf_cloud、mesh、refined_pose/refined_axis。
6. save_session 落盘，比较 FK-only 与 FK+ICP 的表面厚度、重投影残差和多次扫描
   重复性，再依据数据调整 ICP 阈值。

命令见 src/peach_reconstruction_ros2/README.md。

## 已知边界

- 套袋场景得到的是袋子外表面，而不是袋内果实真实表面。
- 刚性 ICP 不能修复深度比例错误、时间戳系统偏移或严重手眼标定错误。
- 若每帧 ICP 修正呈稳定同向偏差，应重新标定手眼外参，不能把 ICP 当作永久补偿。
- CameraInfo 当前与 RGB/Depth 一起近似同步；若真机发现内参消息频率导致同步饥饿，
  再改为独立缓存，不提前增加分支复杂度。
