# peach_reconstruction_ros2

## 简介

本包是套袋桃靠近阶段的唯一局部三维重建包。机械臂以低速连续运动，
Percipio RGB-D 相机约 0.8 FPS；节点不要求机械臂停稳，而是持续接收每个
RGB-D 时间戳并执行：

1. TF2 精确查询 base_link←camera，不允许回退 latest TF；
2. 机器人 FK/手眼外参给出绝对相机位姿；
3. Open3D 点到平面 ICP 只估计有界小修正；
4. 近重复视角只跳过积分，其余合格帧立即积分 Open3D ScalableTSDFVolume；
5. finalize 提取彩色点云、带法向三角网格和目标几何结果。

peach_pose_ros2 仍负责检测、分割和初始目标；原
peach_pose_fusion_ros2 已移除，重建质量门和最终 refined 输出均在本包内。
自动绑定只接受 base_frame 中且不含 tf_stale/tf_unavailable/target_untracked
标记的有限候选；绑定后缓存 target_kind，目标暂时离场也不会误换拟合模型。
机械臂驱动、控制器、Dashboard 和真机 bringup 不属于本包，固定只读。

## 使用方法

构建与离线测试：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select peach_pose_msgs peach_pose_ros2 peach_reconstruction_ros2
source install/setup.bash
cd src/peach_reconstruction_ros2
PYTHONPATH=../peach_pose_ros2:$PYTHONPATH ../../aubo_py3.12/bin/python -m pytest test/ -q
```

真机测试前先检查旧进程：

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run'
```

确认现场急停、限位和低速模式后启动真机链路。机械臂必须由操作者在硬件或
示教器上手动上电；禁止调用 /aubo_dashboard/startup。

```bash
# 终端 1：真机、相机、TF、MoveIt/RViz
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real

# 终端 2：初始目标感知
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
ros2 launch peach_pose_ros2 peach_pose.launch.py

# 终端 3：连续在线 TSDF
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
ros2 launch peach_reconstruction_ros2 reconstruction.launch.py
```

在 RViz2 将速度缩放保持为 0.01，规划相机围绕目标的平滑慢速路径。运动期间
节点持续采集，不需要停下后再采集。建议先观察 12～24 个合格帧，再完成并保存：

```bash
ros2 topic echo /peach/reconstruction/diagnostics
ros2 service call /peach_reconstruction_node/finalize_reconstruction std_srvs/srv/Trigger '{}'
ros2 service call /peach_reconstruction_node/save_session std_srvs/srv/Trigger '{}'
```

RViz2 关注：

- /peach/reconstruction/tsdf_cloud：在线局部 TSDF 点云；
- /peach/reconstruction/markers：相机轨迹、最终网格和 refined 轴；
- /peach/reconstruction/refined_pose：最终目标几何；
- /peach/reconstruction/refined_axis：表面/采摘方向；
- /peach/reconstruction/diagnostics：TF、ICP、TSDF 和重建质量。

### 全链路状态与抓取许可查询

重建只绑定全局计划的 `selected_target_id`，仅接受该 ID 且与深度图完全同时间戳
的掩膜。默认质量门会跳过掩膜小于 300 像素、掩膜内有效深度低于 0.35、目标
相对绑定中心漂移超过 40 mm 的帧，分别覆盖远距/遮挡、室外强光深度空洞和风吹
摆动场景。掩膜外深度在建云、TSDF、失败重放和 session 落盘前全部清零。

```bash
ros2 service call /peach_reconstruction_node/query_reconstruction_state \
  std_srvs/srv/Trigger "{}"
ros2 topic echo /peach/reconstruction/grasp_decision
```

`grasp_decision` 是只读 JSON：只有重建状态为 READY、最终几何存在且 refit 为
ACCEPT 时 `allowed=true`；REOBSERVE、REJECT、目标丢失或重建未完成均为 false。
本包不会据此发送 MoveIt goal。重建事件附着到同一 `harvest_run_id` 的
`harvest_runs/harvest_*/events.jsonl`；详细 RGB-D、位姿、点云和网格仍保存在
`peach_sessions/session_*`，其路径由 `session_saved` 事件反向关联。

## 执行逻辑

```text
RGB + Depth --按 header.stamp 同步--> 深度局部云
                     |
/joint_states -> Robot FK -> TF2 精确时刻 base←camera
                     |
              手眼外参进入 TF 链
                     |
              T_fk + 当前局部云
                     |
       帧到 TSDF 模型的有界鲁棒 ICP
          | 合格                  | 低质量且 FK 也差
          v                       v
       T_refined                跳过本帧
          |
       在线 Open3D TSDF
          |
    PointCloud + TriangleMesh
          |
   中心 / 表面 / 法向 / refined_pose
```

第一帧或模型点数不足时直接使用 FK。后续 ICP 使用 6 mm/3 mm 两尺度、
Tukey 鲁棒核和点到平面误差；默认最多只允许相对 FK 修正 10 mm/3°。
ICP 不合格但 FK 与模型重叠合格时保留 FK；两者都不合格则拒帧。

每帧保存 T_base_camera_fk、T_base_camera_used、fitness、RMSE、修正量和
接受原因。TSDF 不支持任意撤销，因此 remove_last_frame 会新建空体积并重放
剩余合格帧。session 结果位于 peach_sessions/session_*：

```text
frame_XX_rgb.png
frame_XX_depth.npy
frame_XX_camera_info.yaml
frame_XX_T_base_camera.yaml
result/tsdf_cloud.ply
result/tsdf_mesh.ply
metadata.yaml
```

套袋场景重建的是袋子外表面；输出中心、表面和法向描述袋体局部几何，不能
直接解释为袋内桃子的真实尺寸。

## 软件框架

| 文件 | 职责 |
|---|---|
| reconstruction_node.py | ROS 编排：同步、精确 TF、状态、在线积分、发布与服务 |
| candidate_contract.py | 感知候选安全门禁与绑定目标类别记忆（零 ROS import） |
| params.py | frozen dataclass 参数层，和 reconstruction.yaml 双向测试同步 |
| frame_collector.py | 无 ROS 的帧栈和采集状态机 |
| icp_refiner.py | Open3D 两尺度鲁棒点到平面 ICP 与边界/质量门 |
| tsdf_volume.py | Open3D ScalableTSDFVolume、点云和网格提取 |
| geometry_refiner.py | TSDF 表面的圆柱/球几何精化 |
| session_io.py | 原始帧、FK/ICP 位姿、点云、网格和元数据落盘 |
| visualization.py | RViz 相机轨迹、网格和 refined 轴 Marker |

参数默认值以 config/reconstruction.yaml 为权威源。关键真机默认值：

| 参数 | 默认值 |
|---|---:|
| tf_timeout_sec | 1.0 s |
| capture.max_views | 24 |
| capture.auto_min_interval_s | 0.0 s |
| capture.require_robot_static | false |
| capture.auto_finalize_at_max | false |
| capture.require_target_mask | true |
| capture.min_mask_pixels | 300 |
| capture.min_mask_depth_ratio | 0.35 |
| capture.max_target_drift_m | 0.04 m |
| tsdf.voxel_length / sdf_trunc | 3 mm / 12 mm |
| icp.max_translation / max_rotation_deg | 10 mm / 3° |
| icp.min_fitness / max_rmse | 0.35 / 8 mm |
