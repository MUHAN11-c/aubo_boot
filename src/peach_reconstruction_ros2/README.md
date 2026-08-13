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

## 当前开发范围：套袋桃

当前重建主线只面向**套袋桃外表面**：逐帧掩膜、局部 TSDF、圆柱 refit、
袋轴/入口/行程和质量阈值均以套袋场景为当前调参与验收对象。重建结果描述的是
袋体可见表面，不代表袋内桃子的真实尺寸。裸桃球体 refit 与方向先验仍作为
实验兼容分支保留，但未完成当前真机抓取验收，不应作为现阶段工具接触的放行路径。

peach_pose_ros2 仍负责检测、分割和初始目标；原
peach_pose_fusion_ros2 已移除，重建质量门和最终 refined 输出均在本包内。
自动候选门只接受 base_frame 中且不含 tf_stale/tf_unavailable/target_untracked
标记的有限候选；绑定后缓存 target_kind，目标暂时离场也不会误换拟合模型。
机械臂驱动、控制器、Dashboard 和真机 bringup 不属于本包，固定只读。

与 `peach_pose_ros2` 的职责边界、接口映射、时间戳契约、切换规则、真机全流程和
验收清单见 **[桃子首帧感知与连续局部重建联动说明](../../docs/peach_pose_reconstruction_integration.md)**。

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

`reconstruction.launch.py` 默认自动加载包内 `config/reconstruction.yaml`，正常启动
无需手写 `--params-file`。需要临时验证另一份参数时才传 launch 参数
`reconstruction_params_file:=<绝对路径>`。如果另起 `percipio_rgbd.launch.py`，bringup 必须传
`camera_enabled:=false`，避免两个进程争用同一相机。

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

### 全链路状态、目标绑定与抓取许可查询

期望绑定目标来自全局计划的 `selected_target_id`。节点只使用
`/peach/perception/target_observations` 中处于 `OBSERVED`、掩膜非空且与深度图
完全同时间戳的逐目标掩膜。默认质量门会跳过掩膜小于 300 像素、掩膜内有效深度低于 0.35、目标
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

候选选择器对非空 `selected_target_id` 做严格硬绑定：首选目标只有
`REOBSERVE` 或暂时缺失时不会退到其他目标。每次开始采集前仍须确认查询结果中的
重建 `target_id` 与感知 `selected_target_id` 相同；不一致说明节点版本或缓存状态异常，
不得继续融合，应 reset 并重启相关节点。

## 执行逻辑

### 状态机

| 状态 | 进入条件 | 行为 |
|---|---|---|
| `IDLE` | 启动、reset、目标切换后 | 等待安全候选；默认自动开始 |
| `COLLECTING` | 自动绑定或 start 服务成功 | 持续门控并在线积分合格帧 |
| `READY` | finalize 成功 | 发布最终产物，不再接收新帧 |
| `FAILED` | 预留状态 | 当前异常主要通过诊断和服务失败返回 |

默认 `capture.auto_mode=true`，无需手动 start/capture；合格帧到达即积分。默认
`capture.auto_finalize_at_max=false`，达到最大视角数不会自动 finalize，仍由任务管理器
在安全观察位显式调用；设为 true 后才会在最大视角数触发自动完成。

在 `READY` 收到新的首选 ID 时，节点清空旧帧、TSDF、最终产物和掩膜缓存，回到
`IDLE` 后绑定下一目标。`COLLECTING` 期间拒绝中途换 ID，防止两棵目标混入同一
模型；此时应先 finalize/save，或 reset 后切换。

### 单帧处理顺序

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

完整接受顺序为：状态与帧龄 → 选中 ID 的精确时间戳掩膜 → 跟踪/像素数/有效深度/
中心漂移 → 掩膜深度和局部云 → 精确时刻 TF/FK → 视角基线 → ICP/FK 质量门 →
即时 TSDF 积分 → 发布与落盘。任何一步失败都不能把该帧混入模型。

自动连续采集会跳过近重复视角；移动超过期望最大步长时记录警告但仍可能采集。
手动 `capture_frame` 的检查更严格，过大移动会拒绝；手动是否允许重复视角由服务
路径的 `allow_duplicate` 行为决定。现场自动接近仍应连续、低速、平滑。

### FK、ICP 和 refined 的语义

| 结果 | 来源 | 用途 |
|---|---|---|
| initial pose | 感知包单帧检测、分割和深度拟合 | 全局建档、排序和接近初值 |
| FK pose | 关节状态、机器人运动学、手眼外参 | 每帧点云在 `base_link` 的绝对相机位姿 |
| ICP correction | 当前局部云与已有模型 | 只对 FK 做有界小修正，不替代绝对定位 |
| refined pose | 多帧 TSDF 后球体或圆柱重拟合 | finalize 后的最终视觉抓取几何；裸桃球心/半径由 TSDF 精化，轴沿用绑定目标的果梗先验 |

模型尚未建立时诊断为 `model_warmup`；ICP 全部门限通过时为 `mode=icp`；ICP
不合格但 FK 重叠合格时为 `mode=fk`；两者都不合格则拒帧。ICP 修正过大通常是
掩膜串目标、同步、外参或重叠问题，不能通过放宽边界强行接受。

下游应使用 refined pose，而不是等待本包去覆盖感知包的 initial pose。只有
状态 `READY`、最终几何存在且 refit 为 `ACCEPT` 时，`grasp_decision.allowed` 才为真；
仍须额外通过 MoveIt 可达性、碰撞、安全区和末端执行器检查。

### finalize、撤销与保存

每帧保存 `T_base_camera_fk`、`T_base_camera_used`、fitness、RMSE、修正量和
接受原因。TSDF 不支持任意撤销，因此 `remove_last_frame` 会新建空体积并重放
剩余合格帧。finalize 不自动保存完整 session，需另行调用 `save_session`。
session 结果位于 `peach_sessions/session_*`：

```text
frame_XX_rgb.png
frame_XX_depth.npy
frame_XX_camera_info.yaml
frame_XX_T_base_camera.yaml
metadata.yaml
result/tsdf_cloud.ply
result/tsdf_mesh.ply
```

套袋场景重建的是袋子外表面；输出中心、表面和法向描述袋体局部几何，不能
直接解释为袋内桃子的真实尺寸。

## 软件框架

| 文件 | 职责 |
|---|---|
| reconstruction_node.py | ROS 编排：同步、精确 TF、状态、在线积分、发布与服务 |
| candidate_contract.py | 感知候选安全门禁与绑定目标类别记忆（零 ROS import） |
| capture_gate.py | 手动/自动两路采帧公共门禁纯函数（零 ROS import） |
| params.py | frozen dataclass 参数层，和 reconstruction.yaml 双向测试同步 |
| frame_collector.py | 无 ROS 的帧栈和采集状态机 |
| icp_refiner.py | Open3D 两尺度鲁棒点到平面 ICP 与边界/质量门 |
| tsdf_volume.py | Open3D ScalableTSDFVolume、点云和网格提取 |
| geometry_refiner.py | TSDF 表面的圆柱/球几何精化 |
| session_io.py | 原始帧、FK/ICP 位姿、点云、网格和元数据落盘 |
| visualization.py | RViz 相机轨迹、网格和 refined 轴 Marker |

### 订阅话题

| 话题 | 用途 |
|---|---|
| 相机 RGB、Depth、CameraInfo | 近似同步的原始 RGB-D |
| `/peach/perception/initial_pose` | 候选类型、初始位姿和三态质量 |
| `/peach/perception/target_observations` | selected ID、跟踪状态和逐目标掩膜 |
| `/peach/perception/diagnostics` | 感知拟合质量 |
| `/joint_states` | 精确图像时刻 FK 输入 |

### 发布话题

| 话题 | 内容 |
|---|---|
| `/peach/reconstruction/status` | IDLE、COLLECTING、READY 与帧数摘要 |
| `/peach/reconstruction/diagnostics` | JSON 逐帧门控、目标绑定、FK/ICP、TSDF 与 `view_coverage` 诊断 |
| `/peach/reconstruction/grasp_decision` | JSON 抓取许可和拒绝原因 |
| `/peach/reconstruction/local_cloud` | 当前掩膜局部点云 |
| `/peach/reconstruction/tsdf_cloud` | 在线融合点云 |
| `/peach/reconstruction/refined_pose` | finalize 后精化位姿 |
| `/peach/reconstruction/refined_axis` | finalize 后精化主轴 |
| `/peach/reconstruction/refined_diagnostics` | 最终球体/圆柱拟合质量 |
| `/peach/reconstruction/markers` | RViz 相机轨迹、几何和轴线 |

上述发布器使用 transient-local，RViz 或任务管理器后加入时也能收到最近状态。

### 服务

| 服务 | 作用 |
|---|---|
| `/peach_reconstruction_node/start_reconstruction` | 手动从 IDLE 开始会话 |
| `/peach_reconstruction_node/capture_frame` | 手动采集当前同步帧 |
| `/peach_reconstruction_node/remove_last_frame` | 删除最后合格帧并重放 TSDF |
| `/peach_reconstruction_node/reset_reconstruction` | 清除当前会话并回到 IDLE |
| `/peach_reconstruction_node/finalize_reconstruction` | 检查覆盖、提取模型并输出 refined 结果 |
| `/peach_reconstruction_node/save_session` | 保存完整可复现 session |
| `/peach_reconstruction_node/query_reconstruction_state` | 查询状态、绑定 ID、帧数和运行数据路径 |

### RViz 查看

Fixed Frame 设为 `base_link`。同时添加 `local_cloud` 和 `tsdf_cloud`：前者检查
单帧掩膜是否串入背景或邻近目标，后者检查融合是否逐渐完整、有无双层和拖影；
再添加两包的 MarkerArray，对照初始 ID 与 refined 几何。TF 显示应能看到
`base_link → wrist3_Link → camera_link`。点云为空时先查状态和 diagnostics，
不要仅凭 RViz 判断算法没有运行。

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
