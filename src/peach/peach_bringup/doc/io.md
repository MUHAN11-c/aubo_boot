# Peach 系统输入输出

本页只记录跨能力边界与共享设备契约。字段、图名、生产消费方及QoS以 [interface_manifest.yaml](../../peach_interfaces/config/interface_manifest.yaml) 与IDL为准。能力包概述见 [scene](../../peach_scene_perception/README.md)、[reconstruction](../../peach_target_reconstruction/README.md)、[supervisor](../../peach_supervisor/README.md)；包内接口明细见 [arm](../../peach_arm/doc/io.md)、[observability](../../peach_observability/doc/io.md)、[vegetation](../../peach_vegetation/doc/io.md)。系统结构见 [architecture.md](architecture.md)。

## 批次调用顺序

唯一开批action为 `/peach_supervisor/run_harvest`。调度发SurveyScene，拍照位确认成功后BeginScene；感知锁定的target_observations提供选果集合；CheckReachability只查询IK/路径条件，不运动。BuildTargetModel与OBSERVE_ONLY并行，随后按当前策略派PREGRASP_ONLY或FULL。PREGRASP_ONLY成功仅表示预抓取验证；是否允许下一目标由批次策略决定，异常恢复必须人工ACK，前端与runner不得自动确认。ControlTask负责暂停/跳过/取消/ACK，Nav2生命周期PAUSE与批次暂停不同。

感知initial_pose只给重建；调度不据此选果。GraspDecision由重建发布，调度缓存clearance用于goal，臂侧再次校验目标、模型修订、时效和能力。allowed只是汇总观测，网页显示不授予运动权。ledger由调度写；臂不写账本、不调重建Trigger。

SetEnables由supervisor接收并发布 `/peach/batch/enables`；臂收到广播后按心跳及本地回落规则执行。依赖execution→grasp→tool，各层默认关闭；前端或HTTP受理不能替代最后写硬件前的ExecutionAuthority。

调度 `HarvestState` 的 execution/grasp/tool 开关均优先投影操作台 override；没有 override 时沿用本地参数/动作反馈。臂侧 `~/status` 每次发布都重新读取三项实际使能，包含 `tool_enabled`。Web 写入后同时核对调度意图与臂侧状态；广播受理不等于臂已应用。

## 生命周期

自动启动参数拒绝写入时，ROS 参数服务读值与发批缓存一致；不因 GPL 回调顺序使用被拒绝的负 `intent`。`startup_facts.git.branch` 保留完整分支路径，`commit` 为当前引用的 12 位提交号（兼容 packed refs）；无法读取身份时仍明确标为 unknown。

autostart `intent` 只接受 RunHarvest 定义的 PICK_ALL=0、PICK_SELECTED=1、SURVEY_ONLY=2；启动 override 与动态写入均使用同一集合，非法动态写入不改变 ROS 值或发批缓存。

`/peach_lifecycle_manager/manage_nodes` 使用 nav2_msgs/ManageLifecycleNodes。bringup 将 `is_active` 的结果发布至 `/peach/lifecycle/managed_nodes_activated`（RELIABLE、TRANSIENT_LOCAL、depth1）。服务下线、失败或超过两个轮询周期（至少 1 秒）返回时发布 false。自动开批采用最近一次托管状态，`wait_stack_timeout_s` 涵盖等待托管栈、自检和 RunHarvest 服务端的总时间。默认名单为感知→重建→臂→调度；关闭重建进程时从名单移除。四节点接 bond，bond_timeout 默认 0 关闭进程死检。observability、scene_obstacles 和相机在名单外。

## 相机与TF

camera_enabled默认false；camera_frontend默认stereo，可选percipio或replay。该参数只在launch期选择前端，禁止运行期热切：两驱动争同一GigE并双发`/camera/*`。stereo/percipio互斥；预检拒绝残留`stereo_camera_node`/`component_container`/`camera_container`。replay不起驱动。percipio时装入`peach_scene_perception/config/percipio_frontend.yaml`（`max_frame_age_s=2.5`），避免18图案一帧0.4–1s被默认0.50年龄门丢掉。候选袋轴、袋底/袋颈与入口旋转保持同一曝光的几何一致性；身份EMA不覆写候选方向，也不替代时间稳定性检查。共同话题：

- `/camera/color/image_raw`：彩色图。stereo 与官方驱动同厂参 `TYUndistortImage`（棋盘格对着官方去畸变彩图）。
- `/camera/depth/image_raw`：配准深度。stereo 再按彩色厂参最近邻 remap 到与彩图同一网格。
- `/camera/color/camera_info`、`/camera/depth/camera_info`：对应内参（棋盘格 yaml 覆盖）。
- `/camera/depth_registered/points`：彩色配准点云；stereo额外confidence字段，消费方按字段名读取。

本机uint16深度单位为0.25mm，边界按depth_scale_unit换算；32FC1按米处理，不套uint16倍率。不能把uint16类型本身当作毫米保证。感知/重建RGB-D同步slop为0.05s，具体QoS读部署源码与清单；探针使用与发布端兼容的QoS。stereo发布未压缩 Image（话题名仍是 image_raw），不自行再造压缩契约。

URDF负责固定工具链。腕随动时单一`extrinsics_publisher`发布`wrist3_Link→camera_link`；`camera_enabled:=true`且`extrinsics_enabled:=false`时改由`fixed_camera_mount_publisher`发布`base_link→camera_link`（`config/fixed_camera_mount.yaml`，2026-10-13 采集位）。两路互斥，禁止叠同一 child。重建用图像精确stamp TF，latest仅用于诊断并显示陈旧。真实相机物理固定而mock腕移动时必须走固定采集位，不使用模拟腕运动产生新视点。use_sim_time只在统一模拟时钟图中启用，真机禁止误开。

工具档默认bite_shear_v1，shear_v1/adaptive_shear_v1显式选择；aubo_description档案定义tool_axis、sleeve_mouth、cutting_plane及tcp。停位几何由peach_common/config/grasp_standoffs.yaml注入感知/重建等使用方，不各包复制。

## 驱动、场景与外设

部署参数 `move_group_shutdown_workaround` 默认 true，记录在 observability 的 startup facts；只给 `move_group` 进程预加载其默认 capability 库以避免 Jazzy 退出析构崩溃，不改 ROS 图/API、使能或退出码。显式 false 关闭；作用范围与上游问题见 [architecture.md](architecture.md)。

应用Include只读aubo_e5_bringup；mock使用GenericSystem+标准JTC，real使用厂商硬件/透传控制器。柜状态/SetIO使用aubo_msgs，规划/FK/IK使用MoveIt。关节序冻结为shoulder_joint、upperArm_joint、foreArm_joint、wrist1_joint、wrist2_joint、wrist3_joint。不开dashboard、不远程上电；工具IO只能经臂侧门。硬件急停不经ROS。

`/joint_states` 由应用配置的 `joint_state_broadcaster` 按上述顺序发布，名称与对应的
position/velocity/effort 数组保持成对；未提供的 effort 沿用官方广播器的 NaN。
`/dynamic_joint_states` 由无接口过滤的 `dynamic_joint_state_broadcaster` 发布，保留全部
硬件状态。该额外实例的 JointState 输出仅位于 `/dynamic_joint_state_broadcaster/joint_states`，
应用仍只消费公共 `/joint_states`，消费时继续按名称关联，不能把任意 JointState 的数组下标当轴号。

scene_obstacles在Survey后按刷新触发生成快照，仅写PlanningScene world.collision_objects；arm仅写ACM diff，不能相互覆盖分域。该快照保护相机，不是安全扫描器。vegetation仅出二维掩膜，不写PlanningScene。serial_force按开关随栈（只出力传感器，IMU 已删）。IVG/仿真场景独立运行，不能共占设备/域；导航与peach2归档不作为本栈活动入口。

## 观测与验证

五点力新增 `/force/sample`（`peach_interfaces/ForceSample`，RELIABLE/VOLATILE/depth100）
原子帧，旧四 `/force/{points,travel,ad,status}` 保持兼容。完整帧含源进程 UUID、序号、
主机 ROS 接收/解码时间与五路数值/状态；无固件采样时刻保证。observability 的全部录制档
保留原子帧，不走相机媒体节流；observability 只用完整帧形成力页聚合状态。当前力页只读，没有自动分段或手动 20 秒写者；历史接触段 sidecar 仅兼容读取。

`/diagnostics`提供标准健康流，observability聚合到8090；状态/事件带作业身份，会话bag位于runs/session_*，每批ledger在runs/<request_id>。静态页面属于peach_web。调试POST委托现有ROS入口，默认运动调试关闭；不得新增SetIO或旁路门。

排障用 ROS 2 日志：rqt_console 过滤 `peach_supervisor.batch` / `peach_supervisor.contact` / `peach_arm.cycle` / `peach_arm.contact`；目录 `$ROS_HOME/log`（联测 `runs/<本轮>/ros_home/log`）。SELECT 与 Finalize 各打一条 `caller=`，行程不同是预期。读法见 [architecture.md 一批怎么走](architecture.md) 与 [测试指导 §10](../../peach_system_tests/doc/testing.md)。

修改IDL/图名后先构建interfaces及消费包，再运行：

```bash
python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py
```

接线验证及bag封口/精确PID清理见 [测试指导](../../peach_system_tests/doc/testing.md)。清单通过只证明静态一致，节点集成、许可身份与实际终态仍须独立验收。

联合runner的report.json按阶段原子更新，soak只在RViz存活及相机/关节新鲜流成立时计为健康，失效立即终止验收并清理。自动bag报告子进程继承本轮进程组，有界30+2+2秒回收，超时/非零须离线重算；bag可读不代表动作或稳定性验收成功。相机SDK发现需注明网络环境：隔离网络的0设备与主机网络可发现并不矛盾，实际取帧及模型/动作仍须独立证明。

参数装配优先级：包YAML与工具/停位注入 → 显式legacy overlay → 共用 `task_params_file` → supervisor/arm 显式 launch policy。各子 launch 作用域独立，不共享 `params_file` 或覆盖父级 `autostart`；父级全局时钟和启动事实仍被转发。`reconstruction_enabled` 是 composition，不进节点参数；`skip_reconstruction` 省略时 mock 默认 true、real 默认 false，显式传参优先。`skip_reconstruction`/`allow_unrefined_geometry` 是 launch/ops policy，任务YAML含这些键会启动期硬拒绝。`capture.auto_mode` 是重建节点本地运行期参数；组合启动与操作台显式按 !skip 设置，三项读回一致才放行。硬件反馈要求与固定真实相机TF属于环境适配，不通过此文件改业务门。观测startup记录 `reconstruction_enabled`/实际skip；runner另落实际参数读回。

整栈将 `rosbridge_enabled` 和 `rosbridge_port` 写入启动事实。`GET /api/transport` 提供默认传输、WS 地址、参数面板和端点映射。启用桥时默认 rosbridge，关闭桥或独立启动 observability 时默认 HTTP。`/peach_observability/web_{state,force,trajectory}` 发布 std_msgs/msg/String JSON（RELIABLE、VOLATILE、depth1），有订阅者时状态 1 Hz、力和轨迹 4 Hz。写操作经原生 ROS 参数服务、业务 Service 和 Action。8090 提供同源 SDK、图像和历史文件，也可用 `?transport=http` 显式选择 HTTP。

操作员 SetEnables、接受的 ControlTask 和成功 ACK 会强制广播最新 HarvestState/state_seq；执行反馈仍限频。应用限频不能依靠 transient_local 或 RELIABLE 补发被抑制的消息。网页 ACK 先结束旧批次，核对终态 6/8 与双侧空闲，再以当前序号请求恢复，避免旧 PICK_ALL 继续派发。
