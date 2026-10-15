# Peach 系统输入输出

本页只记录跨能力边界与共享设备契约。字段、图名、生产消费方及QoS以 [interface_manifest.yaml](../../peach_interfaces/config/interface_manifest.yaml) 与IDL为准；包内明细分别见 [harvester](../../peach_harvester/doc/io.md)、[arm](../../peach_arm/doc/io.md)、[observability](../../peach_observability/doc/io.md)、[vegetation](../../peach_vegetation/doc/io.md)。系统结构见 [architecture.md](architecture.md)。

## 批次调用顺序

唯一开批action为 `/peach_supervisor/run_harvest`。调度发SurveyScene，拍照位确认成功后BeginScene；感知锁定的target_observations提供选果集合；CheckReachability只查询IK/路径条件，不运动。BuildTargetModel与OBSERVE_ONLY并行，随后按当前策略派PREGRASP_ONLY或FULL。PREGRASP_ONLY到位等待人工ACK，不能自动继续下一颗。ControlTask负责暂停/跳过/取消/ACK，Nav2生命周期PAUSE与批次暂停不同。

感知initial_pose只给重建；调度不据此选果。GraspDecision由重建发布，调度缓存clearance用于goal，臂侧再次校验目标、模型修订、时效和能力。allowed只是汇总观测，网页显示不授予运动权。ledger由调度写；臂不写账本、不调重建Trigger。

SetEnables由supervisor接收并发布 `/peach/batch/enables`；臂收到广播后按心跳及本地回落规则执行。依赖execution→grasp→tool，各层默认关闭；前端或HTTP受理不能替代最后写硬件前的ExecutionAuthority。

## 生命周期

`/peach_lifecycle_manager/manage_nodes` 使用nav2_msgs/ManageLifecycleNodes；`is_active`查询由bringup桥接至 `/peach/lifecycle/managed_nodes_activated`（RELIABLE、TRANSIENT_LOCAL、depth1）。名单为感知→重建→臂→调度。四节点接bond，bond_timeout默认0关闭进程死检。observability/scene_obstacles/相机/IMU不进该名单。

## 相机与TF

camera_enabled默认false；camera_frontend默认stereo，可选percipio或replay。stereo/percipio互斥连接同一相机；replay不起驱动。共同话题：

- `/camera/color/image_raw`：彩色图。
- `/camera/depth/image_raw`：配准深度。
- `/camera/color/camera_info`、`/camera/depth/camera_info`：对应内参。
- `/camera/depth_registered/points`：彩色配准点云；stereo额外confidence字段，消费方按字段名读取。

本机uint16深度单位为0.25mm，边界按depth_scale_unit换算；32FC1按米处理，不套uint16倍率。不能把uint16类型本身当作毫米保证。感知/重建RGB-D同步slop为0.05s，具体QoS读部署源码与清单；探针使用与发布端兼容的QoS。stereo发布raw图像，不自行再造压缩契约。

URDF负责固定工具链；单一extrinsics_publisher发布腕到相机外参，禁止重复child frame。重建用图像精确stamp TF，latest仅用于诊断并显示陈旧。真实相机物理固定而mock腕移动时，必须固定采集位锚定，不使用模拟腕运动产生新视点。use_sim_time只在统一模拟时钟图中启用，真机禁止误开。

工具档默认bite_shear_v1，shear_v1/adaptive_shear_v1显式选择；aubo_description档案定义tool_axis、sleeve_mouth、cutting_plane及tcp。停位几何由harvester/config/grasp_standoffs.yaml注入感知/重建等使用方，不各包复制。

## 驱动、场景与外设

应用Include只读aubo_e5_bringup；mock使用GenericSystem+标准JTC，real使用厂商硬件/透传控制器。柜状态/SetIO使用aubo_msgs，规划/FK/IK使用MoveIt。关节序冻结为shoulder_joint、upperArm_joint、foreArm_joint、wrist1_joint、wrist2_joint、wrist3_joint。不开dashboard、不远程上电；工具IO只能经臂侧门。硬件急停不经ROS。

scene_obstacles在Survey后按刷新触发生成快照，仅写PlanningScene world.collision_objects；arm仅写ACM diff，不能相互覆盖分域。该快照保护相机，不是安全扫描器。vegetation仅出二维掩膜，不写PlanningScene。serial_imu按开关随栈；imu_follow仅adaptive档Include，motion默认false，启用真机运动另授权且不可与MTC同时写控制器。IVG/仿真场景独立运行，不能共占设备/域；导航与peach2归档不作为本栈活动入口。

## 观测与验证

`/diagnostics`提供标准健康流，observability聚合到8090；状态/事件带作业身份，会话bag位于runs/session_*，每批ledger在runs/<request_id>。静态页面属于peach_web。调试POST委托现有ROS入口，默认运动调试关闭；不得新增SetIO或旁路门。

修改IDL/图名后先构建interfaces及消费包，再运行：

```bash
python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py
```

接线验证及bag封口/精确PID清理见 [测试指导](../../peach_system_tests/doc/testing.md)。清单通过只证明静态一致，节点集成、许可身份与实际终态仍须独立验收。
