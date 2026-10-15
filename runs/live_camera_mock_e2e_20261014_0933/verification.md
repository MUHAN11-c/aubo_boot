# 真实相机 + mock 臂 FULL 端到端验证

2026-10-14，北京时间。**完整抓取验收未通过：两轮各发现 1 个目标、尝试 1 个、成功 0 个。**

## 本轮实际运行

- 真实 PS800-E1 `207000152740`，IP `169.254.10.110`，stereo 前端；相机日志约 10.9–11.8 Hz。
- AUBO E5 使用 `mock_components/GenericSystem` + 标准 JTC，MoveIt 与 RViz；不是 Gazebo 接触物理仿真。
- 独立 DDS 域 81，`use_sim_time=false`，工具 `bite_shear_v1`，重建未跳过，`tool=false`。
- 固定相机用前日记录的 `base_link → camera_link` 静态外参，原腕外参关闭。该变换已读取核对，本轮没有重新实测相机物理外参。
- 产品工具内径 104 mm、最大行程 200 mm；未加载会放宽工具尺寸的 frozen overlay。场景障碍护栏是产品默认关闭状态，本轮未验证枝叶碰撞阻挡。
- 未执行真实机械臂运动或 SetIO；未修改业务源码及驱动源码。

## 两轮结果

1. `live_camera_mock_full_product_20261014_0936`：39.6 秒，`result.success=false`，`no_targets_succeeded`。重建约 2.15 秒才进入 COLLECTING，超过产品 2 秒启动等待；账本记录 `build_start_timeout`。未进入接触执行。
2. `live_camera_mock_full_fixed_20261014_0938`：32.2 秒，`result.success=false`，`no_targets_succeeded`。诊断配置只把重建启动等待改为 8 秒、`reconstruct_in_trajectory=true`，避开固定相机无法产生的新机位；整轮仍限 180 秒。重建完成 1 视、2409 点、TSDF 938 点、mesh 1641 顶点，refit ACCEPT。臂侧拒绝 `GraspDecision.allowed=false`，账本为 `SKIPPED_QUALITY`，completion_level=0。

第二轮接触拒绝的具体几何原因是 `cut_plane_fruit_clearance`。袋模型直径 106.3 mm，融合位置 sigma 39.8 mm，径向余量 -103.5 mm，轴向余量 -2 mm；有 `cut_band_unavailable` 和 `keypoint_cloud_axis_conflict` 标记。`cut_to_fruit_m=0.0` 是当前模型输出，不能视为独立测得的果实/刀口净空。

Action `STATUS_SUCCEEDED` 表示批次完成结算。两轮的业务 success 均为 false，成功计数均为 0，不能据 action 状态宣称抓取通过。

## 覆盖边界

已执行：真实 RGB-D 出图、扫场、目标锁定与选择、重建动作、单视融合与拟合、接触许可拒绝、批次结算与账本落盘。

未执行：目标预抓取、套入、剪切 IO、撤退；未验证果梗剪断或果实脱离。质量门提前拒绝，后续阶段没有通过证据。

初次自检 14/18 通过，robot_status、IMU、权重清单、vegetation 四项跳过。末次自检出现 move_group 查询失败，不能宣称全时段自检通过。RViz 首次缺 DISPLAY、单独启动又缺 SRDF；随后使用桌面环境和原 launch 参数恢复显示。

## 录制与重现证据

会话为 `../session_20261014_093504/bag`：1.2 GiB、254.706 秒、53643 条消息。录制未正常封口，用 `ros2 bag reindex` 恢复 metadata 后 `ros2 bag info` 可读；原 MCAP 缺完整尾部/消息索引，完整性仍有此限制。

各大话题独立限流，首个 RGB 的深度偏差 -66.718 ms、点云偏差 +333.593 ms，首帧不满足严格 <50 ms 同步，不能拿 index 0 作冻结输入。完整只读扫描找到 1 组满足同步的录制样本：**RGB index 46**，RGB/深度/点云原始 stamp 均为 `1791941773802610000`，偏差 0 ms；现有 `load_frame` 同时验证图像尺寸、光学 frame 与注册深度内参一致。该组在固定 TF 发布之后采集，可作为后续明确指定 index 的离线输入；本轮没有对它另外发回放抓取。

## 收尾及附带问题

SetEnables 返回 accepted=true；臂参数 `execution.enabled/grasp.enabled/tool.enabled` 全为 false，臂日志确认广播 `execution=0 grasp=0 tool=0`。Web `task_executor.grasp_enabled` 仍显示旧值，不能用该展示字段覆盖臂侧证据。

`cleanup.json` 核对域 81 无剩余进程，8090 已关闭。用户另外运行的仿真及 `ucvcameraservice` 未终止。

现有 `probe_topics.py` 对同一话题同时订阅 Image/PointCloud2，在本机 Jazzy 报 incompatible type；另一次标准 CLI 探针因不支持 `--qos-reliability` 参数退出。相机流证据来自相机日志、自检与实际录制，不把失败探针算作通过。

## 证据入口

- [结构化结果](test_result.json)
- [产品 FULL 输出](product_full.log)、[固定相机诊断 FULL 输出](fixed_full.log)
- [产品账本](../live_camera_mock_full_product_20261014_0936/ledger.json)、[诊断账本](../live_camera_mock_full_fixed_20261014_0938/ledger.json)
- [接触几何记录](../live_camera_mock_full_fixed_20261014_0938/geometry.jsonl)
- [录制同步组审计](rgbd_recorded_groups.json)、[首帧同步审计](rgbd_sync_audit.json)
- [bag 信息](bag_info.txt)、[索引恢复记录](bag_reindex.log)、[清理核对](cleanup.json)
- [真实相机识别画面](perception.jpg)、[launch 日志](launch.log)

下一步应先解决重建启动的时间预算/调度拥塞，以及外袋模型对剪切净空的证据不足，再进行同配置 FULL 复验。扩大工具尺寸或跳过质量门不能用来证明产品抓取成功。
