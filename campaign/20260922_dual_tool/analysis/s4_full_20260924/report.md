# S4 完整流程轮全方位分析（2026-09-24，真相机 stereo + mock 臂）

执行纪律（用户 09-24）：完整感知→抓取全流程（不停在预抓取）；单轮 ≤5 分钟；
全量数据但袋不胀；RViz 视频随轮；监测（数据流必须图像+点云 / 停驻 / 内存 /
日志巡检 / 45s 截图）；停-分析-再开须全关；袋超限删旧只留分析+重点图。

## 1. 轮次与结果

| 轮 | request_id | 结果 | 关键数据 |
|----|-----------|------|----------|
| 1 | e2e_full_unrefined_20260924T165422 | **empty_limit 零派发**（25.3s completed） | `targets_filtered: target_1 → ik_no_solution:sleeve_no_ik`；Survey→锁定链正常（photo_pose_reached→round_locked×2） |
| 2 | e2e_full_unrefined_20260924T171821 | empty_limit 零派发（20.6s，travel 上限 0.08 批时翻转失败） | 同上过滤码 sleeve_no_cartesian |
| 3 | e2e_full_unrefined_20260924T171955 | **FULL 派发成功**：reconfirm 0.36s+approach_insert 9.3s→停驻等 ACK→外因拆栈取消（full_failed） | SELECT 过（travel 0.05）；completion=2 |
| 4 | e2e_full_unrefined_20260924T172313 | 同 3：approach 9.3s 停驻等 ACK，150s 超时取消 | progress 1.0/recovery_required/permissions[4,6]/`5:ready_full` |
| 5 | e2e_full_unrefined_20260924T172716 | 同 3（9.85s，驱动被打断） | 同上 |
| 6 | 172920（终局补发） | 通信失败：**406 个 fastrtps SHM 残留段锁死 port7000**（多轮 kill -9 后遗症） | 预检/收尾清 /dev/shm/fastrtps_* 后恢复 |

**链条证明**：真实感知→YOLO（conf 0.85-0.87）→3D 候选（entry/axis/suggested_travel 0.10m）→锁集→SELECT（FULL 档套入预检）→ExecuteTarget FULL 派发→reconfirm→approach_insert 9.3s 到位→**停驻等操作员 ACK（recovery_required，命令 6）才续套入**——第 3/4/5 轮均停在此检查点，未续因为驱动在 ACK 前被打断（用户叫停/超时取消连带 SIGINT 掉 goal）。

## 2. 根因与边界数据（CheckReachability 原样位姿探测）

- 袋口 entry=(0.578,-0.731,0.543) base_link，**|p|=1.079m**，轴≈(0.23,0.27,0.71)。
- 原位、拍照位种子：travel 0.102/0.08 **0/6 稳定败**（sleeve_no_cartesian / 批时 sleeve_no_ik）；**travel 0.05/0.03/0.02 各 6/6 全过**。
- 入口向基座径向移 **5cm（|p|=1.029）**：满行程 0.1024 IK+笛卡尔全过；10cm 同。
- 结论：**当前物理摆位处于 E5 套入终点可达边界**。满插入深度的完整套入需袋口再近 ~5cm（现场动作）；0.05m 行程档确定性可达（本轮 3/4/5 已用）。

## 3. 停在预抓取的机理（用户问）

FULL 周期在 approach_insert 完成后进入**接触前人工确认停驻**（recovery_required=true，
permissions 含 6=ACKNOWLEDGE_RECOVERY，transaction `5:ready_full`），等 `ControlTask{command:6}`
放行才沿轴套入→撤离→stow。与 PREGRASP 档同一检查点（S3 即 ACK 后收口）。自动化驱动必须
带 ACK 步骤——本轮 3 次未收口均为 ACK 前被外部取消，非系统故障。

## 4. 修复与优化落地（工作区，未提交）

| # | 项 | 位置 |
|---|----|------|
| 1 | **F2 UAF**：executeBlendedCorridor `[&result_future]` 悬垂引用改按值捕获 | grasp_task.cpp:767 |
| 2 | **录制节食**：控制流族补齐（statistics/controller_state/dynamic_joint_states 20Hz）+ PlanningScene 族 1Hz + 感知派生族（debug_image/raw/masks/target_observations）2Hz | recorder.py（12min 8.96G→预计 <2G） |
| 3 | pep257 D213 一处（历史欠账顺修）；包测 41 全绿 | pipeline.py |
| 4 | 停驻监测口径修正：逐消息差分（100Hz 恒小）改采样窗参考快照 | /tmp/s4mon/monitor.py（会话级） |

## 5. 数据与体积

- 袋构成实测（12min/8.96G）：**感知派生 79%**（debug_image 3.03G+raw 1.93G+target_observations 1.18G+masks 0.89G）；控制流族 ~1.1G（旧 O1 未盖住 statistics/controller_state/dynamic_joint_states）；相机 raw 0.77G（std 1Hz 生效）。
- runs/ 14.1G→**73M**（6 个 session mcap 全清，证据=各轮 ledger/perception_data/rvizwin.mp4+本报告+shots×4）。
- 监测全程运行：数据流（图像 0.6-6Hz 随 blender 并发负载、点云 0.1-0.3Hz 订户在才有）、内存峰值可用 11-15G 无风险、bag 守门 6G 告警 1 次、日志巡检命中已知良性 2 条。

## 6. 遗留与下一步

1. **收口一轮完整 FULL**：起栈→SetEnables→pregrasp_only=false→travel 0.05→RunHarvest→**停驻出现即 ControlTask 命令 6**→套入→撤离→stow→SetEnables 全关→停栈。SHM 已清，预计批次 <60s、全程 <4min。满行程（0.10m）版需现场把袋口向基座挪 ~5cm。
2. observability 停栈挂死（-9 后 bag_report 缺失）再现×2——O3 根因仍未明。
3. 使能 1Hz 心跳致 peach_arm 每秒一行 INFO 刷屏（建议降 DEBUG 或去重）。
4. HarvestState.grasp_enabled 是臂侧反馈镜像，无周期时恒 false——观测语义易误读（本次曾误导排查）。
5. teardown 后 406 个 fastrtps SHM 段残留锁死新参与者——拆栈脚本应加 /dev/shm/fastrtps_* 清理步。
