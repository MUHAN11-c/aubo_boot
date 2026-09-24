# S3 真相机轮全方位分析（2026-09-24，裁定⑦ 真相机先行首落地）

执行纪律（用户 09-24）：真相机（peach_stereo 前端，不碰 percipio 官方驱动）+ mock 控制；bag 全量 + RViz 视频随轮；定时截图 + 日志监测修复闭环；**臂不留停驻**（ACK 及时）；停止分析须**全关→分析完→再开**；录制跟随程序开关；**50G 预算**超限删旧，保留分析结果+重点图片；监控内存防崩。

## 1. 轮次与结果

| 轮 | request_id | 内容 | 结果 |
|----|-----------|------|------|
| 拍照位自洽 | e2e_survey_20260924T1446 | intent:2 Survey | 首发即拒（survey_failed）→ **SetEnables 契约**（live 同 E1）→ 重发 **SUCCEEDED 16.9s** |
| 主轮 | e2e_full_unrefined_20260924T1451 | intent:0 真实感知选果→PREGRASP 干跑 | **全链首通**：锁定 target_1→派发→接近→停驻 RECOVERY_REQUIRED(7)→ACK→**COMPLETED 结算**；周期 **9.4s**（reconfirm→approach_insert，completion=2，无 SetIO） |

取证四件套：session bag 26G（report 后删）/ rvizwin.mp4 ×2（16.5+11.3MB）/ 重点截图 4 张（本目录 shots/）/ ledger+perception_data（7076 事件）。

## 2. 实时监测运行记录（本轮新基建）

- **日志监测器**：tail -F 过滤 ERROR/died/Exception → alerts；实况 11 条 ERROR 全部为已知良性（move_group 无 3D sensor 插件、rviz recognize_objects 缺省）；**零进程死亡**。
- **定时截图**：45s/张循环 25 张；抓到检测画面/拍照位点云/停驻/结算四个关键相变。
- **内存守门**：总量 30G、可用 21G、峰值进程 1.04GB（observability 录制）——无 OOM 风险。

## 3. 勘定与发现（基于数据）

| # | 发现 | 数据 | 判定/优化项 |
|---|------|------|------------|
| 1 | **E2E 完全感知抓取链真机级首通** | 周期 9.4s、SUCCEEDED、无 hang | ✅ 端到端成立（真实相机→YOLO 0.84→3D 拟合→选果→接近→停驻） |
| 2 | **点云按需发布**：`stereo_camera_node.cpp:624` 无订阅者跳过生成 | 首测 0Hz，订阅后 0.75Hz | 监测/验收工具必须**先订阅再读数**；RViz/catch-all 订阅即触发 |
| 3 | **深度 0.37Hz vs 设计 2.43Hz、color 9.6s 缺口** | topic hz 多窗实测 | ⚠ 优化项 O1：嫌疑=observability all 档（83.7% 单核）+感知推理争抢；下轮 A/B：all→std 对照帧率 |
| 4 | **感知 worker 丢帧 3041 次**（13.7fps 输入 vs 3-6fps 推理） | WARN 计数 | drop_oldest 设计行为（实况预期）；O2：推理提速或降采样输入可再降延迟 |
| 5 | controller_manager **Overrun ×6**（一次 42.7s） | WARN | 负载尖峰（模型加载/录制）——mock 实时性参考值，真机无此担扰（透传不受 CM 周期约束同程度） |
| 6 | **observability 停栈挂死再现**（SIGTERM 窗不退需 -9） | 09-22 起第三次 | O3：根因仍未明——本轮代价=bag_report 原子写没赶上（离线补跑成功）；建议给 observability 加 watchdog 强退 |
| 7 | **孤儿 lifecycle_manager 挡预检** | 前栈残留 1803326 | O4：拆栈脚本需按名单强清孤儿（本次按预检提示 -9） |
| 8 | SetEnables 契约 live 复现 | survey_failed→使能后通 | 与 E1 勘定一致（Survey=TRANSIT 需 execution）——已入 runbook |
| 9 | RViz 前台争夺（截图拍到 IDE） | 截图证据 | O5：无 wmctrl/xdotool，用「杀-重启 RViz」提窗；建议 apt 装wmctrl |

## 4. 50G 预算执行

runs/ 87G→**26G**（删除 09-22/09-23/09-24 注入矩阵已分析 session，其证据均在已提交的 jsonl+campaign analysis）；主 S3 袋 26G 待 bag_report 完成后删除，仅留本报告+shots/+ledger+perception_data+mp4。

## 5. 后续

- O1 录制档位 A/B（all vs std 帧率对照）→ 决定 S3 常规档
- O3 observability 挂死根因（附 watchdog）
- S3 数据支撑 E1 二期：C25–C28 闪烁/消失用例的真实参数标定（本轮 debug_image 已见真实检测帧序列）
- 真机门前置：本轮全部 mock 控制、tool 恒关，无授权运动

## 6. 50G 预算执行结果（v2 补记）

- runs/ 87G→26G→**33MB**：旧 session 全删→bag_report **两次尝试均失败**（第一次脚本 500s 超时误杀；第二次按用户报告**内存占满闪退**——26G 全量袋全反序列化超出 bag_report 设计，**O6：bag_report 需流式/分块重设计，>几 GB 的袋禁跑**）。
- 替代：`ros2 bag info` 元数据提取（零反序列化秒出）→ `bag_info.txt`（26G 构成=joint_states 413,297 条+CM introspection/statistics ~20 万条×3+dynamic_joint_states 199,801 条+相机 raw——高频控制流主导，非感知数据）。
- 保留：bag_info.txt / report.md / shots×4 / ledger×2 / perception_data(7076 事件) / mp4×2。**O1 修正**：all 档 26G 大头是控制流高频话题而非相机——录制优化应从「control/CM introspection 高频族限频或隔离」入手，而非只限相机。
