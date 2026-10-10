# test/ —— peach_stereo vs percipio_camera 端到端对比（精简测试档案）

2026-09-20/21 相机双前端对比。**09-21 重录终版**（录制链含 composite 全尺寸窗+自检门，视频**检测框/分割/标注清晰可见**；节点含 confidence 偏移修复）。
**入口文档**：[analysis/percipio_vs_peach_stereo.md](analysis/percipio_vs_peach_stereo.md)（函数级+流程级对比分析）· [report/report.md](report/report.md)（**实测终版报告**，含 target_1 深度答复）· [analysis/peach_project_impact.md](analysis/peach_project_impact.md)（**peach 项目影响评估·深度版**：消费者地图/帧率逐层兑现/真实门限/真机验证矩阵/切换硬门槛）。

## 目录

```
test/
├── README.md                            # 本索引
├── analysis/
│   ├── percipio_vs_peach_stereo.md      # ★ 函数级·流程级对比分析 + 缺陷清单 + 选型结论
│   └── peach_project_impact.md          # ★ peach 项目影响评估（集成面/收益/切换清单/运维风险）
├── report/                              # 终版实测（本轮 145/146 帧）
│   ├── report.md                        # ★ 实测报告（感知/点云质量/target_1/录制链修复/复现）
│   ├── fig_f_compare.mp4                # ★ 30s 对比视频（上=composite 三联全尺寸，下=rviz 3D）
│   ├── {hh4,percipio2}_rvizwin.mp4      # 30s rviz 3D 源视频（composite 源视频未随精简归档保留——
│   │                                    #   fig_f_compare.mp4 即其合成终版，重录经 record_session.sh 再生成）
│   ├── fig_f_poster.png                 # 对比视频海报帧
│   ├── fig_cyl.png / fig_pcq.png        # ★ 圆柱拟合 / 点云质量
│   ├── fig_branch_depth.png / fig_branch_overlay.png  # ★ 避障四档对比（枝上深度 JET / 植被叠加）
│   ├── branch/                          # 四档枝分析代表图+逐帧日志（p1 现行/p2 med0tk1/p3 全分辨率/p4 percipio）
│   ├── fig_a_debug.png / fig_b_traces.png / fig_d_metrics.png / fig_e_rviz.png
├── scripts/                             # 可复现集（运行时中转 /tmp/e2e_live；脚本自定位）
│   ├── record_session.sh                # ★ 一条链录制（自检门+双窗 30s+收尾复核）
│   ├── check_frame.py                   # ★ 录制自检门（框/掩膜像素计数）
│   ├── run_analysis.sh                  # ★ 分析总跑器（统计+全部图件）
│   ├── collect.py / run_collect.sh      # A/B 采集器（E2E_GUI=1 开 composite 全尺寸窗）
│   ├── analyze.py / deep_analysis.py / make_fig_*.py / make_fig_f.sh
│   ├── run_percipio.sh（务必 depth_resolution:=640x480）/ run_camera.sh / run_rviz_fixed.sh
│   ├── verify_stack.py                  # 重启验证（30 帧深度健康+点云字段字节级校验）
│   ├── branch_analysis.py               # 枝掩膜×深度质量分析（订 color/depth/branch_mask，避障口径
│   │                                    #   cov/detail_mm/mad；SUMMARY 打 stdout 须落盘，图进 /tmp/e2e_live/branch/）
│   └── probe_raw_depth.py / run_probe.sh # 裸深度探针（09-20 评估期工具）
└── data/
    ├── hh4_frames.jsonl                 # peach_stereo hh4 档 145 帧（终版轮）
    └── percipio2_frames.jsonl           # percipio 健康档 146 帧
```

## 核心结论速查

1. **target_1 深度**（09-21 重录轮 hh4 档的远距小目标）：掩膜中位 **758.2±0.43mm**、entry z 761.4±3.23mm。注意 tid 是会话内轨迹号非稳定物理 ID——跨轮比较按物理目标对齐。
2. **双健康档（终版轮，145/146 帧）**：hh4 优势=拟合稳定性+帧率——entry std **0.39/0.53/1.92mm** vs **2.39/2.88/4.37mm**、跳变中位 2–4×、掩膜密度 1.000、粗糙度 0.33 vs 0.44、帧源 **13.6gps vs ~1–2fps**；**percipio 优势=覆盖与细节（用户图上观察、数据证实）**——左缘 15% 列 hh4 全盲（0.000 vs 0.035）、远端 1–1.5m 覆盖 +40%、细节密度 +62%（hh4 平滑链以细节换稳定）。**连续三轮 entry std z 3.6×/5.7×/2.3× 方向稳定、量级随场景波动**。真实门限=重建体素 3mm/pregrasp 偏置 30mm（讹传的"3mm 门"已修正，见影响评估 §4；远目标 0.95m hh4 MAX 29mm 逼近偏置预算）。
3. **录制链修复（"视频不对"终解）**：rviz 内嵌 Image 面板缩放后标注不可见 → composite 全尺寸窗直录 + `check_frame.py` 自检门（不过门不录）+ 四源拼接。此前同日：confidence@16 与 rgb@16 重叠修复（@20/step24）、挂死 recorder 堵流（清理规则 `.cursor/rules/test-program-cleanup.mdc`）、XML 因果现场重测（值2→0.053、空→0.466）。
4. **纪律**：percipio_camera=官方驱动+仅本机 IP/分辨率；实验值不落 `parameters.xml`；起 percipio 必带 `depth_resolution:=640x480`；切前端采果前重做手眼标定；测试进程 timeout 包裹+收尾复核。
