# 2026-10-15 项目设计交付（PPT + 对比视频）

本目录为 2026-10-15 一轮的交付物，数据全部来自 git 提交记录与 `runs/` 测试账本，未含任何未授权真机运动。

## 交付物

| 文件 | 说明 |
|------|------|
| `桃采摘机器人项目设计_2026-10.pptx` | 18 页项目设计汇报：产品目标 / 12 模块架构 / W01-W08 作业闭环 / 技术栈与安全 / 两月演进（原生周度提交柱状图）/ 感知与运动设计 / 三把剪切手 / 测试金字塔 / 真机验证 / 仿真 P1-P4b / 观测性 / 优化前后对比表 / G0-G4 路线图。经 office 校验 + 18 页视觉验收全过 |
| `优化前后对比_2026-10.mp4` | 1080p30 · 161s · 中文旁白 · **数据驱动版**。八段（素材全部裁剪去 UI，图表由 runs/ 账本实测生成）：片头 → 点云密度对比（密集 vs 稀疏）→ v1/v2 同冻结数据对照图表（完成率 100% vs 66.7%、极差 0.19s vs 14.2s）→ 冻结链完整周期（3D 视图+检测框双栏 + 阶段时间条：套入 15.3/复确认 0.2/撤离 12.5）→ 自动优化轮图表（19 轮 12 成、失败全类型化）→ 相机流 vs 同帧实拍对照 → 关键指标对比卡 → 片尾 |
| `数据对比分析.md` | 视频数据底稿：v1/v2 对照、阶段耗时、几何复测、自动轮归因、真机稳定性、两月工程指标，全部注明账本来源 |

## 数据出处（关键数字 → 来源）

- 205 提交 / 2558 文件 / 183 万行：`git log --since=2026-08-15`（W34–W42 周度分布见 PPT 第 8 页柱状图）
- 轨迹通过率 1/30 → 71.3%：提交 6317c40（v2 证伪）、4cc2e5e（v4 入冠 57/80）、781971b（v4d blend）
- 单果周期 28.4s ×4：`runs/frozen_v1_1791774589` 等 4 个 ledger.json（approach 15.3 + retreat 12.5）
- 真机首抓 9.4s：提交 4b5936c（S3 轮，YOLO 0.84）
- 517 测 0 败 / E2E 64/64 / 852 帧 GATE PASS：提交 3c65425 轮、aa5c73a（0034-0037）、4ba0631
- 袋长 −29% 修正 / 两段式 RANSAC：提交 d6051d4、bebfdb4

## 视频素材出处（均已裁剪，无 RViz 面板/窗口边框）

- `runs/compare_rviz_20261012/v1_frozen_full.mp4` → 裁出 3D 主视图 + 右下 Debug 检测面板（target_0 检测框）双栏
- `_archive/code_cleanup_2026-10-12/src/peach/peach_stereo/test/report/fig_f_compare.mp4` → 裁出下排两主视图并排（密集彩色点云 vs 稀疏点云）
- `_archive/.../percipio2_rvizwin.mp4` → 裁出 3D 点云主视图
- `_archive/reports/2026-09-20-camera-image-analysis/final_cmp_live.png` → 同帧实拍对照（peach_stereo 宿主链 vs percipio 设备链，09-20 相机分析轮）
- **弃用**：`runs/harvest_*/masks/` 掩膜序列——抽检证实 712 帧为同一张近全黑死图（亮像素覆盖率恒 0.42%、帧间零变化），不具展示价值

## 复现

```bash
# 数据图表（账本实测 → 三张卡；matplotlib，Noto Sans CJK）
python3 video_build/gen_charts.py

# PPT（需 pptxgenjs：npm install pptxgenjs react-icons react react-dom sharp）
node build_ppt.js
python3 /home/mu/.agents/skills/pptx/scripts/office/validate.py 桃采摘机器人项目设计_2026-10.pptx

# 视频（卡片 + 分段合成 + 合并；旁白 wav 在 video_build/audio/，含 nb/nc 两段数据段旁白）
python3 video_build/gen_cards.py
bash video_build/assemble.sh
```

`assets/` 为 PPT/卡片用的图片素材（RViz 帧、立体对比、掩膜、YOLO 验证、仿真渲染、Web 截图）。
