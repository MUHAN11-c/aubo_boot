# 过程数据分析报告

> 数据截止:2026-08-24(现场 `runs/` 已归档至 `_archive/runs/root_2026-08-24/`;更早归档在 `_archive/runs/`)
> 分析对象:采摘批次过程数据(ledger / events / reconstruction / approach / summary)
> 可视化:[`harvest-analysis-dashboard.html`](harvest-analysis-dashboard.html)(自包含,浏览器直接打开)

---

## 1. 数据范围与规模

### 现场数据(当前迭代,8/20 – 8/24)
- **批次账本** `harvest_runs/field_full_*/ledger.json`:17 个批次,共 33 个目标周期(8/20 验证模式 6、8/21 17、8/24 10)
- **目标周期明细** `harvest_runs/harvest_<ts>_sN/`:`manifest.yaml`、`events.jsonl`、`latest_reconstruction.json`、`latest_perception.json`、`masks/*.png`
- **Web 运行** `web_runs/run_*/`:`summary.md`、`state.jsonl`、`reconstruction.jsonl`、`approach.jsonl`、`clouds/*.ply`、`images/*.jpg`(数据取 2026-08-20 ~ 08-24 共 14 个带阶段明细的批次)
- 全部使用同一模型/标定:`yolo:6981750d… | mobile_sam:6dbb9052…`,`percipio-640x480-chessboard | hand_eye:import_humble_20260128T114006`
- **现场无一次全程成功**:8/20 五个"精化通过但 grasp.enabled=false"是验证模式;8/21 `coverage_fix` 1 次成功抓取;其余全部失败

### 归档数据(早期迭代,8/13 – 8/14)
- `_archive/runs/harvest_runs/` 42 个目标周期、`web_runs/` 23 个、`peach_sessions/` 4 个
- 对应开发调试期(如 8/14 18:17 批次仅 2.9 s、无目标),属历史阶段,未纳入本报告主体

---

## 2. 8/24 最新一轮:5 批 10 目标全部 skipped

| 批次 | 开始 | target_1(先) | target_0(后) | 失败阶段 |
|---|---|---|---|---|
| `grasp` | 09:28 | MTC CartesianPath `min_fraction 0.9667`(差 3.3%) | `observe_failed: selected_target_stale` | 接近 / 观察 |
| `grasp2` | 09:34 | `observe_failed: selected_target_stale` | `observe_failed: observe_only 未等到 TSDF/精化` | 观察 |
| `grasp3` | 09:38 | `selected_target_stale` | `selected_target_stale` | 观察 |
| `grasp4` | 09:43 | MTC short-path guard `25.16s > 12s` | MTC short-path guard `27.29s > 12s` | 接近 |
| `lin` | 09:58 | MTC `collision-aware move to refined entry (0/1)` | 同上 `(0/1)` | 接近 |

**模式分布(10 目标)**:观察阶段失败 **5** 次(其中 `selected_target_stale` 4 次),MTC 接近阶段失败 **5** 次。

MTC 失败的三种具体形态:
- `short-path guard: 预计时长 25–27 s > 12 s`(grasp4 ×2)→ 规划路径太长,被时长护栏拒绝
- `collision-aware move to refined entry (0/1)`(lin ×2)→ 路径已规划但该运动段 0/1 执行失败
- `CartesianPath min_fraction 0.9667`(grasp)→ 已走到 96.7%,最后一段未达标

---

## 3. 深入:`lin` 批次(最后一批,数据最全)

### 3.1 目标级结果

| | target_0 | target_1 |
|---|---|---|
| 观察耗时 | 14.95 s | 33.5 s |
| 有效视角 | 4(建议 ≥5) | 6 |
| 点云 | 38 225 点,重叠 1.3 / 7.7 mm | 19 487 点,重叠 3.6 / 29.6 mm |
| 重建 refit | ACCEPT(cylinder) | **REOBSERVE**(cylinder,rmse 1.4 mm) |
| 帧拒绝 | 31 | **147** |
| 最终闸门 | — | `perception_reconstruction_axis_mismatch` **58.3°**(重建轴 vs 感知轴) |
| 失败 | MTC `collision-aware move to refined entry (0/1)` | 同上(降级 `degraded_anchor` 后) |

### 3.2 帧拒绝构成(观察瓶颈)

target_1 的 147 次拒绝:
- `robot_not_static` **93 次(63%)** ← 主因
- `near_duplicate` 31、`stale_frame` 11、`target_drift` 5、`same_stamp` 4、`missing_mask` 3

target_0 的 31 次:`robot_not_static` 19、`stale_frame` 3、`missing_mask` 3、其余各 1–2。

**时序证据**(reconstruction.jsonl):
- target_1 09:59:24 开始 COLLECTING,前 ~5 s 内 `robot_not_static` 从 0 涨到 19→27→44;09:59:40 才集满 5 视角
- 之后在 COLLECTING 中又停留 ~18 s 等第 6 视角,期间以 `near_duplicate` + `robot_not_static` 为主
- 有效帧率与感知帧率严重脱节:感知约 **2.5 FPS**(帧间隔中位数 0.4 s),重建有效视角 6 个却用时 33.5 s

### 3.3 观察-接近衔接

- target_0:重建 READY(4 视角)后各阶段(FINALIZING/VALIDATING/APPROACHING/COMPLETING)各仅 0.2 s 即失败——**MTC 规划几乎瞬时失败**,问题不在规划时长而在单运动段失败
- target_1:重建 READY(6 视角)后同样 0.2 s 级失败;重建质量闸门曾以 `axis_mismatch 58.3°` 拒绝,系统走降级路径后仍死在 MTC

---

## 4. 对照:唯一一次全流程成功(8/21)

`field_full_20260821_1645_coverage_fix:target_2` → **succeeded,51.6 s**
- OBSERVING 15.0 s / VALIDATING 0.6 / **APPROACHING 28.0** / RETREATING 7.8 / COMPLETING 0.2
- 有效视角 **15**(远超 8/24 的 4–6),拒绝 39 次以 `same_stamp`(17)+ `near_duplicate`(16)为主,**`robot_not_static` 并非主导**
- 同批 target_3 失败于刻度的另一侧:`missing_mask` 35 次 → 掩码/目标分割问题

**推断**:8/24 观察策略产出的视角覆盖不足且 `robot_not_static` 占比过高,与 8/21 成功案例(15 视角、拒绝以重复帧为主)形成对照。

---

## 5. 数据质量异常

- `harvest_20260824T095902_245575_s2/events.jsonl` 时间跨度 **01:59:02 → 03:05:12 UTC**(本地 09:59 → 11:05),目录 mtime 11:05
- 批次已于 02:00:08 UTC(10:00:08 本地)结束,但该文件在结束后仍继续写入 **~65 分钟**的 `frame_observations`(`observed_target_ids` 全空,6838 条)
- 事件总数 7030:`frame_observations` 6838、`frame_skipped` 178、`frame_accepted` 10、`reconstruction_linked` 1、`reconstruction_finalized` 2、`global_targets_locked` 1
- 分析该 run 时,02:00:10 UTC 之后的记录应视为运行后残留(感知进程在批次结束后仍向旧 run 目录写帧),不计入运行指标

---

## 6. 结论与建议(仅分析层面)

**结论**
1. 8/24 的失败一半在观察段(`selected_target_stale` ×4、未得到精化 ×1),一半在 MTC 段(全部为"入口/插入"运动失败:guard 超时 ×2、`(0/1)` ×2、`min_fraction 0.967` ×1)
2. 观察段最大的帧损失源是 `robot_not_static`(lin target_1 占 63%)——采集窗口与机器人静止判定失配,把 2.5 FPS 的感知流进一步稀释到 ~0.2 有效视角/秒
3. 视角数普遍 4–6,低于推荐 5,且 lin 已出现局部 REOBSERVE 信号;8/21 成功案例为 15 视角
4. MTC 段失败集中在"接近目标入口"而非插入本身,`(0/1)` 表示该运动段整体未规划成功;grasp4 的 25–27 s 路径被 12 s 护栏拒绝,提示目标相对机器人的几何/关节状态不佳

**建议排查方向**(供后续验证,不改变驱动栈)
- 观察:检查拍照触发时机与"机器人静止"判定(速度阈值/里程计来源)的耦合;适当增加观察视点数量并扩大角基线
- MTC:`collision-aware move to refined entry` 的 `(0/1)` 失败需要看该段规划日志(碰撞?IK 奇异?);`short-path guard` 12 s 阈值与 25–27 s 实际值的差距,建议核对入口点选取是否合理
- 跟踪:`selected_target_stale` 高频出现,检查目标身份刷新与 dispatcher 之间的时序
- 数据:批次结束后停止向旧 run 目录写帧,避免 `events.jsonl` 混入运行后残留

---

## 附:字段速查

- `ledger.json`:`claimed`(锁定目标)、`outcomes[]`(target_id/outcome/reason/failure_code/elapsed_s/build_view_count/build_status)
- `summary.md`:批次级终局、逐目标 phase 耗时、事件统计、感知/重建/性能指标、逐目标 skip_reasons
- `reconstruction.jsonl`:重建模块 diagnostics(帧拒绝计数按目标累加)、grasp_decision
- `events.jsonl`:每帧观察/接受/拒绝、重建接合与最终化(时间自 UTC)
