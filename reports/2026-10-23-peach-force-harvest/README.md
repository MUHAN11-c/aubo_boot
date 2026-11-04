# 2026-10-23 汇报：真机抓取 / 五点力 / CoP-剪切位置

4 页 PPT，数据截至 2026-10-22 真机记录。

## 交付物

| 文件 | 说明 |
|---|---|
| `peach_force_harvest_20261023.pptx` | 主汇报，4 页，含原生环形图/柱图、力曲线、CoP 盘面、侧视几何与演讲备注 |
| `peach_force_harvest_20261023.pdf` | 同版 PDF，便于快速预览 |
| `summary.json` / `summary.csv` | 本次引用的全部数值（抓取成功率、力测试、CoP/旋转/刀口位移） |
| `assets/` | PPT 内嵌图（台架 20 s、真机五点、CoP 盘面、侧视几何） |

## 核心结论

- **真机视觉引导抓取成功率 89.1%**：2026-10-22 16:37–17:06，`session_20261022_163539`，46 个周期，41 个达到 `completion_level=6`（套入+撤退）。5 次失败：4 次 `skipped_quality`（接触阶段取消/撤离未确认，target_2/3）、1 次 `skipped_unreachable`（target_3）。46/46 次都有有效视觉目标。
- **全天口径一致**：7 个 session、55 个周期，49 次 level 6（89.1%）；严格按 `outcome=0` 且 level≥6 为 47/55（85.5%）。
- **无采摘确认**：`tool.enabled=false`，level 7 = 0/46。level 6 只表示套入到位，不等于切断成功。
- **五点力链路可信**：10 s 单点峰值 3.619 N；力-行程 65.8 g/mm（r=0.9998）；20 s 四路顺序 3.628 N SAT；手动五点逐点 3.628 N / 6.0 mm；力残差 <1 g。台架三组测试五点同时均为 0 帧。
- **唯一一次真实五点同步**：2026-10-22 15:09:34–38，target_0，426 帧 / 4.26 s；172 帧五点同时 >0.1 N；峰值合计 5.237 N；峰值帧 CoP 8.19 mm；315 帧有效 CoP 的 P50 8.61 mm、P90 12.01 mm；左两点 : 右两点 = 2.95 : 0.30 N（9.8 : 1）。
- **CoP 旋转对剪切位置的影响**：按现行方案绕 TCP ≤5° 纠偏，刀口在 37 mm 力臂处横移 `37·tan5° = 3.24 mm`，膜片平面横移 6.32 mm。3.24 mm = 40% 刀口捕获窗口（8 mm）、108% 刀面标定误差（3 mm）。全回中 P50 需 6.8°、P90 需 9.4°，均超 5° 上限；纠到 2 mm 也需 5.2° / 7.9°，刀口移 3.38 / 5.12 mm。

## 口径与假设

- 成功率分母是**发起抓取的周期**；分子是 `completion_level ≥ 6`。不是“检测到果实数”，也不是“采摘确认数”。
- `skipped_quality` 的实际原因是“接触阶段取消或撤离未确认，须现场人工撤离后确认恢复”，PPT 中简写为“接触异常”。
- CoP 点位：`tcp` 系 5 点 CAD 坐标，膜片平面 `tcp z = −72.29 mm`；刀口 `L_blade = 37 mm`（`bite_shear_v1`）。
- CoP→旋转角 θ 采用**一阶杠杆模型**（CoP 随压头中心平移）；刀口 3.24 mm 是**刚体几何精确值**，不依赖接触力学。真实收敛/符号仍需台架验证。
- 绝对力尚未砝码标定；台架 `mm` 是 `力/66` 的力等效行程，不是独立位移。

## 复现

```bash
cd reports/2026-10-23-peach-force-harvest
python3 analyze.py

NODE_PATH=/home/mu/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules \
  node build_deck.js

python3 /home/mu/.agents/skills/pptx/scripts/office/soffice.py \
  --headless --convert-to pdf peach_force_harvest_20261023.pptx
```

`analyze.py` 只读 `runs/` 与工具档案；`build_deck.js` 需要 `pptxgenjs`（本机在 Codex runtime 的 node_modules 中）。

## 数据源

- `runs/session_20261022_163539/cycles/*.json`（46 周期）
- `runs/session_2026102*/cycles/*.json`（全天 55 周期）
- `runs/session_20261022_150712/force/episode_001.csv`（唯一五点同步）
- `runs/session_20261022_144719/force/episode_001.csv`（单点对照）
- `runs/force_10s_20261021_095244/force.csv`
- `runs/force_20s_20261021_102812/force.csv`
- `runs/force_20s_manual/force.csv`
- `src/aubo_description/config/bite_shear_v1.yaml`（L_blade=37 mm）
- `src/serial_force/doc/geometry.md`（5 点 CAD 位置 / 72.29 mm）
