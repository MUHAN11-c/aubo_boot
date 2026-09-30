# peach2_calibration

Peach v2 每把刀的**误差常数单源**：`results/<tool_id>.yaml`，安装到 `share/peach2_calibration/results/`，
由 `peach2_core.tool.load_tool_calibration()` 读取、`peach2_core.budget.evaluate_budget()` 使用。
刀具**几何**（D_inner / D_outer / L_insert / L_blade）不在这里，只读 `aubo_description/config/<tool_id>.yaml`。

本包现阶段是空壳（ament_python，无节点）：M0 台架工具以后放进 `peach2_calibration/`。
**2026-09-30：** M1 软件骨架已过退出门；本包三份 yaml 仍是 `design_reference`，M0 退出门未过。

## 结果文件

| 键 | 含义（全部 95% 界，米） | 初值来源（profile 同名项） |
|----|------------------------|----------------------------|
| `w_capture_m` | 刀口平面捕获半宽：颈落在刀口平面 ±w 内即可剪断 | `geometry_m.blade_capture_half_width` |
| `e_blade_m` | 刀口平面相对 TCP 的轴向标定误差 | `error_m.blade_plane_calibration_error95` |
| `e_robot_m` | 臂末端轴向定位误差 | `error_m.robot_axial_error95` |
| `e_tcp_m` | TCP 横向标定误差 | `error_m.tcp_calibration_error95` |
| `e_handeye_m` | 手眼（相机外参）误差 | `error_m.hand_eye_error95` |
| `e_runout_m` | 刀具安装跳动 | `error_m.tool_runout95` |
| `c_wall_m` | 套筒内壁留隙 | `geometry_m.wall_clearance` |
| `fruit_clearance_m` | 刀口平面到果顶最小距离 | `geometry_m.fruit_safety_clearance` |
| `status` | `design_reference` / `calibrated` | — |

未映射的 profile 项：`bag_deformation_margin95`（台架确认后再定是否进预算）、`target_motion95`（v2 由实测摆幅
`swing_amplitude_m` 替代）。

**`status: design_reference` 时 `cut_allowed` 恒为 false**（`BUDGET_STRUCTURAL` / `calibration_pending`）；套入
（`sleeve_allowed`）只看径向预算，不受 status 影响。三份文件当前都是 `design_reference`，数值照抄 profile v1.1，
**不是**台架测量值。

## M0 台架怎么标

前提：示教器上电、急停可及、工作空间隔离；每一步真机运动都要口头授权（AGENTS.md MUST）。标定件用刚性假颈
（已知直径的金属/3D 打印棒）固定在台架上，位置用示教器 TCP 触碰或外部测量确定。每项至少 20 次，取 95% 分位
（或 1.96·σ，样本近正态时）写入结果文件。

1. **`e_tcp_m`**：四点/六点 TCP 标定后，用尖端在固定标记点上换姿态触碰 ≥ 20 次，记录 TCP 位置散布，取横向 95% 半径。
2. **`e_handeye_m`**：手眼标定后，把标定板/球放到工作空间 ≥ 20 个位置，比较相机测得位置（base_link）与示教器触碰位置，
   取残差 95% 分位。
3. **`e_runout_m`**：刀具装卸 10 次以上，每次用百分表或相机测套筒口中心相对法兰的偏移，取 95% 分位。
4. **`e_robot_m`**：同一轴向目标以不同接近姿态/速度到达 ≥ 20 次，外部测量（百分表/激光测距）轴向到位误差，取 95%。
5. **`e_blade_m`**：在假颈上以已知轴向位置下刀（空剪或薄纸条），测实际刀口平面位置与 `tcp − L_blade·z` 预测的差，取 95%。
6. **`w_capture_m`**：沿轴向以 1 mm 步长把刀口平面相对假颈扫描 ±20 mm，每个位置剪 3 次，记录“完全剪断”的连续区间，
   `w_capture_m` = 该区间半宽减去该次试验中 `e_blade_m`、`e_robot_m` 的 RSS（避免重复计入）。
7. **`c_wall_m`**：用最大允许袋径的假袋套入 ≥ 10 次，确认不刮壁的最小内壁余量（通常取套筒内壁公差 + 袋表面起伏）。
8. **`fruit_clearance_m`**：真果/假果贴袋底，逐步缩小刀口平面到果顶的距离下刀，取无果皮损伤的最小距离再加安全余量。

全部替换后：把 `status` 改为 `calibrated`，`source` 写成台架记录编号与日期，并在 `docs/testing-log.md` 追加该轮记录。
`test/test_results.py` 只在 `status: design_reference` 时要求数值与 profile 一致；改成 `calibrated` 后只检查键完整和取值合法。

## 怎么测

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
timeout 1200 colcon build --base-paths src/peach2 --packages-select peach2_calibration \
  --build-base build/v2/peach2_calibration --install-base build/v2/peach2_calibration_install
timeout 1200 colcon test --base-paths src/peach2 --packages-select peach2_calibration \
  --build-base build/v2/peach2_calibration --install-base build/v2/peach2_calibration_install
colcon test-result --test-result-base build/v2/peach2_calibration --verbose
```

## 接口需求

- 无。

## 已知限制与 TODO(M0)

- TODO(M0)：以上 8 项全部为设计参考值，需台架实测替换后才允许 `cut_allowed`。
- TODO(M0)：三把刀当前误差常数完全相同（profile 如此），实测后应各自不同。
- TODO(M0)：台架采集/统计脚本尚未实现（本包 Python 模块预留）。
