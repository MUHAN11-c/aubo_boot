# 真机测试 2026-08-28 — PREGRASP_ONLY 停预抓取

目的：完整批次走到预抓取后停住，目视袋轴方向与定位。不开套入、不 SetIO。对照 [docs/testing.md](../../docs/testing.md)。

| 图 | 文件 |
|----|------|
| 设计：全部判断与执行门 | [pregrasp_only_flow.mmd](pregrasp_only_flow.mmd) |
| 轮次 B 实测走过的边 | [round_b_actual.mmd](round_b_actual.mmd) |

## 档位（各轮相同）

- `hardware_mode:=real` `camera_enabled:=true` `robot_ip:=169.254.10.98`
- 调度 `execution_enabled=true`，`execute_pregrasp_only=true`（yaml 默认未改，运行期 `ros2 param set`）
- 技能 `execution.enabled=true` `grasp.enabled=true` `tool.enabled=false`
- 手眼 `active.yaml` ≈ `[0.045, 0.108, 0.002]`
- launch 不自动 `RunHarvest`；不起 `aubo_dashboard`

---

## 轮次 A — 11:56 `field_pregrasp_20260828`

冒烟通过。Survey PTP 拍照位 goal-hold。绑定 `target_10` 后重建发 `PregraspVerification` 时 ndarray `or` 崩溃；场景感知 `self.lighting` 使锁定后观测中断。ledger：`observe_failed` / `reconstruction_data_stale`。无 SetIO。

已修：`cut_pt is None` 回退、`_vec_or`、`lighting_min_conf_mean`。

---

## 轮次 B — 13:40 `field_pregrasp_20260828b`（修后）

冒烟：`e_stopped=0` `motion_possible=1` `drives_powered=1`；六节点 Active；8090=200；观测锁定，`target_0`/`target_1` 为 `robust_bag_pose`。重建未再崩，无 `AttributeError`/`ValueError`。

批次 `termination_reason=no_targets_succeeded`，`attempted=2` `succeeded=0` `skipped_quality=2`，约 24 s。无 SetIO，未进 `MovePregrasp` / `HoldPregrasp`，无需 ACK。

### 逐门实测

| 门 | 结果 |
|----|------|
| pgrep 无残留 / 真机 launch | 过 |
| dashboard 未起 | 过 |
| robot_status 可动 | 过 |
| 手眼非名义 | 过 |
| execution→grasp、tool=false | 过 |
| execute_pregrasp_only=true | 过 |
| BeginScene | 过 |
| SurveyScene PTP 拍照位 | 过（goal-hold） |
| SELECT `target_0` | 过（袋策略，非 fruit） |
| 技能 ExecuteTarget 锁定集 | **拒×4**：`不在锁定集且缓存目标=（无有效锚点）` → `observe_failed` |
| 再 Survey → SELECT `target_4` | 过 |
| Build 2 s 内 COLLECTING | 过 |
| OBSERVE_ONLY 当前位+短 PTP | 过 |
| 重建积分 | 4 视、3629 点、重叠 mean=2.7 mm p95=10.2 mm；TSDF 325 点；**少于推荐 5 视** |
| refit | **REOBSERVE** `final=True` axis≈`[0.38,-0.056,0.923]` d=76.7 mm |
| OBSERVE_ONLY 终局 | SUCCEEDED（只观察+精化，未靠近） |
| PREGRASP_ONLY `skip_observation` | 发了 |
| FinalizeAndValidate / `GraspDecision.allowed` | **false** `refined_quality_not_allowed`（门内 `axis_angle_deg=25.08`）；禁止降级接触 |
| Reconfirm / MovePregrasp / VerifyPregrasp / HoldPregrasp | **未执行** |
| 空扫上限 | 过 → 结算 |

账本：`runs/field_pregrasp_20260828b/ledger.json`。

### 为何停在质量门而不是预抓取

技能 `readyToGrasp` 要求 `refined_accept && GraspDecision.allowed`。本轮圆柱 refit 保持 `REOBSERVE`（未 ACCEPT），故 `allowed=false`，BT 在 `FinalizeAndValidate` 失败，按设计不 PTP 预抓取。

`target_0` 拒观察是调度已选 id、技能观测缓存尚未写入锁定锚点（Survey 刚结束立刻 DISPATCH）。

整栈仍在跑，臂停在最后观察位附近。看完可在 launch 终端 Ctrl+C。再试须新 `request_id`；要到预抓取须 `GraspDecision.allowed=true`（精化 ACCEPT）。

---

## 轮次 C — 15:37 `field_pregrasp_20260828c`（拆门后）

先一次 launch 因 `PYTHONPATH` 指向 `src/peach_perception`、生成参数模块只在 `install/`，场景/重建 import 退出，lifecycle 未齐。清环境重拉后：六节点 Active；`e_stopped=0` `motion_possible=1` `drives_powered=1`；8090=200；手眼 `[0.045, 0.108, 0.002]`；相机约 2.4 FPS；dashboard 未起。档位同前。

批次约 25 s，`termination_reason=no_targets_succeeded`，`discovered=1` `attempted=1` `succeeded=0` `skipped_quality=1`。`target_0` 观察约 15 s 后 `observe_failed`：`达到扫描上限仍未收敛（有效视点 0/1）: insufficient_views`。透传有三次 goal-hold（约 0.26 s / 4.1 s / 3.2 s），未进 `MovePregrasp` / `HoldPregrasp`，无 SetIO，无需 ACK。账本：`runs/field_pregrasp_20260828c/ledger.json`。

本轮没走到融合几何，拆门是否让预抓取放行**尚未验证**。方向/定位仍须停在预抓取后现场评。

---

## 轮次 D — `field_pregrasp_20260828d`（宽底极性：袋底→袋口）

目的：重启真机栈，走完 PREGRASP_ONLY；Debug Image 黄箭头须从宽头（袋底）指向窄头（扎口）。不开套入、不 SetIO。`request_id=field_pregrasp_20260828d`。

源码相对 C：沿袋长轴两端宽度强制宽=底；分割掩膜两端宽度与重力/3D 半径打架时跟窄头；融合后再做一次宽底不变量。install 感知 `pipeline.py` 时间戳 16:08，含 `mask_taper_over_3d`。

### 启动前

| 项 | 结果 |
|----|------|
| 时刻 | 2026-08-28 16:15+08 |
| pgrep launch/container/peach/move_group/ros2_control | 无残留 |
| aubo_dashboard | 未起 |
| ping `169.254.10.98` | 通，rtt ~0.18 ms |
| 手眼 `active.yaml` xyz | `[0.04485, 0.10840, 0.00170]`（非名义 2 cm） |
| numpy | 1.26.4 |
| `PYTHONPATH` | launch 前 `unset` |
| 极性是否已编入 install | 是 |

```bash
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
export DISPLAY=:0
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

launch 16:16:32，`PYTHONPATH` 仅为 install/site-packages（无 `src/peach_perception`）。lifecycle：`managed nodes Active（仍须显式 RunHarvest）`。

### 冒烟（未开抓取、未发批次）

| 门 | 结果 |
|----|------|
| 六节点 lifecycle | 全 Active（含 observability） |
| aubo_dashboard | 未起 |
| `robot_status` | `e_stopped=0` `drives_powered=1` `motion_possible=1` `in_error=0` `mode=2` |
| `/joint_states` | 六轴名正确（序与权威不同但集合对） |
| 8090 `/api/state` | 200 |
| 手眼 TF | 启动前 yaml 非名义；echo 未在冒烟脚本里收完（随后观测已是 `base_link`） |
| 调度/技能档 | 仍为默认关：`execution_enabled=false` `grasp.enabled=false` `tool.enabled=false` |
| `execute_pregrasp_only` | true（未改） |

未 `param set`、未 `RunHarvest`。现场先看 Debug Image 极性。

### 现场：`target_1` 方向反了（16:19 观测快照）

`/peach/perception/target_observations`，`base_link`：

| | `target_0` | `target_1` |
|--|------------|------------|
| flags | `taper_neck`（无 over_gravity） | **`taper_neck` + `taper_over_gravity`** |
| bottom xyz | `0.215, -0.644, 0.561` | `0.424, -0.708, 0.567` |
| neck xyz | `0.220, -0.649, 0.662` | `0.336, -0.687, 0.546` |
| 轴（颈−底） | 主要 **+Z**（朝上） | 主要 **−X**，颈比底更低 |
| 黄箭头像素 底→口 | `(403, 419) → (398, 336)` 朝上 | `(235, 402) → (302, 421)` 朝右略下 |
| bbox | 89×135 | 111×102（近方） |
| `mask_taper_over_3d` | **无** | **无** |

`target_0` 箭头从画面下方指向扎口，与「袋底宽、底→口」一致。`target_1` 3D 半径剖面认为窄头不在朝上，跟重力对打后把口标到另一头；目视该箭头从窄头指向宽头。

原因：分割宽度校正把整图像素当成检测框内坐标，端带是空的，2D 梯形从未改写 3D。已改为：掩膜在框内自测宽头，投影减框原点后再对调。

本轮 **未开批次**。SIGINT 停栈后编进感知，作为 **D2** 再拉。

---

## 轮次 D2 — 分割梯形对齐口底（续 D）

相对 D：分割掩膜 ROI 自测宽头；`bottom_px` 落在窄头则对调 3D。感知已重新 colcon。16:22:17 SIGINT launch python PID，4 s 内退出，无残留、无 dashboard。`request_id` 仍未发；先看 `target_1` 黄箭头是否改为宽→窄，flags 是否出现 `mask_taper_over_3d`。

### D2 冒烟

16:22:36 launch；lifecycle 五节点 Active；`e_stopped=0` `motion_possible=1` `drives_powered=1`；8090=200；dashboard 未起。仍未开抓取、未开批。

### Debug Image（已存）

- 全图：`runs/field_test_20260828/d2_debug_image.png`
- `target_1` 裁切：`runs/field_test_20260828/d2_target_1_crop.png`

`target_0`：竖袋，箭头从袋底朝上指向扎口（枝），对。
`target_1`：仍 `taper_over_gravity`，无 `mask_taper_over_3d`。箭头 `(243,398)→(309,416)` 沿袋宽朝右略下，**指向鼓起的果、背离挂枝**。bbox 103×98 近方，分割两端宽度差不够，2D 校正没触发。3D PCA 把袋宽当成了长轴（世界系轴几乎水平，ΔZ≈−2 cm）。

处理：主方向相对重力 |axis·g|<0.5 视为袋宽，长轴改用逆重力（口朝上），再沿这条轴做宽=底。

---

## 轮次 D3 — 近水平 PCA 改走逆重力

先看 `target_1` 箭头是否改为朝挂枝/窄口；flags 期望 `axis_from_gravity_prior`，不应再沿袋宽。

### D3 现场（16:43）

图：`d3_debug_image.png`、`d3_target_1_crop.png`。
`target_1` flags：`axis_from_gravity_prior`、`taper_polarity_swapped`、`mask_taper_over_3d`。箭头 `(257,384)→(273,432)` 朝下偏右，指到鼓的果。**目视：窄口几乎与检测框左边平行**，箭头应指向左边扎口，不该改成竖轴。重力当长轴比 D2 更错。

处理：撤销「近水平改竖轴」。长轴仍用点云主方向。分割沿该轴切成两半，窄半为口（对 `target_1` 即框左边），只对调口底。

---

## 轮次 D4 — 斜袋保持长轴，两半宽度定口底

期望：`target_1` 箭头从宽头指向左边窄口（与左框边平行的扎口）；`target_0` 仍朝上指向枝。未开批。

### 启动前

| 项 | 结果 |
|----|------|
| 时刻 | 2026-08-28 16:50+08 |
| pgrep | 无残留 |
| aubo_dashboard | 未起 |
| ping `169.254.10.98` | 通 |
| install 极性 | 16:47，斜袋不改竖轴，分割两半窄=口 |

```bash
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
export DISPLAY=:0
ros2 launch peach_task_executor harvest_system.launch.py \
  hardware_mode:=real camera_enabled:=true robot_ip:=169.254.10.98
```

### D4 现场（16:52）

用户：`target_1` 还是反的，**没有任何变化**。D4 栈仍在跑，未存 `d4_*.png`。

原因：两半「宽度」把左边竖缝当成宽头（缝在垂直方向很长）。3D 半径剖面同样把竖缝当宽头，箭头继续指向右下果鼓。紫线仍在果–颈中带，不在左框极限。

处理：撤销半宽口底。两端只比**沿轴朝外那条检测框边**的贴合（左缝贴左边、右果离右框有空隙则口在左）；果鼓贴框底不再被当成口。剪切/紫线放到袋口（分割贴框极限），果距不足只否决 `allowed`、不把刀挪到中点。须停栈编进感知后再看，作为 **D5**。

---

## 轮次 D5 — 贴朝外框边定口，剪切在分割/框极限

期望：`target_1` 箭头从右下果鼓指向左边竖缝；紫线/剪切在左框边（红掩膜贴绿框处）。`target_0` 仍朝上指向枝，紫线在上沿扎口。未开批、不开套入、不 SetIO。

flags 期望 `target_1` 出现 `mask_bbox_flush_mouth`。

### D5 现场（17:05）

图：`d5_debug_image.png`。`target_0` 仍朝上对。`target_1` 箭头仍朝右下指果鼓，紫线在右下，**还是反的**。贴框没压过 3D `taper_over_gravity`（竖缝被当成宽头，窄头判到下端）。

用户：袋底→袋口应按悬挂先验，**从下往上、左右最多 90°（水平），不能出现从上往下的趋势**。不把斜袋长轴改成竖的，只禁止朝下的符号。

处理：不再跟会翻到下半球的窄头。最后用上半球夹紧（`polarity_upper_hemisphere`）。作为 **D6**。

---

## 轮次 D6 — 袋底→袋口只许上半球

期望：两袋箭头都从下往上（`target_1` 朝左上/左指向左框竖缝，不得朝右下）；`target_0` 仍朝上。紫线在袋口。未开批。

### D6 观测（17:11，未开批）

图：`d6_debug_image.png`、`d6_target_1_crop.png`、`d6_target_0_crop.png`。

| | `target_0` | `target_1` |
|--|------------|------------|
| flags | `taper_neck`（无朝下翻） | `taper_lower_hemisphere_ignored` + **`polarity_upper_hemisphere`** |
| 底→口 Δxyz | `+Z` 为主（0.107 m 朝上） | `+X` 为主、`+Z` 0.019 m（不朝下） |
| 黄箭头像素 底→口 | `(405,423)→(389,332)` 朝上 | `(307,422)→(234,403)` **朝左略上** |
| 紫线/口 | 上沿 | 左框边 |

3D 上 `target_1` 口比底高；箭头从右下果鼓指向左边竖缝。待现场目视确认。

---

## 轮次 E — PREGRASP_ONLY 完整批次 `field_pregrasp_20260828e`

在 D6 栈上开执行/抓取、刀具保持关，intent=PICK_ALL。走到预抓取后停住目视方向与定位。不开套入、不 SetIO。看完须 ACK（命令 6）才再 Survey。

### 档位

17:17 冒烟：六节点 Active；`e_stopped=0` `motion_possible=1` `drives_powered=1`；8090=200；dashboard 未起。运行期 `execution.enabled`/`grasp.enabled`/`execution_enabled`=true，`tool.enabled=false`，`execute_pregrasp_only=true`。

### 结果

17:18:47 开批，约 185 s，`termination_reason=no_targets_succeeded`，`discovered=2` `attempted=2` `succeeded=0` `skipped_quality=2`。透传四次 goal-hold（Survey / 再 Survey），**未进** `MovePregrasp` / `HoldPregrasp`，无 SetIO，无需 ACK。账本：`runs/field_pregrasp_20260828e/ledger.json`。

| 门 | 结果 |
|----|------|
| SurveyScene PTP 拍照位 | 过（goal-hold） |
| SELECT | `target_0` 后 `target_1` |
| Build `target_0` | **2.0 s 未 COLLECTING** → `build_start_timeout` |
| Build `target_1` | **拒**，空等 `action_timeout` 180 s → `build_rejected` |
| OBSERVE_ONLY / PREGRASP_ONLY | **未发** |
| SetIO | 无 |

原因：`target_0` 的 Build 取消后调度立刻派 `target_1`；重建单槽未结束，下一颗 goal 被拒。重建全程无「强制开始」日志。

处理：取消未进采集的 Build 后先等动作结束再派下一颗；绑定后立刻反馈 COLLECTING。编进后再开 **F**。

---

## 轮次 F — PREGRASP_ONLY `field_pregrasp_20260828f`（Build 取消后等待）

档位同 E。期望：Build 进 COLLECTING，观察后 PTP 预抓取并 Hold。不开套入、不 SetIO。

### 结果

17:25:57 开批，约 31 s，`termination_reason=no_targets_succeeded`，`discovered=2` `attempted=2` `succeeded=0` `skipped_quality=2`。无 SetIO，未进 `MovePregrasp` / `HoldPregrasp`，无需 ACK。账本：`runs/field_pregrasp_20260828f/ledger.json`。

| 门 | 结果 |
|----|------|
| SurveyScene PTP | 过 |
| `target_0` DISPATCH | OBSERVE **拒**：技能锁定集未跟上（同轮次 B） |
| `target_1` Build | 过（进采集，有 REFINING） |
| `target_1` OBSERVE_ONLY | 当前位采帧 + 两次短 PTP（goal-hold 4.4 s / 3.4 s）后 **16.7 s 有效视点 0/1** → `insufficient_views` |
| PREGRASP_ONLY | **未发** |
| SetIO | 无 |

与轮次 C 同一观察门：日志有 REFINING（TSDF 已抽出上千点），但随后 `TSDF 在线积分失败: The truth value of an array with more than one element is ambiguous`。根因：融合成功后写 `geometry.jsonl` 对 `cut_pose`（3 维 ndarray）用了 Python `or`，异常被当成体积失败并回滚，RViz TSDF Cloud 空、技能有效视点 0。**方向/定位仍未停到预抓取。**

---

## 轮次 G — PREGRASP_ONLY `field_pregrasp_20260828g`（TSDF 回滚修复后）

档位同 E。期望：观察帧留在体积里，RViz 有 TSDF Cloud；过观察门后 PTP 预抓取并 Hold。不开套入、不 SetIO。

### 结果

17:44:00 开批，约 23 s，`termination_reason=no_targets_succeeded`，`discovered=2` `attempted=2` `succeeded=0` `skipped_unreachable=2`。无 SetIO，未进 `HoldPregrasp`，无需 ACK。账本：`runs/field_pregrasp_20260828g/ledger.json`。

| 门 | 结果 |
|----|------|
| SurveyScene PTP | 过 |
| `target_0` Build / OBSERVE | 过：2 视、12053 点、TSDF 2072 点、`refit ACCEPT` |
| `target_0` PREGRASP_ONLY | 发了；`ptp to on-axis pregrasp` MTC **0/1** |
| `target_1` Build / OBSERVE | 过：2 视、12425 点、TSDF 1974 点、`refit ACCEPT` |
| `target_1` PREGRASP_ONLY | 发了；同样 MTC **0/1** |
| HoldPregrasp | **未到** |
| SetIO | 无 |

F 轮的 numpy `or` / 体积回滚未再出现。`geometry.jsonl` 已写出 `cut_pose` 三维 list。观察中 TSDF 有点；批次结束后重建复位，当前 `/peach/reconstruction/tsdf_cloud` 为空，须在下一颗观察时看 **TSDF Cloud**。下一步卡在预抓取 PTP 规划，不是重建。**方向/定位仍未停到预抓取。**


