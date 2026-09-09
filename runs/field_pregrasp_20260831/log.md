# 真机 2026-08-31 — PREGRASP_ONLY

档位：两边 `execution=true`，`grasp.enabled=true`，`tool.enabled=false`，`execute_pregrasp_only=true`。显式 `RunHarvest` `field_pregrasp_20260831`。结束后已把三档使能改回 false。

结果：`no_targets_succeeded`；发现 2、尝试 2、成功 0、`skipped_unreachable` 2。约 23 s。无 SetIO，未停预抓取，无需 ACK。

| 目标 | 观察/重建 | 预抓取 |
|------|-----------|--------|
| target_0 | Build 2 帧、TSDF 1894、refit ACCEPT，约 6.6 s | Pilz LIN `NO_IK_SOLUTION`，`lin to on-axis pregrasp (0/1)` |
| target_1 | Build 2 帧、TSDF 2159、refit ACCEPT，约 5.0 s | 同上 |

账本：同目录 `ledger.json`。栈仍在跑，未再开批。
