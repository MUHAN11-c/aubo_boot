# 咬合式剪切手（bite_shear_v1）主测轮 · 2026-09-28 起

三把剪切手（决策 0033）换代后的第一轮工具战役，**主测档 = `bite_shear_v1`**；
`shear_v1` / `adaptive_shear_v1` 随切随跑（完整步骤见 [docs/testing.md](../../../docs/testing.md) §「感知稳定 + 三种末端」）。

## 约定

- 切换 = **整栈隔离重启** `tool_profile:=<p>`（唯一切换参数，禁止混跑；sim 侧 `--tool-profile` 与栈错配即拒跑）。
- launch 默认 `adaptive_shear_v1`；bite/shear 必须显式传（忘传会起 imu_follow）。
- 三档接线冒烟门（先过再跑轮次）：`colcon test --packages-select peach_system_tests` 的
  `test_tool_profile_smoke_{shear_v1,bite_shear_v1,adaptive_shear_v1}`（域 91/92/93：
  TF `wrist3_Link→tcp`==档案 ±2 mm、`/peach_arm tool.profile_id` 对齐、imu_follow 按名单起/不起）。
- 本战役全程 `hardware_mode:=mock`，不开刀、无 SetIO、不自动 `RunHarvest`（MUST）。
- 测完即停：launch 终端 Ctrl+C 后 `pgrep -af 'ros2 launch|component_container|ros2 run|bag record'` 清残留。

## bite 专属预期（档案几何所致，不按缺陷报）

- D_inner 0.030 对袋颈：`d_bag95 > 0.026` 即 `bag_d95_exceeds_tool` / `radial_budget_negative`，deny 高占比。
- body 包络 0.260 m 长喉道：①层工具胶囊审查更严，`tool_clearance_failed` / skip_select 升高。
- IO = fun3/pin0 与另两把共用；BST-YF 推杆反馈通道待真机核（档案 notes）。
- 套入/撤退走 MTC LIN（与 shear_v1 同代码路径，不在 `usesImuFollowContact` 名单）。

## 脚本

```bash
# 终端 1：起栈（默认域 62，可用 CAMPAIGN_DOMAIN 覆盖）
campaign/20260928_bite_shear/scripts/launch_stack.sh bite_shear_v1
# 终端 2：网格（栈须已起且 tool_profile 一致）
campaign/20260928_bite_shear/scripts/run_grid.sh bite_shear_v1
```

## 轮次台账（追加，request_id 不复用）

| 日期 | request_id | 档案 | 轮次 | 结果 | 备注 |
|------|------------|------|------|------|------|
| （待首轮） | | bite_shear_v1 | grid full | | |

已知红（换代记档，09-28）：E1 注入器 known-good 夹具按旧圆柱 TCP 标定，重标前 C03 预期红（testing-log 09-28）。
