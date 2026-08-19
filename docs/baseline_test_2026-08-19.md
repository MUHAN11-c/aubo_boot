# 重构前置基线（2026-08-19）

> 在破坏性重构开始前冻结：全量 Release 构建与 `colcon test` 结果。
> 本文件只记录事实，不把基线失败当作新架构已修好。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

结果：20 个包全部完成。无构建失败。

## 测试总表（首次全量）

```bash
colcon test && colcon test-result --verbose
```

| 项 | 值 |
|---|---|
| 包数 | 20 |
| 测试总数 | 986 |
| 失败 | 5 |
| 错误 | 1 |
| 跳过 | 76（含 cppcheck 性能跳过与 open3d 条件跳过） |
| 墙钟 | 约 4 min 23 s |

失败/错误包：`peach_approach_grasp`、`peach_reconstruction_ros2`。

## 失败与错误明细

| 包 | 用例 | 性质 | 现象 |
|---|---|---|---|
| `peach_approach_grasp` | `cpplint` `scan_budget.hpp:51` | lint | `whitespace/comments`：代码与行尾注释之间不足两空格 |
| `peach_approach_grasp` | `ProtectedZoneEntry.EntryInsideZoneRejectsMtcWithUnreachable` | 错误 | 最小 URDF 无 `tcp` 坐标系时观察移动 TF 查找失败；补 `tcp` 固定关节后仍可能因无 TF 广播超时 |
| `peach_reconstruction_ros2` | `test_flake8` | lint | 导入名顺序 |
| `peach_reconstruction_ros2` | `test_pep257` | lint | `icp_target_cache.py` / `publish_throttle.py` D213 等多处 pep257 |

## 派发协议 DDS 复测（G5 盲区）

首次复测：`test_dispatch_protocol` 14 例仍全部 `GTEST_SKIP`：

> 假能力端 action 服务不可达：本机 DDS 端点匹配异常（假 RunTargetCycle action 服务 matched_count=0）

内核已是 `6.8.0-138-generic`（用例注释里「升级未重启」已不再成立）。`test_policy_service` 仍全绿：该用例 `fork`/`execl` 真实 `harvest_orchestrator_node` 可执行文件。

处理：将假能力端改为独立 Python 进程 + `launch_testing` 夹具（`test/fake_capability_node.py`）。

## 其余编排器测试

`test_state_machine`、`test_policy_service`、`test_yaml_defaults`、`test_parameter_validation` 全绿。
