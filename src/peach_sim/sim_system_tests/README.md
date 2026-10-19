# sim_system_tests

仿真子工作空间系统测试：isolated launch_testing 验收门，与 `peach_system_tests`
同构分层（不进运行 launch）。

## P2 验收门（`test/test_sim_controllers.py`）

起 `sim_bringup/sim.launch.py`（无头、`moveit:=false`——move_group 已在手动
验收覆盖），等待并断言：

1. `/joint_states` 出现，六关节名 = AUBO 契约集合，且带 position 与 velocity；
2. `joint_state_broadcaster` 与 `joint_trajectory_controller` 双双 active
   （`/controller_manager/list_controllers` 轮询，单次往返超时不算失败、下轮重试）。

预算：重世界（368 模型）首载 40-90 s，用例语义等待各 300 s；CMake
`add_launch_test(... TIMEOUT 900)`——总超时必须盖过首载+等待+拆栈，调小会在
慢机/并发负载下产出 missing_result。

## 隔离（必读，2026-10-15 实证）

本机并发 gz 实例（用户侧探针/其它栈）与测试**共享 gz-transport 默认分区时**，
`ros_gz_sim create` 请求会被别的 server 应答——实体建进别人的世界，本域永远
等不到 controller_manager（症状：`/joint_states` 永不到来，300 s 超时失败）。
测试内已按进程唯一隔离两套总线（`GZ_PARTITION=sim_system_tests_<pid>`、
`ROS_DOMAIN_ID=(pid%74)+25`），与 launch_testing 的 ROS domain 隔离同理；
手工复跑时若机上有其他 gz 实例，同样须设独立 `GZ_PARTITION`。

**FastDDS SHM 端口是机器级共享**（跨 ROS domain）：并发栈并存时
`init_port fastrtps_port7018: open_and_lock_file failed`，域内服务调用
（spawner→`/controller_manager/list_controllers`）静默失联。测试子树连同
launch_testing 父进程统一 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`（ROS 2 CI
同款）绕开 SHM；代价仅回环带宽。手工复跑同机多栈时同样建议设置。

## 运行

```bash
colcon test --packages-select sim_system_tests
colcon test-result --verbose
```

测试跑完整 gz 世界，属重测试（分钟级）；速查配置一致性用 `sim_bringup` 的
`test_nav_config.py`（零 ROS）。P3 感知门（begin_scene → target_observations）
现以手动验收口径执行（见 `sim_bringup/README.md`），未落本包。
许可 BSD-3-Clause。
