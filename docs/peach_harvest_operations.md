# 桃子自动采摘联动、Web 调试与接口手册

## 软件边界与启动

`peach_harvest_orchestrator` 是批次唯一所有者；`peach_approach_grasp` 只执行一个稳定
`target_id`；感知负责身份；重建负责有序会话；Web 只代理类型化命令。业务栈不包含 AUBO
上电、松刹车或安全恢复，也未修改冻结驱动。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch peach_harvest_orchestrator harvest_system.launch.py
```

Web 默认 `http://127.0.0.1:8090`。真机上电由现场人员操作；首次运动速度/加速度缩放不高于
0.1。

## 自动流程与状态

编排器要求：锁定且新鲜的目标快照、新鲜重建诊断、在线单目标 Action，以及默认要求的
RobotStatus（未急停、未故障、已上电、可运动）。通过后派发选中的 target ID。失败或接触
结果不确定进入 `RECOVERY_REQUIRED`，不自动恢复。安全暂停在检查点生效；立即取消独立。
维护模式释放自动所有权，Web 才允许调用旧 Trigger 调试服务。

## Web 操作关系

- 批次：安全暂停、恢复自动、进入/退出维护、立即取消。
- 策略：自动开始、规划执行、抓取动作、工具 IO；依赖固定为
  `execution → grasp → tool`，周期中拒绝修改。
- 感知：查询计划、完成目标、重置目标。
- 重建：开始、采集、移除末帧、完成、保存、查询、重置。
- 靠近抓取：查询、预览、完整接触、启动、取消、恢复确认、Arm/Disarm。

写请求校验同源、HttpOnly SameSite Cookie、业务 revision 和维护模式；重置、立即取消、
Arm、接触动作及开启运动策略需要二次确认，审计缓存保留最近 200 条。

## 类型化接口

| 名称 | 类型 | 含义 |
|---|---|---|
| `/peach_harvest_orchestrator/state` | `HarvestState` | 状态、阶段、阻塞、使能、revision |
| `/peach_harvest_orchestrator/events` | `HarvestEvent` | 过程和审计事件 |
| `/peach_harvest_orchestrator/run_harvest` | `RunHarvest` Action | 长时批次入口、反馈与取消 |
| `/peach_approach_grasp_node/run_target_cycle` | `RunTargetCycle` Action | 单目标运动能力 |
| `/peach_harvest_orchestrator/control` | `ControlHarvest` | 暂停、恢复、维护、取消 |
| `/peach_harvest_orchestrator/set_operation_policy` | `SetOperationPolicy` | 原子三级使能 |

旧 Trigger 与 JSON 话题保留一个迁移周期；新客户端使用类型化接口。

## 并发、档案与交付默认值

感知使用容量 1 的最新帧 worker；重建使用容量 3 的串行单写者 worker，满载拒绝新帧，
保证 ICP/TSDF 不并发。命名档案位于 `peach_profiles/`，原子写入且防路径穿越。自带
`sim_safe` 与 `production_disabled` 均关闭运动、抓取和工具 IO；档案不能操作上电或安全
回路。
