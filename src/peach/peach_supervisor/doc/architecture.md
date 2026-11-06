# peach_supervisor 架构

## 职责与源码入口

`executor_node.py::TaskExecutorNode` 是 `peach_supervisor` Lifecycle 节点，
通过 `RunHarvest` 显式开批。调度只从 `target_observations` 选果；
`grasp_decision` 仅缓存接触令牌，不作为选果输入。不调用 dashboard、驱动
运动或 SetIO；运动、工具与最终授权由 `peach_arm` 执行。

- `harvest_fsm.py`：事件、Reaction 与命令表，控制权限、阻塞词表和终局映射。
- `domain/reducer.py`：批次/目标阶段、运行模式、恢复闩锁及事务世代的纯值归约；
  丢弃跨会话、旧世代与已结案事务的事件。
- `executor_node.py`：ROS 边界、命令处理器、等待/取消、状态发布与落账。
- `batch.py`：候选资格、顺序与窗口、控制序号、结果转换和账本白名单。
- `observe.py`、`cycle_core/view_policy.py`、`view_planner.py`：FAST 补视纯核。
- `cycle_core/batch_policy.py`：策略校验、单调时限与补采类别/文件。
- `run_folder.py`：将批次账本归入当前记录会话 `cycles/`，保留来源时钟。
- `params.py`：GPL 参数实时读取视图、整数及跨字段窗口校验。

## 批次命令链

`RUN_REQUESTED` 从 WAITING_READY 进入 DISCOVERY。固定座 NAVIGATE 直接
产生 NAV_OK；首次 Survey 成功后才 BeginScene，读回 `scene_epoch` 并清旧
观测缓存，然后 WAIT_LOCK。回访只 Survey/dwell/发布快照，不再次 Begin。
Survey 成功发布 `obstacles_refresh`，让独立场景障碍节点更新快照。

SELECT 要求观测世代匹配且集合已锁定。共用资格谓词排除未确认、已 claim、
裸果、贴边及 `tool_clearance_failed` 候选。普通选择按 priority 升序、同级
框面积降序，再过深度与 IK 窗；IK 不可用时回退入口半径窗，FULL 另查套入
终点半径。点名列表只允许名单内目标且仍过资格谓词，但不受深度/IK 窗限制；
`selection_priority>0` 再与本场锁定顺序取交集，无合格点名目标则回访。

DISPATCH 有三种路径：

1. `skip_reconstruction=true`：不发 Build/补视，带 `unrefined:*` 修订派臂；
   臂还须允许未精化几何，适用范围由 launch 作业策略决定。
2. 默认 `reconstruct_in_trajectory=true`：Build 进入 COLLECTING 即派臂，
   使用完整 `pending:<epoch>:<target_id>` 占位。臂先等同身份决策几何，再运动。
3. 显式关闭上述并行模式：Build 与补视并行。FAST 用 MoveTo 相机短 LIN，
   总机位封顶三视；CONSERVATIVE 用 ExecuteTarget OBSERVE_ONLY。
   补视后等 Build 终态与最小机位数，成功才派接触周期。

EXECUTE_FULL 根据 `execute_pregrasp_only` 选择 PREGRASP_ONLY 或 FULL，
均设置 `skip_observation=true`。未精化/并行 Build 路径不装未完成令牌，
臂从同身份决策快照复检；串行路径仅在缓存目标相符时装配冻结 Clearance。
默认 PREGRASP_ONLY 到轨迹净空点 A 停住，周期可成功，恢复 ACK 后才可继续。
单颗终局归入 outcomes；点名名单已全部 claim 时结算，否则返回 Survey。

## 状态、时限与失败

批次态经 FSM/reducer 和 `_apply` 落地。暂停由 `operation_mode` 表达，保留
工作流批次态；Survey/可中断等待会取消在途动作，接触周期等当前动作收口。
恢复闩锁独立于批次态，ACK 须先收到臂侧确认，再解除并更新序号。维护态仅有
现行控制投影，未形成独立完整作业流程；以当前 `permissions` 为可调用依据。

单果时限从 SELECT 选中目标起，扇区时限从 RunHarvest 开始起，使用单调墙钟。
各等待取请求截止与剩余作业预算的较小值。超时先取消并有界等终态；迟到受理
仍追踪/取消，未收口前不派下一颗。单果超时记录 TIMEOUT 并入补采；扇区超时
进入 INTERRUPTED，原因 `sector_timeout`。修改预算保留原起算点。

正常采摘意图至少一个周期 SUCCEEDED 才 `Result.success=true`；SURVEY_ONLY
可零目标正常结算。该计数包含 PREGRASP_ONLY，不证明实际切断；真实采摘须看
臂结果 `harvest.grasped` / level7。取消、失败、质量跳过分别保留业务结果码。

## 生命周期与验证边界

configure 创建 ROS 实体与 bond；activate 后才接批次。deactivate/shutdown
先阻断新派发并级联取消；仍有批次/受理在途时 cleanup/重新 activate 失败，
保留资源等待收口。初始与运行参数分别做 GPL/整数/窗口验证，拒绝不提交视图。

包内 `test/` 仅官方 ament lint。静态契约核对使用
`python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py`；
mock/replay 与现场验证统一见 [测试指导](../../peach_system_tests/doc/testing.md)。
