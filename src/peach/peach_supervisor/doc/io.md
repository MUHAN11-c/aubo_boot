# peach_supervisor IO

本页依据 `executor_node.py::_configure_ros`、参数文件与
[接口清单](../../peach_interfaces/config/interface_manifest.yaml)。节点图名
`peach_supervisor`；接口字段权威在 [peach_interfaces](../../peach_interfaces/README.md)。

## 对外入口

- `~/run_harvest`，RunHarvest action：唯一开批入口。非 Active、托管栈未就绪、
  已有批次、在途动作/受理未收口或恢复未 ACK 时拒绝。非法意图/策略同样拒绝。
- `~/control`，ControlTask：按当前 permissions 与非零预期 state_seq 控批；
  协议允许 0 跳过序号检查。ACK 转发 `/peach_arm/acknowledge_recovery`，
  失败不消耗序号。客户端未发 action cancel 时，CANCEL_NOW 以 abort 收口。
- `~/set_enables`，SetEnables：校验 execution→grasp→tool 依赖并广播覆盖值；
  由臂命令门强制执行。至少设置一次后，仅 Active 时每秒重发使能。
- `~/set_batch_policy`，SetBatchPolicy：有限采收率 `[0,1]`、非负有限时限、
  FAST/CONSERVATIVE；可更新当前批或一次下批默认，非法请求无副作用。
- `~/fire_step`，FireStep：仅 PHOTO 已实现，经 MoveTo 到 `global_photo_pose`；
  其余枚举调用会拒绝。PHOTO 也需 execution 使能与臂侧授权。

## 订阅与客户端

选果订阅 `/peach/perception/target_observations`（RELIABLE/VOLATILE，depth10）；
令牌订阅 `/peach/reconstruction/grasp_decision` 与栈就绪
`/peach/lifecycle/managed_nodes_activated`（RELIABLE/TRANSIENT_LOCAL，depth1）。
不订 `initial_pose`、重建 diagnostics 或 ForceSample 作为许可输入。

客户端为 BeginScene、SurveyScene、BuildTargetModel、ExecuteTarget、
CheckReachability、MoveTo 和恢复 ACK。前五者的名字来自部署参数；MoveTo/
ACK 使用当前 `/peach_arm/*` 名字。固定座不创建 NavigateToWorksite 客户端。
具体名字/字段见接口清单，调用顺序见 [架构](architecture.md)。

## 发布与落盘

`~/state`（HarvestState）、`~/scene_snapshot`（SceneSnapshot）、
`/peach/batch/enables`（Enables）、`/peach/scene/obstacles_refresh`（Empty）
均为 RELIABLE/TRANSIENT_LOCAL depth1；`~/events`（CanonicalEvent）同可靠性/
持久性，depth50。诊断走标准 `/diagnostics`。状态普通更新最多10Hz，关键终态/
取消/暂停/恢复及操作员服务变更立即广播；事件细节当前编码在 `message` JSON，
`details` 数组与 `sequence` 未填充。

批次 ID 为空时用动作 UUID 派生 `auto_<UUID>`。调度 run_id 与该请求 ID 同源；
模型 run_id 来自感知锁定轮，臂在周期内转换并绑定，两者不可直接等同。
选中目标后 cycle_id 为 `run_id:target_id`。

`runs/<safe_request_id>/ledger.json` 保存 claimed、outcomes 及白名单遥测；
非成功目标可另写 `rework_list.json`。非法目录段回退 `harvest`。批次 finally
统一取消在途并落账；落盘失败告警，账本失败计入 blockers。有效记录会话
通过 `session.json` 中的写者身份确认，将账本归入会话 `cycles/`；来源时钟与
身份保留，旧账本缺完整时钟对时才使用 mtime/elapsed 估算并标记。

记录格式与恢复限制详见 [README](../README.md)。本页只描述接口与现行实现，
不替代 [系统验证](../../peach_system_tests/doc/testing.md)。
