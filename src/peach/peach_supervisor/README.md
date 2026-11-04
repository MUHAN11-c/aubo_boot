# peach_supervisor

Peach harvest batch supervisor lifecycle node.

## 开关语义

- `skip_reconstruction`：本批是否跳过 Build/补视，使用锁定集几何；节点参数基准 false，整栈省略时由 launch 按模式选择 mock=true、real=false，显式传参优先。
- 该参数由 bringup 的显式 launch policy 决定；`task_params_file` 含该键会启动期拒绝。
- `reconstruction_enabled=false` 时 supervisor 被强制 skip，但 reconstruction 进程是否启动由 bringup 决定。

## 生命周期与参数校验

去激活和 shutdown 先拒新派发、取消在途动作并唤醒批次收口。批次回调或动作受理仍在进行时，cleanup 和重新 activate 返回 FAILURE，待远端终态和落账结束后重试；正常重新 activate 清本地取消旗标。诊断 Updater 每个节点只构造一次并只添加一次任务，cleanup 取消 timer，重新 configure 复用并 reset；可达性客户端和障碍刷新发布器随清理释放。

GPL listener 构造后再注册只读范围/跨字段验证，使它们先于内部视图提交执行；拒绝的运行期写入同时保留 ROS 参数值与实时读取值。构造后还显式验证全量当前值，初始非法整数或倒置窗口拒绝启动。

## 作业时限

单果预算从 SELECT 选中目标起算，扇区预算从本次 RunHarvest 开始起算，均使用单调墙钟；0 表示不限。服务/动作的可用性、受理与结果等待共用各自固定截止，并取单果/扇区剩余预算的最小值。Build、锁定等待、回访驻留以及暂停/恢复等待也观察作业截止。`SetBatchPolicy(apply_to_current=true)` 按原起算点重算当前预算；已经触发取消的超时不因后续加预算而恢复。

预算耗尽先取消在途动作，再沿现有最多 10 秒终态等待收口；这段停轨等待可以超过作业预算。单果仅记录一次 `FailureCode.TIMEOUT` 并进入 timeout 补采类别。扇区经 `sector_timeout` FSM 事件进入 INTERRUPTED，RunHarvest 返回 `termination_reason=sector_timeout`。迟到受理仍被取消并追踪；未获远端终态时保留在途/恢复门，不能派下一颗。超时不代替人工恢复 ACK，ACK 服务自身的有界等待不继承已取消批次的预算。

## 操作员状态发布

SetEnables、ControlTask 接受和 ACK 成功后立即广播最新 HarvestState（包括 state_seq），不受高频反馈的 100 ms 限频抑制。网页 ACK 必须先结束旧批次并使用当前非零序号；后端仍保留严格序号检查，状态变化须重新确认，不能以 expected_state_seq=0 绕过网页恢复确认。

点名采摘只在 `target_ids` 中派果。名单内的目标暂未锁定或不合格时进入回访。也可在开批前设置 `selection_priority>0`，由本场景锁定集提供顺序号。两者都提供时取交集。`PICK_SELECTED` 缺少这两种选择依据时拒绝开批。省略 `request_id` 的请求以动作 UUID 派生批次 ID，传给子动作、事件和账本。

## 账本与周期归档

补采清单 `rework_list.json` 与同批账本共用目录。含路径分隔符、上跳段或 NUL 的批次 ID 统一落到 `runs/harvest/`。

补采类别匹配前统一失败码大小写，`TIMEOUT`/`BUILD_TIMEOUT_*` 归 timeout，`SKIPPED_UNREACHABLE` 归 unreachable，刀具/套入/撤退失败归 contact_failed。

`runs/<request_id>/ledger.json` 的每颗结果保留生产端已有的可选来源字段：
`start_unix_ns`、`end_unix_ns`、`start_ros_ns`、`end_ros_ns`、`time_basis`、
`clock_domain`、`cycle_id`、`scene_epoch`。保存与断点恢复使用同一白名单，
纳秒整数和零值原样往返；缺失或 `null` 的字段不补默认值，未知附加键不落盘。

账本落盘后，当前录制会话的 `cycles/` 归档保留来源时钟、时钟域和周期身份。
有效的 Unix 起止时钟优先于账本 mtime；旧账本或缺少有效起止对时，仍按
mtime 与 elapsed 估算并标记 `estimated_ledger_mtime`，重复归档保留首次估算区间。
归档中的 `time_basis` 描述实际采用的区间依据；仅有终点时钟不视为完整来源区间。
