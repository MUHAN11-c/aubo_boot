# move_to_pose 全流程与错误码 -4 分析

## 一、完整调用链（从前端到机械臂）

```
[前端 app.js]
  moveRobotToPoseCartesian() / executeRobotPose()
    → fetch('/api/set_robot_pose')  单次请求（已有 _setRobotPoseInFlight 防重）
        ↓
[桥接 http_bridge_server.py]
  handle_set_robot_pose() → ros2_node.move_to_pose()
    → call_async(MoveToPose) + spin_until_future_complete()  单线程，串行
        ↓
[C++ move_to_pose_server]
  moveToPoseCallback() → moveToPose()
    → move_group_->plan(my_plan)
    → move_group_->execute(my_plan)  阻塞，等待 MoveIt 执行结果
        ↓
[MoveIt2 MoveGroupInterface]
  execute(plan) 内部：
    - 把 plan 转成 FollowJointTrajectory Goal
    - 作为 action client 发送给 follow_joint_trajectory action
    - 等待 action 返回 result（或超时后取消/报错）
        ↓
[aubo_ros2_trajectory_action]
  handleGoal() → ACCEPT_AND_EXECUTE
  handleAccept() → publishTrajectory() 发布到 joint_path_command (ROS1 桥)
  → 等待 aubo/feedback_states 反馈
  → 当 checkReachTarget() 为真时 active_goal_->succeed(result)
  或 在以下情况 abort：
    - watchDogTimer(): 接受目标后 2 秒内未收到任何 feedback → "watchdog_no_feedback_2s"
    - handleCancel(): 客户端请求取消 → "client_cancel"
    - moveitExecutionCallback("stop"): 收到 trajectory_execution_event=stop → "trajectory_execution_event_stop"
    - handleGoal() 时已有 active goal → "new_goal_received"
        ↓
[ROS1 端 / 真实控制器]
  接收 joint_path_command，执行轨迹，向 aubo/feedback_states 发布反馈
```

## 二、错误码 -4 的含义与来源

- **MoveIt 侧**：`MoveGroupInterface::execute()` 返回 `MoveItErrorCode::CONTROL_FAILED`（值为 -4）时，会打印：
  - `MoveGroupInterface::execute() failed or timeout reached`
- **含义**：FollowJointTrajectory **action 未返回 SUCCESS**，而是被 **aborted** 或 **超时**。
- **因果链**：轨迹 action 一旦 `abort(result)`，MoveIt 的 action client 就会得到 aborted，进而 `execute()` 返回 -4。

因此：**-4 的根因在“轨迹 action 为何 abort”**，而不是规划或 HTTP 层。

## 三、轨迹 action 的四种 abort 原因（对应日志）

在 **aubo_ros2_trajectory_action** 节点日志中会看到：

`JointTrajectoryAction: abort, reason=XXX`

| reason | 含义 | 可能触发场景 |
|--------|------|----------------|
| **client_cancel** | 客户端（MoveIt）调用了 cancel | MoveIt 执行超时后主动取消；或其它逻辑发 cancel |
| **watchdog_no_feedback_2s** | 接受目标后 2 秒内未收到 aubo/feedback_states | 控制器/ROS1 未及时发反馈；话题名或频率不对 |
| **trajectory_execution_event_stop** | 收到 trajectory_execution_event 的 "stop" | MoveIt 的 trajectory_execution_manager 发 stop（如内部超时/校验失败） |
| **new_goal_received** | 在处理新 goal 时已有旧 goal 在执行 | 同一或另一客户端在未完成时又发了新 goal（重复请求/并发） |

**排查 -4 时**：先在同一时刻的 aubo_ros2_trajectory_action 日志里确认是哪一个 `reason`，再对下表处理。

## 四、与“执行超时”相关的配置

- **moveit_controllers.yaml**：`allowed_start_tolerance: 0.1`（起始关节容差）。
- **aubo_moveit_bridge_ros1.launch.py** 已设置较宽松的执行时间：
  - `allowed_execution_duration_scaling: 15.0`
  - `allowed_goal_duration_margin: 5.0`
  即允许时间 ≈ 轨迹时长×15 + 5s，正常 1～2s 的轨迹不会因“轨迹执行时间”配置而超时。
- 日志中的 **"execute() failed or timeout reached"** 是 MoveGroupInterface 在等待 action 结果时的总体失败提示，可能对应：
  - action 被 **abort**（此时以轨迹 action 的 `abort, reason=XXX` 为准）；
  - 或 MoveIt 内部 action client 的 **等待超时**（与 trajectory_execution 参数可能不是同一路）。
因此出现 -4 时，**必须先看 aubo_ros2_trajectory_action 的 abort reason**，再判断是反馈慢、MoveIt 取消，还是其它。

## 五、建议排查步骤（当出现 -4 时）

1. **看轨迹 action 的 abort 原因**  
   在 move_to_pose_server 报 -4 的同一段时间内，看 aubo_ros2_trajectory_action 的终端/日志：
   - 若为 `reason=client_cancel` → 多为 **MoveIt 执行超时** 或其它逻辑发 cancel。
   - 若为 `reason=watchdog_no_feedback_2s` → **反馈未在 2s 内到达**，查 aubo/feedback_states 的发布与桥接。
   - 若为 `reason=trajectory_execution_event_stop` → **MoveIt 内部发了 stop**，需查谁发布 trajectory_execution_event、以及 MoveIt 的 trajectory_execution 配置。
   - 若为 `reason=new_goal_received` → **同一时刻有多个 goal**，需确认是否仍有重复/并发请求（前端/桥接/C++ 是否只发一次）。

2. **确认“单次用户操作只对应一次请求”**  
   - 前端：控制台是否有 `[set_robot_pose] 忽略重复请求` 或 `请求 #N` 与预期一致。
   - 桥接：SEND/RECV 的 req_id 是否与前端请求一一对应、无重复。
   - C++：若曾出现同一逻辑请求两次 before_exec/after_exec（1 与 -4 成对），说明曾有多处并发执行，需保证单线程/互斥与单进程。

3. **若确认为 MoveIt 超时（client_cancel）**  
   - 适当增大 `allowed_execution_duration_scaling` 或 `allowed_goal_duration_margin`（在 moveit_controllers.yaml 或 launch 中）。
   - 或检查 MoveIt 源码/文档中 execute 的 timeout 参数，看是否可调大。

4. **若确认为 2s 内无反馈（watchdog）**  
   - 检查 ROS1→ROS2 的 aubo/feedback_states 是否持续、及时发布。
   - 必要时将轨迹 action 的 2s 看门狗改大（仅作诊断用，长期应保证反馈正常）。

## 六、错误码 -6（TIMED_OUT）与“两次 goal”现象

### 6.1 -6 的含义

- **MoveIt**：`MoveGroupInterface::execute()` 返回 **-6** 表示 **TIMED_OUT**（MoveItErrorCodes）。
- **含义**：MoveIt 的 trajectory_execution 在 **允许时间上限内** 没有收到 action 的 SUCCESS，于是 **主动取消** 当前执行并报超时。
- **允许时间**：`trajectory_duration × allowed_execution_duration_scaling + allowed_goal_duration_margin`（例如 0.63×15+5≈14.45 s）。

### 6.2 你提供的日志说明的问题：同一逻辑请求触发了两次 ExecuteTrajectory goal

时间线摘要：

| 时间 | 事件 |
|------|------|
| 1772008029.140 | move_group 收到第一个 **ExecuteTrajectory** goal，开始执行 |
| 1772008029.159 | aubo_ros2_trajectory_action 接受第一个 goal，发布轨迹 |
| 1772008029.924 | **reach target**（第一个轨迹执行完成） |
| 1772008029.987 | move_group：**Execution completed: SUCCEEDED**（第一个 goal 成功） |
| **1772008029.988** | move_group：**Received goal request**（**第二个** ExecuteTrajectory goal，与第一个仅隔约 1 ms） |
| 1772008030.007 | aubo_ros2_trajectory_action 接受**第二个** goal，发布第二条轨迹 |
| 之后 | **没有** 第二次 “reach target” |
| 1772008044.462 | **waitForExecution timed out**（约 14.45 s），MoveIt 取消第二个 goal（client_cancel），报 **TIMED_OUT** |

结论：

1. **同一次“用户操作”触发了两次 ExecuteTrajectory**：第一个 goal 正常完成，第二个 goal 在约 14.45 s 后被 MoveIt 因超时取消，所以你看到 -6。
2. **-6 来自“第二个 goal”**：第二个 goal 从未出现 “reach target”，可能是：  
   - 前端/桥接对同一位姿发了**两次** set_robot_pose（两次 move_to_pose 请求），或  
   - 某处逻辑对**同一次** move_to_pose 调用了**两次** execute()（当前 joint 分支应只有一次 execute()，若仍出现需再查）。
3. **为何第二个 goal 一直不完成**：可能原因包括：控制器/ROS1 端对**背靠背**两个 goal 只执行了第一个、第二个被忽略或未正确反馈；或第二条轨迹与当前状态/配置不匹配导致从未 “reach target”。

### 6.3 建议排查与修改（针对 -6 与“两次 goal”）

1. **确认“一次操作只对应一次请求”**  
   - 前端：同一次点击/操作是否触发了两次 `set_robot_pose`（看 network 或控制台 req_id / 防重日志）。  
   - 桥接：同一 req_id 是否只调用一次 move_to_pose 服务；SEND/RECV 是否成对且无重复。  
   - C++：同一 move_to_pose 回调内是否只调用一次 `move_group_->execute()`（关节分支应只有一处 execute）。

2. **利用已有去重**  
   - move_to_pose_server 已做“同目标、2 s 内成功则直接返回成功”的去重。若第二次请求的**目标位姿与第一次完全一致**，应被去重、不会发第二次 execute。  
   - 若仍出现两次 goal，说明：要么两次请求的**位姿不完全相同**（例如浮点/四舍五入差异），要么第二次请求在**第一次尚未返回成功**时就已经发出（例如双请求并发），需要从前端/桥接收紧“防重”或请求合并。

3. **允许时间与 velocity_factor**  
   - 当 **velocity_factor 很小**（如 0.1）时，实际执行时间 ≈ 规划时长/velocity_factor；当前配置 15×+5 s 一般够用。  
   - -6 的直接原因是**第二个 goal 在 14.45 s 内未完成**；根本上是**不该有第二个 goal**，而不是单纯把超时时间调大。

4. **若确认为“背靠背两个 goal、第二个不完成”**  
   - 优先保证：**一次用户操作 → 一次 move_to_pose 请求 → 一次 execute() → 一个 FollowJointTrajectory goal**。  
   - 其次再视需要：在 trajectory action 或控制器侧处理“连续两个 goal”的行为（例如队列、或拒绝过快的第二个 goal），避免第二个 goal 一直挂起导致 -6。

---

## 六（原）小结（-4 与 -6）

- **-4**：MoveIt execute() 得到“失败”，来源于 **FollowJointTrajectory action 被 abort**（client_cancel、watchdog、stop、new_goal 等）。
- **-6**：MoveIt execute() 得到 **TIMED_OUT**，即允许执行时间内未收到 action 的 SUCCESS，MoveIt 主动取消；常见诱因是**同一逻辑请求触发了两次 goal**，第二个 goal 一直未完成。
- 机械臂若实际已到位，说明底层已执行完轨迹，但 **action 在 MoveIt 等待结果之前或之内被 abort/超时**。
- 定位时以 **aubo_ros2_trajectory_action 的 `abort, reason=XXX`** 及 **move_group 的 “Received goal request” 次数** 为准，再结合上述原因逐项排查。

---

## 七、典型现象：Execute 接受后约 20ms 内被 abort

### 7.1 日志特征（与你提供的日志一致）

- `Execute request accepted` 与 `Execute request aborted` 间隔约 **19～44 ms**。
- 轨迹时长本身为 0.08～0.09 s（关节 2 点）或笛卡尔约 1.4 s，**不是** 2 秒看门狗触发。
- 说明：abort 发生在“刚接受执行”之后极短时间内。

### 7.2 可排除的原因

- **watchdog_no_feedback_2s**：需“接受后满 2 秒仍无反馈”才触发，与 20ms 不符，可排除。
- **new_goal_received**：单次请求、串行调用时，一般不会在 20ms 内又来新 goal，除非有并发或重复请求，需结合 trajectory action 的 reason 确认。

### 7.3 最可能的原因：client_cancel（MoveIt 侧起始校验失败）

- **client_cancel** 表示 **FollowJointTrajectory 的客户端（MoveIt）主动发了 cancel**。
- MoveIt 在执行前/刚下发轨迹时会做 **起始状态校验**：用当前 **joint_states**（或 planning scene 中的机器人状态）与 **轨迹起点** 比较；若超过 **allowed_start_tolerance**，会认为“起点偏差过大”并 **取消本次执行**。
- 表现就是：Execute 刚被 accept，很快就被 abort，时间在几十毫秒量级，与你的日志一致。

可能的具体情况：

1. **规划用的状态与执行时用的状态不一致**  
   规划时可能用了 `demo_robot_status` 或某时刻的 joint_states，执行时校验用的是最新的 `/joint_states`；若两者有延迟或不同源，容易超容差。

2. **allowed_start_tolerance 偏严**  
   当前为 0.1 rad（约 5.7°）。若机械臂实际到位略有抖动、或 joint_states 更新略慢，单轴差个几度就会超 0.1 rad，触发取消。

3. **trajectory_execution_event=stop**  
   若有节点在 Execute 后立刻发布 `trajectory_execution_event` 的 "stop"，轨迹 action 会因 **trajectory_execution_event_stop** 而 abort；时间上也可以是几十毫秒。需在 trajectory action 日志中确认 reason。

### 7.4 建议操作（按顺序）

1. **确认实际 abort 原因**  
   在出现 -4 的同一时刻，查看 **aubo_ros2_trajectory_action** 终端或 debug 日志中的 **`abort, reason=`**：
   - 若为 **client_cancel** → 按下面 2、3 处理。
   - 若为 **trajectory_execution_event_stop** → 查谁在发布 `trajectory_execution_event`，为何在刚执行就发 stop。

2. **适度放宽起始容差（仅当 reason=client_cancel 时推荐）**  
   在 `moveit_controllers.yaml` 或 launch 的 trajectory_execution 参数中，将 **allowed_start_tolerance** 从 0.1 提高到 **0.15**（约 8.6°）再试；若仍偶发 -4，可再试 0.2。

3. **保证 joint_states 与规划一致**  
   确认 `/joint_states` 来源（如 ROS1 桥、真实控制器）频率足够、无大延迟；若规划用的是 `demo_robot_status`，确认与 `/joint_states` 是否同源或至少不要长时间不同步。

---

## 八、诊断日志：如何根据 debug 日志定位是哪一部分出问题

轨迹 action 会向 **`.cursor/debug-157b9d.log`**（或当前 session 的 debug 日志）写入 NDJSON，用于区分“谁在什么时刻取消了执行”。

### 8.1 关键事件与字段

| 事件 | 含义 | 关键字段 |
|------|------|----------|
| **accept** | 轨迹 action 接受了新 goal | `trajectory_start_positions`：本轨迹的起点关节角（rad） |
| **abort** | 本 goal 被中止 | `reason`、`time_since_accept_sec`、`feedback_received_before_abort`、`trajectory_start_positions`、`last_feedback_positions`、`max_delta_start_vs_feedback_rad` |
| **success** | 本 goal 执行成功 | `time_since_accept_sec` |

### 8.2 根据 abort 日志判断“哪一部分”出问题

1. **看 `reason`**  
   - **client_cancel**：取消来自 **MoveIt（action 的 client）**，即“执行/校验”这一侧（MoveIt 或 trajectory_execution_manager）认为不能继续而发 cancel。  
   - **trajectory_execution_event_stop**：有节点发布了 `trajectory_execution_event=stop`，问题在“谁在发 stop”。  
   - **new_goal_received**：同一时刻有多个 goal，问题在“请求是否重复/并发”。  
   - **watchdog_no_feedback_2s**：2s 内未收到 aubo/feedback_states，问题在“反馈话题/控制器”。

2. **看 `feedback_received_before_abort`**（仅对 client_cancel 有用）  
   - **false**：abort 前 **从未收到** 控制器 feedback。说明 cancel 发生在“刚下发轨迹、机器人几乎还没动”的阶段，多半是 **MoveIt 的起点校验**（轨迹起点 vs 当前 joint_states）不通过，或内部超时/逻辑取消。  
   - **true**：abort 前 **已经收过** feedback。可再看 `max_delta_start_vs_feedback_rad`：若明显大于 `allowed_start_tolerance`（如 0.2），说明“轨迹起点”与“控制器反馈的当前位”偏差大，有助于确认是起点/状态不一致导致被取消。

3. **看 `time_since_accept_sec`**  
   - 若 **&lt; 0.1 s**（几十毫秒）：与“接受后很快被 abort”的现象一致，通常对应 **起点校验失败** 或 **trajectory_execution 立刻发 stop**，而不是 2s 看门狗。

4. **对比同一请求的 accept 与 abort**  
   - 用时间戳或前后事件顺序，找到“同一次执行”的 `accept` 和 `abort`。  
   - `accept` 里的 `trajectory_start_positions` = 本轨迹起点。  
   - `abort` 里的 `trajectory_start_positions` 应与之相同；若有 `last_feedback_positions`，则 **轨迹起点 vs 最后 feedback** 的差值（`max_delta_start_vs_feedback_rad`）可用来判断是否“起点偏差过大”。

### 8.3 结论对应关系（快速查表）

| 现象 | 日志特征 | 出问题的部分 |
|------|----------|--------------|
| 接受后几十 ms 就 -4，reason=client_cancel，feedback_received_before_abort=false | 刚接受就被 MoveIt cancel，且没有控制器反馈 | **MoveIt 侧起点校验**（轨迹起点 vs joint_states / planning scene）或内部超时 |
| 接受后几十 ms 就 -4，reason=trajectory_execution_event_stop | 有 stop 事件 | **谁在发 trajectory_execution_event=stop**（如 trajectory_execution_manager 的配置或其它节点） |
| 同一逻辑请求出现两次 accept 或两次 before_exec | 同一 pose 对应多个 goal | **请求重复/并发**（前端/桥接/C++ 串行与去重） |
| 接受后约 2s 才 -4，reason=watchdog_no_feedback_2s | 2s 内无 feedback | **aubo/feedback_states** 未发布或未桥接 |
| abort 时 feedback_received_before_abort=true 且 max_delta 很大 | 有反馈，但起点与反馈位偏差大 | **轨迹起点与当前状态不一致**（规划用状态 vs 控制器/joint_states） |
