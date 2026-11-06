# Web 调试与测试使用手册

本文说明当前五页 Web 界面的操作、测试入口与证据判定。Web 静态资源由 `peach_observability` 托管；不要用独立静态服务器打开 HTML，否则 `/api/*` 没有对应后端。本文给出操作方法，不表示模型配额、完整抓取或三十分钟稳定性验收已经通过。逐功能核对与待测项见 [调试功能验收清单](debug-feature-checklist.md)。源码入口见 [`peach_web/README.md`](../README.md)；构建后阅读副本位于 `$(ros2 pkg prefix peach_web)/share/peach_web/doc/debug-testing.md`，源码交叉链接按仓库目录组织。

## 1. 适用范围与操作前提

- **默认在 mock 使用。** mock 可验证接线、参数、动作协议、模型身份和账本；不能证明真实碰撞、切断、接触力或硬件急停安全。真实相机 + mock 仍使用真实相机、GPU、HTTP 端口等独占资源。
- **真机另需操作员授权。** 未授权不得真机运动或 SetIO；操作员发起 real launch/操作台指令具有授权语义。网页确认框、某个 `true`、`allowed=true` 都不能代替人的授权。先在 mock 核对，再按授权范围逐档操作；工具与动臂同级管理。
- 硬件急停在柜/示教器，不经 ROS。网页的取消、停止、`RobotMoveStop` 是应用停轨。保护停止后先处理现场、取消并丢弃旧 goal、示教器复位，再重新授权下发；禁止恢复旧轨迹。不使用 dashboard 远程上电或复位。
- 驱动栈只读。Python 使用工作区 `aubo_py3.12`，`numpy == 1.26.4`；不要升级 numpy 或另装同名 ROS Python 包。
- 一轮一个新输出目录和新请求 ID；不将历史截图、账本或浏览器协议模拟当成本轮物理链路证据。所有测试结束必须停栈、封口 bag 并核对本轮进程。

完整的安全、真机命名及验收规则见 [系统测试指导](../../peach_system_tests/doc/testing.md) 和仓库根 `AGENTS.md`。

## 2. 准备环境与启动

### 2.1 构建与环境检查

从工作区新终端执行，先 ROS，再工作区，再 venv：

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to peach_system_tests peach_web peach_stereo peach_vegetation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
source aubo_py3.12/bin/activate
export PYTHONPATH="$PWD/aubo_py3.12/lib/python3.12/site-packages:$PYTHONPATH"
python3 -c 'import sys,numpy,cv_bridge,open3d; print(sys.executable,numpy.__version__,cv_bridge.__file__,open3d.__version__); assert numpy.__version__ == "1.26.4"'
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|ros2 bag|move_group|ros2_control_node|robot_state_publisher|peach_|lifecycle_manager|rviz2'
```

确认解释器来自工作区 venv，`cv_bridge` 来自 Jazzy。若环境污染，开新终端按顺序重载；依赖或模型资产缺失应记为环境失败。启动前确认本轮 DDS 域、相机、力串口与 HTTP 端口可用；发现其他人的实例只报告，不清理。更换 DDS 域不能解除相机、端口或 GPU 占用。运行读取 `install/` 中的资源，修改静态页、脚本或配置后须构建对应包。

### 2.2 无相机 mock：仅检查启动与页面连接

```bash
export ROS_DOMAIN_ID=81
export ROS_LOCALHOST_ONLY=1
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=false force_enabled:=false rviz:=false autostart:=false \
  reconstruction_enabled:=true skip_reconstruction:=true
```

默认保留重建节点供页面切换，但本轮不使用重建。`reconstruction_enabled` 是部署选择，`skip_reconstruction` 是运行策略，两者不能混用。观察生命周期、自检、诊断和页面连接。没有相机时不能期待场景锁定、模型或抓取；零目标正常结算不是抓取成功。`force_enabled=false` 时五点力显示未启用是预期。完成后按第 9 节停栈。

### 2.3 真实相机 + mock：复用现有联合 runner

不要同时启动上面的整栈。先核对 `src/peach/peach_system_tests/config/live_camera_mock.yaml` 的固定采集位校准：`base_link → camera_link`、来源、相机是否移动及 mock 初始关节位。该 TF 只适用于对应固定相机试验，不能把历史数值当成当前标定。相机移动后须重测；未完成锚定只检查静止出图，不发抓取。runner 关闭腕随动外参，自己发布这一条固定 TF，不要额外起重复广播器。

```bash
ros2 run peach_system_tests run_live_camera_mock.py --help
ros2 run peach_system_tests run_live_camera_mock.py \
  --rounds 0 --models 0 --no-reconstruction --soak-seconds 1800 --domain 86 \
  --output "runs/web_debug_$(date +%Y%m%d_%H%M%S)"
```

默认 `--models 0`、不使用重建；`--no-reconstruction` 在示例中显式说明比较基线。需要开启重建并采集一个模型时，单独运行：

```bash
ros2 run peach_system_tests run_live_camera_mock.py \
  --rounds 0 --models 1 --reconstruction --soak-seconds 0 --domain 86 \
  --output "runs/web_model_on_$(date +%Y%m%d_%H%M%S)"
```

`--models > 0` 必须同时指定 `--reconstruction`。runner 两种模式均部署重建节点；默认关闭自动采集与 Build 路径，空闲时可在 Web 切换。参数名和值必须分开写，例如 `--rounds 0`、`--models 0`，不能写成 `--rounds0`。上方 `--models 0 --soak-seconds 1800` 的默认交互命令启动真实 stereo 相机、mock 臂、观测服务与 RViz，不由 runner 发模型采样或抓取轮，留出最多 1800 秒供网页操作。它是有时间上限的联合入口：预算到或失败会收尾；需要提前结束时先完成网页收尾，再在 runner 终端按 Ctrl+C。人工中断保留失败/中断记录，不要求报告变绿。不要把设置了 1800 秒当成三十分钟长测通过；人工动作也应单独记录，不能算作 runner 的零抓取轮配额。

runner 要求域内没有既有采摘进程，并只接受 mock；默认用 8090，冲突时选 8091–8098，实际端口见终端及本轮 `report.json` 的 `web_port`。它不占 8099。以下 URL 的端口须随之替换。默认开 RViz；`--no-rviz` 可用于无头排障，但本轮不能声称验证了 RViz。

runner 工具固定关闭，不自动 ACK。只模型/soak 不打开执行与抓取；默认产品配置为 PREGRASP_ONLY。需要终端驱动一轮预抓取时可用 `--rounds 1 --models 0 --soak-seconds 0`；到位 L2 后停止后续请求并保留恢复等待。需要 runner 驱动 FULL 时，用 `--task-params-file /绝对路径/job.yaml` 提供标准 ROS 参数文件，例如：

```yaml
peach_supervisor:
  ros__parameters:
    execute_pregrasp_only: false
    action_timeout_s: 90.0
peach_arm:
  ros__parameters:
    moveit.execute_timeout_s: 15.0
```

该文件设置本作业执行档与等待预算；工具仍关。90/15 秒是此处完整干跑的显式示例，须结合实际阶段耗时核对，不改变产品默认或质量门。不要在作业文件设置 `record.root_dir` 或 `session.root_dir`，runner 拥有本轮 `records/`。不要在作业文件写 `skip_reconstruction`、`capture.auto_mode` 或 `quality.allow_unrefined_geometry` 等运行策略，使用 runner 的 `--reconstruction` / `--no-reconstruction`，或在空闲时用页面统一写入并读回。Web 手动操作与终端动作 runner 不应同时发任务。

### 2.4 真机入口

仅在已获授权、示教器就绪、工作空间隔离、硬件急停可及时使用生产入口：

```bash
ros2 launch peach_bringup harvest_system.launch.py \
  hardware_mode:=real robot_ip:=169.254.10.98 camera_enabled:=true \
  reconstruction_enabled:=true skip_reconstruction:=false \
  autostart:=false rviz:=true
```

保持使能和工具关闭，确认机器人状态、关节、相机/TF与工具状态后，再按本次授权逐档写入。真机速度/加速度/转移缩放不得超过 0.1；网页可输入范围不等于真机工艺许可。不得复制固定相机 mock 校准、跳过重建、允许未精化几何或其他测试放宽项到真机。`run_live_camera_mock.py` 和 Web E2E 脚本都不能用作真机测试入口。

## 3. 打开五页

默认同一后端地址为 `http://127.0.0.1:8090`。不要只凭页头“已连接”判定每路相机、模型和力数据都是新鲜的。

页面从 `/api/transport` 读取部署元数据。默认整栈启用回环地址 `127.0.0.1:9090` 的 WebSocket，可用 `rosbridge_port` 更改端口。实时页按需加载同源的 `ros2_web_link` SDK，并按页面需要订阅 `/peach_observability/web_state`、`web_force` 或 `web_trajectory`。五点力页应只有 `web_force` 订阅，历史分析页应没有 WebSocket。参数使用标准 List/Get/Set 服务，业务操作使用现有 ROS 服务和 Action。独立观测节点及 `rosbridge_enabled:=false` 部署以 HTTP 为默认传输。

保留 HTTP 显式对照入口，例如 [HTTP 调试页](http://127.0.0.1:8090/debug.html?transport=http)；默认 [rosbridge 调试页](http://127.0.0.1:8090/debug.html)。切换前先结束当前请求，分别保存传输方式、URL、请求身份和证据。WS 断线不自动降级重发业务请求；重连只恢复订阅，不重放服务或动作。默认只允许应用端点，不能将驱动或 SetIO 暴露给网页。原 HTTP `debug.motion_enabled` 门与 `debug_audit` 只覆盖 HTTP 调试桥；直接 ROS 调用依赖节点自身使能及臂侧 `ExecutionAuthority`，不能声称 HTTP 门或审计自动覆盖 WS。

HTTP 与 ROSbridge 请求均以单调时钟核对本地等待时限。浏览器渲染阻塞导致定时器迟到时，超时或取消后收到的响应不能当成本地等待成功；已收到的逐项确认仍作为核对证据保留。自动选择传输消耗同一时限，HTTP 回退前再次检查，不能在本地超时后才发出命令；迟到的网络异常也按时限分类。未获得确认的写命令结果未知，须读回状态，不自动重发，也不等同于远端动作已取消。

ROSbridge 的初始化、调试门读回、多项参数及最终服务/动作发送共用该绝对截止时间。每次调用 SDK 前重新检查剩余预算；前一步已耗尽预算时，不再发送后续参数或命令。检查脚本支持 `--transport rosbridge --only debug`，草稿检查同时统计 HTTP POST 和 WS 写入帧；带 HTTP 响应夹具的页面仍走 HTTP 对照测试。

多项写入中途超时或取消时，HTTP 与 ROSbridge 已收到的逐项确认均保留在错误响应中并显示在参数面板。ROSbridge 发送前截止的项标为“未发送”，已交给 SDK 但无确认的项标为“结果未知”；后续项停止发送。部分成功不回滚，也不自动重试。

总览和过程监控的 FPS 使用感知发布的 `timing.fps`；`timing.total_ms` 是单帧处理耗时。不能用处理耗时的倒数、模型纯推理速度或网页轮询频率替代相机/感知实际吞吐。

- [总览](http://127.0.0.1:8090/overview.html)：先看运行模式、图像、自检、流程、当前完成证据与诊断；根路径 `/` 也到总览。
- [过程监控](http://127.0.0.1:8090/monitor.html)：看实时/感知图像、3D、目标顺序、几何、实际工具/TCP、阶段时序、事件和 ROS 日志。提供“回到拍照位”“取消执行”“取消批次”；回拍照位可能运动，不能用它代替恢复确认。兼容路径 `/index.html` 对应本页。
- [单步调试](http://127.0.0.1:8090/debug.html)：三档测试草稿、本轮设置、单轮点选、多轮策略、白名单参数；建议与过程监控并排打开。
- [数据分析](http://127.0.0.1:8090/analysis.html)：选择本轮顶层记录目录，点击“解析”，核对逐目标终态、完成级别、原因、阶段与几何。解析失败时不能沿用旧结果；离线分析不会改变监控页当前批次。
- [五点力](http://127.0.0.1:8090/force.html)：看五通道、合力、CoP、行程/AD/状态与视角。断线或陈旧时 CoP 不应继续显示为实时值；力源关闭时仍可查看标称点位。力读数不是抓取许可或切断确认。

## 4. 本轮设置与三档按钮顺序

先等状态在线、参数读取完成，确认没有在途动作和恢复锁。查看“当前批次证据”的运行环境。**选择目标档只更新说明；点击“载入为待写设置”只填本地草稿；点击“写入”并在确认框选择“写入并应用”才发送请求。** 开始单轮/多轮时还会重新写入当前设置并读回，不能把草稿勾选当成已生效。

写入先设置并读回 `debug.motion_enabled`、`execute_pregrasp_only`、`skip_reconstruction`、`capture.auto_mode`、`quality.allow_unrefined_geometry`，成功后才应用 `SetEnables`。随后调度与臂侧三项使能都须读回一致；超过等待窗口仍不一致就是写入未确认，不得继续。

勾“执行/抓取/工具”会联动打开运动放行；抓取会带上执行，工具会带上执行和抓取；关闭运动会清掉三项使能草稿。**`FULL` 的含义是允许走套入等后续阶段，工具是否动作还取决于工具使能。** 本手册三档一律工具关；**载入三档保留“本轮使用重建”的当前选择**，默认未勾选。工具开关只用于明确授权的工艺操作，本手册不测试 SetIO。

### 4.1 空闲切换重建并比较前后效果

1. 停止单轮和多轮，确认动作空闲、无恢复锁/未知请求；等本轮参数读取完成。取消确认框不发送写请求，确认写入使用冻结草稿，读写期间保持互斥。
2. 默认 OFF：不勾“本轮使用重建”→“写入”→确认。逐项读回 `skip_reconstruction=true`、`capture.auto_mode=false`、`quality.allow_unrefined_geometry=true`；含义是使用感知几何，不调用 Build。
3. ON：勾选后同样确认写入。逐项读回 `skip_reconstruction=false`、`capture.auto_mode=true`、`quality.allow_unrefined_geometry=false`，才算进入 Build/精化路径。上述三项不是跨节点原子事务；部分成功或混合模式应阻止开始，先核对实际值再处理差异。
4. 分别以新 request/epoch 运行同一档，记录图像、目标、耗时、几何来源、模型修订/有效期、终态和失败原因。OFF 的旧重建模型不得显示为当前证据；ON 拒绝几何不能靠切 OFF 改判为同一请求通过，两组各自留存。
5. `reconstruction_enabled:=false` 表示节点未部署，页面不能开启节点；需要以 `reconstruction_enabled:=true` 重新部署。节点存在、处于 Active、自动采集开启、本轮使用 Build 是不同事实，图里存在节点不代表本轮正在重建。

目标 TSDF/Build 是局部目标几何精化；MoveIt `PlanningSceneMonitor` 维护机器人状态和碰撞环境。关闭目标重建不等于关闭 PlanningScene、TF 或碰撞检查，也不证明环境安全。

### 4.2 观察与建场景

1. 选“01 · 观察与建场景”→“载入为待写设置”。核对运动、套入、执行、抓取、工具均未勾选；重建选择保持比较组的 OFF 或 ON。
2. 点“写入”→“写入并应用”；确认调度意图及臂侧实际使能均为“关 / 关 / 关”。
3. 点“启动单轮点选”→确认“启动单轮”，等待新 epoch 和目标锁定。这个操作只建立场景并保持点选会话。
4. 在过程监控核对图像新鲜度、锁定目标、优先级、TF和标注；**观察档不要点击顺序按钮或“抓取已勾选”，不要启动多轮**。这些按钮会尝试下发抓取请求，节点自身使能与臂侧授权应拒绝实际执行，拒绝不能当观察成功。
5. 点“停止单轮”，确认会话退出。

### 4.3 预抓取核验 · L2

1. 选“02 · 预抓取核验 · L2”→载入草稿。核对运动、执行、抓取开；套入、工具关；重建选择保留，逐项核对第 4.1 节对应组。
2. 点“写入”并确认；调度和臂侧均须执行开、抓取开、工具关。
3. 启动单轮并确认；锁定后只点一颗当前目标的顺序按钮。
4. 记录本次 request/target、epoch、模型身份和动作结果。期望预抓取到位并核验后账本 **L2**，保持停驻，进入人工恢复等待；不套入、不 SetIO、不自动回 stow。独立目视/测量核对实际到位位置和方向，不能仅凭 L2 文本认为几何正确。
5. 后续队列应停止。点击“停止单轮”清队列/取消在途批次，等动作不再活动，再按第 6 节人工确认恢复。不要自动 ACK 后继续下一颗。

### 4.4 完整干跑 · L6

1. 选“03 · 完整干跑 · L6”→载入草稿。核对运动、套入、执行、抓取开；**工具关**；重建选择保留。
2. 点“写入”并确认；两侧使能读回均为“开 / 开 / 关”，重建三参数与所选 ON/OFF 一致。
3. 启动单轮后选一颗；或按第 5 节选多轮策略。
4. 核对同一请求动作终态成功、账本 `SUCCEEDED` 且 L6、已撤退、没有终局恢复锁。FULL 进行中的临时接触恢复标志不能直接判为已停驻待 ACK；要结合批次状态与动作是否仍在运行。
5. 关闭使能并按第 9 节收尾。工具关闭时 L6 只证明干跑套入和撤退，不能证明切断，更不是 L7 采摘确认。

## 5. 单轮点选与多轮

### 5.1 单轮保持点选

“启动单轮点选”确认一次后重新开场景，等待当前 epoch 目标锁定；本轮一直保持，直到停止、失败或恢复等待。按钮上的“顺序 1、2…”来自 `priority`，不是 `target_N` 的数字大小。

- 直接点某颗顺序按钮：将这一颗加入队列，抓完继续等待点选。
- 勾多个“点选”框后点“抓取已勾选”：按当前选择发送同场景目标；每个目标仍须下游检查，不代表批量授权绕过质量门。
- 正在执行时再次点选会排队，页面显示后续目标；不要将重复点击当成查询。
- 换 epoch 会清选择和队列，旧场景目标不得继续使用；失败或恢复等待会停止并清空后续队列。
- “停止单轮”先撤销保持和排队；仍在执行或结果未知时发送 `CANCEL_NOW`。提示“单轮已停止”本身不等于臂已停，仍须核对动作终态和臂 `running=false`。

### 5.2 多轮三种策略

先停单轮，写入设置，再填 **1–30 的整数**轮数、选方式→“启动这些轮”→确认“启动 N 轮”。每轮使用新 request 和新场景；前一轮必须全部成功，失败、未知或人工恢复等待会阻止后续轮。成功 Action 结果与状态话题可能先后到达：页面最多等待 5 秒，核对同 request 已结算、动作/臂空闲、双侧恢复锁清除且状态新鲜后才开始下一轮；超时保留待核对请求，不自动重试。

- **每轮按顺序换一颗**：本轮锁定目标数为 N，轮索引依次选 `(index % N) + 1`。这是每场优先级轮换，不保证跨场相同 ID 就是同一物理果实。
- **每轮第一颗**：每轮重开场景，选当前 `priority=1`。
- **每轮全部**：使用 `PICK_ALL/AUTO`，不限顺序号；本轮全部目标都应核账本。不能用一颗成功代替整批成功。

“停止后续轮”也会发送 `CANCEL_NOW`，并非只禁止下一轮。恢复等待或结果未知时，该取消入口仍可使用。查看最新轮次日志、同请求账本和动作状态确认结束。PREGRASP_ONLY 设计上到 L2 后等待人工恢复，不能期待无人确认连续跑完多轮。需要多轮干跑时使用工具关闭的 FULL，并保留所有失败轮。

## 6. 恢复锁、未知结果和取消

“确认恢复”只在状态新鲜、调试开启、旧批次已结算或中断（值 6/8）、`recovery_required=true`、动作空闲、臂不运行且有有效 `state_seq` 时开放。`RECOVERY_REQUIRED`（值 7）说明旧批次仍可能在等恢复：先点“停止单轮”或“停止后续轮”，等取消结果和批次终态，不要直接解除锁让旧 `PICK_ALL` 继续派下一颗。

操作员核对现场接触、工具、停驻位后，再点击“确认恢复”→“现场已确认”。页面提交当前非零状态序号；取消确认框不得发 ACK，确认期间状态变化会要求重新核对。返回接受后，核对调度 `recovery_required=false`、臂 `contact_recovery_required=false`、臂空闲，以及事件 `recovery_acknowledged`。旧 L2 账本仍须为 L2，不能变成 L6。ACK 后应没有新动作或目标自动派发；需要继续时重新载入/写入设置，以新 request 启动。刷新页面不会恢复本地队列或重发任务；若服务器仍在运行或处于 L2 恢复等待，停止入口会按新鲜服务器状态开放，须显式点击取消并确认终态后再 ACK。ACK 不恢复旧轨迹，不自动武装单轮或继续队列。

默认 WS 路径先看 ROS 响应中的 `accepted/success`、逐项参数结果、Action feedback/result 和对应请求身份；服务成功不等于动作成功。WS 超时/断线时操作可能已执行，应保留未知请求并停止后续，不自动重放，重连后查同 request 或取消。SDK 等待超时后保留本页句柄并阻止重复提交。同一 WS 会话可请求取消。重连后通过调度或技能取消入口核实原请求终态。HTTP 对照路径的状态和业务结果要分别看：

- **200**：仍要核对 `accepted`、内层 `accepted/success` 和动作终态；服务受理不是抓取完成。
- **202**：动作仍在执行/未返回终态，保留句柄与 request；不自动重发。单步会话保留未知请求并停后续轮，先查同请求状态或取消。
- **400**：请求结构、类型、范围或白名单失败。参数修改全部失败也返回 400。检查具体 `results[].message`，更正草稿后再提交。
- **207**：参数部分成功；已成功的项可能已生效，不能当事务回滚。保留草稿，读回逐项对照，仅处理未达预期的项。
- **423**：运动类门未放行等明确拒绝。参数桥对运动参数未放行也可能以 400/207 加逐项拒因返回；不应把所有门拒绝都只按 423 搜索。
- **500 / 网络超时或断线**：ROS 操作可能已执行，视作结果未知；保存 request、响应/异常和最后状态，勿直接重发。**503** 通常为调试关闭、后端/服务不可用，同样先核对具体消息。

取消可用单步页“停止单轮 / 停止后续轮”，或过程监控“取消批次 / 取消执行”。“取消批次”提交调度的 `CANCEL_NOW`。“取消执行”经确认调用臂侧 `/peach_arm/cancel_cycle`，作用于臂当前操作，可从另一个页面使用。取消请求受理也不等于已经停止。确认调度 `action_active=false`、臂 `running=false`、对应 action 的终态；未知锁只在同 request 空闲且无恢复要求后解除。重开网页不能证明旧动作已结束。

## 7. 参数读写与证据

### 7.1 参数操作

1. 停止单轮/多轮，确认没有在途动作。展开“参数（白名单读写）”。参数按节点分组，显示用途、类型、范围、“运动/重启”标记。
2. 在“搜索参数”输入参数名、用途或节点，如 `selection_priority`、`速度`、`peach_arm`；“只看待写入”仅保留与当前值有差异的行。
3. **“当前”是上次读回值，“待写值”是本地输入。** 修改后行高亮且待写数量增加，改回相同数值应清除差异；`1` 与 `1.0` 对数值类型视为相同。
4. 输入不能为空；数字须有限、整数项不能填小数，且遵守行上范围。布尔项与枚举只能从选项取值。上下限等跨参数关系仍由节点检查，不因每项单独合法就保证组合有效。
5. 点“应用已修改”，必须全部写入并读回一致才报成功。400/207 或其他失败时保留输入核对；参数写入不是原子事务，不保证之前已写的项回滚。
6. 本轮设置和参数面板的“重新读取”都会刷新当前值，**保留尚未应用的参数草稿，并按最新当前值重新计算差异**。如果外部已把参数改为草稿值，该项将不再标为待写；仍有差异的项继续保留。参数读取失败也保留原输入，并提示“当前值尚未刷新”；此时不要把旧当前值当作最新读回。写入成功后草稿应与读回值一致。

未运行节点、未声明参数不可编辑；先查服务/声明，不用默认值替代未知。HTTP 对照路径中标“运动”的参数须先打开 `debug.motion_enabled`；WS 使用标准节点参数服务，节点自身参数约束、使能和臂侧授权与真机限值仍须成立。“重启”参数（如录包开关/档）写入后须按部署配置重启并读回确认，当前会话不会因参数显示改变就自动重开录包。参数修改只改运行期 ROS 参数，不自动持久化源 YAML。

**超时参数不能混用：** `peach_supervisor.action_timeout_s` 是 Survey/Build/Execute 的动作等待上限；`build_start_timeout_s` 只管 Build 启动；`peach_arm.moveit.execute_timeout_s` 是单段轨迹执行等待。参数面板可分别搜索并读回。完整过程包含接近、套入、撤退和回拍照位，必须给动作等待留足全部阶段预算。`per_target_timeout_s` 是 RunHarvest goal 字段，不是 supervisor 节点参数；把它写进节点 YAML 不会替代 `action_timeout_s`。日志 `action result timeout` 后即使账本已到 L6，若 outcome=CANCELED 仍是失败，不可只按级别判断。

HTTP 调试桥的写入含拒绝会审计到本轮根下 `debug_audit/<YYYYMMDD>.jsonl`；普通整栈默认根为工作区 `runs/`，runner 子进程根为输出目录的 `records/`。GET 参数读取不产生写审计。WS 直接参数/业务调用不经过此 HTTP 审计，需保存 ROS 返回值、Action 结果及本轮会话记录。

### 7.2 一次请求应保留什么

至少同时记录：

1. 硬件模式、配置/版本、DDS 域、相机输入 stamp 与帧龄；页面状态新鲜不代表模型仍有效。
2. `request_id` 与调度 `run_id`、`scene_epoch`、`target_id`、`priority`；换场可能复用 `target_0`，不能只凭 ID 关联历史证据。
3. 模型 `model_revision`、工具/标定/配置修订、生成时间和 `valid_until`，以及同身份的 GraspDecision/clearance。重建 ACCEPT、原始 `allowed`、臂实际授权分别判断；过期许可不能刷新成新许可。
4. 调度使能意图与臂侧实际使能分别读回。调度全开而臂未更新/已拒绝时，不应继续。
5. action 状态/业务 `success`、尝试/成功/失败数量、稳定 `failure_code`，及同请求账本 `outcome`、`completion_level`、原因。Web 当前证据只显示与当前 `run_id` 匹配的账本。

L2=预抓取已核验并停驻待恢复；L6=撤退已确认（工具关时是完整干跑）；L7=采摘已确认，要求真实切断及撤退证据。HTTP accepted、action SUCCEEDED、重建 ACCEPT、页面 allowed、L6 都不能单独充当 L7。界面协议断言或参数校验数量也不能换算成真实采摘验收。

## 8. 怎样验证界面确实可用

### 8.1 人工浏览器核对

在本轮 mock 后端上打开五页和浏览器开发者工具，记录时间、URL、窗口尺寸及截图：

1. 三档分别选择和“载入为待写设置”，在 Network 只观察这两步，确认没有业务写入或 Action 请求（WS 检查消息帧，HTTP 检查 POST）；核对草稿与第 4 节一致。关闭确认框或按 Escape 不应提交。
2. 参数搜索→修改→只看待写→改回，核对差异计数；以 `selection_priority=1.5` 检查整数拒绝，以空值、越界值检查未写入。非有限数/非法类型可由现有协议模拟检查，不要为此向真机探测。核对读写期间提交锁和节点离线只读。
3. 用观察档启动单轮，核对只开场景、目标锁定与双侧关使能；不点抓取。先验证取消确认框无请求，再验证停止会话。
4. mock 的 L2/FULL 分开跑，记录同 request/epoch/模型/终态/账本；L2 到位后恢复框先取消一次，确认锁仍在；状态变化时旧 `state_seq` 不得被当作新确认。人工确认后新请求才能继续。
5. 监控页切图像源、暂停/恢复、放大关闭、3D 视角、日志过滤/暂停。另用浏览器离线或协议模拟断线，核对陈旧提示、普通操作禁用、取消入口保留；恢复网络后旧数据显示不可冒充新数据。
   展开事件详情和账本阶段耗时，等待多次快照及新事件；未变化行应保持展开。新批次不沿用旧批次详情状态，日志过滤/暂停仍需生效；无变化列表不应持续替换 DOM。
6. 分析本轮目录，再输入不存在路径，确认明确报错且旧结果隐藏。五点力分别核对在线、源未启用、陈旧、断线；源不可用时不得显示旧 CoP 为实时值。
7. 320、390、768、1440 像素宽分别检查导航、主要按钮、参数行和图像无横向溢出；Tab 可到跳转正文和操作按钮，确认框 Escape 可退出。记录控制台错误及失败网络请求。

协议模拟验证错误处理；实际 ROS 操作验证接线和终态，两类结果分别报告。

### 8.2 现有 Playwright 脚本

入口为 `src/peach/peach_web/scripts/e2e-console.js`。它依赖可被 Node 解析的 `playwright` 和浏览器；包的 `npm test` 只打印 lint 提示，不运行浏览器测试。脚本先检查 `hardware_mode=mock`，默认覆盖五页、响应式、可访问性和协议模拟，并实际创建一次关闭执行的单轮场景会话。它要求后端已有可显示目标/记录等数据，无相机零目标栈不具备完整前提。

先定位已有依赖，不硬编码 npx 缓存哈希，也不要为运行测试自动安装：

```bash
node -p 'require.resolve("playwright")'
# 若上一步不能解析，查找已有缓存中的包，核对后选择本机实际路径：
rg --files --hidden "$HOME/.npm/_npx" -g package.json | rg '/node_modules/playwright/package.json$'
# 将查到的某条绝对路径填入此变量；不要照抄占位值。
playwright_package=/实际缓存路径/node_modules/playwright/package.json
export NODE_PATH="$(dirname "$(dirname "$playwright_package")")${NODE_PATH:+:$NODE_PATH}"
node -p 'require.resolve("playwright")'
command -v google-chrome
```

找不到依赖/浏览器时记录环境缺失。`--browser` 接受实际可执行文件路径；默认 `CHROME_PATH`，再默认 `/usr/bin/google-chrome`。后端实际端口不是 8090 时替换下列 `--base-url`：

```bash
node src/peach/peach_web/scripts/e2e-console.js \
  --base-url http://127.0.0.1:8090 --transport http \
  --out "runs/web_e2e_$(date +%Y%m%d_%H%M%S)"
```

脚本 `--transport http|rosbridge` 默认 **http**，与页面默认rosbridge不同：五页/错误注入fixture套件用HTTP；原生ROSbridge动作核对须显式 `--transport rosbridge --only harvest --harvest`。`--base-url` 只写站点根地址，不把查询串塞入该参数。可用 `--only debug` 单查调试页，`--headed` 显示窗口。`--only` 仅接受 `overview`、`monitor`、`debug`、`analysis`、`force`、`a11y`、`responsive`、`harvest`；未知值会拒绝，`--only harvest` 必须同时带 `--harvest`。前置条件失败会写入失败结果，`report.json` 的 `failed` 应大于 0，不能把没有执行检查看成零失败通过。界面/协议检查不等于实际 FULL 通过。完整网页 mock 干跑单独执行：

```bash
node src/peach/peach_web/scripts/e2e-console.js \
  --base-url http://127.0.0.1:8090 --transport rosbridge \
  --only harvest --harvest --reconstruction off \
  --out "runs/web_harvest_off_$(date +%Y%m%d_%H%M%S)"
```

开启重建的比较组使用同一命令，将 `--reconstruction off` 改为 `--reconstruction on`，另用 `runs/web_harvest_on_时间戳` 输出目录。两个组都须实际核对终态，不预设会到 L6。HTTP动作对照将 `--transport rosbridge` 改为 `--transport http`，同样使用新输出目录。

`--harvest` 会真实写入设置、打开 mock 执行/抓取、选择当前顺序第一颗，按 `--reconstruction on|off`（默认 off）验证重建三参数、工具关闭、同请求 action 成功、账本 L6 且无恢复锁。两组应使用各自新输出目录，并确认没有其他控制者发任务。写入后先输出 `harvest_settings.json`，核对 FULL 及所选模式的 skip/auto/allow 三参数；`harvest_timeouts.json` 记录实际调度/单段轨迹等待秒数，缺失或无效则不发抓取。动作阶段输出 `report.json`、`harvest.json`、截图；即使未到 L6 也保留失败证据。其 finally 会尝试取消、关三项使能和运动门并读回验证；它**不停止外部 ROS 栈**，失败/异常后仍须人工核对第 9 节。不要只看浏览器检查通过数忽略 `harvest.json` 或清理失败。

## 9. 测试结束必须收尾

1. 停止单轮或后续轮；有在途请求用现有取消入口，等待动作终态，核对调度 `action_active=false` 与臂 `running=false`。恢复等待须保留记录，不用自动 ACK 消掉证据。
2. 能正常写入时，将执行、抓取、工具及运动放行都关闭→“写入并应用”，核对调度与臂两侧均关闭。若恢复锁/未知状态阻止本轮设置，先取消并核对；需要直接关使能时使用既有 `SetEnables`，不要为了解锁关闭按钮盲目 ACK：

   ```bash
   ros2 service call /peach_supervisor/set_enables peach_interfaces/srv/SetEnables \
     '{execution: false, grasp: false, tool: false, reason: "Web test cleanup"}'
   ros2 param set /peach_observability debug.motion_enabled false
   ```

   在同域终端执行，确认服务 accepted 并从网页/状态读回两侧关闭。命令返回不代替读回；失联时先按现场安全纪律处理，再清理自己拥有的进程。
3. 停探针、采集器、回放和浏览器测试。手动 launch 在自己的终端 Ctrl+C；runner 也在自己的终端 Ctrl+C 或等待预算结束，由它取消→关闭使能→关闭 ROS→清理自有进程组。等待会话 bag 封口与报告收尾，不把外层 timeout 返回当作所有子进程已退出。
4. 查看 runner 的 `report.json`、`owned_processes.json`、进程退出/残留、bag 封口记录。若出现 MoveIt 等子进程退出 `-11` 或其他异常码，**保留失败码不可忽略**，保存日志与退出证据再定位；不能以网页通过或最终无残留覆盖该失败。

   当前整栈 launch 参数 `move_group_shutdown_workaround` 默认 `true`，只对 `move_group` 进程预加载 MoveIt 官方默认 capability 库，不修改驱动或 `/opt`。当前文档不重新认定历史探针或联测结果；须以对应运行目录中的日志与退出码为证据，不能把收尾通过计为抓取通过。需要 A/B 排查时显式设置 `move_group_shutdown_workaround:=false`，保留两组配置、日志和退出码。
5. bag 停止写入后确认 `metadata.yaml`、存储文件和可读性：

   ```bash
   ros2 bag info /本轮绝对路径/session_目录/bag
   # 已封口的 bag 可离线重生成报告：
   ros2 run peach_observability peach_bag_report /本轮绝对路径/session_目录/bag
   ```

   元数据/文件存在不代表全 bag 可读。缺 metadata 时先保留原件且确认记录器已停，再按测试指导 reindex；禁止边录边修复。报告生成成功也不代表抓取成功。
6. 盘点进程，只清理本轮拥有的 PID；不要宽泛 `pkill` 或按全机名称杀其他人的栈：

   ```bash
   pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|ros2 bag|move_group|ros2_control_node|robot_state_publisher|peach_|lifecycle_manager|rviz2'
   # 12345 替换为已确认属于本轮的 PID：
   ps -p 12345 -o pid,ppid,pgid,args
   tr '\0' '\n' < /proc/12345/environ | rg '^ROS_DOMAIN_ID='
   kill -INT 12345
   ```

   先 INT 并等待；仍残留时确认无运动且 bag 不再写，才按已核实 PID 升级 TERM，最后才 KILL 并记录原因。再执行 pgrep，确认本轮 PID 全消失、端口/相机已释放；他人进程只报告。

## 10. 常见问题定位

- **页面可开但状态失败**：检查是否由 observability 托管、实际端口、WebSocket 9090 连接/订阅（HTTP 对照看 `/api/state`）及后端日志；不要直接双击 HTML。runner 可能改用了 8091–8098。
- **启动按钮灰/参数不可编辑**：依次看连接新鲜度、`debug.enabled`、设置读取/写入锁、单轮保持、在途 action、恢复锁、未知 request。参数行另看节点是否运行及参数是否声明。
- **单轮等待锁定超时**：核对当前 epoch、真实相机是否持续出图、源 stamp、深度单位、精确 stamp TF和目标检测。没有相机不能靠重复点启动得到目标。
- **图像卡住**：区分 raw 与感知处理节拍，检查源帧龄、QoS、订阅数、相机独占和残留 bag；旧 RELIABLE 大图订户可能拖住下一轮。不要把仍有图片当作仍有新帧。
- **Web allowed 但臂拒绝**：对齐 request/run/cycle/target、epoch、模型修订与有效期，查 clearance、臂使能读回和最近 `failure_code`。页面汇总 allowed 不是臂 CONTACT/TOOL 最终授权。
- **L2 后不继续**：这是预抓取档预期恢复等待；核对终态、停驻几何后人工处理，不能自动 ACK 补成连续 FULL。
- **FULL 无 L6/规划失败**：查监控日志、`peach_supervisor.batch/contact`、`peach_arm.cycle/contact`，区分 SELECT 与 FINALIZE 的行程、精化结果、IK、碰撞与笛卡尔完成比。保留失败原因，不通过放宽质量门或改成跳过重建冒充通过。
- **`bag_axis_envelope_ill_conditioned` / L0**：袋轴拟合未获得可用几何，动作应拒绝继续。查看本次模型采集帧数、有效点数、`missing_mask`/重复帧计数、图像遮挡及轴可观测性；区分“原始相机持续出图”与“目标有可用的同时间戳掩膜/深度”。保留本轮失败与原始模型，核对相机/目标后用新请求测试；不要打开跳过重建或未精化几何来将失败改成 FULL 通过。
- **OFF 后切 ON，已有新模型但臂一直等决策**：对齐同一请求的 Build、TSDF、GraspDecision 与臂接收日志，排查上一周期感知几何缓存钉扎，而不是直接认定几何质量不合格。已定位的 `TargetCache.promoteUnrefinedGeometry` 旧路径会保留 `unrefined_hold_`，导致后续新决策未被接收；当前实现于执行周期收尾释放旧钉扎并撤销旧几何/许可，仍须核对新决策的身份、修订与时效，不放宽质量门。修复的实际 ON/OFF 运行验收须看本轮日志与账本，本文不提前认定通过。
- **参数写了一半/读回不一致**：看 400/207 逐项结果，再读回实际值。已成功项可能生效；先核对再提交差异，不能一键重复整批假设回滚。
- **分析结果和当前动作不同**：先查路径与 request 身份；分析是历史目录，当前证据只认匹配本次 `run_id` 的账本。
- **RViz 消失或 GL 错误**：保存 `rviz.log`、Ogre/显示环境和退出时间；Web 3D 可用不能替代 RViz 存活验收。MoveIt 异常退出同样保留失败码和日志，不计作正常退出。
- **退出慢/报告失败**：区分 bag 封口、离线报告与动作线程；检查本轮进程组和退出记录，按第 9 节精确清理。不能因报告超时而忽略仍在写的 bag 或残留节点。

## 11. 源码权威与交叉核对

行为变化时，以源码、当前接口清单及同轮包级文档为准；不要按旧页面截图或旧 README 推断按钮行为。

- 页面结构/按钮：[`web/debug.html`](../web/debug.html)、[`debug.js`](../web/debug.js)。
- 会话、请求 ID、epoch、防重、单轮/多轮/取消：[`debug-session.js`](../web/debug-session.js)、[`debug-shared.js`](../web/debug-shared.js)。
- 三档草稿及当前请求账本投影：[`debug-evidence.js`](../web/debug-evidence.js)。
- 五页 URL 与共享确认框：[`shell.js`](../web/shell.js)；请求/新鲜度：[`api.js`](../web/api.js)。
- 现有浏览器检查与 CLI：[`scripts/e2e-console.js`](../scripts/e2e-console.js)。
- 参数白名单、范围、类型、逐项结果：[`param_panel.py`](../../peach_observability/peach_observability/param_panel.py)；HTTP/调试桥：同目录 `http_server.py`、`debug_actions.py` 与 `observability_node.py`。
- 联合 runner 与固定相机环境：[`run_live_camera_mock.py`](../../peach_system_tests/scripts/run_live_camera_mock.py)、[`live_camera_mock.yaml`](../../peach_system_tests/config/live_camera_mock.yaml)。
- 验收分母、真机与清理要求：[`testing.md`](../../peach_system_tests/doc/testing.md)；系统启动/图名见 `peach_bringup/doc/architecture.md`、`io.md` 与 `peach_interfaces/config/interface_manifest.yaml`。


## 12. ROS 2 设计依据

- [Jazzy 参数模型](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Basic/About-Parameters.rst)：节点拥有参数；列表、读取、设置走标准服务。客户端须检查逐项结果并读回。
- [ROS 2 Actions](https://design.ros2.org/articles/actions.html)：长任务具有 goal、反馈、结果和取消；服务受理与最终成功必须分别记录。
- [Managed nodes](https://design.ros2.org/articles/node_lifecycle.html)：部署节点和 configure/activate 是不同状态，网页运行策略不能替代生命周期部署。
- [rosbridge ROS 2 协议](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)：WebSocket 转发话题、服务和 Action；网络恢复不构成业务重试授权。
- [MoveIt Planning Scene Monitor](https://moveit.picknik.ai/main/doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial.html)：维护规划场景，与本仓目标 TSDF 精化分工不同；规划碰撞不是现场功能安全。

## 13. 多轮、ACK 与连续 30 分钟联合验收

使用第 2.3 节 `--rounds 0 --models 0 --no-reconstruction --soak-seconds 1800` 入口，保留默认 RViz。只有一套栈占用相机和端口；计时从 `report.json.phase=soaking` 开始，不能将两次启动、修复前后的时段相加。打开五页观察，另用一个调试窗口执行下列操作，其他窗口只读。全程保持工具关闭、重建三参数为 OFF。

1. FULL 档写入并读回后，依次执行轮换 3 轮、第一颗 2 轮、全部 2 轮。每组等完整结束再切换下一组。保存每轮 request、实际场景 epoch、selection_priority、目标 priority、Action 终态与逐颗账本；轮换至少覆盖两个顺序号，全部策略须核对所有目标。rotate/first 的网页前置锁场和 RunHarvest 内部建场可能各产生一个 epoch，不能以 epoch 仅加 1 作为验收条件；选取结果以执行场景为准。
2. L2 档选择“每轮全部”、2 轮。首颗到 L2 后须停止后续目标与后续轮；此时即使臂空闲，也不能直接 ACK 仍在等待的旧批次。点击“停止后续轮”，等取消结果及中断态，再测试 ACK。
3. 打开恢复确认框后取消：请求计数不得增加，双侧恢复锁保持。再次确认“现场已确认”：保存当前非零 expected_state_seq、新 request、accepted 响应，以及 recovery_acknowledged 事件。核对双锁清除，旧账本仍为 L2；等待几秒确认没有自动 Survey/ExecuteTarget，也没有新目标被派发。最后以新 request 跑一轮工具关闭 FULL，证明可以重新执行。
4. 余下时段可隔一段时间显式启动 1 轮 FULL/第一颗，使长测包含重复运行负载；每次先检查剩余预算，不在临近自动停栈时新发长任务。所有动作必须在 1800 秒预算结束前收口并关闭使能；不以请求次数替代连续存活时长。
5. 同时保存相机/深度/关节的实际时间戳推进、状态接收时间、页面错误、TF/诊断、CPU/GPU/内存和磁盘采样。runner 的 raw 图像验收订阅匹配本机 stereo/冻结回放发布端，采用 RELIABLE、depth=1；关节仍为 sensor_data。大图 BEST_EFFORT 探针可能丢帧，不能把探针的丢帧直接等同于设备停流。仍要求图像源/接收帧龄均不超过 3 秒、窗口内至少两个不同时间戳；不以旧帧或重发时间代替源时间。重建 OFF 时，不得出现本轮新 Build 或非空 TSDF；旧模型显示不算新证据。结束后检查精确起止时间、RViz 退出码、bag metadata 和本轮所有 PID。正常运行但退出异常仍应判失败。

操作员服务变更会立即发布最新 state_seq；高频执行反馈仍限频。若另一个客户端改变状态，ACK 的序号校验仍会拒绝旧确认。重新读取并重新人工确认，禁止通过零序号或自动重试绕过。

### RViz 图像退出兼容与排障

当前应用配置的两路图像使用 [`peach_rviz_plugins/Image`](../../peach_rviz_plugins/README.md)。它保留官方 ImageDisplay 的图像传输、属性、QoS 和渲染，仅使重复取消订阅幂等。需构建并 source 本工作区；找不到该插件时检查 ament 索引和安装，不要删掉图像显示来制造通过。

已分析的历史 core 位于另一次 RViz 退出事件，调用栈为 ImageDisplay 析构 → message_filters::Signal1::removeCallback → system_error；本机 Jazzy 库取消订阅后未清空 Connection，重复断开可能访问已销毁的 Signal1。该路径有源码和 core 证据，但没有匹配 core 的其他历史异常仍须分别记录，不能推断全部同因。不要改系统安装库、全局 ptrace/core 设置或以强制杀进程冒充正常退出。若复现，保存同次配置、版本、退出日志和匹配 core/全线程回溯；图像窗口隐藏、显示器禁用、topic/QoS切换和最终关窗都属于应核对的生命周期操作。上游修复发行并通过相同验收后，应移除临时兼容插件。
