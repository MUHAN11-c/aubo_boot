# peach_observability 架构

## 职责

只读观测聚合、FastAPI/uvicorn 8090 调试桥、会话 bag 与报告；静态页面由 peach_web 安装。调试请求委托现有ROS服务/动作，不能绕过臂侧门或新增SetIO。

## 源码地图

`observability_node.py`：订阅镜像/生命周期；`http_server.py`、`debug_actions.py`：HTTP与委托；`image_preview.py`：JPEG缓存；`tcp_trajectory.py`：真实TF缓存；`state.py`、`job.py`、`pipeline.py`：状态解释；`recorder.py`、`catch_all_recorder.py`、`retention.py`：录制/回收；`bag_reader.py`、`bag_report.py`：离线报告；`selfcheck/`：启动检查。

## 入口、输入输出与参数

整栈随起，独立用 observability.launch.py；离线 `ros2 run peach_observability peach_bag_report <bag>`。输入事件、模型、状态、diagnostics、TF/关节等；输出8090、作业/指标、自检和runs/session_*产物。参数声明由config/observability_parameters.yaml生成ParamListener，部署值在config/observability.yaml，ObservabilityParams.attach维护运行期快照；详细API见 [io.md](io.md)。

## 生命周期、并发与失败

参数 listener 与实时快照绑定到 Node 生命周期，同一节点反复 configure/cleanup 复用它们，避免累积生成器和整数校验回调。cleanup 释放运行资源但不撤销 ROS 参数；此期间的合法更新在下次 configure 读取时可见，非法更新仍先拒绝并保持缓存一致。

只读参数轮询按节点保存当前 future 与单调截止，响应预算为 2 秒；到期后在下一次既有轮询定时器执行时撤下 rclpy pending request 并重试。迟到响应必须同时匹配当前 future 且未过期才能提交，不能清掉新请求或覆盖新值；cleanup 撤销全部在途身份。失联时保留上一镜像及原更新时间，不伪造新鲜参数。

性能采样停止最多等待 2 秒；若旧线程还未退出，保留句柄并告警，重新激活也不会清除它的停止信号或另起并行采样线程。退出后再次启动可恢复；各次启动使用独立停止事件，停用期间完成的旧样本不再提交。极慢采样导致本次重启被拒时，采样保持停止并在日志中可见，须等旧线程退出后再次启动。

关键进程按执行入口（原生可执行文件、Python 脚本或模块）和首条适用的合法 ROS 节点改名精确匹配，不匹配临时参数文件路径或 `peach_arm:/topic:=...` 这类话题 remap 子串。同一 PID 每轮只读一次 CPU 差分；不存在或不可读的配置项保留空指标。采样范围仍是宿主全局，同名取 RSS 最大者，不按 ROS_DOMAIN_ID 或某次 launch 归属隔离；多域同名进程须另用运行器进程清单确认。

账本在调度身份切换或进入目标/批次终态时立即增量读取一次，普通心跳不触发磁盘读；既有参数轮询继续负责补读。文件签名包含 inode、mtime 与 size，原子替换可被识别；读失败不缓存成功签名，文件消失清掉旧成功行并保留 error。刷新锁忙时直接略过，不等待；读取在身份短锁之外，提交再次核对请求及 watch 实例。被丢弃的未提交读取清除其缓存，避免 A→B→A 回到同一文件后永久无更新。

录包打开失败、累计写入错误或封口错误在 `session_recorder` diagnostics 中报告 ERROR；尚未获得 bag 目录与队列丢帧报告 WARN，不能把已丢失记录的会话显示成健康录制。保留策略删除前先追加并 flush/fsync 删除意图；审计不可写时保留 bag。删除结果另行追加，便于区分意图与已完成的回收。

Lifecycle节点自激活，不进系统四节点名单；双线程executor外有HTTP、编码、发现等工作线程。HTTP只读缓存图像/TF，不阻塞ROS获取新帧；每流最新槽、公平编码总计最多5Hz。cleanup/shutdown/destroy停止HTTP、线程、订阅与配置期发布器并封口bag；远程shutdown迁移也立即收尾。诊断Updater在节点进程内只创建一次，cleanup取消其timer，重新configure时复用并恢复，避免重复声明参数和累积任务。自动报告在封口后用当前Python启动子进程，继承launch进程组；默认等30秒，超时先terminate、2秒后kill并回收，保留已封口bag供离线重算，报告失败不伪装成功。accepted仅代表受理，202仍在途，网络/超时是结果未知，POST不自动重发。allowed不证明臂侧许可，level6不是level7；陈旧/身份缺失明确标记，不能推断成有效模型。

## 验证与限制

`colcon test --packages-select peach_observability --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。

## 状态投影与静态资源

原始精化诊断、绑定来源及 typed/raw/decision 合并集中在 pipeline.py，节点 ROS 回调负责接线。state.py 按已有身份与诊断标记投影。嵌套 candidate/fitting 空身份或与外层 target_id 不符时为 null。BagFitting 负值与非有限标量转 null，合法零值保留。`anchor_from_memory` 保留历史关键点，置信度、相机距离、袋径和行程置 null。

作业票的单果终局与完成度读取同一非空 request_id、target_id 的账本，在途动作期间不套用旧终局；有 cycle_id 时须匹配，双方有效 scene_epoch 不同则拒绝，旧账本缺字段仍兼容。终态先到而账本尚缺时显示 pending，不由 SUCCEEDED 技能文本推断完成度。已有完成度不因收尾关闭使能而消失，失败/取消/跳过不显示单果成功。L6 只证明撤退，不能推出切断；当前账本缺独立切断字段，因此工具仅在 L5 或 L7 标 done，L6 保持 pending（已有失败状态保留）。无目标、无在途动作且读到同批合法空 rows/claimed 的终态另标 empty_batch，表示批次结束但未执行目标，不生成单果 completion。

轨迹和路标保存最近一个非空目标所有者；调度在周期结束发布空 target_id 后，下一颗目标仍与上一所有者比较。A→空→B 与直接 A→B 都清理上一颗轨迹、路标和可视化缓存；同目标恢复不重复清空，换批次始终清空。

TCP 摘要的路径几何按实际收点版本缓存，静止时重复采样复用几何计算。收点（含同位置的时间/阶段更新）、环形截头和 clear 均使缓存失效；TF 成功与失败计数每次读取当前值。计算在锁外完成，只有点集版本仍匹配才回写缓存；返回的字典与 xyz/quat 列表独立，不允许调用方修改污染后续读取。20 Hz TF 采样、8000 点默认容量、轨迹导出和 RViz 发布规则保持原有行为。

8090 按扩展名托管 peach_web 安装的页面。SDK 和许可使用固定 URL，从 ament 索引定位的 ros2_web_link 安装目录取文件。ROS 投影按各自订户数计算，独立 HTTP 读取照常取当前快照。自检跟踪最近一次托管状态，真机要求 robot_status 时未收过一帧即失败，关闭重建进程时自检跳过 Build 动作。普通面板写参后继续沿用启动事实决定的探针范围。

Vegetation 自检优先服从 `startup_facts.vegetation_enabled`：false 表示未部署并 SKIP；true 时从未收到或超过 30 s 的状态均 FAIL。整栈不部署此独立节点并注入 false。独立启动未声明事实时，使用实际发布者数决定是否检查，自身订阅使话题出现在 ROS 图不算发布者。

`/diagnostics` 按 `(hardware_id, task name)` 合并，最多保留最近接收的 256 项；新帧只更新其携带的任务，发布者之间不覆盖。每项保存接收 wall time 并用 monotonic 计算年龄，超过 5 s 在快照投影为 STALE（3），旧 OK 不会因 HTTP 持续读取而保持健康。原等级和原消息仍留作排障，容量淘汰次数可见。

Updater 启动时空 `hardware_id` 是临时占位，呈现为 unknown；同一任务收到正式硬件身份后移除空身份占位，迟到的同任务空身份也不重新引入。不同非空硬件的同名任务仍分别保留并独立过期。

录包会话的 `session.json` 在关闭/封口时保存 `written`、`dropped`、`drops`、`write_errors`、`open_failed`、`close_error` 与 `sealed`。written 计成功返回的 writer.write；dropped 计已进入录制流程但因队列淘汰、写失败或 bag 未打开而未写入的消息，drops 单列队列丢弃。主动限速/媒体策略过滤不计作异常丢弃。关闭超时保留 closing 和错误，写线程随后完成时刷新最终计数；sealed 只证明 writer.close 返回成功，不掩盖此前写入或关闭等待错误。

状态快照按需读取当前 `Recorder.info()`，释放 State 锁后才进入 Recorder 锁，不逐消息推送计数。关闭时保存最终 info；只保留一个会话读取器直到下次 configure 替换，因此关闭等待超时后的迟后封口也会反映到 Web，旧会话不会覆盖新会话的实时计数。


记录器是会话身份所有者：原子 session.json 声明写者与状态，调度周期归档据此关联活会话；8090 力页只读最新帧，不创建接触段文件。原子 /force/sample 在 core/std/all 录制档均不受媒体策略/主动限速过滤；固定订阅与发现订阅不重叠写入。Recorder控制屏障不可丢，停止接收后排队封口，只有确认sealed才启动自动报告。离线保留数值数组与源ns时间；旧mtime时间推算明确标estimated，时间重叠关联不能替代目标身份。

TopicMetadata 类型来自生成消息类的真实 msg/srv/action 命名空间；未知非生成类报错并走现有写失败计数，不伪造 msg 类型。旧 bag 中误写为 msg 的动作反馈仍由读取侧兼容解析。本机 Jazzy rosbag2_cpp 0.26.11 的定义解析器不嵌入动作反馈及 GoalStatusArray 的 schema，且 Python SequentialWriter.create_topic 仅接受 TopicMetadata；相关原始警告保留。类型正确、CDR 可在同接口环境读回与 MCAP 自包含 schema 完整是不同结论，sealed/written 不证明后者。

启动事实仍投影到状态；仅启用录制时启动落盘线程。写者绑定本次Recorder，等待会话目录时可由cleanup/shutdown立即唤醒退出；确认退场后才清句柄，避免旧facts写进重配后的会话。

空批次的 `empty_batch` 仅表示未执行目标，不抹掉批次终局：COMPLETED 显示“批次已结束，未执行目标”；INTERRUPTED 保留原失败/取消原因和失败状态，只将未执行的靠近、工具、撤离标为 skipped。

参数面板将 `moveit.approach_clearance_m` 标为运动参数，展示机械臂轨迹相对视觉预抓取的额外后退距离。后端仅转发并读回臂参数，不修改感知/重建位姿，也不移动视觉标记；视觉预抓取和实际轨迹停点因此保持各自含义。写入仍经过原调试运动许可和臂空闲检查。

共享预抓取后撤只从统一配置载入，不再提供独立的臂侧 Web 编辑入口。面板只展示仍参与当前规划的速度控制，近果套入/撤退与接近转移分开；旧混合走廊专用的加速度参数不再显示为可生效控制。
