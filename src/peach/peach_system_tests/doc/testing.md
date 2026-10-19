# Peach 测试与验收

本页是现行可执行流程。整栈仅从 `peach_bringup/harvest_system.launch.py` 启动；包职责见 [系统架构](../../peach_bringup/doc/architecture.md)。历史实现、peach2及临时 `/tmp` 试验脚本不作为交付入口。本文给出方法，不代表这些验收已完成。

## 1. 安全与测试边界

未授权不得真机运动或 SetIO；操作员发起 real launch/操作台指令属于授权语义。驱动栈只读，不起 dashboard，示教器上电/抱闸。硬件急停在柜/示教器，不经 ROS；取消和 RobotMoveStop 是应用停轨。保护停止后处理现场、取消并丢弃在途 goal、示教器复位，重新授权后下发新目标，禁止恢复旧轨迹。

默认 mock、相机关、autostart=false、执行/接触/工具使能关、PREGRASP_ONLY。SURVEY_ONLY 可能移动到拍照位；tool=false 只关闭工具IO，不代表不动臂。所有测试和探针结束必须停止并核对本轮进程，见§9。

## 2. 环境与构建

从新终端执行，先 ROS，再工作区，再 venv。numpy 必须是1.26.4；ROS依赖用Jazzy apt，不额外pip同名ROS包。

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to peach_system_tests peach_web peach_stereo peach_vegetation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
source aubo_py3.12/bin/activate
export PYTHONPATH="$PWD/aubo_py3.12/lib/python3.12/site-packages:$PYTHONPATH"
python3 -c 'import sys,numpy,cv_bridge,open3d; print(sys.executable,numpy.__version__,cv_bridge.__file__,open3d.__version__); assert numpy.__version__ == "1.26.4"'
```

python应指向工作区aubo_py3.12；cv_bridge应来自Jazzy。若污染，开新终端或unset PYTHONPATH后按顺序重载。source venv不会改变已生成CTest的解释器，检查 `build/peach_system_tests/CTestTestfile.cmake`；显式site-packages用于系统Python节点发现已固定的非ROS依赖。模型资产检查 `peach_harvester/model/` 的 best.pt/mobile_sam.pt，缺资产与依赖属于环境失败。

IDL变更先编peach_interfaces，再编消费包。只改某能力可按其依赖构建；运行时读install配置，源码YAML修改后须重建对应包。参数事实源：Python各包config YAML+attach，arm GPL声明src/arm_parameters.yaml+部署config，停位几何grasp_standoffs.yaml，工具几何只读aubo_description。

## 3. 分层测试

依据 [Jazzy测试指导](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html) 和 [launch_testing](https://docs.ros.org/en/jazzy/p/launch_testing/)，先纯核/接口、再节点接线、再数据回放与现场。

```bash
colcon test --packages-select peach_common peach_interfaces peach_harvester peach_arm \
  peach_bringup peach_observability peach_vegetation peach_stereo peach_web \
  --return-code-on-test-failure
colcon test-result --verbose
python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py
node src/peach_web/scripts/run-tests.cjs
ctest --test-dir build/peach_system_tests -N
```

只跑测试包静态组时，先确认筛选命中，再执行：

```bash
ctest --test-dir build/peach_system_tests -N -R 'test_(preflight|replay_approach|perf_baseline|baseline_inventory|frozen_camera_bag|drive_harvest)'
colcon test --packages-select peach_system_tests --return-code-on-test-failure \
  --ctest-args -R 'test_(preflight|replay_approach|perf_baseline|baseline_inventory|frozen_camera_bag|drive_harvest)'
colcon test-result --test-result-base build/peach_system_tests --verbose
```

核对测试数量、失败/error/skip及本轮XML时间；0测试、No tests found、旧轮XML汇总、bag缺失导致skip都不能当验收通过。日志在 `log/latest_test/` 与各构建包 `Testing/Temporary/LastTest.log`。前端node测试只验证行为，浏览器布局/WebGL仍须独立验证。

## 4. 隔离域 mock 节点集成

启动前执行§9的进程盘点，确认域和设备空闲。CMake已为launch_testing设置ROS_DOMAIN_ID及ROS_LOCALHOST_ONLY；不要在普通pytest中初始化默认域。

- 域89：test_mock_launch，使能默认关、无自动goal、关节序和Active。
- 域90：test_e1_supervisor_chain，合成观测+RunHarvest，包含预抓取等待ACK。
- 域91/92/93：三工具profile的TF、参数与标签。
- 域94：test_scene_obstacles，合成点云/目标/假PlanningScene服务。
- 域98：test_nav2_lifecycle_debug，生命周期调试。

```bash
colcon test --packages-select peach_system_tests --return-code-on-test-failure \
  --ctest-args -j1
colcon test-result --test-result-base build/peach_system_tests --verbose
```

这些域必须不被IVG、仿真或其他任务共用；不同DDS域不能解决相机、端口、GPU争用。观察节点Active及action终态，也检查退出与残留。`bond_timeout=0`默认关闭进程死检，启用非零值前必须验证bond依赖和故障传播。

人工无相机mock入口：

```bash
export ROS_DOMAIN_ID=81
export ROS_LOCALHOST_ONLY=1
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=false force_enabled:=false rviz:=false autostart:=false
```

无相机栈只能证明启动。完整合成动作链优先使用E1测试夹具，不拿零目标正常结算当抓取成功。固定关节序：shoulder_joint、upperArm_joint、foreArm_joint、wrist1_joint、wrist2_joint、wrist3_joint。

## 5. 冻结 RGB-D 回放

现有可靠联合入口如下；bag目录必须含metadata.yaml，GPU/依赖/模型齐全，域81空闲。先核对CLI，不从/tmp运行试验脚本。

```bash
ros2 run peach_system_tests run_frozen_grasp.py --help
ros2 run peach_system_tests run_frozen_grasp.py \
  --bag runs/peach_camera_capture_20260930/stereo_10s \
  --index 70 --hz 2 --ready-timeout 120 --timeout 180
```

runner默认无头（rviz=false），需桌面时显式`--rviz`，`--no-rviz`显式保持无头；launch也默认关闭RViz并透传开关。runner在本域启动 frozen_camera_grasp.launch.py，出生于拍照位，冻结重发camera话题；离开拍照位后pause图像/点云。overlay单视配置、测试口径与关闭障碍门属于回放夹具，不可复制到real部署。`skip_reconstruction=false`、tool=false；ready预算与goal受理后的抓取预算分开。

历史夹具曾将 `grasp.maximum_travel_m` 硬设为 0.02 m，当前 index 70 的合格候选建议行程约 0.03 m，在选果 `CheckReachability` 阶段即以 `cut_travel_out_of_range` 过滤，表现为 discovered=1 / attempted=0。这是夹具覆盖与产品门冲突，不是空感知或抓取成功。现已删除该行程覆盖，沿用产品正常范围；后续精化几何、IK、笛卡尔路径和有效期门仍须独立通过。冻结 bag 不能保证当前工具/模型下 FULL 成功，不得通过放宽产品门或点名绕过选果取得绿灯。

冻结runner只清理自己创建的PGID；`--skip-launch`不停止外部栈，仅对自己request取消并关闭自身使用的使能，不按全域匹配清进程。取消/关闭使能真实失败会令wrapper退出非零；当前request已由外部作业替换时只记正常skip，不触碰对方。测试包test_frozen_camera_bag只验证overlay/数据条件，缺bag可skip；FULL联合驱动不属于默认无设备测试。回放只能证明固定数据与当前夹具下的链路。普通 `ros2 bag play --clock` 才使用use_sim_time=true；冻结重发runner按自身launch配置，不能任意混时钟。

修改接近/融合/护栏/包络后，跑test_replay_approach（现场案册+分层/随机语料，对照replay_baselines.json）；基线变化必须有原因和分母，不能重封基线掩盖退化。

## 6. 真实相机 + mock + RViz

此档连接真实相机，机械臂仍mock，不授权真机运动。相机物理固定，mock腕移动不会创造新机位。先核对固定采集位base_link→camera_link锚点，关闭腕随动外参的重复广播；锚点必须来自真实采集位，不编造数值。未完成锚定时只检查静止出图/TF，不发抓取批次。

```bash
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=true camera_frontend:=stereo force_enabled:=false \
  extrinsics_enabled:=false autostart:=false rviz:=true
# 另一同域终端，固定锚定已由本轮独立入口提供后检查
ros2 run peach_system_tests probe_topics.py --duration 10
ros2 run peach_system_tests check_planning_scene.py
```

RViz Fixed Frame=base_link，检查RGB、深度点云、感知/融合轴、预抓取/剪切参考与实际工具TF。界面几何与数值必须对应同目标同场景；共同主轴构造出的0°不是独立真值。tool始终false，重建保留，不能用放宽漂移/多视门凑通过。只读探针均需有限时间，完成即退出。

当前联合验收（`--rounds 10 --models 30 --soak-seconds 1800`）未通过：已记录30组模型，10轮抓取中6轮达到level6、4轮质量门跳过；soak只有253条资源采样、约21分钟，RViz已退出，不满足30分钟可视化稳定性要求。模型、执行和稳定性须分别判定，不能用模型配额或早先短测成功替代整项通过。RViz监控配置和退出处理已更新，仍须按新配置重新完成验收。新主机网络monitor probe已完成3组模型，RViz在15秒观察窗内存活；抓取轮在180秒预算后FAILED，bag已封口且无本轮进程残留。它只证明该次模型/短时显示/收尾，不能算抓取或完整验收成功。2026-10-16 短测 5 轮见下节，不替代本项。

### 真实相机联合入口

`scripts/run_live_camera_mock.py`收编重复模型、动作链、RViz与稳定性采样。参数为独立token，`--rounds 10`，不是`--rounds10`：

```bash
ros2 run peach_system_tests run_live_camera_mock.py --help
ros2 run peach_system_tests run_live_camera_mock.py \
  --rounds 10 --models 30 --soak-seconds 1800 --domain 86 \
  --output runs/live_mock_YYYYMMDD_HHMMSS
```

RViz默认开启，`--no-rviz`只用于无头排障，不能算可视化验收。先构建测试包以安装脚本/config，再执行；输出目录须为本轮新目录，域86、相机须空闲。监控口优先 8090，已被他人占用则依次试 8091–8098 并透传 `observability_port`，不占用 8099、不清理其他进程。

`config/live_camera_mock.yaml`保存test-only固定采集位校准，来源为20261013固定相机记录，用户确认相机未动；parent=base_link、child=camera_link，mock初始关节也由该配置提供。该校准只用于本测试，不注入产品手眼文件；相机一旦移动必须重测，不继续沿用。`config/live_camera_mock.rviz`是本轮显示配置。产物包括report.json、stack.log、camera_tf.log、rviz.log、rviz_model_params.yaml、startup_state.json、model_N_state.json、round_N_state.json、测试专用brain_overlay.yaml和arm_overlay.yaml（mock 接近/转移速度缩放到 0.8，把接近+撤离从约 20 s 压进单果 40 s 预算；产品 `peach_arm.yaml` 仍 0.1，真机不得高于 0.1；近果套入仍 0.05）。owned_processes.json在每次创建进程组后立即保存owner PID、PID/PGID、命令、域与时间，信号退出按cancel→disable→close→自有组清理顺序处理。report.json采用同目录临时文件后原子replace，在启动/模型/动作/soak/清理等阶段写入phase与updated_at检查点；中断时保留最后完整状态。soak每次资源采样先落检查点，再检查RViz存活和相机/关节新鲜帧；任一失效立即失败并进入清理，不继续累计成合格时长。通过还要求实际采样时长达到预算且全部样本健康。

RViz重建点云默认订阅 `/peach/reconstruction/preview_cloud`（标准PointCloud2，最多50000点，保留采样点颜色，TSDF优先、否则局部累加云）。原local_cloud/tsdf_cloud仍完整发布，算法输入不变；RViz不同时启用这两条完整云。此预算只约束重建预览，DetectionCloud保持原固定分辨率stride输出，Image/PointCloud2仍使用BEST_EFFORT、queue1、decay0。

联合长测采用监控专用RViz配置：仅一份RobotModel，移除会另建场景机器人/规划机器人/交互标记的MotionPlanning显示插件；规划和操作入口仍在Web。保留两路图像、Detection/50k重建预览、感知关键点、实际TCP轨迹及工具轴/套筒口/剪切平面TF。该简化是针对Ogre顶点缓冲异常的A/B候选，不能仅凭配置测试就宣称GL故障已消除；须重新记录全程RViz存活、渲染错误和进程/资源证据。重复初始化订阅日志不等于运行中存在重复Display，Ogre无时间戳异常行也不能用来确定崩溃时刻。

模型阶段只BeginScene一次，在固定epoch轮换有效目标；Build结果身份包含target/epoch/revision/generated_at/valid_until，并有界等待Web同revision和diag.model_stamp对应，不能以refined接收时间替代。重复Build使用新request及新生成时间去重；最多models×3次尝试，配额不足判失败。models≥3时每个已采目标至少3样本，统计同目标D95、entry、cut及轴角P95。动作各轮新scene，mockACK仅依据当前request/target的终态证据发出，避免预抓取恢复锁死锁。

产品reconstruct_in_trajectory默认true：Build绑定后启动臂侧Finalize等待，同目标精化与许可到齐才规划，不代表使用粗几何先运动；测试overlay显式true不改变TTL或接触能力门。按实际runner产物核对每组模型/每轮终态/采样时长和退出清理，不能仅看总返回码。

短测（不替代上面 10×30×1800）示例：

```bash
ros2 run peach_system_tests run_live_camera_mock.py \
  --rounds 5 --models 0 --soak-seconds 0 --domain 86 \
  --output runs/live_mock_YYYYMMDD_HHMMSS
```

2026-10-16 `runs/live_mock_20261016_183842`：`--rounds 5 --models 0 --soak-seconds 0`，`passed=true`，5/5 level6，`residual_pids` 空。账本单果 24–27 s，RunHarvest 38–43 s（首轮 43 s，其后 38–40 s），相对人工偏保守的单果预算 40 s。runner 墙钟 45–51 s 含每轮 `new_scene`，不算产品节拍。点名 `target_ids` 摘完即 `EMPTY_LIMIT`，不再空扫。立体相机 `peach_stereo_camera_node` **12.0–13.1 gps、drop=0**（隔离 hh4 约 13.7；此前整栈钉 `OMP_NUM_THREADS=1` 时约 6.4 gps、仍 drop=0）。runner 不再默认钉 OpenMP，`OPENBLAS`/`MKL` 仍 1。mock overlay 速度 0.8，产品 `peach_arm.yaml` 仍 0.1。此项不证明 30 组模型、10 轮长测或 30 分钟 soak。

## 7. 量化验收与结果口径

每轮唯一request_id，结果放runs下本轮目录，保存启动配置/git身份/域、原始RGB-D、模型与geometry.jsonl、许可身份及时效、action终态、ledger、diagnostics、RViz截图和退出证据。禁止复用旧轮次文件充当本轮证据。

1. **30组模型**：每组记录可见目标、输入帧、袋底/袋口极性、轴方向、D95/拟合质量及四项能力；包含正常、低质深度、遮挡、无效/陈旧TF和身份切换。逐组报告ACCEPT/拒绝原因，外部真值不足时标未知。
2. **10轮链路**：每轮独立request_id，记录重建ACCEPT、GraspDecision、臂侧受理/拒因、PREGRASP_ONLY或FULL、恢复ACK、终态及是否无SetIO。失败轮保留，不能靠重复成功轮替换。
3. **30分钟稳定性**：连续观察真实相机+mock+RViz的帧率/帧龄、TF、诊断、CPU/GPU/内存、磁盘、订阅数量与渲染错误；开始/结束记录精确时间。掉线恢复、旧图/旧许可不得无声变新。结束后bag封口和进程清理亦属验收。

三项分别报告样本分母、通过/失败/跳过及证据，未完成明确写未完成。重建ACCEPT≠许可有效；Web allowed≠臂授权；action SUCCEEDED≠业务成功；level6干跑完成≠level7真实采摘。只有切断与撤退确认才允许业务grasped；tool=false不可能证明真实剪切。

PREGRASP_ONLY到位应SUCCEEDED并recovery_required，停在预抓取等人工ACK，不回stow、不SetIO；残差/方向仍需独立目视或测量。HTTP accepted/202只是受理/在途，超时或网络失联是结果未知，不自动重发。

## 8. 真机验收另授权

先完成mock并确认操作员授权范围，现场无人/隔离、硬件急停可及、工具状态已知、示教器就绪。real参数为hardware_mode:=real、camera_enabled:=true、robot_ip:=169.254.10.98；保持autostart=false与所有门关闭，确认机器人状态/关节/TF/深度后才按授权逐档开启。不得通过8090按钮或YAML true推定授权。

先预抓取评方向定位，再独立授权套入/工具；刀具、电流、急停回路和人在环不是mock覆盖项。真机不得套用冻结overlay、allow_stale_anchor或关闭质量/碰撞门。

## 9. 停栈、bag封口与精确清理

有在途批次先通过现有取消入口取消，等终态；ControlTask.CANCEL_NOW=4，ACK/RESUME不等于取消。关闭execution/grasp/tool，停止探针、回放、注入器和采集器；launch终端Ctrl+C，先等待bag封口。自动报告使用继承本轮进程组的独立subprocess，不再依赖非守护分析线程；默认等待30秒，超时TERM后等2秒，仍不退出则KILL并等2秒回收。超时/非零会保留已封口bag并提示离线重算；这不是报告成功，不应无限等待，也不能在bag未封口时跳过记录器收尾。

```bash
pgrep -af 'ros2 launch|component_container|extrinsics_publisher|ros2 run|ros2 bag|move_group|ros2_control_node|robot_state_publisher|peach_|lifecycle_manager|rviz2'
# 以下12345替换为本轮已确认PID，先查命令/父进程/DDS域
ps -p 12345 -o pid,ppid,pgid,args
tr '\0' '\n' < /proc/12345/environ | rg '^ROS_DOMAIN_ID='
# 仅对本轮拥有的PID发送信号，先INT，等待后再决定TERM
kill -INT 12345
```

不使用宽泛pkill，不杀别人的域/栈；同域runner的清理也要求域独占。仍挂住先确认已无运动、bag不再写，再TERM；最后才KILL并记录原因。外层timeout退出不保证子进程退出。清理后重跑pgrep并核对本轮PID全部消失；其他任务只报告。

bag封口后检查metadata及可读性：`ros2 bag info <本轮bag目录>`，报告可用 `ros2 run peach_observability peach_bag_report <本轮bag目录>` 重生成。异常退出缺metadata时，在确认记录进程已停止且保留原件后用ros2 bag reindex恢复，不能边录边reindex。报告生成成功不代表动作成功。

## 10. 排障顺序

- **无帧**：先区分主机网络与隔离网络，确认SDK发现设备，再查设备独占/IP→实际取帧→发布者→QoS→帧stamp/年龄→订阅数/残留bag。主机SDK当前可发现相机，先前probe的0设备来自隔离网络环境，不能据此判相机损坏或产品无设备；发现成功也不等于新probe已成功取帧。传输探针用probe_topics，不把timeout结束当收到帧。
- **重建无积分**：绑定目标/scene_epoch→精确stamp TF→关节静止门→深度单位→采帧窗口；不以latest TF补积分。
- **Web允许而臂拒绝**：对照同一request/run/cycle/target及模型修订、clearance与valid_until，查臂侧最近拒因与enables心跳；页面显示不能替代命令门。
- **不可达/规划失败**：先对齐两次行程再看完成比。SELECT 用感知建议行程，Finalize 用精化行程，二者不同是预期。用 **ROS 2 日志**（`/rosout`、rqt_console、`$ROS_HOME/log`），不要只翻 `stack.log`。

```bash
rqt_console   # Include: peach_supervisor.batch, peach_supervisor.contact, peach_arm.cycle, peach_arm.contact
rg 'peach_supervisor\.(batch|contact)|peach_arm\.(cycle|contact)' \
  "${ROS_HOME:-$HOME/.ros}/log" runs/<本轮>/ros_home/log
```

怎么从日志落到具体位置：

1. logger `peach_supervisor.batch` 最后一条 `command=` 是调度停在哪一步（`wait_lock` / `select` / `dispatch` / `execute_full` / `none`）。
2. 已到 `execute_full` 后看 `peach_arm.cycle` 方括号阶段：`[FINALIZE]` → `[READY_FOR_GRASP]` 或 `[RECONFIRM]` → `[MTC_APPROACH_INSERT]` → `[FAILED]`。
3. `peach_supervisor.contact` 的 `caller=SELECT` 与 `peach_arm.contact` 的 `caller=FINALIZE` 必须分开读，重点对比 `travel=` 和 `code=`。
4. `code=sleeve_no_cartesian` 时看同秒 `peach_arm.contact` 的 `各滚转 fraction:`；完成比门槛 0.95。
5. 接近阶段在 `peach_arm.contact` 搜 `跳过` 与 `接近筛滚转`：`insert_travel=0` 表示 PREGRASP_ONLY，不筛套入直线。
6. `peach_arm.contact` 的 `failure_code=` 对照 `peach_interfaces/msg/FailureCode.msg`。账本 `runs/<request_id>/ledger.json` 与 `/peach_supervisor/events` 对同一 `target_id`。

`[READY_FOR_GRASP]` 不是规划失败，是 `grasp.enabled=false`。不修改驱动兜底。
- **RViz GL错误**：保存Ogre日志、显示/驱动环境、点云/marker规模和截图，检查是否残留重复RViz；明确记录渲染失败，不计入可视化验收通过。
- **退出卡住**：先区分报告写盘与动作线程滞留，按本轮PID/域清理，保留LastTest.log和进程证据；不能因post_shutdown豁免退出码而忽略残留。

新帧缓存年龄门 12 秒、96 帧/192MiB，关节 Cache 8192 条，覆盖感知处理滞后；8 秒门曾拒 8.42 s 已配对帧。不同缓存/显示配置的验收结果不得混计。历史性能与本轮验收数据保留在各自runs产物，不把中间测量复制成产品精度保证。单果完整周期墙钟预算 40 s（相对人工偏保守）：点名 `target_ids` 摘完即结算，不含摘后空扫。

### 有界诊断与复验边界

MTC任务销毁问题用独立100次创建/销毁探针隔离，`runs/peach_optimization_20261015/mtc_lifecycle_comparison.json`保留对照：默认构造后关闭introspection的旧方式约12秒超时，SIGINT后才退出；直接`Task(task_name,false)`约0.115秒退出且未超时。结合MTC IntrospectionExecutor及本机Jazzy rclcpp启动/取消源码核对，该证据针对cancel-before-spin/join竞态，不代表抓取链已通过。排障先对齐候选create/plan/reset计时及cancel generation，再判断阻塞阶段。

模型采集为0帧时先对齐曝光到处理延迟与 12 秒帧龄门。runner 只钉 `OPENBLAS_NUM_THREADS`/`MKL_NUM_THREADS=1`，不钉 `OMP_NUM_THREADS`（SGBM 走 OpenMP）。比较时保留线程数、主机CPU/load、处理耗时、曝光stamp和captured计数；他程序抢核可把立体组率从 12+ gps 打到约 6 gps，低组率本身不构成质量门失败。不能将环境过载记作几何通过或靠放宽门补帧。

UI布局/行为证据沿用已有截图验证。本轮CUA浏览器策略拒绝访问，未绕过该限制，不能声称已用新浏览器会话复验当前UI。RViz存活、浏览器UI与真实抓取分别记录。
