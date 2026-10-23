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

## 3. 仓库里的自动检查

`test/` 只保留 ROS 2 官方 lint。Python 包是 `test_flake8.py` 与 `test_pep257.py`；C++ 与 ament_cmake 包由 `ament_lint_auto` 跑 uncrustify、cppcheck、xmllint。不再维护自定义 gtest、pytest 或 launch_testing。行为对不对，看节点日志、`ros2 topic echo` 和后面各节的手工步骤。

```bash
colcon test --packages-select peach_common peach_interfaces peach_harvester peach_arm \
  peach_bringup peach_observability peach_vegetation peach_stereo peach_web \
  peach_system_tests --return-code-on-test-failure
colcon test-result --verbose
python3 src/peach/peach_interfaces/scripts/check_interface_manifest.py
```

接口清单脚本不在 `test/` 里。改了 IDL 仍要跑。`colcon test` 绿只说明 lint 过了，不代表采摘链路正确。

## 4. mock 冒烟

启动前执行§9的进程盘点，确认域和设备空闲。用日志和话题确认栈起来了。域不要和 IVG、仿真或其他任务共用；换 DDS 域解决不了相机、端口、GPU 争用。看节点是否 Active、action 终态、退出后有没有残留。`bond_timeout=0` 默认关闭进程死检，启用非零值前先在 mock 上看 bond 是否真能传故障。

人工无相机 mock 入口：

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

历史夹具曾将 `grasp.maximum_travel_m` 硬设为 0.02 m，当前 index 70 的合格候选建议行程约 0.03 m（该冻结夹具记录的是旧 0.03 m；现行感知 `(neck-entry)-margin_neck` 为 0.05–0.10 m），在选果 `CheckReachability` 阶段即以 `cut_travel_out_of_range` 过滤，表现为 discovered=1 / attempted=0。这是夹具覆盖与产品门冲突，不是空感知或抓取成功。现已删除该行程覆盖，沿用产品正常范围；后续精化几何、IK、笛卡尔路径和有效期门仍须独立通过。冻结 bag 不能保证当前工具/模型下 FULL 成功，不得通过放宽产品门或点名绕过选果取得绿灯。

冻结runner只清理自己创建的PGID；`--skip-launch`不停止外部栈，仅对自己request取消并关闭自身使用的使能，不按全域匹配清进程。取消/关闭使能真实失败会令wrapper退出非零；当前request已由外部作业替换时只记正常skip，不触碰对方。FULL联合驱动要有 bag 和 GPU。回放只能证明固定数据与当前夹具下的链路。普通 `ros2 bag play --clock` 才使用use_sim_time=true；冻结重发runner按自身launch配置，不能任意混时钟。

修改接近、融合、护栏或包络后，用冻结回放或 mock 跑一轮，对照日志里的 failure_code 和行程。不要靠改口径把退化盖掉。

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

RViz Fixed Frame=base_link，检查相机raw、独立实时debug_image_raw、深度点云、感知轴、预抓取/剪切参考与实际工具TF。界面几何必须对应同目标同场景；共同主轴构造出的0°不是独立真值。相机固定在base_link是本档允许的环境差异，腕动不会产生新视角；业务默认与real相同，缺覆盖应明确失败，不自动跳重建或改变质量门来通过。只读探针须有限时间，完成即退出。

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

`config/live_camera_mock.yaml` 保存固定采集位TF，来源为20261013记录及相机未移动确认；parent=base_link、child=camera_link，mock初始关节也在其中。该环境校准不写产品手眼文件，移动相机后须重测。RViz统一使用 `peach_bringup/config/harvest.rviz`，不改驱动配置。runner不再生成brain/arm业务overlay；用产品默认，或传 `--task-params-file /path/job.yaml`，同一文件原样传给real launch的 `task_params_file:=/path/job.yaml`。速度、超时、重建和质量门不依硬件模式分叉。report.json保存实际读回的effective_params和required_completion_level；任何模型/轮失败或恢复等待立即收口，不自动ACK。PICK_ALL必须全部目标成功。产物包括stack.log、camera_tf.log、rviz.log、启动/模型/逐轮状态和owned_processes.json；结束按cancel→disable→close→自有组清理，report原子检查点保留中断证据；soak要求实际采样达到预算且每次相机/关节/RViz健康。

RViz重建点云默认订阅 `/peach/reconstruction/preview_cloud`（标准PointCloud2，最多50000点，保留采样点颜色，TSDF优先、否则局部累加云）。原local_cloud/tsdf_cloud仍完整发布，算法输入不变；RViz不同时启用这两条完整云。此预算只约束重建预览，DetectionCloud保持原固定分辨率stride输出，Image/PointCloud2仍使用BEST_EFFORT、queue1、decay0。

联合长测采用监控专用RViz配置：仅一份RobotModel，移除会另建场景机器人/规划机器人/交互标记的MotionPlanning显示插件；规划和操作入口仍在Web。保留两路图像、Detection/50k重建预览、感知关键点、实际TCP轨迹及工具轴/套筒口/剪切平面TF。该简化是针对Ogre顶点缓冲异常的A/B候选，不能仅凭配置测试就宣称GL故障已消除；须重新记录全程RViz存活、渲染错误和进程/资源证据。重复初始化订阅日志不等于运行中存在重复Display，Ogre无时间戳异常行也不能用来确定崩溃时刻。

模型阶段只BeginScene一次，在固定epoch轮换锁定目标；Build结果身份包含target/epoch/revision/generated_at/valid_until，并有界等待Web同revision和diag.model_stamp，不能用诊断接收时间代替生成时间。每次Build用新request，首次失败就停止，不额外重试补出成功配额。models≥3时每个已采目标至少3样本，统计同目标几何离散度。动作各轮新scene，目标按本场锁定priority选择；人工恢复等待使runner退出并清理，禁止自动ACK。

抓取档由实际共享作业参数决定：默认 `skip_reconstruction=false`、`execute_pregrasp_only=true`；显式选择skip时，mock/real使用同一配置，须一并写入supervisor.skip_reconstruction、reconstruction.capture.auto_mode、arm.quality.allow_unrefined_geometry，不因固定相机自动开启。参数面板逐项写入结果和读回值必须一致才允许使能。PREGRASP_ONLY仅验证预抓取；FULL不切刀看level6；采摘确认看level7和切断/撤离反馈。`--models>0`时独立模型采样仍走真实Build，不用接受心跳代替模型成功。

短测（不替代上面 10×30×1800）示例：

```bash
ros2 run peach_system_tests run_live_camera_mock.py \
  --rounds 5 --models 0 --soak-seconds 0 --domain 86 \
  --output runs/live_mock_YYYYMMDD_HHMMSS
```

2026-10-16 `runs/live_mock_20261016_183842`：`--rounds 5 --models 0 --soak-seconds 0`，`passed=true`，5/5 level6，`residual_pids` 空。账本单果 24–27 s，RunHarvest 38–43 s（首轮 43 s，其后 38–40 s），相对人工偏保守的单果预算 40 s。runner 墙钟 45–51 s 含每轮 `new_scene`，不算产品节拍。点名 `target_ids` 摘完即 `EMPTY_LIMIT`，不再空扫。立体相机 `peach_stereo_camera_node` **12.0–13.1 gps、drop=0**（隔离 hh4 约 13.7；此前整栈钉 `OMP_NUM_THREADS=1` 时约 6.4 gps、仍 drop=0）。runner 不再默认钉 OpenMP，`OPENBLAS`/`MKL` 仍 1。mock overlay 速度 0.8，产品 `peach_arm.yaml` 仍 0.1。此项不证明 30 组模型、10 轮长测或 30 分钟 soak。

### 2026-10-21 行程修正与 A/B（不替代 10×30×1800）

`skip_reconstruction` 抓取轮在 17:45 轮 target_16 指令 0.030 m、实际 TCP 插入 29.5 mm，入口→袋颈 94.5 mm，TCP 距颈 65 mm；同场景全重建成功目标 travel 0.080–0.082、TCP 0.7576、距颈 15.3–15.4 mm，位置大致正确。根因是感知 `min(s_safe, L_insert=0.03)` 封顶，不是重建/感知两套算法；`blade_offset_m=0`（TCP 与剪切平面重合）。

修正：`compute_travel_range` 去掉 `L_insert` 封顶（`s_max=s_safe`）；`promoteUnrefinedGeometry` 把 `suggested_travel_m` 置 0，臂走 `(neck-entry)-neck_margin` 回退。感知 SELECT 行程 0.05–0.10 m，与全重建 0.07–0.09 m 同量级；差约 0.01 m 为融合修剪，不是旧 0.03 vs 0.09 的封顶差。

短测 `runs/live_mock_unified_rviz_20261021_1813`：3/3 level6，travel 0.079–0.091，TCP 0.703–0.763，距颈 15–24 mm，RViz 托管、camera 三轮新鲜、`passed=true`、`residual=[]`。

A/B（每边 2 轮 PICK_ALL，RViz + 8090 实时）：

- A 全重建 `runs/AB_A_full_20261021_1829`：6 次目标尝试、2 次 level6（target_1/5，33.3%），travel 0.080–0.082、TCP 0.7576、距颈 15.3/15.4 mm；失败 target_0/2/4 refit（`bag_axis_envelope_ill_conditioned`/`bag_axis_silhouette_unobservable`）、target_3 MTC short-path。
- B skip `runs/AB_B_skip_20261021_1832`：8 次尝试、8 次 level6（100%），travel 0.054–0.092、TCP 0.691–0.763、距颈 15–32 mm（多数 15–26）。
- 两跑 `passed=true`、`residual=[]`；产物在各自 `records/{session,harvest,round}`，一跑一文件夹；分析页每跑只列一个顶层选项。B 的 level6 仍不是 level7 真剪切（tool=false）。

历史 30 mm 对照：`live_mock_20261021_1730/1745` 5/5 level6，但 TCP 0.69–0.704、长袋距颈 65–82 mm（仅 target_5 27 mm），不能把 level6 当到颈。A 的失败是固定相机多视不足导致的 refit，不是全重建不可行；同日 09:31 全重建 8/9、15:xx 50–55%。

### 2026-10-22 抓取顺序与 4 颗成功率（修正前一轮的重复抓取）

2026-10-21/22 的 5 轮、3 轮和 A/B B 用的是旧 `choose_target`：按 `target_id` 排序后过滤 `LOST/RETIRED`，固定相机下只有前排 long bag 被跟踪，索引实际把 target_0→8→15→16 都指到同一颗。5/5、8/8 是同一颗的重复抓取，不能当 4 颗目标的成功率。

修正（runner，test-only）：

- `choose_target` 按场景 `priority`（抓取顺序 1..N）选；`LOST` 的 memory 候选（`anchor_from_memory`）也参与，是否执行由 supervisor/臂门决定。
- 每轮记录 `grasp_slot=1..N`；`target_id` 直接使用场景源 ID（也是 2D/3D/账本 ID），2026-10-22 起不再另造 `grasp_id` 重映射。`priority` 与 `target_id` 不一定相等。
- 相机证据采样前 `spin_for(0.5)` 让驱动回调查到，3 s 新鲜门不放宽。2026-10-22 第一次 5 轮 target_0 的 camera 15 帧/最后帧 3.4 s 前 false；加 spin 后第二轮 target_0 22 帧 true。

4-slot 短测 `runs/goal_skip_4slot_20261022_0908`：4 轮 = priority 1..4，报告 `target_0..3`（当时旧 runner 仍做抓取顺序重映射；场景源 target_0/4/9/14），全部 level6，`passed=true`、`full_level6_rounds=4`、`residual=[]`；travel 0.097/0.096/0.081/0.070 m；camera 四轮 fresh；3 颗 LOST 的 memory 候选由 `allow_unrefined_geometry` 接受（日志 `target=target_4/9/14`）。**4/4 = 100%**，4 颗不同物理目标。

第二次 4-slot 重复 `runs/goal_skip_4slot_b_20261022_0915`：4/4 账本 level6（旧 runner 报告 `target_0..3`，场景源 target_0/4/9/14；travel 0.080/0.095/0.080/0.070 m），`residual=[]`；但 slot 4 的 camera 新鲜门 false，`full_level6_rounds=3`、`passed=false`。这是测试证据采样（驱动相机队列/关节洪泛）问题，不是抓取失败——该轮 ledger 仍是 `SUCCEEDED` level6。两次 4-slot 合计 **8/8 level6**，按抓取顺序的成功率 **4/4 = 100%**；第一次 `passed=true` 已证明可视化/相机门在全跑内可过。

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
