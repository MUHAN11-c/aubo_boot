# 机械臂运动评估标准（准确性 / 平稳性 / 平滑性 / 实时性）

本文档定义 `tools/motion_analyzer.py`（rec 模式）输出的四大类指标的含义、数值来源与判定阈值。
标准来源以 UR 驱动生态（ur_robot_driver）的实测口径为主，辅以 ISO 9283 的测量方法学
与运动平滑性文献。

## ISO 9283 范围声明（先读）

ISO 9283《Manipulating industrial robots — Performance criteria and related test
methods》只规定**测量方法**，不规定限值；且其全部特性（位姿精度/重复性 AP/RP、
距离精度 AD/RD、路径精度 AT/RT、拐角偏差 CR/CO、位置超调、稳定时间、漂移等）
都定义在 **TCP 笛卡尔位姿**上，需要激光跟踪仪等外部计量设备。

本工具只订阅 `/joint_states`（编码器关节数据），**无法**进行 ISO 9283 合规测量。
下文 B 类（平稳性）中的超调、稳定时间、漂移等指标是 ISO 9283 §7.4/§7.5/漂移条款的
**关节空间类比量**，仅用于纵向对比与同机调参，不构成 ISO 9283 合规结论。

## A. 准确性 —— UR 驱动生态标准

数值取自 Universal Robots 官方 ROS2 驱动的控制器默认配置与集成测试断言：

| 指标 | UR 数值 | 来源 |
|---|---|---|
| 目标点关节误差 | 0.01 rad（集成测试严格值）/ 0.1 rad（goal_tolerance 默认） | [integration_test_passthrough_controller.py](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/test/integration_test_passthrough_controller.py)、[ur_controllers.yaml](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/config/ur_controllers.yaml) |
| 路径偏差（逐关节） | 0.2 rad（scaled JTC constraints.goal 默认） | [ur_controllers.yaml](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/config/ur_controllers.yaml) |
| 目标时间容差 | 1 s（passthrough 集成测试）；0 = 不检查 | 同上集成测试 |
| 停止速度容差 | 0.2 rad/s（stopped_velocity_tolerance） | ur_controllers.yaml |
| abort 后静止漂移 | < 0.01 rad（2 s 窗口） | [test_trajectory_aborts_on_violation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/test/integration_test_passthrough_controller.py) |
| MoveIt 执行监控默认 | 时长 ≤ 1.1×标称 + 0.5 s；起点偏差 ≤ 0.01 rad | [trajectory_execution_manager.cpp](https://github.com/moveit/moveit2/blob/main/moveit_ros/planning/trajectory_execution_manager/src/trajectory_execution_manager.cpp)（`allowed_execution_duration_scaling=1.1`、`allowed_goal_duration_margin=0.5`、`allowed_start_tolerance=0.01`） |

**本项目现状对照**：`src/aubo_e5_moveit_config/launch/moveit.launch.py:72-74` 将
MoveIt 执行监控放宽为 **5.0× / 10 s / 0.15 rad**（passthrough 蓝本值，整段轨迹一次
下发、硬件侧自行插补，耗时与 RIB 流控相关，余量须比流式 JTC 宽）。因此分析器同时
给出 MoveIt 默认规则（1.1×+0.5 s）的判定，便于识别"在默认口径下会不会被 MoveIt
判超时"。

速度/加速度限值校验并入本组：|q̇|max 对照 xacro 限值（前 3 关节 2.596177 rad/s、
后 3 关节 3.110177 rad/s）；|q̈|max（速度差分）对照 17.30878 / 20.73676 rad/s²。

## B. 平稳性（steadiness）

以下均为 ISO 9283 关节空间类比量（见范围声明），配合控制理论惯例：

- **位置超调量 / PO%**（ISO 9283 §7.5 类比）：越过最终位置的最大幅度（rad），
  及相对首末位移的百分比。段首末净位移 ≈0（如往返轨迹）时该项不适用（N/A）。
- **位置稳定时间**（ISO 9283 §7.4 类比）：从"名义到达时刻"（有标称轨迹时为
  段开始+标称时长，否则为峰值速度时刻）起，逐关节误差进入 ±2% 位移带
  （下限 0.001 rad）且不再离开所需时间。录制窗口（到达后 0.5 s）内未稳定则如实报告。
- **到达后振荡次数**：到达时刻之后各关节速度符号变化次数（带 0.01 rad/s 死区），
  取各关节最大值。
- **静止漂移**：段末 0.5 s 各关节位置 max−min，对应 UR abort 静止漂移判据
  （< 0.01 rad）与 ISO 9283 漂移 dAP 的关节类比。
- **振动带 RMS**：到达后窗口内速度模 ||q̇(t)|| 的 FFT，>5 Hz 分量按 Parseval
  定理折算的 RMS。无标准阈值，仅作同机不同配置间的相对对比量。

## C. 平滑性（smoothness）

速度信号取关节空间速度剖面 v(t) = ||q̇(t)||（6 关节速度矢量模）。

- **SPARC（频谱弧长，主指标）**：对速度幅值谱归一化后算弧长，无量纲、对噪声与
  时长鲁棒。实现：V=|rfft(v)|，V̂=V/V[0]，自适应截止
  ωc=min(2π·20 Hz, 使 V̂ 其后恒低于 0.05 的最小 ω)，
  SAL=−∫₀^ωc √((1/ωc)²+(dV̂/dω)²) dω。**≥ −1.6 判平滑**。
  来源：Balasubramanian et al. 2015, *On the analysis of movement smoothness*,
  J NeuroEngineering and Rehabilitation, [DOI 10.1186/s12984-015-0090-9](https://doi.org/10.1186/s12984-015-0090-9)（MPiNets 等规划文献沿用同一阈值）。
  20 Hz 截止远低于本系统 100 Hz Nyquist（200 Hz 采样），条件良好。
- **LDLJ（对数无量纲 jerk，辅助指标）**：DLJ=−(t₂−t₁)⁵/v_peak²·∫(d²v/dt²)²dt，
  LDLJ=−ln|DLJ|。健康点到点运动参考值 ≈ −6。对噪声与采样率极敏感，计算前固定做
  **50 ms 零相位滑动平均**预处理（双向滑动平均，不引入 scipy），且仅作同配置
  纵向对比。来源：Hogan & Sternad 2009, *Sensitivity of smoothness measures to
  movement duration, amplitude, and arrests*, J Motor Behavior,
  [DOI 10.3200/JMBR.41.6.529-534](https://doi.org/10.3200/JMBR.41.6.529-534)。
- **速度峰数**：局部极大且高于 0.1×峰值的个数，简单鲁棒的粗糙指标。
- **mean/max 速度比**：越接近 1 越接近梯形速度剖面。

**明确不用 RMS jerk 做跨段/跨轨迹对比**：文献结论（Balasubramanian 2015）RMS jerk
非无量纲，随时长/幅值变化，不同轨迹间不可比。

## D. 实时性（real-time）

- **控制周期抖动**：`/joint_states` 到达间隔的 mean/std/max；标称周期取中位数
  （sim 为 5 ms / 200 Hz）；超期次数 = 间隔 > 中位周期 + 1 ms；实测发布频率。
  `/joint_states` 由 joint_state_broadcaster 以 controller_manager 频率发布，
  是控制环抖动的良好代理。更深的 read/write/update 级统计可看 controller_manager
  的 `~/statistics` 话题（[pal_statistics](https://github.com/pal-robotics/pal_statistics)，
  需 controller_manager 开启 `enable_tcp_nodelay` 同级统计开关）。
- **时间膨胀比**：段墙钟时长 / 标称时长（与 A 类目标时间容差是同一现象的两个视角）。
- **RIB 水位**（项目既有流控判据）：执行期应 >0 且 <400；统计 min/max/mean 及
  饿死（=0）/溢出（≥400）计数。订阅不到 `/aubo_io_controller/rib_status` 时跳过。
  注意 **sim 模式下该判据不适用**：sim 插件在轨迹收齐后把整段点流一次性重采样
  入虚拟板载队列，RIB 峰值≈整段点数×6（远超 400），溢出计数为仿真口径假象，
  仅在 real 模式解读饿死/溢出。
- 文献口径：调好的 PREEMPT_RT 系统延迟在 ~10–100 µs 级
  （[Linux Foundation RT wiki](https://wiki.linuxfoundation.org/realtime/start)），
  200 Hz（5 ms 周期）下即使 100 µs 抖动也仅占 2%；"deadline-is-the-period"是公认
  判据。本项目 sim 模式无 RT 要求，real 模式亦已取消 RT 预检（普通内核直接
  运行），本工具只做测量报告，不做 RT 合规判定。

## 本工具判定阈值表

`tools/motion_analyzer.py` rec 模式默认值（全部可命令行调整）：

| 判定项 | 默认阈值 | 命令行参数 | 类别 |
|---|---|---|---|
| 终点逐关节误差 | ≤ 0.01 rad | `--goal-tol` | A |
| 路径偏差 max/RMS | ≤ 0.2 rad | `--path-tol` | A |
| 执行时长偏差 | \|墙钟−标称\| ≤ 1.0 s | `--goal-time-tol` | A |
| MoveIt 默认规则 | ≤ 1.1×标称+0.5 s（固定，对照用） | — | A |
| 停止速度 | ≤ 0.2 rad/s | `--stop-vel-tol` | A |
| 速度/加速度限值 | xacro 蓝本值（固定） | — | A |
| 静止漂移 | ≤ 0.01 rad | `--drift-tol` | B |
| SPARC | ≥ −1.6 | `--sparc-min` | C |
| 超调量/PO%、稳定时间、振荡次数、振动带 RMS | 无标准阈值，INFO 输出 | — | B |
| LDLJ、速度峰数、mean/max 速度比 | 参考量，INFO 输出 | — | C |
| 周期抖动、时间膨胀比、RIB 水位 | 测量报告，INFO 输出 | — | D |

无标称轨迹时（段开始前 2 s 内未捕获 `/display_planned_path`）自动跳过与标称对照的
判定项（终点误差、路径偏差、执行时长、MoveIt 规则、时间膨胀比），并注明"无标称轨迹"。
