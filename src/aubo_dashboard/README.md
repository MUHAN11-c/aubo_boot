# aubo_dashboard — 控制柜慢速操作的独立服务节点（仅 real 模式）

## 简介

独立 rclcpp 节点（`aubo_dashboard`），通过自带的一条 SDK `ServiceInterface`
连接（登录 8899 端口）对外提供**非运动类**服务：上电/断电/松刹车/停止/
快速停/碰撞恢复/负载设置，以及 SDK 侧 FK/IK 求解。无话题、无 action，
对外接口共 9 个服务。由 `aubo_e5_bringup` 仅 real 模式拉起；消费方有
`tools/fk_ik_check.py`（用 get_fk/get_ik 做运动学自洽检查）与真机分阶段
流程的上电步骤（`docs/usage.md` 第 6/7 节）。独立成节点的原因：TCP2CAN
激活期间 SDK 运动类 API 由硬件插件独占，本节点刻意不触碰运动 API，避免
与硬件插件的 sendLoop/ioLoop 竞争同一控制通道
（`src/aubo_dashboard_node.cpp:29-37`）。

## 使用方法

构建：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select aubo_dashboard
source install/setup.bash
```

启动（不自启，由 bringup 在 real 模式自动拉起，
`src/aubo_e5_bringup/launch/bringup.launch.py:141-144`）：

```bash
ros2 launch aubo_e5_bringup bringup.launch.py hardware_mode:=real robot_ip:=<控制器IP>
```

单独拉起（调试用；注意 SDK 按进程 CWD 读 `./config/`，必须 cwd 到装了
SDK 配置的 share 目录）：

```bash
cd "$(ros2 pkg prefix aubo_dashboard)/share/aubo_dashboard"
ros2 run aubo_dashboard aubo_dashboard_node --ros-args -p robot_ip:=169.254.10.98
```

常用服务调用：

```bash
ros2 service call /aubo_dashboard/startup std_srvs/srv/Trigger            # 上电初始化
ros2 service call /aubo_dashboard/shutdown std_srvs/srv/Trigger           # 断电（先停运动）
ros2 service call /aubo_dashboard/release_brake std_srvs/srv/Trigger      # 松刹车
ros2 service call /aubo_dashboard/stop std_srvs/srv/Trigger               # 停止
ros2 service call /aubo_dashboard/fast_stop std_srvs/srv/Trigger          # 快速停
ros2 service call /aubo_dashboard/collision_recover std_srvs/srv/Trigger  # 碰撞恢复
ros2 service call /aubo_dashboard/set_payload aubo_msgs/srv/SetPayload "{payload: 2.5}"

# FK：6 关节角(rad) → pos[xyz] + ori[四元数 w,x,y,z]
ros2 service call /aubo_dashboard/get_fk aubo_msgs/srv/GetFK "{joint: [0, -1.57, 1.57, 0, 1.57, 0]}"
# IK：ref_joint 为求解种子/参考位形，pos + ori(w,x,y,z) → 6 关节角
ros2 service call /aubo_dashboard/get_ik aubo_msgs/srv/GetIK \
  "{ref_joint: [0, -1.57, 1.57, 0, 1.57, 0], pos: [0.4, 0.0, 0.4], ori: [1, 0, 0, 0]}"
```

节点参数（`declare_parameter` 默认值，bringup 只透传 `robot_ip`）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `robot_ip` | 169.254.10.98 | 控制器 IP（bringup 的 `robot_ip` 透传） |
| `server_port` | 8899 | SDK 服务端口（旧控制器固件） |
| `sdk_username` | aubo | SDK 登录用户名 |
| `sdk_password` | 123456 | SDK 登录密码（真实凭据不要提交进仓库） |

安全约定：本节点自身不驱动运动，但 `startup` 是真机流程的上电入口——
之后的任何运动测试，速度/加速度缩放必须先压到 0.1，确认行为符合预期后
再逐步放宽（AGENTS.md 第 10 节）。`startup` 为阻塞调用，期间不要重复
发起；急停/防护停由本体安全回路主导，本节点不提供"恢复安全停止"的服务。

## 执行逻辑

- **构造即登录**（`aubo_dashboard_node.cpp:65-75`）：构造函数内
  `robotServiceLogin` 最多重试 5 次、间隔 500ms（蓝本语义）。登录失败
  仍创建全部 9 个服务，但每个回调入口先查 `connected_`：未连接时
  Trigger 类返回 `success=false, "not connected"`，FK/IK（响应无
  success 字段）直接丢弃请求——对未登录的 `ServiceInterface` 发起调用
  的行为 SDK 未定义，故宁可不响应也不触碰 SDK。
- **串行化**：单线程 spin，所有服务回调在 rclcpp 执行线程上跑，经
  `sdk_mutex_` 串行化后访问 `sdk_`（`ServiceInterface` 单实例不并发，
  `aubo_dashboard_node.cpp:245`）。无独立 IO/发送线程，无周期任务——
  节点完全是请求驱动的。
- **startup**（:82-100）：`rootServiceRobotStartup`，蓝本参数基线——
  工具动力学全零（无工具）、碰撞等级 6、readPose=true、
  staticCollisionDetect=true、boardMaxAcc=1000、IsBlock=true（阻塞等待
  启动完成）。`success` 要求接口调用成功且结果为 `ROBOT_SERVICE_WORKING`。
- **shutdown**（:101-114）：蓝本顺序先 `RobotMoveStop` 停运动，再
  `robotServiceRobotShutdown(true)` 断电。
- **set_payload**（:213-235）：只写负载质量，其余动力学字段保持全零
  （同 startup 的无工具约定）。SDK 约束：工具动力学按设计应在上电前
  设置，运行中调用可能被固件忽略或延迟采纳——节点只 WARN 提示，不改
  行为；稳妥做法是先 `shutdown`、设负载、再 `startup`。
- **FK/IK 失败路径**（:167-212）：请求维度不足（joint<6、pos<3、
  ori<4）或未连接时直接 return，客户端拿到默认空响应；SDK 调用失败时
  响应字段同样保持默认。调用方须自行判断空响应。
- **退出**：`main` 在 `rclcpp::shutdown()` 前先 `node.reset()` 析构
  节点，保证 SIGINT 后先 `robotServiceLogout` 登出再关 ROS（:258-266）。
- **SDK 配置**：`ServiceInterface` 按进程 CWD 读 `./config/auborobot.conf`
  与 `./config/tracelog.properties`，bringup 已把节点 cwd 设为本包
  share 目录（launch 注释见 bringup.launch.py:138-140）。

## 软件框架

```text
src/aubo_dashboard_node.cpp   # 唯一源码：class AuboDashboardNode : rclcpp::Node
                              #   成员 ServiceInterface sdk_ + std::mutex sdk_mutex_
config/auborobot.conf         # SDK 读取：DH 标定参数（全零）+ robot_type
config/tracelog.properties    # SDK log4cplus 日志配置（控制台 + logfiles/ 滚动文件）
vendor/                       # 旧 SDK v1.3.1 头文件 + lib64 二进制
                              #   （aubocontroller/config/log4cplus/protobuf，AMENT_IGNORE）
CMakeLists.txt                # 链接 vendor libauborobotcontroller.so；
                              #   -Wl,--disable-new-dtags 强制 DT_RPATH（传递依赖
                              #   libprotobuf.so.9 不走 RUNPATH）；INSTALL_RPATH=$ORIGIN/vendor，
                              #   vendor 库装到 lib/aubo_dashboard/vendor
```

对外接口（节点名 `aubo_dashboard`，服务全在 `~/` 下）：

| 服务 | 类型 | 语义 |
|---|---|---|
| `/aubo_dashboard/startup` | std_srvs/Trigger | 上电初始化（阻塞，含无工具动力学写入） |
| `/aubo_dashboard/shutdown` | std_srvs/Trigger | 先停运动再断电 |
| `/aubo_dashboard/release_brake` | std_srvs/Trigger | 松刹车 |
| `/aubo_dashboard/stop` | std_srvs/Trigger | RobotMoveStop |
| `/aubo_dashboard/fast_stop` | std_srvs/Trigger | robotMoveFastStop |
| `/aubo_dashboard/collision_recover` | std_srvs/Trigger | 碰撞恢复 |
| `/aubo_dashboard/get_fk` | aubo_msgs/srv/GetFK | 6 关节角 → pos xyz + ori(w,x,y,z) |
| `/aubo_dashboard/get_ik` | aubo_msgs/srv/GetIK | ref_joint + pos + ori(w,x,y,z) → 6 关节角 |
| `/aubo_dashboard/set_payload` | aubo_msgs/srv/SetPayload | 负载质量(kg) → success |

依赖：`rclcpp`、`std_srvs`、`aubo_msgs`（GetFK/GetIK/SetPayload）+
vendored SDK。无插件、无话题、无 action；被 `aubo_e5_bringup`
exec_depend 并在 real 模式集成拉起。lint 走 `ament_lint_auto`（vendor/
有 `AMENT_IGNORE`，厂商代码不参与）。
