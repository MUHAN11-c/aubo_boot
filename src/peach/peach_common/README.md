# peach_common

peach 能力包的共享设施**单源**（W1 起，角色对齐 Nav2 `nav2_common`）：
`yaml_params`（yaml 直读 + `attach`）、`param_rules`（三包规则并集）、
`qos`（QoS 工厂）、`paths`（目录段净化与 `runs/` 根单源）、
`lifecycle`（名单外节点自转换 + bond 心跳守卫）、`ros_log_bridge`
（stdlib logging → /rosout 桥）、`event_meter`（纯核事件计数摘要）、
`grasp_standoffs` / `tool_profiles`（launch 几何注入）与 `recording_session`
（记录写者身份和 active 会话判定）。
能力包直接导入 `peach_common`。原 harvester/bringup/vegetation 的六个参数转发模块已退役；外部脚本也应改用下列公共 API。参数键与规则行为不变。

## 公有 API

- `peach_common.yaml_params`：
  `attach`、`load_ros_parameters`、`leaf_keys`、`set_dotted`、`dict_to_ns`、
  `expand_share`、`package_yaml`、`flatten_leaves`、`snapshot`；类型别名
  `ValidateFn` / `PreviewFn` / `CommitFn`
- `peach_common.param_rules`：
  `check`（kind：`gt` / `gt_eq` / `lt` / `lt_eq` / `bounds` / `nonempty` /
  `one_of` / `seq_gt`）、`check_min_max`、`check_enable_deps`
- `peach_common.paths`：`safe_component`（目录段净化，非法回退 fallback）、
  `ensure_within`（写入路径包含性防御）、`runs_root`（`runs/` 根唯一解析）
- `peach_common.lifecycle`：`ensure_lifecycle_active`（名单外节点自转换到
  active）、`create_bond` / `break_bond`（W14 bond 心跳；缺
  `ros-jazzy-bondpy` 时守卫降级 WARN 并返回 None）
- `peach_common.ros_log_bridge`：`RosLoggerHandler`、
  `bridge_module_loggers`（纯核模块 stdlib logger 转发进节点 /rosout）
- `peach_common.event_meter`：`EventMeter`（第 1 次必记、之后每 every 次
  一条摘要；默认 WARN，可指定 level）
- `peach_common.qos`：`latched()`（RELIABLE+TRANSIENT_LOCAL+depth 1）、
  `stream(depth=10)` 与其显式别名 `reliable(depth=10)`
  （RELIABLE+VOLATILE+KEEP_LAST）、`sensor(depth=5)`
  （BEST_EFFORT+VOLATILE+KEEP_LAST）；rclpy 延迟 import，模块顶层零 ROS 依赖
- `peach_common.grasp_standoffs`：读取共享 YAML，并分别生成 scene / reconstruction /
  manipulation overlay；不包含 arm 的额外轨迹净空 `approach_clearance_m`
- `peach_common.tool_profiles`：`parse_tool_profile`、`load_tool_profile` 与各节点
  参数 overlay；从 `aubo_description/config/<profile_id>.yaml` 读取工具档案
- `peach_common.recording_session`：`write_session_state` 原子写 `session.json`；
  `active_session` 仅接受唯一 active 声明且 PID/start tick 仍匹配的会话，
  不以 bag 文件 mtime 判活

## 用法

```python
from peach_common import attach, package_yaml
from peach_common.param_rules import check
from peach_common.qos import latched
params = attach(node, package_yaml('peach_vegetation', 'vegetation.yaml'))
```

## Build / Test

```bash
colcon build --packages-select peach_common
colcon test --packages-select peach_common && colcon test-result --verbose
```

只跑官方 `flake8` / `pep257`（2026-10-23 测试策略）；分层验证与系统接线见
`peach_system_tests/doc/testing.md`。

## 依赖

依赖 `rclpy`、`rcl_interfaces`、`python3-yaml`、`ament_index_python`、
`launch` / `launch_ros`、`aubo_description` 与 `bondpy`（见 `package.xml`）；
不引入 numpy/opencv。各子模块的导入边界和文件 IO 见 [doc/io.md](doc/io.md)。

## 许可

BSD-3-Clause。分层与决策快照见 [peach_bringup/doc/architecture.md](../peach_bringup/doc/architecture.md)。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。
