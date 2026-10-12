# peach_common

peach 能力包的共享设施**单源**（W1 起，角色对齐 Nav2 `nav2_common`）：
`yaml_params`（yaml 直读 + `attach`）、`param_rules`（三包规则并集）、
`qos`（QoS 工厂）、`paths`（目录段净化与 `runs/` 根单源）、
`lifecycle`（名单外节点自转换 + bond 心跳守卫）、`ros_log_bridge`
（stdlib logging → /rosout 桥）、`event_meter`（纯核事件计数摘要）。
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
  一条 WARN 摘要）
- `peach_common.qos`：`latched()`（RELIABLE+TRANSIENT_LOCAL+depth 1）、
  `stream(depth=10)` 与其显式别名 `reliable(depth=10)`
  （RELIABLE+VOLATILE+KEEP_LAST）、`sensor(depth=5)`
  （BEST_EFFORT+VOLATILE+KEEP_LAST）；rclpy 延迟 import，模块顶层零 ROS 依赖

## 用法

```python
from peach_common import attach, package_yaml
from peach_common.param_rules import check
from peach_common.qos import latched
params = attach(node, package_yaml('peach_harvester', 'peach_supervisor.yaml'))
```

## Build / Test

```bash
colcon build --packages-select peach_common
colcon test --packages-select peach_common && colcon test-result --verbose
python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach/peach_common/test/       # 七个测试文件；qos 用例无 rclpy 环境整体 skip
```

（原 `scripts/r0_gate.sh` 纯核门已于 2026-10-10 随 `scripts/` 清理删除，
等价命令见 `peach_system_tests/doc/testing.md`。）

## 依赖

`rclpy`、`python3-yaml`（见 `package.xml`）；纯核测试只需 PyYAML。

## 许可

BSD-3-Clause。分层与决策快照见 [peach_bringup/doc/architecture.md](../peach_bringup/doc/architecture.md)。
