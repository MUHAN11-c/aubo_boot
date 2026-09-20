# peach_common

peach 能力包的共享设施**单源**（W1 起，角色对齐 Nav2 `nav2_common`）：
`yaml_params`（yaml 直读 + `attach`）、`param_rules`（三包规则并集）、
`qos`（QoS 工厂）。`peach_harvester` / `peach_bringup` / `peach_vegetation`
的旧 import 路径保留为转发 shim，行为零变化。

## 公有 API

- `peach_common.yaml_params`（= 各包旧 `yaml_params` 路径）：
  `attach`、`load_ros_parameters`、`leaf_keys`、`set_dotted`、`dict_to_ns`、
  `expand_share`、`package_yaml`、`flatten_leaves`；类型别名
  `ValidateFn` / `PreviewFn` / `CommitFn`
- `peach_common.param_rules`（= 各包旧 `param_rules` 路径）：
  `check`（kind：`gt` / `gt_eq` / `lt` / `lt_eq` / `bounds` / `nonempty` /
  `one_of` / `seq_gt`）、`check_min_max`、`check_enable_deps`
- `peach_common.qos`：`latched()`（RELIABLE+TRANSIENT_LOCAL+depth 1）、
  `stream(depth=10)` 与其显式别名 `reliable(depth=10)`
  （RELIABLE+VOLATILE+KEEP_LAST）；rclpy 延迟 import，模块顶层零 ROS 依赖

## 用法

```python
from peach_common import attach, package_yaml      # 或旧路径 shim
from peach_common.param_rules import check
from peach_common.qos import latched
params = attach(node, package_yaml('peach_harvester', 'supervisor.yaml'))
```

## Build / Test

```bash
colcon build --packages-select peach_common
python3 -m pytest -q -p no:cacheprovider --import-mode=importlib \
  src/peach_common/test/          # qos 用例无 rclpy 环境整体 skip
bash scripts/r0_gate.sh           # 已含 peach_common 组
```

## 依赖

`rclpy`、`python3-yaml`（见 `package.xml`）；纯核测试只需 PyYAML。

## 许可

BSD-3-Clause。分层与决策快照见 [docs/architecture.md](../../docs/architecture.md)。
