# peach_interfaces

四个能力包之一：**跨包唯一契约**。无节点、无 launch、无参数默认值。感知、重建、技能、调度、监控只依赖本包消息；导航 IDL 预留（manifest `reserved_interfaces` 区）。

- 契约清单（名称/类型/QoS/生产消费方）：[`config/interface_manifest.yaml`](config/interface_manifest.yaml)
- 漂移检查：`python3 scripts/check_interface_manifest.py`
- 语义与消费方：[docs/io.md](../../docs/io.md) §2；架构定位：[docs/architecture.md](../../docs/architecture.md) §3

改字段只改本包 IDL，先编本包再编下游，同轮改 manifest 与 io.md。
