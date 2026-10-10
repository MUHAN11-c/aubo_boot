# `peach_common` 架构（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/architecture.md` 拆写入包（职责短页）。

共享设施库（W1，对齐 `nav2_common`）：`yaml_params`（yaml 直读 + `attach(node)`）/ `param_rules`（规则并集）/ `qos`（QoS 工厂）/ `paths`（`safe_component` + `runs_root` 单源）。**不跑节点、不进 launch、不载业务契约**；harvester/bringup/vegetation 的包内旧路径为转发 shim（键名与行为零变化）。文件树与全栈位置见 [peach_bringup/doc/architecture.md](../../peach_bringup/doc/architecture.md)。
