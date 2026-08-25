# CLAUDE.md

完整约定见 [`AGENTS.md`](AGENTS.md)。设计和改动依赖三份活文档，并与源码互相更新：[`docs/architecture.md`](docs/architecture.md)、[`docs/io.md`](docs/io.md)、[`docs/testing.md`](docs/testing.md)。改代码改文档同一改动完成，禁止只改一边。

驱动栈冻结；bringup 不起 `aubo_dashboard`（示教器 + MoveIt；停轨在硬件）。Python 用 `aubo_py3.12`，numpy 1.26.4。过程数据在 `_archive/runs/` 与现场 `runs/`，不要删。不要加业务测试或仿真采摘测试。launch 不自动开始采摘。
