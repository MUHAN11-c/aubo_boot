# CLAUDE.md

完整约定见 [`AGENTS.md`](AGENTS.md)。流程、阅读地图与用法：[`docs/flow.md`](docs/flow.md)、[`docs/usage.md`](docs/usage.md)。

驱动栈冻结；bringup 不起 `aubo_dashboard`（示教器 + MoveIt；停轨在硬件）。Python 用 `aubo_py3.12`，numpy 1.26.4。过程数据在 `_archive/runs/`，不要删。不要加业务测试或仿真采摘测试。launch 不自动开始采摘。
