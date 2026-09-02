# CLAUDE.md

完整约定见 [`AGENTS.md`](AGENTS.md)。设计和改动依赖三份活文档，并与源码互相更新：[`docs/architecture.md`](docs/architecture.md)、[`docs/io.md`](docs/io.md)、[`docs/testing.md`](docs/testing.md)（流程与命名）。改代码改文档同一改动完成，禁止只改一边。真机轮次只追加 [`docs/testing-log.md`](docs/testing-log.md)，不把它当设计权威。

驱动栈冻结；bringup 不起 `aubo_dashboard`（示教器 + MoveIt；停轨在硬件）。Python 用 `aubo_py3.12`（依赖分层 venv-first：ROS 依赖走 apt，其余第三方一律 `requirements.txt` 钉版入 venv），numpy 1.26.4（<2，cv_bridge ABI 硬约束）。过程数据在 `_archive/runs/` 与现场 `runs/`，不要删。不要加业务测试或仿真采摘测试。launch 不自动开始采摘。
