# CLAUDE.md

完整约定见 [`AGENTS.md`](AGENTS.md)。设计和改动依赖三份活文档，并与源码互相更新：[`docs/architecture.md`](docs/architecture.md)、[`docs/io.md`](docs/io.md)、[`docs/testing.md`](docs/testing.md)（流程与命名）。改代码改文档同一改动完成，禁止只改一边。过程记录不驱动现行设计：真机轮次只追加 [`docs/testing-log.md`](docs/testing-log.md)；工程整理只追加 [`docs/REFACTORING.md`](docs/REFACTORING.md)。

驱动栈冻结；bringup 不起 `aubo_dashboard`（示教器 + MoveIt；停轨在硬件）。Python 用 `aubo_py3.12`（依赖分层 venv-first：ROS 依赖走 apt，其余第三方一律 `requirements.txt` 钉版入 venv），numpy 1.26.4（<2，cv_bridge ABI 硬约束）。过程数据在 `_archive/runs/` 与现场 `runs/`，不要删。不要加业务测试、DDS 假现场或仿真采摘测试（允许零 ROS 纯核 pytest）。launch 不自动开始采摘。旁路视觉抓取三包不进 harvest launch。
