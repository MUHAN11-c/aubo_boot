# CLAUDE.md

完整约定见 [`AGENTS.md`](AGENTS.md)（ROS 2 工作流百科：MUST / DEFAULT / KEEP 仅完美适配 / SNAPSHOT / UNWIND）。功能安全与应用护栏分层见 AGENTS 第 2 章：**硬件急停不经 ROS**。

现行系统看三份活文档（SNAPSHOT）并与源码互相更新：[`docs/architecture.md`](docs/architecture.md)、[`docs/io.md`](docs/io.md)、[`docs/testing.md`](docs/testing.md)。**怎么改看 AGENTS**：非完美适配当前真机/产品则跟 ROS 2 与优秀 GitHub 主流，同轮改这三份。不要把三份活文档里的当时否决（不上 pluginlib、禁止 launch_testing、已删 GPL）当成不可破红线。

过程记录不驱动现行设计：真机轮次只追加 [`docs/testing-log.md`](docs/testing-log.md)；工程整理只追加 [`docs/REFACTORING.md`](docs/REFACTORING.md)。`docs/` 不新增第四份活文档。
