# CLAUDE.md

完整约定见 [`AGENTS.md`](AGENTS.md)（ROS 2 工作流百科：MUST / DEFAULT / KEEP 仅完美适配 / SNAPSHOT / UNWIND）。功能安全与应用护栏分层见 AGENTS 第 2 章：**硬件急停不经 ROS**。

现行系统看**包级活文档**（SNAPSHOT，2026-10-10 用户裁定拆包入库）并与源码互相更新：系统级 [`src/peach/peach_bringup/doc/architecture.md`](src/peach/peach_bringup/doc/architecture.md) 与 [`doc/io.md`](src/peach/peach_bringup/doc/io.md)；各能力包 `doc/architecture.md` / `doc/io.md`；测试指导在两测试包 `doc/testing.md`。**怎么改看 AGENTS**：非完美适配当前真机/产品则跟 ROS 2 与优秀 GitHub 主流，同轮改对应包活文档。不要把活文档里的当时否决（不上 pluginlib、禁止 launch_testing、已删 GPL）当成不可破红线。

过程记录类文档（docs/testing-log.md、docs/REFACTORING.md）已于 2026-10-10 用户裁定删除：轮次结论随提交说明，历史在 git。`docs/` 只留 v1/v2 方案书（非活文档）。
