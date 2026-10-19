# peach_stereo 测试数据

`data/` 的两份 frames.jsonl 保留为现行仿真审计输入，`src/peach_sim/sim_worlds/reconstruction/audit_sources.py` 会读取它们。包内常规质量门由 CMake 的 ament_lint_auto 注册。

2026-09-20/21 的 A/B 分析、报告、录像和实验脚本已移入工作区 `_archive/code_cleanup_2026-10-12/src/peach/peach_stereo/test/`，保持原结构，属于本地历史证据，不随包安装。当前源码和参数见本包 README 与 doc/；现行系统测见 peach_system_tests。
