# Peach 项目文档

本目录是 Peach 生产项目的架构与验证文档入口。范围只包括 `src/peach`（含 `peach_web`）、`src/serial_force`；`src/peach_sim` 是独立环境仿真，不修改、不纳入验收。

## 阅读顺序

1. [总体架构](architecture/00-overview.md)
2. [包依赖](architecture/01-package-graph.md)
3. [运行数据流](architecture/02-runtime-flow.md)
4. [感知与推理](architecture/03-perception.md)
5. [力传感与仿真源](architecture/04-force.md)
6. [验证与证据](architecture/05-verification.md)
7. [数据记录](architecture/07-recording.md)
8. [错误码](architecture/08-failure-codes.md)
9. [Web 后端](architecture/09-web.md)
10. [日志规范](architecture/10-logging.md)
11. [Orin 部署档案](architecture/11-orin.md)
12. [全功能回归矩阵](architecture/12-regression.md)
13. [参考来源](architecture/13-references.md)
14. [全栈适配性评估](architecture/14-adaptability-review.md)
12. [待迁移清单](architecture/06-roadmap.md)

当前状态：功能、包拆分、GPL、单 Web、单录制、错误码、日志规范与 Web/RViz 单轮已完成；自动 10×30×1800 全量回归仍按文档记录未通过；Orin NX 未实测。

## ADR

- [包边界](adr/0001-package-boundaries.md)
- [GPL 参数](adr/0002-gpl-parameters.md)
- [FastAPI Web](adr/0003-fastapi-web.md)
- [单一录制](adr/0004-recording.md)
- [FSM 不迁移 BT](adr/0005-fsm.md)
- [独立进程](adr/0006-process-model.md)
