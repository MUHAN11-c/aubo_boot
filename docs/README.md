# Peach 项目文档

本目录提供 Peach 生产项目的专题导航、ADR 与带日期的验证记录，范围为 `src/peach`、`src/serial_force` 及 Web 共用的 `src/ros2_web_link`。2026-11-06 按当前源码核对；系统现状以包级 [architecture.md](../src/peach/peach_bringup/doc/architecture.md)、[io.md](../src/peach/peach_bringup/doc/io.md) 与接口清单为准。`src/peach_sim` 是独立子工作区，Gazebo 当前暂停，不计入生产采摘验收。

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
15. [维护与待验证事项](architecture/06-roadmap.md)

验证章节记录对应日期的结果，不构成当前工作树的重新验收。自动 10×30×1800 全量回归仍按测试文档记录未通过；Orin NX 未实测。2026-10-13 旧 Wiki、未实施 ForceSeat 计划和无调用的历史源码副本已移入 本机归档（仅本机；公开历史见 Git 提交记录），含原路径与 SHA-256。

## ADR

- [包边界](adr/0001-package-boundaries.md)
- [GPL 参数](adr/0002-gpl-parameters.md)
- [FastAPI Web](adr/0003-fastapi-web.md)
- [单一录制](adr/0004-recording.md)
- [FSM 不迁移 BT](adr/0005-fsm.md)
- [独立进程](adr/0006-process-model.md)

## 工作区其他源码

完整导航见 [工作区 README](../README.md)。IVG 12 包在 [独立栈导航](../src/ivg/README.md)，仿真六包在 [peach_sim](../src/peach_sim/README.md)，标定与相机分别见 [标定包](../src/aubo_hand_eye_calibration/README.md) 和 [Percipio](../src/percipio_camera/README.md)。这些文档描述当前实现与已知缺口，历史结果不作本轮运行验收。只读驱动文档的旧路径/IMU/octomap/标定字段差异集中记在 [整栈架构](../src/peach/peach_bringup/doc/architecture.md#只读驱动文档的现状差异)，驱动文件本身不改。
