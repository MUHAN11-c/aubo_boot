# 维护与待验证事项

按 2026-11-06 当前源码整理。包级活文档记录实现，历史运行结果见 [验证与证据](05-verification.md) 和 [回归矩阵](12-regression.md)。

1. 参数：生产能力、观测和 bringup 辅助节点使用 GPL；独立 vegetation 保留 `attach`。升级生成器后重新检查整数范围与回调先后语义。
2. 错误码：维护 FailureCode 稳定值；保留旧事件文本兼容，逐项审查纯核中剩余数值而不凭字符串猜语义。
3. Web：整栈默认 SDK/ROSbridge + 同源 HTTP；HTTP 调试门、审计与 ROS 节点最终授权的范围须分别验证。
4. 记录：会话 bag 由 observability 单写者维护；力分段仅兼容读取历史 sidecar，当前没有自动分段写者。
5. 性能：x86 历史基准不能替代 Orin 实测；TensorRT/Orin 验收仍待目标机。
6. 回归：按受影响包官方检查和手工 mock/replay 验证；感知已有图像解码 pytest 不扩展为新行为套件。10×30×1800 未通过，不能用短测补成通过。
7. 归档：旧 Wiki、会话转录、未实施 ForceSeat 与未安装的 sim_perception/vision 快照已归档，原路径/校验见工作区 `_archive/source_docs_2026-11-06/manifest.json`。

Gazebo 当前暂停；仿真子工作区的离线文档整理不表示恢复运行授权。
