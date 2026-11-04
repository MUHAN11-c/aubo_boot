# 待迁移清单

1. ~~**GPL 参数**：六个生产/观测包已完成。~~ `peach_vegetation` 不在主链范围，仍保留 `attach`。
2. **日志/错误码**：已完成 FailureCode 枚举化、`failure_code_n`、`CUT_FRUIT_CLEARANCE` 拆分与日志规范文档。
3. **录制**：已完成单一路径（节点内 rosbag2 MCAP）；旁路 `record_bag` 已删除。
4. **Web**：FastAPI 后端与 `peach_web` 五点力页已完成；8099/8100 第二套入口已删除。
5. **性能/Orin**：x86 实测 13.74 FPS / p95 78.9 ms；Orin profile 已提供，TensorRT/Orin 实测待目标机。
6. **文档**：最终架构图、流程图、参数生成文档、ADR；删除旧包文档中的过期描述。
7. **回归**：真实相机 + RViz、感知 benchmark、bag 录制/读取、force serial/sim、接口/包图检查。

`peach_sim` 始终不修改。
