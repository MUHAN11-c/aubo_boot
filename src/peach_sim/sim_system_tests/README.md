# sim_system_tests

本包只保留官方 ament lint。`colcon test` 不包含自定义 gtest、pytest、launch_testing 或 xacro 结构断言，不能证明导航、抓取和落筐正确。

在 `peach_sim/` 根运行：

```bash
scripts/sim build
scripts/sim test
scripts/sim run --duration 65
scripts/sim run --mission --max-targets 1 --duration 480
scripts/sim run --mission --cancel-after 30 --duration 240
```

第一类验证构建与官方 lint；启动检查核对 clock、joint_states、RGB、odom；任务检查必须同时核对 Nav2 实际位移、MoveIt 轨迹执行、实际接触残差、Gazebo 连接/释放、释放后稳定入筐和最终 action result；取消检查核对在途动作终止及本次进程清理。可加 `--gui --rviz` 检查两个可视化窗口与同一场景。

运行器把证据写到 `.runtime/session_*/` 的 summary.json、launch.log 和节点 JSONL，按本次会话复查，不沿用历史日志替代当前结果。2026-10-29 已有一次 65 秒启动检查通过：clock 28,504、joint_states 9,731、RGB 214、odom 2,919；这不代表导航/采摘/视觉已通过。

默认真值模式只验证仿真操作闭环。`--perception vision` 的检测、锁定、建模与精确时间 TF 要独立取证；field 整体 mesh 只作导航/视觉预览。失败或取消后若发生切断/持果不确定，需要重启仿真恢复，不自动重新导航。

使用 `scripts/sim` 保持项目外只读、独立 ROS domain/Gazebo partition 与有限时长进程组收口；不要全局 pkill 其他会话。测试结束必须确认本次进程组已退出。
