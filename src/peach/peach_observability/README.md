# peach_observability

观测为主：8090 Web（过程页只读）+ 调试 POST 桥 + 会话 bag + `peach_bag_report`。实现模块与参数 yaml（`config/observability.yaml`，W11 起随包走）都在本包，节点 `ObservabilityParams.attach`。删除本包后核心调度仍可构建，但 8090 不起。

**调试面不是纯只读**：`debug.enabled`（默认 true）放行 `POST /api/debug/<action>` 转发 18 个既有动作/服务端点（含 RunHarvest/Survey/Execute——运动类另需 `debug.motion_enabled`，默认 false 拦 423；每次操作含被拒审计落 `runs/debug_audit/`）。话题/端点/参数契约见 [doc/io.md](doc/io.md) 与 [doc/architecture.md](doc/architecture.md)。

## 公有入口

```bash
ros2 launch peach_observability observability.launch.py
ros2 run peach_observability peach_bag_report runs/session_*/bag
```

会话 bag 由 observability 节点内唯一 rosbag2 写入线程管理；有界关闭走节点生命周期，不经 Web。图名仍是 `peach_observability`。会话 bag 随栈启停（`runs/session_*/bag/`，MCAP 限速进袋）并有体积预算回收（`record.max_total_bag_gb` 默认 100 GB，最旧先删、报告/账本永不删、逐条审计；分析后清理用 `peach_observability.retention.purge_analyzed_sessions` Python API）。Web 静态页资源在顶层资源包 `peach_web`（`share/peach_web/web`，本包 HTTP 层托管），另发 `/peach/observability/{tcp_path,markers}` RViz 可视化。

## 构建 / 测试 / 许可

8090 通过 `/vendor/ros2-web-link.js` 提供 ament 安装树 `share/ros2_web_link/dist/web` 中的 SDK。首次构建依赖包时按其说明准备浏览器资源，再构建 Peach。独立启动本包时，页面默认通过 HTTP 读写。见 [ros2_web_link 集成说明](../../ros2_web_link/doc/integration.md)。

`colcon build --packages-select peach_observability`。`colcon test --packages-select peach_observability`（官方 `flake8` / `pep257`）；行为核对见测试指导。BSD-3-Clause。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。
