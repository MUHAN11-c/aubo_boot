# peach_observability

观测为主：FastAPI/uvicorn 托管 8090 Web、状态投影、HTTP 调试桥、会话 bag 与 `peach_bag_report`。参数声明在 `config/observability_parameters.yaml`，部署值在 `config/observability.yaml`，由 `ObservabilityParams.attach` 绑定节点。静态资源由 `peach_web` 安装；核心调度不依赖 HTTP。

**调试面不是纯只读**：HTTP 调试桥转发 19 个既有动作/服务端点，另有取消与只读自检入口。普通转发受 `debug.enabled`（默认 true）控制，运动类另需 `debug.motion_enabled`（默认 false，拒绝时 423）。进入调试命令处理的请求按 `debug.audit_enabled` 写入 `runs/debug_audit/`；自检独立受 `selfcheck.enabled` 控制。整栈默认的 rosbridge 路径直接调用 ROS 接口，HTTP 审计不覆盖它；最终执行仍由目标节点和臂侧命令门复核。详细契约见 [doc/io.md](doc/io.md)。

## 公有入口

```bash
ros2 launch peach_observability observability.launch.py
ros2 run peach_observability peach_bag_report runs/session_*/bag
```

会话 bag 由节点内唯一 rosbag2 写入线程管理；configure 开会话，cleanup/shutdown/destroy 有界关闭，deactivate 只停运行期服务。图名是 `peach_observability`。MCAP 位于 `runs/session_*/bag/`；预算回收默认 100 GiB，按 bag 目录 mtime 从旧到新处理已封口目录，保留报告/账本并先持久化删除审计。分析后显式清理用 `peach_observability.retention.purge_analyzed_sessions` Python API。节点另发 `/peach/observability/{tcp_path,markers}` RViz 可视化和 `~/web_{state,force,trajectory}` 浏览器投影。

## 构建 / 测试 / 许可

8090 通过 `/vendor/ros2-web-link.js` 提供 ament 安装树 `share/ros2_web_link/dist/web` 中的 SDK。首次构建依赖包时按其说明准备浏览器资源，再构建 Peach。独立启动本包时，页面默认通过 HTTP 读写。见 [ros2_web_link 集成说明](../../ros2_web_link/doc/integration.md)。

`colcon build --packages-select peach_observability`。`colcon test --packages-select peach_observability`（官方 `flake8` / `pep257`）；行为核对见测试指导。BSD-3-Clause。

## 当前维护入口

职责、源码地图、参数来源、生命周期/并发与失败处理见 [包架构](doc/architecture.md)。运行环境、分层验证、冻结回放、30组模型/10轮/30分钟验收与精确清理见 [测试指导](../peach_system_tests/doc/testing.md)。本包测试通过不代表现场采摘成功；未授权不得真机运动或SetIO，硬件急停不经ROS，程序结束必须清本轮进程。
