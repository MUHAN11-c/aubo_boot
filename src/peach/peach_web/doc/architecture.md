# peach_web 架构

本包安装五页静态资源，不创建 ROS 节点或 HTTP 服务。`CMakeLists.txt` 将
`web/`、`doc/` 安装到 `share/peach_web/`；8090 服务由
[`peach_observability`](../../peach_observability/doc/architecture.md) 的
FastAPI/uvicorn 托管。页面、脚本和许可均从本机提供。

## 源码与职责

重建性能由现有状态镜像 `reconstruction.diagnostics.timing` 投影。监控渲染器只展示固定阶段白名单中的有限非负采样，不增定时器、订阅或控制。阶段是最近完成值，可能跨次且嵌套，页面明确不能相加；慢阶段的实时 WARN 来自节点 `/diagnostics`，沿用诊断卡片与日志。

- `overview.html/js`：只读总览；`monitor.html`、`app.js`、
  `monitor-renderers.js`、`trajectory-renderers.js`：过程、账本、日志和常用操作。
- `debug.html/js`、`debug-session.js`：本轮参数、单轮和多轮调试；
  `debug-shared.js` 处理请求状态和操作门，`debug-evidence.js` 展示完成口径。
- `analysis.html/js`：只读历史分析；`force.html/js`：只读五点力与 CoP。
- `api.js`：请求时限、单在途轮询、新鲜度和错误反馈；`rosbridge.js`：
  将同一页面请求映射为 SDK 订阅、标准参数服务或既有业务服务/Action。
- `shell.js`、`ui-common.js`：导航、确认框与共享展示；`visualization.js`、
  `viz3d.js`：JPEG 和本地 Three.js 三维视图。
- `scripts/three-entry.js`：Three.js 静态 bundle 入口；
  `scripts/e2e-console.js`：显式运行的 Playwright mock 检查，不随 `colcon test` 执行。

## 传输与请求生命周期

实时页先读 `/api/transport`。整栈启桥时每页共用一个 `RosClient`，按需订阅
状态/力/轨迹投影；参数经标准 List/Get/Set 服务，业务操作经现有 ROS 接口。
独立观测节点、停桥部署或显式 `?transport=http` 使用 HTTP 对照路径。
图像与历史分析始终通过 HTTP，分析页不建立 WebSocket。

WS 断线仅恢复订阅，不改走 HTTP 重发命令。请求共用单调截止时间；超时、
中断或关闭页面不证明远端动作已取消。动作结果未知时保留身份并阻止重复提交，
由现有状态/取消入口核对。HTTP 调试门和落盘审计仅覆盖 HTTP；WS 路径虽读取
调试开关，最终许可仍由目标节点与臂侧执行门强制。

本地测试档只填草稿。写入逐参数确认并读回后才发 SetEnables，随后核对调度与
臂侧实际使能。场景变化、部分写入、失败或人工恢复等待停止后续请求；不自动 ACK。

接口与新鲜度见 [io.md](io.md)，完整操作及验证步骤保留在
[debug-testing.md](debug-testing.md)。`colcon test` 只检查本包官方 ament lint；
浏览器模拟、实际 ROS mock 周期、RViz 和真机验收须分别记录。
