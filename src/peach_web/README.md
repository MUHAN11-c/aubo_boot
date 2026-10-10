# peach_web

8090 感知抓取过程页的**静态前端资源**（2026-10-10 自 `peach_observability/web` 拆出为顶层资源包）。只装文件、不跑节点、不进 launch / lifecycle。

- `web/index.html` / `web/app.js` / `web/app.css`：安装到 `share/peach_web/web/`。
- HTTP 托管与 `/api/state`、`/api/trajectory`、`/api/debug/*` 后端都在 [peach_observability](../peach/peach_observability/README.md)（`ObservabilityNode.start_http` 经 `get_package_share_directory('peach_web')` 解析静态根）。接线契约见 [peach_observability/doc/io.md](../peach/peach_observability/doc/io.md)。
- 8090 仍是纯调试客户端：无令牌，运动类操作另受 `debug.motion_enabled` 门控，绕不过 `ExecutionAuthority`。

## 构建 / 测试 / 许可

`colcon build --packages-select peach_web`（`peach_observability` 对本包有 `exec_depend`，整栈构建自动带上）。无单元测试；页面可用性随 observability mock 冒烟（起栈后 `curl http://127.0.0.1:8090/`）。BSD-3-Clause。
