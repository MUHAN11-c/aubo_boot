# peach_web

8090 采摘过程控制台的静态前端资源。`web/index.html`、`app.js`、`app.css` 安装到 `share/peach_web/web/`；本包不运行 ROS 节点，不进入 lifecycle。HTTP 托管与 API 在 [peach_observability](../peach/peach_observability/README.md)，接口契约见该包 [io.md](../peach/peach_observability/doc/io.md)。

## 页面与反馈

- 首屏显示运行模式、启动自检、诊断摘要、运行期使能与当前作业。顶栏固定当前阶段（批次态、目标阶段、技能周期、目标 id）。节点/关节硬件，以及系统负载、参数、自检、诊断默认展开。阶段时序按服务器缓冲全量在面板内滚动。节点日志读 `/api/state` 的 `logs`：时间、级别、节点、消息；级别是该级及以上（默认 INFO），节点名是包含过滤；离开底部不抢滚动。只读，不增加 SetIO 或运动按钮。
- 过程/调试两个视图支持键盘左右键、Home/End 与焦点提示；表格窄屏在容器内滚动，支持 reduced-motion。页面不使用外部字体或 CDN，符合后端 self-only CSP。
- 状态快照每 1 s、轨迹每 0.4 s 尝试更新，每接口至多一个在途请求，GET 等待上限 6 s。5 s 无新快照或请求失败时标明陈旧/断线并保留旧数据，暂停普通操作。后台页面不发新的 GET；恢复可见或浏览器历史恢复时继续刷新。
- 操作提交期间同类请求防重复，轮询不会解除提交锁；取消动作与 CANCEL_NOW 独立。前端在确认之后、发送之前重新核对状态新鲜度和后端调试开关，不能代替 ROS 授权门。
- HTTP `accepted` 与 `result.success` 分开：服务 success=false 显示操作失败，HTTP 202 显示仍在执行并保留取消提示，HTTP 504/502/500 等或网络失联显示结果未知。只有明确 success=true 才显示操作成功；其他结果提示查看详情。
- POST 不自动重试。浏览器等待上限 195 s（后端常用 180 s 加 HTTP 余量；后端预算由其参数独立控制），超时/关闭页面不会取消 ROS goal；应核对状态或使用已有取消入口，不能直接重复发送。

8090 仍是纯调试客户端：无令牌；后端 `debug.enabled`、`debug.motion_enabled`、调度使能与臂侧 `ExecutionAuthority` 照常强制。运动按钮沿用确认流程，网页不增加 SetIO 或授权开关。取消是应用停轨，不能替代示教器/柜硬件急停。

## 构建与测试

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select peach_web
source install/setup.bash
colcon test --packages-select peach_web --return-code-on-test-failure
colcon test-result --test-result-base build/peach_web --verbose
# 直接运行同一组前端行为回归，无 ROS 图或网络
node src/peach_web/test/web_state.test.cjs
```

测试依赖 Node.js 18+ 与 `ament_cmake_test`；使用 Node 内置 test/vm/assert，无 npm 依赖，也不把 Node 作为页面运行依赖。CTest 注册 `test_web_state` 一组，Node 输出 15 项断言及失败返回码；ament 包装生成 XUnit。覆盖请求重叠、超时恢复、明确失败/未知/仍在执行、提交锁与取消、断线、确认后陈旧、BFCache恢复、键盘切页、日志级别过滤和顶栏当前阶段。VM 使用最小 DOM 与假的 fetch；实际 DOM/布局需浏览器验证。

浏览器验证使用独立静态站点与假的 `/api/state`、`/api/trajectory`、`/api/debug/*`，覆盖 320/390/768/1440 px 两视图、慢请求、断线恢复、POST错误和重复点击；不向真机发请求。安装后在现有 observability 服务刷新页面即可加载新静态资源；整栈 mock 启停与清理见 [v1 测试指导](../peach/peach_system_tests/doc/testing.md)。任何临时 HTTP 服务器或测试浏览器结束后都要关闭，不残留进程。

参考：[Fetch 与异常处理](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API/Using_Fetch)、[ARIA tab](https://developer.mozilla.org/en-US/docs/Web/Accessibility/ARIA/Reference/Roles/tab_role)、[Node test runner](https://nodejs.org/api/test.html)。

许可：BSD-3-Clause。
