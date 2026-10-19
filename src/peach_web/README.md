# peach_web

8090 采摘过程控制台的静态前端资源。`web/index.html`、`app.js`、`app.css` 安装到 `share/peach_web/web/`；本包不运行 ROS 节点，不进入 lifecycle。HTTP 托管与 API 在 [peach_observability](../peach/peach_observability/README.md)，接口契约见该包 [io.md](../peach/peach_observability/doc/io.md)。

## 页面与反馈

- 首屏显示运行模式、启动自检、诊断摘要、运行期使能与当前作业。两个视图共用常用操作区：「回到拍照位」转发既有 `photo_pose_service`，保留发送前确认，取消执行/批次保持显眼且独立。批次执行档读取 `/api/state.params./peach_supervisor.execute_pregrasp_only`，明确显示 PREGRASP_ONLY/FULL；当前条件解释运行期使能关闭与调度 blockers，许可允许时的 reason 仅作诊断。当前完成证据沿用 `job.why`，取消不会抹掉已验证的预抓取；感知 HARVESTED 只表示已结算，采摘确认以当前账本 completion_level=7 为准。常用区反馈操作结果，详细响应保留在单步调试。顶栏固定当前阶段（批次态、目标阶段、技能周期、目标 id）。节点/关节硬件、系统负载、参数与系统诊断默认折叠。阶段时序按服务器缓冲全量在面板内滚动。节点日志读 `/api/state` 的 `logs`：时间、级别、节点、消息；级别是该级及以上（默认 INFO），节点名是包含过滤；离开底部不抢滚动。
- 过程/调试两个视图支持键盘左右键、Home/End 与焦点提示；日志、目标与账本表在窄屏切换为堆叠条目，关节表等宽表仍在容器内横向滚动，支持 reduced-motion。页面使用本地字体、不使用 CDN；本地 JPEG 使用 blob URL，后端 CSP 仅对 img-src 增加 blob:，脚本仍为 self。
- 状态快照每 1 s、轨迹每 0.4 s 尝试更新，每接口至多一个在途请求，GET 等待上限 6 s。5 s 无新快照或请求失败时标明陈旧/断线并保留旧数据，暂停普通操作。后台页面不发新的 GET；恢复可见或浏览器历史恢复时继续刷新。
- 操作提交期间同类请求防重复，轮询不会解除提交锁；取消动作与 CANCEL_NOW 独立。前端在确认之后、发送之前重新核对状态新鲜度、后端调试开关和对应的动臂门。回拍照还呈现恢复锁与臂周期运行的阻挡；不自动 ACK、武装或开使能。臂侧 execution_enabled=false 时服务可以只规划，页面提示「仅规划拍照路径」，不额外禁止服务。前端不能代替 ROS 授权门。
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
node src/peach_web/scripts/run-tests.cjs
```

测试依赖 Node.js 18+ 与 `ament_cmake_test`；使用 Node 内置 test/vm/assert。CTest 分别注册 `test/*.test.cjs`，直接运行每个测试文件并生成 XUnit。独立 runner 同样逐文件执行，传播子进程失败、信号与启动错误。覆盖请求与确认复检、提交锁与取消、结果未知、恢复锁、预抓取证据、目标选择与换场景、模型来源与错轴、无效几何、图像帧龄与 TF 失联，以及 WebGL 降级；VM 测试之外还须浏览器布局验收。

## 分步实验台与定位方向

- 页面使用暖白主题（暖纸底、深墨文字、桃色强调），桌面主区并列相机与三维，右侧集中路径证据和几何计算；监控面板分两列，窄屏保留全部入口并纵向布局。关键点 P=预抓取、E=袋外入口、C=剪切参考；袋底、袋口、重建中心与历史轨迹末端均有标签，工具方向仍只使用真实 TF。密集标签避让，详细坐标可展开。
- 分析显示后撤、剪切行程、轴向投影、横向偏差、行程差、初值/融合入口位移、轴夹角、拟合 RMSE、内点比、D95 和径向/轴向余量。公式直接使用同目标、同场景的 base_link 几何；缺失为“—”，负余量保留，距原点距离不表示可达半径。预算条以零为中线，通过 DOM 样式更新绘制，兼容既有同源 CSP。
- 几何/预抓取/套入/剪切四项能力沿用 TargetModel 的 0=VALID、1=INVALID、2=UNKNOWN；汇总 allowed 与机械臂路径证据独立。路径证据引用当前目标规划失败或完成级别，未验证时明确显示未知。显式可达性查询逐个位姿展示 reachable 和 error_codes，HTTP 请求成功不代表求解成功。

- 六步为拍照与建场景、扫场与选目标、重建与核对、只规划、预抓取、完整周期。切换步骤和目标不会发送命令。人工选择与实验标识保留，场景 epoch 改变后旧选择失效；确认后再次核对捕获的目标与场景。
- 图像支持稳定识别图/即时检测图切换、放大和暂停；暂停只冻结画面。JPEG 独立于录包；来源、曝光 ROS 时间、帧龄和不可用原因可见。重建查询、finalize、保存、复位和取消，以及生命周期、自检、恢复确认、批次和显式可达性查询沿用现有 POST 接口。
- Three.js 图像对照显示感知初值、融合模型、袋轴、预抓取、剪切参考、实际末端坐标轴及 TCP 轨迹，统一 base_link、米、Z 向上。工具轴只取实际 tool_axis TF；缺失或陈旧位姿不画默认方向。模型来源无法对应当前场景时不绘制，原始诊断仍可查询。错轴是红色诊断几何，不代表可执行许可。
- 图像、模型、TF 分别展示时间与新鲜度；异步观测不声明同步。三维支持拖动、缩放、居中、俯视、侧视、复位；刷新保留用户视角。WebGL 不可用或丢失时保留原二维轨迹及数值。
- 桌面采用步骤/对照/操作三栏；窄屏同时保留相机与三维并上下排列，画面按钮用于快速定位，不再隐藏另一画面。监控页优先显示定位、方向和完成证据。日志支持级别、节点、关键词与暂停滚动。

Three.js **0.186.1**、esbuild **0.28.2** 由本包 package-lock.json 固定，编译后的 `web/vendor/peach-three.js` 与许可随静态资源安装；页面运行无需 npm 或网络。更新 bundle：

```bash
cd src/peach_web
npm ci
npm run build:vendor
npm test
```

浏览器验证使用独立静态站点与假的 `/api/state`、`/api/trajectory`、`/api/debug/*`，覆盖 320/390/768/1440 px 两视图、慢请求、断线恢复、POST错误和重复点击；不向真机发请求。安装后在现有 observability 服务刷新页面即可加载新静态资源；整栈 mock 启停与清理见 [v1 测试指导](../peach/peach_system_tests/doc/testing.md)。任何临时 HTTP 服务器或测试浏览器结束后都要关闭，不残留进程。

参考：[Fetch 与异常处理](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API/Using_Fetch)、[ARIA tab](https://developer.mozilla.org/en-US/docs/Web/Accessibility/ARIA/Reference/Roles/tab_role)、[Node test runner](https://nodejs.org/api/test.html)。

许可：BSD-3-Clause。

监控页光照统计展开为目标有效深度、检测置信度与连续低质帧，避免将对象显示成字符串。
缺失的精化轴显示为不可用；零向量不代表有效单位方向。
缺失或目标身份不符的拟合为 null，不显示默认 ACCEPT。记忆锚点保留历史关键点，
目标表和三维标注“无本帧拟合”；未测置信度、尺寸与行程显示“—”，不套用本帧时间。
有身份的真实零点数、零深度比例以及 0=VALID/ACCEPT 枚举仍按原值解释。

实时联调：首次打开也允许加载已编码的陈帧 JPEG，并保留曝光时间/帧龄与陈旧标记；陈帧显示不改变状态新鲜度、重建采帧或执行门。几何条形图不会把 null 计为 0 mm。

当前验收须分别记录模型ACCEPT、页面allowed、臂侧许可、动作终态和账本完成级别；level6干跑不等于level7真实采摘。30组模型、10轮链路、30分钟稳定性与bag封口/清进程见测试指导，既有部分成功与RViz GL报错不能记为全通过。

## 前端源码地图

`web/app.js`保留页面组合；`ui-common.js`提供共享展示函数，`monitor-renderers.js`绘制监控，`trajectory-renderers.js`绘制轨迹，`request-runtime.js`管理请求生命周期。HTTP服务按同源白名单托管四模块。模型READY/accepted、四能力、页面allowed和臂侧许可分别呈现；诊断心跳不代表模型重新生成。脚本拆分的离线行为回归不替代真实相机+mock+RViz验收。
