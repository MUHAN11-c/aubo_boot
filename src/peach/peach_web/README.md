# peach_web

8090 采摘操作台的静态资源包，不运行 ROS 节点。默认通过 rosbridge WebSocket 使用 ROS 话题、标准参数服务与现有业务服务/Action；[peach_observability](../peach_observability/README.md) 提供状态聚合、静态页面、图像、历史分析与 HTTP 对照桥，接口见其 [io.md](../peach_observability/doc/io.md)。页面运行只使用本地资源，无 CDN。

首次使用请看 **[Web 调试与测试使用手册](doc/debug-testing.md)**：环境与启动、五页操作、三档测试、参数读写、恢复/取消、结果判定、自动测试和停栈收尾。逐控件测试、预期和未测项见 **[调试功能验收清单](doc/debug-feature-checklist.md)**。构建后文档同时安装到 `share/peach_web/doc/`。文档中的步骤不代表本轮已通过。

## 五个页面

- `/`、`/overview.html`：系统总览、相机预览、四能力链、作业与账本、诊断、录制和五点力摘要；只读。
- `/monitor.html`：过程监控、九阶段流水线、相机/三维/TCP轨迹、目标与账本、节点日志。`/index.html` 是兼容入口。常用操作区保持文档内原位，不随滚动吸附或遮盖下方内容。执行已使能时，手动回拍照位需要臂侧一次性授权并由操作员确认。确认“取消执行”会向臂侧 `cancel_cycle` 请求停止当前操作，可用于调度或其他页面发起的操作。“取消批次”转发调度的 `CANCEL_NOW`。
- `/debug.html`：本轮参数与使能、单轮点选、多轮抓取、人工恢复确认、白名单参数读写。没有第二套运动或 SetIO 接口。
- `/analysis.html`：会话/批次选择、六项成功率、逐果周期与失败分型、历史记录；无效路径明确报错并隐藏旧结果。
- `/force.html`：五通道原始数值、CAD 点位、视场切换和 CoP；只读，不参与抓取许可。

各页共用导航、连接状态、键盘跳转、确认框和通知。支持320/390/768/1440 px；窄屏保留全部入口，宽表在容器内滚动。支持 reduced-motion、当前页 aria-current 和 Escape 关闭确认框。

## 状态与操作语义

页面读取 `/api/transport` 中的部署配置。整栈默认通过同源 `/vendor/ros2-web-link.js` 加载 `ros2_web_link` 的浏览器 SDK，每个实时页面共用一个 RosClient，连接回环地址 `127.0.0.1:9090`（端口可通过 `rosbridge_port` 更改）。总览订阅状态与五点力，监控订阅状态与轨迹，调试页只订阅状态，五点力页只订阅力。三个投影均为 `std_msgs/msg/String`，采用可靠、易失、深度 1 的 QoS。参数通过节点标准 List/Get/Set 服务读写，业务操作复用现有 ROS 服务与 Action。

历史分析页只走 HTTP。整栈设置 `rosbridge_enabled:=false` 或独立运行观测节点时，页面按部署配置使用 HTTP。也可显式打开 [HTTP 调试页](http://127.0.0.1:8090/debug.html?transport=http)。HTTP 模式按需取图像与快照，免加载 ROS SDK。

快照超过5秒或断线时标记陈旧并暂停普通操作，取消入口保留；取消入口可点不等于离线请求已送达。HTTP在线、WS在线与传感器在线分别判断。力帧接收或源时间超过1.5秒时标记数据陈旧并清除读数/CoP；力投影本身停流时也按1.5秒门限清除，不等待6秒请求超时，切换视场不会恢复旧帧。时钟回退使缓存力帧处于未来时，后端返回不可用，等待同一时间域的新帧。HTTP对照轮询同接口至多一个在途请求，后台页暂停新轮询；WS重连只恢复订阅，不重放服务/动作。

调试页写参数后逐项读回，再提交 SetEnables，并同时核对调度意图与臂侧实际 execution/grasp/tool 三项状态。依赖链是 execution→grasp→tool；失联、场景epoch变化、部分写入、拒绝或人工恢复等待均停止后续。FULL的RUNNING接触暂存恢复标志不提前结束等待，只有FSM明确进入RECOVERY_REQUIRED才停止后续并保留请求身份。运行中拒绝改本轮设置；不自动ACK。单轮点选绑定当前场景，跨轮重新选目标，不复用上一场ID。

调试页测试档提供观察/建场景、L2预抓取、L6完整干跑三档说明与步骤。载入只填草稿，三档均关闭工具，保留“本轮使用重建”的当前选择，不写参数、不发使能。默认不使用重建，空闲时可切换ON/OFF后确认写入，分别记录新请求比较效果。当前证据分别显示调度意图与臂侧实际使能、动作/恢复状态及匹配当前request_id的账本；模型allowed是原始观测，不替代执行前复检，不把旧账本当新测试通过。恢复ACK须先取消旧批次，待已结算/中断（6/8）、恢复锁仍在、两侧动作停止、快照新鲜时开放；发送前须人工确认，状态序号变化则拒发。多轮每轮成功后有界等待同请求的终态与双锁清除，防止Action结果先到、状态仍旧而误判。

参数表按应用参数清单元数据展示类型、范围、当前值和待写值，支持参数名/用途/节点搜索、只看待写入和准确撤销差异。节点离线或参数未声明时只读。整数小数、非有限数、越界值和非法轮数在发送前拒绝；参数写入与本轮写入互斥，写入直到读回结束保持提交锁。HTTP400/207、ROS逐项拒绝或读回失败保留草稿，不显示成功。

本轮重新读取期间也持有互斥锁，禁止载入草稿、编辑或提交。写入确认使用冻结的设置快照，不在确认后重新解释变化的复选框；本轮读回更新参数当前值时保留尚未应用的参数草稿。

参数面板“重新读取”也保留未应用的输入，并按最新当前值重新计算差异；读取失败保留草稿并提示当前值未刷新。

业务写入与Action不自动重试；同类请求防重复。WS断线保留未知请求，重连后先查同身份或取消。SDK 等待动作超时后，本页保留目标句柄并阻止重复提交。同一 WS 会话可请求取消。重新连接后应通过调度或技能取消入口核实原请求终态。HTTP受理与ROS服务的 accepted/success 分开解释；202保留在途请求身份，500/网络超时显示结果未知并提示核对/取消，423表示拒绝。浏览器等待上限195秒，关闭页面不会取消ROS动作。

总览按接口枚举显示L0至L7：L2=预抓取已核验，L6=撤退已确认，L7=采摘已确认。L6在tool=false时只证明干跑。节点诊断异常数只统计非OK节点；WARN与ERROR分别着色。

监控账本按当前FailureCode的0至35显示稳定常量名与中文原因，包含作业超时、使能未开、重建启动/采集/精化失败及剪切间隙；未知新码仍保留数值。

8090是调试客户端。HTTP调试桥的debug运动门和debug_audit仅覆盖HTTP；rosbridge直接ROS调用依赖节点自身使能与臂侧ExecutionAuthority，不能将HTTP门或审计当作WS授权层。上电、硬件急停与保护停止复位由现场负责；网页取消只属于应用停轨。

部署参数 `reconstruction_enabled=true` 默认保留重建节点，运行策略 `skip_reconstruction=true` 默认关闭本轮Build/精化。ON逐项读回skip=false、capture.auto_mode=true、quality.allow_unrefined_geometry=false；OFF读回true/false/true。混合模式或未知读回阻止开始，部分成功不假设回滚。未部署/未运行时页面不能启动重建节点，须重新部署。OFF的旧模型不作当前证据；目标TSDF开关不关闭MoveIt PlanningScene、TF或碰撞检查，mock也不证明现场安全。

## 源码地图

- `rosbridge.js` 将 Peach 的投影、参数清单和调试请求接入 SDK。`api.js` 管理 HTTP 对照、超时和状态更新。`shell.js`：共享导航、确认框、通知。
- `overview.js`：总览；`app.js` + `monitor-renderers.js` + `trajectory-renderers.js`：监控；`visualization.js`：相机/三维。
- `debug.js`：调试页组合与参数；`debug-session.js`：单轮/多轮会话；`debug-evidence.js`：只读测试说明与证据；`debug-shared.js`：请求分类与门控；`ui-common.js`：共享展示函数。
- `analysis.js`：历史分析；`force.js`：五点力。

三维统一base_link、米、Z向上；工具方向只用实际TF，缺失不画默认方向。模型身份不能对应当前目标/场景时不绘制。相机、模型和TF分别呈现时间，不宣称同步。WebGL不可用时保留二维轨迹及数值。

## 构建与验证

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to peach_web
source install/setup.bash
colcon test --packages-select peach_web --return-code-on-test-failure
colcon test-result --test-result-base build/peach_web --verbose
```

`colcon test`运行官方lint；浏览器行为用现有mock栈和Playwright脚本验证：

```bash
node src/peach/peach_web/scripts/e2e-console.js --base-url http://127.0.0.1:8090 --transport http \
  --out runs/web_e2e_YYYYMMDD_HHMMSS
# FULL mock单轮，默认重建OFF；要求同请求成功且L6，结束取消并关闭使能
node src/peach/peach_web/scripts/e2e-console.js --base-url http://127.0.0.1:8090 \
  --transport rosbridge --only harvest --harvest --reconstruction off --out runs/web_full_off_YYYYMMDD_HHMMSS
# 独立ON比较组，新请求与新输出目录
node src/peach/peach_web/scripts/e2e-console.js --base-url http://127.0.0.1:8090 \
  --transport rosbridge --only harvest --harvest --reconstruction on --out runs/web_full_on_YYYYMMDD_HHMMSS
```

脚本`--transport`默认http，五页fixture套件使用HTTP；原生WS动作核对用`--transport rosbridge --only harvest --harvest`，HTTP动作对照改成`--transport http`并另存目录。页面采用部署指定的传输方式。Playwright须在Node模块搜索路径中可用；可用`NODE_PATH`指向本机已有安装。脚本在任何操作前要求后端hardware_mode=mock。默认检查也会创建一次不开执行的调试会话；它并非纯只读。脚本覆盖五页、四屏宽、视觉控件、日志过滤、连接恢复、错误结果与提交锁，输出report.json及截图。`--only <页面名>`可定向复查；`--only harvest --harvest`仅做静态检查和FULL单轮，按`--reconstruction on|off`（默认off）选择重建，在抓取前保存harvest_settings.json逐项核对FULL及skip/auto/allow实际值。依赖、CLI或模式前置失败均计入失败报告。真实数据联调和RViz流程见[测试指导](../peach_system_tests/doc/testing.md)。测试完成必须停栈并核对无本轮进程残留。

`ros2_web_link` 提供通信 SDK 与第三方许可。首次从干净源码部署时，先在该依赖目录执行 `npm ci`，再运行上面的 colcon 构建。Peach 自有的三维工具视图使用 Three.js 0.186.1，本包通过 `npm ci && npm run build:vendor` 更新其静态资源。项目许可为 BSD-3-Clause。
