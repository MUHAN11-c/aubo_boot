<!-- 0.8.3当前行为补充；下文具名旧版本结果保留为历史证据。 -->

0.8.3当前Service预算与缓存开关独立：默认cache0，所有连接共享线程创建前的准入限额128，创建等待5秒。`python3 scripts/service_budget_probe.py`在已source Jazzy的终端检查执行器阻塞、迟到工厂、异常隔离与关闭唤醒，不创建ROS实体。`npm run workbench`检查项目模板订阅选项保留。旧`--trace-late-client`及`--late-reader`属于归档诊断，当前Node在启动前明确拒绝；下文对应旧版复现命令不能用作0.8.3验收。

完整0.8.3主安装验收使用无界面Chromium，相关参数、逐档结果、清理与局限见[性能参数文档](jetson-orin-nx-performance.md)。禁止用有界面自动点击压测干扰当前桌面操作。

# 功能自测与使用

压力与部署参数新增[NX 16GB 本机浏览器压力测试](jetson-orin-nx-performance.md)。本轮使用隔离域220、HTTP22688/WS22690与独立Chromium；每档检查自己的原始判定、输入哈希和退出状态。0.8.2的cache0超时失败保留；0.8.3默认改为cache0并加入进程局部DDS恢复配置，按相同门限重测。ARM64实机尚未连接，不能用桌面PASS代替目标机验收。

以下步骤覆盖本包无硬件示例与只读录制数据，按对应配置选择入口。不要启动真实机械臂或 IO。浏览器尺寸模拟只能验证布局，不能替代 Android 实机记录。

0.7.1历史默认1800秒2691RPC0失败、SDK17/网页表单10/demo按钮12/随包browser CLI21项分别通过。当前验收状态按development.md I63–I66及performance.md页首读取；下文带版本/轮次的小节保留当时失败与待办，不把历史“未通过”当作当前状态，也不把历史通过扩展至新配置。Android物理平板按用户2026-10-30指示本轮暂不测试，不是完成的验收项。

0.7.1真实表单回归覆盖10组操作及12项demo按钮：启动中停止标准图像、PNG/JPEG恢复与超限/特殊深度拒绝、Topic、Service、Action、九类参数恢复、Lifecycle、图谱/类型和四视图并用。详见I65原始步骤；图谱与参数面板使用有界预览，不期待完整大数组，必要时导出完整项目报告或用公有SDK读取。标准图像停止会移除src/回收URL，不停止独立ImageView。

回放容量核对可在`run_transport_benchmark.py`加`--write-queue-size 16`，或给`replay.launch.py`传`bridge_write_queue_size:=16`。这是上游每连接出站消息数，不是字节数；1..4096，省略/launch空值保持web.launch原默认64或部署方已有全局配置。可选参数使用不同名称，避免共享launch作用域中的空值遮蔽子默认。缩小队列可能丢消息，必须同时核对晚加入静态TF、服务回复、逐流恢复，不能只看RSS；也不能由只读回放推导真机Action/命令回复可靠。显式`launch_queue_probe.py`核对CLI、实际Include与子参数求值，不执行ROS进程，不代替容量测试。


## 0.8.1 界面核对

本轮入口为工作区 `runs/ros2_web_ui_20261102/run_browser.py`，使用独立域 219、HTTP 22588 / WS 22590，普通安装位于同目录 `install`。沿用 3 GiB 后代 sumRSS、4 GiB 主机可用内存和 260 秒单轮守卫；外层 systemd 有时间/内存上限。每轮检查原输入哈希、自有 PID 创建时间与端口归零。

- `scripts/workbench_browser_probe.mjs`：8 组工作台数据、编辑与收尾核对；按新页签与折叠入口操作。
- `scripts/workbench_layout_probe.mjs`：10 组导航、离线空态、连接面板、键盘、实时画布、Topic 筛选、接口定义、1280/768/390 宽度与断线状态检查，并输出截图。此项不是平板实机验收。
- `scripts/browser_probe.mjs`：21 项经典调试流程；导航助手先点击真实导航/折叠控件并等待可见，不强制点击隐藏元素。
- `npm run graph`：保留图谱采集边界，新增共用发布者的无关 Topic 不应进入搜索结果的复现。`npm run workbench`、`npm run preview` 和 `npm run check`继续核对既有逻辑。

布局探针首轮在懒加载完成前读取空态，第二轮在 hashchange 处理前断言当前面板；两轮失败保留。修正为等待实际控件/面板可见，未增加产品超时或放宽断言。关系图原筛选沿两跳扩展，把同一发布者的无关 Topic 带入，`graph-filter-red.log`复现后修复。完整结果、截图与清理结论见本轮 `summary.json`。

启动后按右上角“连接设置”选择项目并连接；进入“可视化工作台”打开组件。完成手测后仍须在 launch 终端 Ctrl+C 并核对进程，不因页面导航改变而默认结束任务。0.8.0 的 120 秒 Peach 回放和 0.7.1 的长测保留原版本归属，本轮没有重新宣称其完整性能门已覆盖全部新视图。

## 0.8.0 工作台核对入口

先按 [workbench.md](workbench.md) 启动。无硬件示例的实际浏览器流程8组、SDK17项、经典页面21项通过；CPU graph/workbench/observe/preview/audit 与37项官方lint分别记录。新增显示、类型/QoS属性编辑与回滚、不同Topic/同Topic双云、逐路TF错误、真实节点退出、IDL字段、配置保存/导入、经典工具互斥、断开清理均有对应步骤。唯一fresh只读审查的六条问题在 review-red.log 中复现，修复后 review-green-v2.log 通过；不把纯CPU当成GPU验收。

原生有界 runner 位于工作区 runs/ros2_web_workbench_20261102/run_browser.py。测试须等待HTTP、WebSocket和demo节点就绪，不能以静态HTTP先开作为桥就绪。桌面本轮为 DISPLAY=:0；headless在本机无法创建WebGL，旧:1不可用，两次失败保留。scripts/workbench_browser_probe.mjs 默认桌面模式；旧 browser_probe.mjs 用 WEB_LINK_HEADED=1 显式选择桌面模式。

Peach使用已结束的真实相机+mock录制，未启动相机、硬件、Peach业务或Gazebo。native-peach-03完成120秒新版显示负载、4路点云/8路空间/11路总显示、6活动关节、12/12模型资产、21 TF与RGB-D/曲线、2次clock回退。所有动态输入使用同SDK实例上的额外轻量回调只计数/时间戳，不保存payload，不增加网络reader；10秒流推进、3秒RPC、原队列和3GiB树sumRSS门未改变。

录制首个完整云在起始后3.319419981秒，因此clock回退会先清空旧云，并等待该录制段的新帧；测试采用离线recv_timestamp分析得到的4320ms窗口（首帧偏移向上取整+1秒交付余量），仍小于10秒流推进门。错误要求瞬时非零的native-peach-01与错误假设1500ms的native-peach-02保留FAIL；recorded-gaps.json给出原数据空窗，不以失败轮的目标120秒宣称通过。

查看每轮resources.json的status、inputHashesUnchanged、PID创建身份、remaining与端口；systemd-run结束显示的一行MemoryPeak不代表所有后代峰值，应读runner实际后代sumRSS采样。本轮UI06为1920.90MiB，Peach03为2175.23MiB；这些是含浏览器的功能负载，不是1GiB无浏览器通信门。最终分发重建与公有入口记录独立保存在工作区artifacts/ros2_web_link/。

## 启动与构建检查

按 README 构建，在启动前检查占用端口与现有 ROS 进程。可设置独立 `ROS_DOMAIN_ID`（例如 87）隔离工作区内其他系统；所有自测终端必须使用同一个域。启动 `self_test.launch.py`，浏览器访问 8088，在右上角“连接设置”中点击连接。网页打不开先核对是否完成 npm build 和 colcon install；网页能打开但未连接，核对 9090、绑定地址和防火墙。

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws/src/ros2_web_link
npm run check
npm run build
npm run api
npm run observe
# 启动 self_test.launch.py 后，在另一个终端运行：
npm run probe -- ws://127.0.0.1:9090
# 使用已安装的本机 Chrome；也可 CHROME_PATH 指定：
npm run browser -- http://127.0.0.1:8088
# source ROS 与本包环境后运行，与示例节点处于同一 ROS_DOMAIN_ID：
npm run fault
```

colcon test 只运行 ament 官方 lint，绿灯不等于行为验收；显式 CLI probe 会对真实 rosbridge/DDS 示例验证返回值、故障和清理，不依赖自定义 gtest/pytest/launch_testing。browser probe 会真实点击所有操作组并生成截图与 JSON，默认输出 /tmp/ros2-web-link-browser，可由 WEB_LINK_EVIDENCE 更改。fault probe 仅启动并停止自己在 19090 端口的临时桥，验证断线后请求结束和订阅恢复，不停止用户正在使用的 9090 桥。

## 网页逐项核对

1. **联通**：连接显示已连接，手动刷新图谱可看到 demo 节点与接口。断开后操作按钮禁用；重新连接后恢复可用。
2. **一键自测**：点击“运行功能自测”，逐项显示结果；失败时保留原因，可导出 JSON 操作记录。自测只使用 /web_link_demo。
3. **Topic**：订阅 echo，再向 input 发布一次；应显示原样消息与计数。发送过程中按钮应禁用，连续点击或长按 Enter 不得排队；消息取点击时的 JSON，发送等待期间修改输入不得改变该次内容。取消后计数不增加。订阅 sensor，类型 Float64、QoS sensor，应持续变化。订阅 retained，类型 String、QoS latched，即使启动许久仍能收到 retained-once。
4. **Service**：AddTwoInts 的 3+4 应返回 7。将服务改为 slow、类型 Trigger、请求 {}，超时设 100ms，应提示 TIMEOUT；设 5000ms 应成功；等待中“停止等待”应显示 WAIT_ABORTED。修改不存在的接口或非法 JSON 必须报告错误。
5. **Action**：order=8 显示连续反馈和 status=4；order=20 后请求取消，等到 status=5；order=-1 显示拒绝/未知原因；order=13 返回 status=6。超时不得显示取消成功。
6. **参数**：列举、读取值与描述；gain type=3 写 0.75 后读回；写 -1 返回 successful=false；写 read_only 必须被节点拒绝。原子写入中混合有效与无效项时，全部不生效。空数组必须携带明确 type；字符串不能默默转数字。不安全整数应在发送前被拒绝。
7. **Lifecycle**：读取 unconfigured，选择 configure→inactive、activate→active、deactivate→inactive、cleanup→unconfigured，核对每步返回与可用转换；无效转换不应显示成功。
8. **标准数据**：订阅示例后图像每秒更新，TF/JointState/PointCloud2/diagnostics 都带原生字段与时间；停止后活动订阅数下降、图像资源释放。
9. **3D**：点击开始，状态应显示 URDF已加载、4关节（2活动）、5 TF帧、3 Marker、400点、20轨迹点；工具STL应可见，无资源错误。关节随示例改变；逐项关闭/打开5图层、切换demo_world、填写不存在坐标系应显示原因、恢复demo_base、复位视角。停止/重新开始至少3次，活动订阅回到基线，无多余canvas；拖动与缩放不发送ROS命令。
10. **图像**：使用彩色配置→显示，核对PNG的96×48；使用深度配置→显示，核对16UC1的96×48且depthScale=.001。把单位清空应显示明确错误；恢复后继续更新。停止后画布/Blob释放，反复切换无冲突。
11. **曲线**：sensor/data应持续更新；改错字段显示明确原因；最多600点，停止后样本订阅释放。JointState用position.0等字段；不要eval表达式。
12. **类型**：分别选择Topic/Service/Action，使用ticks/add_two_ints/fibonacci识别类型并查询字段；不存在接口返回空类型或明确错误，不能让rosapi退出。
13. **记录**：错误与成功均可查看，最新记录在前并显示浏览器本地时间；大图谱响应不能把最新结果挤出预览。导出 JSON 文件包含操作和 ISO 时间，最多保留最近 200 条完整记录。清空旧网页记录后保留这次clear-log的OK条目，不操作 ROS。

## 故障与长期行为

- 保持 Topic 订阅，停止并重新启动桥：页面进入重连、恢复数据；已有 Service/Action 返回断线未知，不能重新执行。用 demo 输入回显/服务调用次数或节点日志确认没有补发。
- 发布者晚于订阅者启动，sensor/latched 显式 QoS 仍需工作。用第二客户端测试不同 QoS 时注意 rosbridge 按主题共享 ROS 订阅的上游限制。
- 多次 connect/disconnect、订阅/取消订阅、成功/超时/取消后，SDK `stats().pending` 回到 0。页面关闭后释放本地订阅、定时器与图片 Blob URL。
- 绕过网页直接请求被禁止的 topic/service/action，确认服务端 glob 生效。上游某些拒绝仅写日志，客户端应超时而非伪成功。
- 普通低带宽示例运行 5 分钟检查计数、内存和错误；当前版本不把百万点云/高分辨率视频作为性能保证。

## 平板人工验收（本轮用户暂不测）

电脑 `address:=0.0.0.0`，Android Chrome 打开电脑局域网 IP。记录设备型号、Android/Chrome 版本、连接地址与时间；实际点击连接、Topic发布、服务、参数、Action取消、断线重连，检查横竖屏触控与结果。模拟窄屏只记“布局验证”，没有物理平板就明确记“实机待验收”。不嵌入完整Lichtblick界面；需额外手测3D旋转缩放、图层、彩色/深度与曲线。

## 收尾

停止 CLI 和浏览器探针，关闭自测页面，运行 launch 的终端 Ctrl+C。核对本轮启动的 `rosbridge_websocket`、`rosapi_node`、`serve.py`、`demo_node.py` 与 ros2 launch 已退出；只清理本轮 PID，不批量杀死用户其他 ROS 进程。端口 8088、9090 应释放；不得留下后台测试程序。

## 本轮功能范围与验收口径

核心通信、类型/参数/生命周期使用真实rosbridge与DDS示例探针；图像/点云/TF边界通过独立 observation_probe CLI核对。页面一键自测覆盖通信和标准示例消息到达；渲染、图层、交互、启动/停止的资源释放仍须按上文逐项检查，不能将一键绿色等同于所有RViz功能已完成。

新版统一入口还需验证：web.launch.py默认不启动demo；self_test.launch.py启demo；web=false时仅桥/rosapi；任何一个必需进程退出时其余子进程均退出。独立端口/域验收示例为 web_port=8188,ws_port=9190,ROS_DOMAIN_ID=89，勿与人工8088环境混用。

MCAP录放使用原生rosbag2与replay.launch.py，不在浏览器实现存储播放器。当前不包含MoveIt规划/关节轨迹回放、激光地图面板或生产生命周期管理器；业务包继续使用原生ROS接口。Android物理设备、生产高带宽负载和完整外部模型资产验收分别记录，未执行时不得记为通过。

## 2026-10-28 v0.2 本机核对

- npm严格类型检查、前端构建、colcon build完成；官方ament lint报告14项，0失败。
- 实际rosbridge/DDS SDK探针16项通过；观测边界31组与场景定位4组通过。
- 页面一键自测12项通过；实渲染显示4关节（2活动）、5 TF帧、3 Marker、400点与20路径点，STL资源HTTP 200。
- 手动点击核对5图层、缺失坐标系反馈/恢复、复位视角、彩色PNG/16UC1深度、深度单位缺失、曲线缺字段、Action字段查询；三次3D启停后活动订阅0、相关canvas0；图像与曲线停止也回0。
- 隔离桥退出后页面订阅重连恢复，图像和曲线重新更新；不补发指令。第一次Action类型查询复现rosapi退出，加入公开Jazzy graph API适配后16项回归通过。
- 390px与768px浏览器布局无横向溢出；Android物理平板未验收。完整外部DAE/GLTF资产、百万点云/生产视频未作为通过项。

核对日志和截图位于工作区 runs/ros2_web_link_v02_20261028/（运行证据，不作为源代码提交内容）。

## v0.3 项目验收台

按project-testing.md启动真实相机+mock，当前工程默认17项只读、3项可选交互。先只读核对，再显式选择交互；组合顺序须遵循SurveyScene→BeginScene，不得反转后把世代切换导致的失败当成Web传输失败。保留失败报告及原生message，不重试补绿。

CLI边界核对：`npm run audit`（取消、恢复失败、mock守卫、状态确认等）；已有`npm run observe`与`npm run probe`继续使用。`scripts/static_tf_probe.mjs`核对多客户端静态TF，参数说明在脚本头部；仅用自己的桥与DDS域。UI需核对项目切换、只读默认选择、清空/单项、停止/收尾、全部NOT_RUN不能标PASS、报告与实际原生响应一致、已有手动订阅被暂停后无QoS误报、交互期连接按钮禁用、离页保护、窄屏无溢出。


## v0.4 传输专项

新增scripts/run_transport_benchmark.py（项目契约、JSON/CBOR、资源守卫、清理）与roundtrip_probe.mjs（640×480和1280×720原生DDS大图双向哈希）。核心16项、audit20组、观测31组及场景4组继续有效。官方lint当前13项0失败；npm pack核对包含所有公有入口、声明与源码。

完整点云Web项目数据检查10/10。新增场景资源5组核对已纳入`npm run observe`；点云/Path/Marker资源复用后，稳定阶段CPU渲染提交p95约4.5ms。开始3D后展开“3D性能与资源证据”核对资产是否全部加载、CPU p95、shader创建数、几何体/纹理是否随更新无界增加；停止后SDK订阅应回基线。首载仍存在秒级长任务，不能记为流畅度完成。15秒CBOR慢读者下，独立服务查询与恢复通过；分片/长期行为和Android实机仍待验收。专项日志、失败与通过对照见performance.md。不要将本轮结果表述为通用框架的全部性能已完成。

当前官方lint四项命令均通过，colcon test-result汇总17项0失败；观测31组、场景定位4组、资源5组及audit20组通过。这些计数代表各自范围，不能相加当成完整业务验收。

## v0.5 有界传输与集成回归

`npm run wire`启动自己的临时WebSocket协议服务，36组检查覆盖：过大帧/UTF-8字节、分片乱序与重复/冲突、聚合字节/ID/片数/固定超时、主线程繁忙时的迟到片、CBOR类型数组/深度、发送积压、拒绝后的订阅/发布者回滚、注销积压、任意抛出值的消费/状态/反馈回调、连接状态回调重入、未知/迟到响应与内部路由碰撞。它不替代原生ROS/DDS回归；每组finally释放连接与服务器。可用WEB_LINK_WIRE_CASE子串筛选定位。

新增网页“传输预算与连接统计”：观察收发字节、分片缓存、发送积压与lastError；停止观察/断开后缓存归零。自定义profile transportLimits由SDK在连接前校验，不把配置错误当ROS端故障。对照执行`npm run probe`、`npm run roundtrip -- ws://host:port`和`npm run fault`，以及真实录制的大数据基准，确认限制没有破坏正常QoS、参数、Action终态与大图往返。

## v0.6 首载诊断与多客户端

开始3D前，在“配置观测话题”的SceneOptions JSON中加`"profileShaders":true`，展开“3D性能与资源证据”。核对bootstrap的模块/构造/start耗时，以及rendererInitMs、firstRenderMs、renderMaxMs、shaderCalls。默认不启用探针；测试后移除或设false并重新开始。它只测CPU调用，不是GPU计时。需要Three完整编译日志时设置`"shaderDiagnostics":"verbose"`；默认errors-only仍检查LINK_STATUS并显示失败。YAML profile可部署同样字段，非法类型会在启动时拒绝。

`npm run observe`现在包含31观测/4图像视图/4定位/9资源/4上下文丢失/7离散事件/12着色器/4负载进展边界组。资源探针确认ROS reset后保留仍在使用的着色器错误、绘制异常跨上报限频保留且失败不计首次完成；v0.6.1补充JointState缓存边界与URDF切换。v0.6.12检查绘制前/期间上下文丢失、恢复保留错误历史和dispose后不再提交。GL边界故障模拟不等于真实驱动故障注入。正常真实模型加载/绘制另查浏览器；firstRenderCompleteAtMs只代表首次CPU提交返回，不是GPU执行或屏幕呈现完成。

离散点击观测：开始3D前在SceneOptions JSON加`"profileInteractions":true`，展开性能证据的eventTiming。supported=false表示浏览器没有这项能力；无报告时耗时为null。阈值请求16ms、duration按规范量化为8ms，仅覆盖浏览器报告的页面级受信离散事件；不覆盖连续pointermove/wheel，不是INP。p95取最近最多120个已报告duration，最大值为本次观测累计值。停止3D即disconnect observer；再次开始新建一轮。测试完移除该选项或设false，不要将人为忙等按钮加入业务页。实际本机验证与局限见development.md I31。

性能CLI的`--multi-client-seconds 300 --client-count 3`使用3个SDK加慢读者，循环回放并重建客户端。查看multi-client.json的TF、时钟回退、p95、错误和每代清理，以及resources.json的PID/RSS/remainingPids。加强判据300秒已通过：每流10秒空窗、慢读者7次resume均有进度且未关闭；首次TF发现10秒、就绪后丢失6秒。启动期6秒门失败轮也保留。不是数小时或3个真实浏览器验收。完整命令、数据、限制与首载未通过结论见[performance.md](performance.md)。

## 服务超时的标准诊断

性能CLI加`--trace-services`，仅限multi-client模式。服务事件写入service-events.jsonl，原生DDS对照汇总service-trace.json，Web请求时间写入rpc-trace.jsonl。查询超过1秒时最多请求10组自有bridge/rosapi的Python线程栈，见stack-dumps.jsonl和launch.log；3秒RPC门不变，失败后多保留5秒用于取证，仍记FAIL。额外诊断会改变负载，修复后还需不加trace复测。

若METADATA模式改变了故障复现条件，可另用`--trace-waits`（与`--trace-services`互斥）。它只观察`/rosapi/nodes`：在上游Event.wait返回False之后、Future.cancel之前采样公开done/cancelled/is_set，再在原调用返回后写证据。不增加ROS实体或Future回调，不改等待/取消/重试策略。此模式不启用线程栈信号；不要向进程自行发SIGUSR1。默认关闭，两种CLI诊断均清除未选模式的继承环境。

两种诊断模式的负载探针失败后均保留5秒取证，再关闭订阅/连接并清理；RPC失败仍按原3秒门记录，不追加查询或改为成功。`node scripts/service_failure_grace_probe.mjs`核对实际失败处理分支，默认模式立即收尾、两种诊断保留取证时间且原异常不变。缺少这段时间时，SDK可能先于桥端等待到期而退出，空诊断文件不能排除桥端等待故障。

若快照已确认Future未完成，可在`--trace-waits`后追加`--trace-server`，用标准服务introspection继续观察服务端。只有rosapi进程为目标服务启用METADATA；客户端工厂不替换，不启用客户端事件发布。被动collector只订阅`/_service_event`，不创建对照客户端或发送查询。此组合会增加一个服务端事件发布者和一个观测节点/订阅者，仍会改变负载，不能称为无扰动测试。普通模式和单独wait模式保持原有边界。

检查service-trace.json的mode=passive-server、native.calls=0，以及phaseCounts中1（REQUEST_RECEIVED）/2（RESPONSE_SENT）；没有0/3不能推断请求丢失，因为该模式未开启客户端事件。服务端GID+sequence用于配对服务端阶段，不能直接映射桥CID；rpc-trace.jsonl提供会话/时间窗口，时间接近也不构成逐CID关联。RESPONSE_SENT不证明客户端收到响应。组合模式不发线程栈信号。

原生小样本验证可给`service_wait_trace_probe.py`追加`--server-metadata`：标准Trigger应产生两条接收、两条发送事件，没有客户端阶段或主动对照查询。`node scripts/service_rpc_trace_probe.mjs`验证request/response/error三个日志写失败阶段：原RPC结果和统计保留，诊断失败单独记录，整轮仍失败，不把写盘错误冒充RPC超时。

输出`service-wait-ready-PID.json`与`service-waits-PID.jsonl`。只适用已核对源码哈希的rosbridge2.7.1，专用诊断遇不同源码会拒启。每进程保留最近64条纯元数据，最多尝试写8条失败记录，每条最多128KiB；不保存请求/响应内容或原生对象。`waitState.futureDone`为True只表示Future成功或异常完成，不能直接说明是哪种；False表示采样时未完成。采样不是原子快照，比较sampleStartedAt/sampleFinishedAt与Event时间；Future取消也会触发Event.set，故setEnteredAt/setReturnedAt不能直接当作ROS响应到达时间。samplingError表示不分类。

显式原生诊断核对（独立未占用端口/DDS域，40秒及1GiB守卫，自动收尾）：

```bash
python3 src/ros2_web_link/scripts/service_wait_trace_probe.py \
  --out /absolute/fresh_wait_evidence --domain 147 --port 20290
```

它使用标准Trigger服务，先验证正常响应，再故意延迟以触发0.1秒桥等待失败；这是诊断正确性验证，不是实际长测故障的复现。`service_wait_boundaries_probe.py --case done|sampling|write|flush`不初始化ROS，使用真实Future/Event及上游调用函数，但受控替换实体与调度；核对已完成但通知未执行、异常采样、坏磁盘与二级日志、引用释放。`service_trace_modes_probe.py`在首个外部ROS查询前截断真实CLI路径，验证12种模式/继承环境组合，不创建节点。以上均为手动CLI，不加入ament行为测试。

`--rpc-period-ms 100`用于短轮高请求压力；默认2000。不要把高请求短轮通过替代默认周期长测，也不要自动重试参数、服务或Action。当前原始30分钟目标在638秒出现过服务超时，未通过；具体数据与解释限制见performance.md。

跨RMW复测按官方环境变量选择，在空DDS域中对整组子进程使用同一个RMW；不要重启或杀掉用户其他域的daemon。CLI预检使用--no-daemon。当前FastDDS的15分钟trace与Cyclone核心/短轮通过，原30分钟目标仍有一次638秒失败；不能把不同条件的绿灯混为一次完整长测。

## v0.6.1 模型缓存与30分钟回放

JointState的空position仍允许（仅速度/力矩数据），不清除已有位置；部分关节更新保留当前模型其他关节值。模型尚未加载或消息属于其他模型时，只保留最近1024个未匹配关节值，单名最长1024字符；更旧或更长的未匹配值不保留。URDF到达后应用匹配缓存，模型替换剔除旧关节名；断开/停止清空两个缓存。页面“3D性能与资源证据”的jointStateCache显示modelValues、unmappedValues与unmappedLimit。Peach正常回放应为6个活动关节位置，未匹配数量保持有界；先到消息、多个模型共用话题或切换描述时尤其要检查。

异常JSON的name必须为真正数组，position可为普通数组或CBOR数值TypedArray，拒绝伪造length的对象与DataView；错误显示在场景状态，不能冻结UI。资源探针验证该边界，真实浏览器另核对合法JointState的模型同步；只看探针不算实际URDF解析验收。

`cyclone-long-frozen/`是从独立安装的0.6.0本地tgz运行的1800秒完整通信轮：3个SDK、慢读者40次暂停/40次恢复、30次时钟回退、2682次RPC零失败，进程树峰值750.6MiB、退出无残留。该轮trace关闭、查询周期仍2000ms、超时仍3秒。它证明该条件下30分钟通过，不证明修复了FastDDS那次超时，也未执行本节新增的3D缓存路径；缓存另由新探针与实际浏览器验证。

## 原生服务客户端对照

当Web服务偶发超时，先用Jazzy公有客户端在同一隔离域查询只读`/rosapi/nodes`，区分原生服务通路与Web通路。统一守卫模式与UI/multi-client模式互斥，父进程一起管理播放器、桥、rosapi和原生探针的PID/RSS/期限/收尾：

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 src/ros2_web_link/scripts/run_transport_benchmark.py \
  --bag /absolute/recording --project peach_replay_large \
  --asset-packages ros2_web_link,aubo_description \
  --evidence /absolute/fresh_evidence --domain 132 \
  --native-churn-seconds 120 --rpc-period-ms 2000
```

`--rpc-period-ms`在此表示一组持久/新建客户端对照的周期，默认2000ms；改为100ms用于高频创建压力，不能与默认周期的结果直接等同。原生模式不建立WebSocket订阅，回放本身不会让桥接收全部图像/点云。直接CLI形式为`ROS_DOMAIN_ID=同域 python3 src/ros2_web_link/scripts/service_churn_probe.py --out /fresh/path --seconds 120 --period-ms 100`，需自行维护已启动服务的生命周期。它只允许固定只读rosapi查询，不用于执行业务服务。

输出`service-churn.jsonl`逐次记录模式、发现时间、响应时间和状态，`service-churn.json`保留全部样本的p95/次数、RMW、RSS和失败。服务发现最多1秒；请求提交和响应等待合用绝对3秒预算，迟到完成仍失败，绝不重试。每次交换两种模式的先后顺序；空结果、零样本、中断、超预算均不能PASS。未完成请求先移除pending再取消；最后销毁节点。JSONL以独占创建原子占位，汇总也拒绝覆盖；误用同一目录的并发实例在ROS初始化前拒绝。

纯时间边界探针（不创建ROS上下文）：`python3 src/ros2_web_link/scripts/service_churn_deadline_probe.py`。它验证3.1秒后完成不得PASS、发送本身消耗同一个预算以及超时pending清理。真实4秒延迟服务另验证3秒门失败、一次调用且无重试。当前FastDDS的两轮高频原生对照均通过，但没有复现既往638秒超时，不能据此删掉原失败或宣称DDS根因已解决；数值和失败轮见performance.md。

## v0.6.3 忙碌主线程与请求截止时间

`npm run wire`在v0.6.3时为42组协议边界。新增五组：主线程忙碌后迟到Service成功响应必须TIMEOUT；迟到Action反馈不投递、晚结果忽略；响应处理跨过截止时间不得成功完成；迟到的相关桥错误仍遵守本地截止时间；真实握手的open回调被延后时拒绝过期连接。另验证合法最大等待时间经截止时间浮点取整后不会溢出成1ms。请求用例使用真正发出的ID，经原生WebSocket MessageEvent进入实际SDK；模拟定时器尚未执行的事件顺序，不能以这类协议故障注入替代原生DDS或真实浏览器渲染验收。

可分别用`WEB_LINK_WIRE_CASE='expired service response' npm run wire`、`WEB_LINK_WIRE_CASE='response processing crossing' npm run wire`或`WEB_LINK_WIRE_CASE='expired connection open' npm run wire`定位。所有临时服务/客户端均由finally清理，不连接机器人。源码归档0.6.2作为不可变对照，五个用例均先失败；最大定时器用例也先复现溢出再修复。当前SDK全部42组、观测31/定位4/资源9/着色器5/负载进展4组与audit20组通过。真实节点与实际录制数据仍单独回归，指标范围见performance.md。

## v0.6.4 未完成请求上限

`npm run wire`包括51组：在42组基础上，覆盖Service/Action共用名额、默认256项突发、错误/超时/Abort/发送拒绝后复用、断线重连、无效配置、预算不可变、选项getter重入、错误Signal与注册/清理异常。可用`WEB_LINK_WIRE_CASE='admission' npm run wire`筛选部分边界。超限消息不得出现在临时服务器收到的帧中，所有临时客户端/服务器在finally中清理；该工具不连接ROS或硬件。

`node scripts/probe.mjs ws://127.0.0.1:9090`现含17项原生场景，新增同一客户端的slow服务+Fibonacci Action占满两项预算、后续请求未发送、显式取消后收到CANCELED、原服务成功、释放名额后可继续服务调用。需本包self_test.launch.py，无相机/机械臂。`npm run observe`另以实际离线RosClient核对逐资源释放断言，含配置元数据时不误报、任一非零资源仍失败。profile可配maxPendingRequests，范围/复合图谱查询占用见io.md。

0.6.4本机核对已完成：51组协议边界、17项原生SDK、双尺寸大图精确哈希、独立离线重建和19项官方lint通过。真实录制的完整云Reliable配置30秒通过，常规sensor配置出现彩色断流失败；须查看performance.md中的并列结果，不能笼统标记“所有项目配置通过”。

## v0.6.5 QoS配置与面板一致性

手工浏览器回归：在projects_config中准备Reliable图像用例与depth=7的显式QoS图像/Topic用例；分别点击“填入面板”，确认图像预设仍为Reliable、显式对象的reliability/durability/depth完整保留。通过公开按钮订阅时，由同一SDK使用该策略；停止图像/Topic后活动资源应归零。切换预设或项目后不积累旧自定义选项。该修复同时让省略选项的用例按audit默认sensor/none/0填入。

qos-controls下保留旧页面先失败、修复后通过的CUA表单证据。原生对照使用Jazzy官方ros2 topic echo --field header.stamp --csv及显式QoS参数，只有时间戳输出，不保留原始图像。测量在独立域执行、固定34秒并清理；Web对照则保持bag/点云/3客户端+慢读者一致，仅改变RGB-D QoS。

项目切换回归：先从depth7显式QoS用例填入Topic面板，再切到内置manual项目，应清空Topic名/类型、恢复sensor/none/0、旧自定义option数量为0；切回原项目重新按配置填入。real-ui保留旧版失败与修复后的DOM结果。

real-ui使用相同真实录制与现有Peach消息类型，已执行10/10只读用例；显式Reliable depth7深度图在实际页面显示640×480。ros2 topic info --verbose确认桥端订阅为RELIABLE，但FastDDS端点发现把History显示为UNKNOWN，因此不以该输出声称外部验证了depth7。页面停止/断开后活动订阅0、等待0/256，3D/图像/曲线canvas均0；300秒fixture正常到时清理全部子进程。

## 0.6.8响应时间线诊断

启用`--trace-waits`或`--trace-services`时，多客户端CLI额外保存`service-wire.jsonl`；追加`--trace-server`仍只由服务端标准METADATA记录ROS阶段。相同负载与3秒本地等待不变，不能将诊断通过当修复。默认模式不替换原生WebSocket。

WAIT正常退出生成`service-wait-final-PID.json`：最近64条已完成调用、CID、wall锚点、进入/等待/通知/返回单调时间，最大128KiB/一次IO尝试。获取锁最多50ms，snapshotAvailable=false表示采样不可用；这不是整个退出时长保证。文件IO由外层进程守卫兜底，强杀时文件可能没有生成。

原生WS元数据只含小型JSON服务头，send/sendReturn/receive保存CID及会话标签；receive是原生message事件执行时间，不是网卡接收时间。每轮最多24000条/8MiB，先缓冲，SDK关闭后flush，budgetExceeded或sink错误均保留FAIL。bufferedRecords/bufferedBytes应在最终报告归零。信号/系统中止且未执行finally时尚未flush的记录可能丢失。Python单调时间为秒，Node performance.now为各进程起点的毫秒，跨进程必须通过CID、wall锚点和各自时差解释，不直接相减。

以下独立探针无需ROS节点；Python仍需先source Jazzy以导入已安装IDL，32模式检查还需source已安装本包读取项目配置。Node探针会创建并关闭自己的本地回环socket，不是浏览器/Android测试：

```bash
python3 src/ros2_web_link/scripts/service_wait_final_probe.py --case success
python3 src/ros2_web_link/scripts/service_wait_final_probe.py --case write
python3 src/ros2_web_link/scripts/service_wait_final_probe.py --case flush
python3 src/ros2_web_link/scripts/service_wait_final_probe.py --case busy
node src/ros2_web_link/scripts/service_wire_trace_probe.mjs
WEB_LINK_TRACE_SLOW_PROBE=1 node src/ros2_web_link/scripts/service_wire_trace_probe.mjs
```

受控Future提前完成的检查不覆盖真实executor通知并发；native-smoke及loaded-smoke分别补真实服务与30秒完整录制负载。45个CID跨原生事件/桥退出快照精确匹配已核对，1800秒失败复现与关闭诊断的最终回归仍未通过。审查及红/绿、lint原始失败均保留在service-response-timeline，详见performance.md。

后续buffered-rpc候选同时缓冲旧rpc-trace；wire/rpc各自24000条/8MiB。full METADATA的stack旗标改为异步且最多一个在途写，收尾await后再判errors。`node src/ros2_web_link/scripts/service_rpc_trace_probe.mjs`和`node src/ros2_web_link/scripts/service_stack_trace_probe.mjs`直接检查实际CLI函数，不启动ROS或socket。新loaded-buffered-smoke单独保存，不与首个0.6.8归档混用。

## 0.6.9原生Client边界诊断

当桥Future未完成且服务端已有发送事件时，可在多客户端命令中同时加`--trace-waits --trace-server --trace-native`。native不依赖服务端模式，但必须有同端点WAIT。默认关闭，未选中的继承环境变量由父工具清除；直接启动桥时，`WEB_LINK_NATIVE_SERVICE`必须与显式`WEB_LINK_WAIT_SERVICE`相同。升级或其他平台不支持该诊断时应拒绝启用，不能直接替换哈希绕过审查。普通通信不依赖此诊断。

观察器仅在桥进程包装已审阅的rclpy7.1.12 pybind Client.send_request/take_response，核对本机二进制SHA256及Python call_async/Executor._take_client源码哈希。不创建ROS实体、不加Future回调、不解引用原生地址；pointer仅作为十六进制标识，与Python对象id组成clientKey。短命Client销毁后标识可能复用，必须按本进程时间窗关联最近一次send，不能全局仅按地址配对。请求CID来自已有WAIT工作线程局部上下文，executor取样线程没有CID，依Client标识与sequence关联。

正常退出一次生成`service-native-PID.json`，字段为sendEnter/sendReturn/takeEnter/takeReturn（或异常类型）、wall/单调时间、线程、Client标识、请求序号及响应头是否存在。无请求、响应、图像、异常消息或原生对象。最多24000条/8MiB事件元数据；运行时仅缓存，非阻塞获取观察锁，丢记录/采样错误/预算超限均显式计数。退出锁最多等50ms，文件IO本身依父守卫；信号终止未执行atexit时可能缺文件。该hook改变时序，不能称零扰动。

父工具在清理后读取快照，缺失、超限、丢记录或错误均使诊断验收失败，`resources.json`同时保留原通信failure与独立diagnosticFailure。`--evidence`必须指向尚不存在的新目录，父工具原子创建；既有目录一律拒绝，不删除旧证据。已取到对应sequence只能证明原生取样返回，未看到take也不能直接证明网卡丢包。

```bash
python3 src/ros2_web_link/scripts/service_native_trace_probe.py --case basic
python3 src/ros2_web_link/scripts/service_native_trace_probe.py --case budget
python3 src/ros2_web_link/scripts/service_native_trace_probe.py --case sink
python3 src/ros2_web_link/scripts/service_native_trace_probe.py --case guard
python3 src/ros2_web_link/scripts/service_trace_modes_probe.py
# 以下创建真实Jazzy服务；选空域/空端口及新目录，结束由父工具清理。
python3 src/ros2_web_link/scripts/service_wait_trace_probe.py \
  --out /absolute/new_native_evidence --domain 160 --port 21390 \
  --server-metadata --native-boundary
```

模式探针现在为80种模式/继承组合，另核对旧证据目录拒绝，不启动ROS进程。native四组检查用受控Client替代仅验证观察器；真实Trigger与完整录制负载分别验证真实绑定和端到端，不能相互替代。旧32组结果保持其原覆盖范围。证据见`runs/ros2_web_transport_20261029/service-native-boundary/`。

0.6.10成功takeReturn另有sourceTimestampNs/receivedTimestampNs十进制字符串。FastDDS下是写入/reader历史插入时刻；不是消息header.stamp、网卡时间或executor唤醒时间。空header为null，0/负值原样保留但无交付含义；时钟倒退/未对齐不能直接作延迟差。本机fixture要求FastDDS的正时间，命令前显式`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`；不将该fixture限制推广为其他RMW不支持产品通信。已有0.6.9记录缺这两个字段，不能补推。

## 完整点云与图像字节往返

在已启动的独立无硬件自测桥上执行`node scripts/roundtrip_probe.mjs ws://127.0.0.1:端口`（本包源码目录），仅两个`/web_link_demo/roundtrip_image`与`roundtrip_cloud`话题需发布/订阅许可。调用显式发布确定性合成数据，不能对业务/硬件话题改名后直接运行。每项5秒门、无重试，结束清理所有订阅/发布者/等待；程序外仍须限时/RSS守卫并停止自己启动的桥。

原两图为921600/2764800字节，CBOR接收；新增4136928字节PointCloud2分别JSON与CBOR接收，完整SHA256覆盖坐标、RGB和padding，布局及首/中/末显示采样点数值另核对。它是协议/原生DDS/解码的完整性检查，不是相机原帧哈希、GPU渲染、频率或长时容量检查。本轮父守卫与PASS/两次测试侧FAIL都保留在`runs/ros2_web_transport_20261029/payload-integrity/`。先完成构建再运行；构建会重建dist，不能与引用同一dist的探针并行。

## 超时后reader保留（0.6.11，默认关闭）

`--trace-waits --trace-server --trace-native --trace-late-client`可观察已超时调用的迟到回执。最后一项只依赖native+WAIT，服务端metadata可选。环境变量WEB_LINK_LATE_CLIENT_SERVICE必须与两者端点相同；未选模式会清继承变量。升级时额外核对原_destroy_client_async函数哈希，不直接改哈希绕过审阅。

原3秒SDK/桥超时仍判FAIL、Future仍取消、不发第二次请求；只在原清理处短暂保留reader2秒，最多3次，之后调用原executor调度器。标记阶段只存id，Timer期间临时保留Node/Client，正常请求立即清理。该模式改变超时后的生命周期，不能当作修复或原生命周期验收；成功结果也不能复活已经失败的Web请求。

退出service-late-client-PID.json只含CID/时间及最多3条记录，8KiB检查；缺证、预算、提交错误、退出取消或未提交均保留FAIL。cleanupSubmittedAt是提交时刻，不是实际销毁完成；仍需看资源/进程清理。真正修复后必须关闭此模式及其他诊断，按原负载和期限复测。

受控五组不建ROS实体：`python3 scripts/service_late_client_probe.py --case normal`，case另有constructor/budget/exit/sink。模式继承检查现为192组，旧80/32组结果仍仅代表原覆盖。真实fixture使用`service_wait_trace_probe.py --native-boundary --server-metadata --late-reader`，应同时看到网页timeoutRejected=true、失败CID的晚take及2秒后的清理提交，父工具限时/RSS并清进程。完整录制/正常调用和稀有超时另行验证。


## 可选 RPC / 大数据分连接对照（0.6.15）

仅在run_transport_benchmark.py的--multi-client-seconds模式加--separate-control。默认关闭；显式CLI把WEB_LINK_MULTI_SEPARATE_CONTROL写为0/1，父环境不会意外开启。每个逻辑客户端使用同一地址上的两个现有RosClient：data订阅TF/URDF/关节/图像/点云，control只查询/rosapi/nodes；3逻辑客户端加慢读者为7条WS连接。原3秒期限、数据量、TF及释放断言不变，报告增加separateControl、socketsPerHealthyClient及逐代controlCleanup/controlWire。它不提供桥事件循环或DDS隔离，也没有改变网页默认连接方式。

例如，在已source Jazzy和本包独立安装前缀后，执行性能CLI的完整原回放命令并增加`--multi-client-seconds 90 --separate-control`；不要与--ui-seconds或--native-churn-seconds混用。容量/XML覆盖仍需分别显式选择，不由该选项暗中改变。完整长测仍沿用1800秒判据，短测不能关闭历史问题。


## 实际录制数据全字节和画布核对（本工作区证据）

image-copy/recorded-frames-typed保存实际color/depth/cloud原始数据与严格IDL类型清单，三帧独立、非同步。运行该目录上级recorded_payload_probe.mjs且省略WS参数，仅离线核对全payload和独立像素/几何参照；给出显式WS地址则只在固定/web_link_demo/roundtrip_image、roundtrip_cloud自测话题发布，原生DDS后分别JSON/CBOR接收。用新WEB_LINK_EVIDENCE输出文件，已有文件会拒绝覆盖。必须使用无硬件self_test桥，不能把测试发布改成机器人命令。测试nonce替换frame_id，只证明数据/布局/显示，原header另存。

需要重跑本机实际浏览器验证：source Jazzy和本包安装前缀后，运行`python3 runs/ros2_web_transport_20261029/image-copy/run_native_canvas.py --evidence /tmp/ros2-web-native-canvas-新编号`。它拒绝已占用域183和21988/23590/22688端口，使用新证据目录，先运行6次原生往返，再提供22688页面；点击连接、彩色、深度，核对PASS、原始与画布SHA、allocations=1和sameArray=true，最后点击停止检查资源/Canvas全0。网页等待阶段最多360秒（另有启动及原生探针阶段），或新证据目录stop.request提前停止，自动清理自有进程；结束仍复核进程与端口。这是工作区具体证据脚本，不是npm包运行依赖，也不代表Android或持续性能验收。

## 0.6.17通用项目/页面分连接核对

`npm run audit`包含原20组及新增9组控制/观测分工：Topic和TF使用观测侧、Service/mockGuard使用控制侧、观测断线释放、原生Action取消确认、参数恢复、跨URL预检拒绝及默认共用。`npm run preview`包含原14组摘要及新增8组实际HTML/connection handler核对：元素ID、默认/双连接、部分失败释放、双侧断开、旧代回调、配置迟到取消和双侧握手期间新发送禁用。均为显式CLI边界检查，不是浏览器/DDS性能测试，也未向test目录添加行为测试。

手工页面按顺序核对：默认模式连接并运行6项只读；断开、勾选分离观测连接，再连接确认两侧就绪及模式锁定；标准服务3+4得到7，项目报告为separate。完整demo回归仍是页面中的12项，不能误写为17项。分别启停3D、图像、曲线并检查订阅/等待/画布归零；只选Action后运行再停止，应确认原生终态5。双侧首次握手未完成时不得发送新Service/Action/发布，取消/停止等待/断开须可用；失败后可以重连。完成后关闭临时标签、停止自建launch，按PID身份与端口清理。

本机记录位于ui-channels：首轮demo-ui为页面初始化FAIL，demo-ui-fixed为完整功能复测，demo-ui-handshake-fixed为握手修复后的最终正常流程。受控握手失败红绿与实际浏览器正常握手分别记录，不互相替代。彩图停顿I36仍开放；这里不改变原长测的输入、超时或通过判据。

## 0.6.18显示时钟与真实配置加载

先npm run check/build，再npm run observe；其中scene_time_probe.mjs有10组实际Scene/TF回调检查，包含系统/ROS模式、首帧零、暂停、到期释放、回退与大秒值纳秒精度、TTL更新及清理。无WebGL或ROS实体，不能代替构造器/浏览器集成。source Jazzy后运行python3 scripts/config_probe.py，调用真实load_config与仓库YAML，3组核对默认demo/robot、项目时钟定义和布尔值拒绝；不启动HTTP服务器。

实际页面应显示系统时间或ROS时间；回放SceneOptions须useSimTime:true，独立TfBuffer须显式clockTopic。暂停自己拥有的回放后启动新Scene，首次Clock前应显示等待/clock并使用ROS零；恢复后按记录时钟推进。可在隔离域向专用自测Marker话题发布有限寿命对象，核对初始、暂停、恢复及删除/释放。切换模式要重建组件，断开重连后重获TF/数据；不得向真实机器人命令话题发布，也不得用旧0.6.17长测替代本版本浏览器结果。

## 0.6.19：显式断开与图像预算回归

`node scripts/ui_connections_probe.mjs`当前12组执行实际页面连接/stopOptional/closeSubscriptions、统计更新与挂起Scene启动续体，验证双连接、本地资源只收尾一次、Blob撤销、旧异步模块不在新会话创建视图、不调用远端Goal取消，以及没有定时器tick时直接断开仍立即显示0订阅。它使用CLI依赖替身，不直接执行GPU/RAF或原生ROS。

`node scripts/image_budget_probe.mjs`为13组受控边界：PNG/JPEG超像素、compressedDepth/无效头/codec不符、空format JPEG、合法PNG子视图、非法像素/字节预算、构造前拒绝、非法data类型、跨realm Uint8/伪造类型和非目标ftyp不进入通用解析器。小型头夹具不创建真实超大图，也不验证浏览器像素解码；该脚本已纳入`npm run observe`。

0.6.20当前为15组，另加未知/原型编码拒绝与合法RGB正对照、数值数组uint8范围/类型/稀疏（含继承数字索引）及合法边界对照。它们针对公开解码API，ROS序列化自身会拒绝部分非法uint8输入，不能假装这些值已通过实际DDS传输；0.6.20原生Peach10项、单像素constructor拒绝与rgb8恢复、640×480RGB-D和直接断开0资源已单独核对；冻结范围另记。

真实页面需启动独立普通安装，打开模型/TF/点云/图像/曲线后直接点击断开，不预先单击各组件停止；核对活动订阅即时0、等待0、容器及图片src清理，再重连核对视图可恢复。共享与分离模式分别验证。新demo用真实4×3 JPEG验证空format回退，用96×48 PNG验证正常显示，5000×5000的33B头和compressedDepth只验证预解码拒绝；Peach完整回放验证640×480原始彩色/深度、明确0.00025m单位、12/12模型资源、21TF及直接断开。证据见image-headers/demo-native、peach-ui及metrics-fixed/peach-ui，保留首次计数延迟PARTIAL。所有fixture/端口/临时标签已清理，RSS不含浏览器，未据此宣称GPU内存或冷首屏卡顿已解决。

实际页面步骤：连接并启动Topic、标准数据、3D、图像和曲线；直接点“断开”（不先各自停止），核对画布/图像/曲线/Blob与活动订阅归零、状态停止，观察到页面动画循环停止。再连接时按需重新开始观察。另按既有故障步骤测试意外断线后的订阅恢复，确认两种意图都正确；无硬件示例的在途Action断开不得自动触发取消。

0.6.21上下文恢复专项：`npm run observe`的shader12组区分真false、null不可验证、丢失前/查询中/日志中丢失，保留恢复前后真实错误，确认只读hook恢复失败不阻止其余收尾。Scene的4组调用实际帧路径和contextChanged，检查两事件清pending/设dirty、dispose晚事件无效；构造器实际注册由真实浏览器静态网格验证。使用新origin避免缓存，固定根尺寸，先核对成功帧稳定，再用公开WEBGL_lose_context扩展丢失/恢复，期间不能拖动、重置视角、改变尺寸或直接设dirty；恢复后须自动增帧，停止后画布/订阅/监听0。原生脚本/红绿证据见development.md I48。本项不证明整窗闪退或长任务修复，也不等于完整Peach context故障注入。


## 0.6.21默认共享连接持续流复测

先source待测普通安装前缀，核对没有自定义RMW/DDS XML覆盖、工作目录没有自动加载的默认XML，启动前确认空域/空端口。以下复用原性能CLI，不传队列、分连接或诊断覆盖；新证据目录拒绝覆盖：

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws/src/ros2_web_link
python3 scripts/run_transport_benchmark.py \
  --bag /home/mu/Desktop/aubo_e5_jazzy_ws/runs/ros2_web_live_camera_mock_20261028/capture_before_camera_off/peach_rgbd_mock_60s \
  --evidence /tmp/ros2-web-default-shared-新编号 \
  --project peach_replay_large --domain 201 \
  --web-port 24088 --ws-port 25490 \
  --multi-client-seconds 1800 --rpc-period-ms 2000
```

本机I54在381.95秒失败，567RPC中3次超时；成功p95<25ms不能判通过。查看multi-client.json的失败与每代rpcFailed、resources.json实际时长/内存/remainingPids，不把seconds=1800目标当实际完成。第9次慢读恢复和450秒首次重建未达到，不能给它们通过。结束必须核对所有自有PID创建身份与两端口已清理。

需要细分该故障时，可在新的独立诊断运行显式追加已有`--trace-waits --trace-server --trace-native`，按前述版本/预算限制采元数据；这会增加诊断负载，不能当作默认无观察器验收。保持原3秒截止与数据，不用增加超时、默认XML或重试覆盖失败。精确结果与边界见development.md I54及performance.md。

I56的Client复用是工作区临时副本诊断，不是本包可用的产品launch开关。13组CPU→隔离真实AddTwoInts短测→相同Peach录制1800秒顺序执行，必须分别说明范围；长测没有取消事件，取消/迟到应仍引用短测。正式负载只变Node拥有的目标Client寿命，保持原call_service、timeout:3、q64/shared/默认RMW/无XML，实际入口/环境/YAML/loaded library和全部输入hash均需核对。不要直接把固定/rosapi/nodes helper用于多服务产品。实验源码、预声明门和有限通过证据见development.md I56及其plan/ledger，原始FAIL和中间修订不可覆盖。

离线核对先等同一正式会话退出，再查PID创建身份/端口，最后调用本轮postprocess；它先逐项分析，再汇总所有门，status与transportStatus分开，缺失/截断会另写FAIL/PARTIAL。JSONL真实行数/Counter集合、DDS正时间字段、原生错误、SDK每代计数都要核对，协议result=true不代表本地SDK在截止前成功。完整1800依赖冻结Node单调循环完成，半秒RSS采样最后时刻只作下界。37门通过也仅是实验范围；本轮1007.10MiB子树峰离守卫16.90MiB，通用生命周期、无诊断产品回归和长期内存仍需后续验收。

## 测试守卫停止与中断（0.6.22）

手测使用`--ui-seconds`时可在本轮新证据目录创建`stop.request`正常结束；终端Ctrl+C或仅向本轮benchmark守卫PID发SIGTERM则记录中断并非零退出。不要向其他操作员进程发信号，不把终止测试守卫等同于机械臂安全停止。所有路径都要等守卫终态，再读resources.json的failure、terminationSignal、cleanupErrors、remainingPids，按processInventory的PID创建时间与监听端口复核。中断是失败/取消证据，不能记性能PASS；guardReturncode非零本身也不能证明清理成功。

预检`ros2 node list`同样在信号请求保护内，沿用10秒子进程上限；信号不会在任意Python字节码处抛出异常打断Popen登记。清理先停自有进程组，再收可选诊断，错误隔离后原子提交resources.json；提交前是完成边界，晚到信号不开启新操作。SIGKILL/OOM/断电无法由Pythonfinally保证，报告缺失必须保留FAIL_EVIDENCE并核对/清理确切自有组。本机自动测试另套独占systemd临时service的RuntimeMaxSec/MemoryMax/MemorySwapMax/KillMode=control-group，明确source安装环境，并核内核cgroup实际限额与后代归属；不要仅凭systemd-run的一行MemoryPeak推内存峰。

可复查的真实回放受控信号夹具、6组同SHA回归、detached后代red/green、4组实际finally AST故障注入及范围见development.md I59。AST探针没有原生ROS实体，不能代替真实回放；可处理信号通过也不能解释I58未知杀父信号。独立浏览器轨迹须检查dataLossOccurred和阶段双mark，先核实际GPU/浏览器/视口/数据流；有损或缺守卫终态只作PARTIAL，不将旧CPU采样权重报告当耗时证据。

## 暂存Service池的开发验证（I60）

I60历史快照：当时冻结安装0.6.22没有缓存开关，候选位于runs/ros2_web_transport_20261030/service-client-design/candidate；当前开发树已在I61–I62接入，尚未冻结新版本。不要把候选短测当作任意安装Web的验收。先读该目录native-plan.md的环境、输入快照、已知失败与新所有权契约；完整记录在development.md I60。

CPU检查必须先source Jazzy，按cpu-factory-matrix.json的script/case执行；所测输入在cpu-factory-inputs，格式后的候选与该输入AST等价。不要把每次导入pool_probe重复运行的7基础组累加为新用例。旧invalid-import输出不是有效反例。

原生复跑只用空闲ROS域中的自建数学服务。run_native_pool.py要求全新--out、--domain、--mode single或multi、--case all；推荐套与证据同样的独占临时systemd用户service（105秒/MemoryMax512M/Swap0/KillMode control-group），父守卫另设90秒/RSS512MiB/free4096。它会保存实际输入源码快照、每组结果、native primitive序号和resources；不启动真机/相机/IO/Gazebo/Web。先核主机自有残留与域占用，结束必须核PID创建身份/线程join/Node.clients及临时unit清除，不把shutdown返回值单独当排空。

使用约束：一个Node只对应一个池和一个稳定executor；先close池、实际停止并join executor，再Node.destroy。factory异常可能已注册实体，隔离slot后需清理旧Node再重建，不能在旧Node上另建池绕容量。未知布局/EventsExecutor支持还未完成，接入正式包前由Task2另行验证；不要直接复制候选绕开该门。

## 受管Service与诊断的当前使用（I61开发树）

先source Jazzy和本轮构建的工作区，使用Node22+（systemd用户单元的PATH可能仍指Node18，需显式选择）。self_test.launch.py启动无硬件示例；web.launch.py为可组合入口。0.8.3默认缓存0项；显式service_client_cache_size:=64才进入已审缓存路径；三个缓存参数均只读，须停止旧节点再改部署参数。普通ROS Node仍走原函数，不被受管池接管。

```bash
ros2 launch ros2_web_link self_test.launch.py service_client_cache_size:=64
# 另一个已source的终端，显式连接本轮端口
node src/ros2_web_link/scripts/probe.mjs ws://127.0.0.1:9090
node src/ros2_web_link/scripts/roundtrip_probe.mjs ws://127.0.0.1:9090
```

前者当前17项，涵盖Topic/Service/参数/Action/Lifecycle/权限/断开清理；后者真实ROS逐字节往返两图及4.14MB云。数据发布只在/web_link_demo下，不用于真机命令或IO。结束Ctrl+C并按实际PID身份检查本轮进程；不要用宽泛pkill停止其他操作者程序。

独立数学服务诊断会自行启动/停止自己的ROS组；每轮换用从未存在的out目录、空闲domain与端口。下面四阶段METADATA+WAIT/native走缓存；late-reader显式原始对照：

```bash
python3 src/ros2_web_link/scripts/service_wait_trace_probe.py \
  --out /tmp/web-wait-cache64-new --domain 230 --port 20290 \
  --server-metadata --client-metadata --native-boundary --cache-size 64
python3 src/ros2_web_link/scripts/service_wait_trace_probe.py \
  --out /tmp/web-wait-cache0-new --domain 231 --port 20290 \
  --native-boundary --late-reader --cache-size 0
```

第一次的server-only与bridge客户端metadata处于独立进程；单进程仍拒绝混合两模式。检查guard-result.json、各phase的service-events.jsonl、service-waits中的responseCaller/waitState与native CID/sequence，不只看端口或ready文件。该短夹具超时0.1秒，不是正式网页3秒门。缓存开时late-reader在监听前拒绝；未知/装饰的运行方法必须匹配实际code，否则明确拒绝，不能删指纹或绕过版门换PASS。

性能CLI新增--service-client-cache-size 0..1024；普通默认不传则使用0.8.3的launch0，--trace-late-client自动0，显式非0拒绝。I62已经实际核对Peach录制回放的64/0/默认64、q64与JSON/CBOR完整负载，并通过正式3秒wire和关闭边界；I62当时版本冻结及1800秒仍待；后续0.7.0正式完成见I63，0.7.1另列I65。当前短测不代表WebGL/Android/长测已通过，详见development.md I61–I62。

## 3秒与退出复核（I62）

独立夹具位于runs/ros2_web_transport_20261030/managed-lifecycle，输入源码及普通安装路径记录在每轮inputs/input-snapshots中。复跑须换新out、空闲域/端口，并显式改为本次待测安装；这些是工作区证据脚本，不是本包依赖。deadline用真实server4秒响应与标准timeout:3，核同CID的false/原超时文字、随后fast成功及远端继续；8秒观察期限不替代3秒请求门。infinite在远端执行时timeout:0后SIGINT；queued在真实工厂排队而未创建Client时SIGINT，检查call返回/无复活/Node.clients0，不仅看stats。

退出复核必须等守卫终态，再检查launch.log无escalation、bridge clean0、resources无失败/残留，并按PID创建时间和端口复核。发布者测试使用真实advertise/publish/subscribe后直接断开，保留上游默认10秒Timer，新launch可等待约10秒，15秒后才升级TERM。不能因之前5秒launch升级TERM后最终清干净，就记为graceful；I61两轮旧声明已撤回。

读取实际部署参数时使用`ros2 param get --no-daemon --spin-time 1 /rosbridge_websocket service_client_cache_size`，避免为测试生成后台daemon；同法核write_queue_size。源码CLI仍按本章从scripts执行，ROS入口/资产来自source过的普通安装。三轮短测每种编码约10秒，6种信号夹具是q16/UI模式；都不能替代默认q64/shared/noXML/3秒/1800秒容量门。

## 当前随包浏览器CLI复测（I66）

先在独立空域启动self_test.launch.py，保持运行；另一个终端source相同环境，在源码或恢复lock的解包目录执行`WEB_LINK_EVIDENCE=/absolute/fresh_ui_output npm run browser -- http://127.0.0.1:8088`。可用CHROME_PATH指定已安装Chrome；脚本默认headless，自己关闭浏览器，外部launch仍由启动者Ctrl+C并核对PID/端口。它仅应连接无硬件demo，运行21组（含多轮12项demo按钮），不要向在线真机域运行写参数/Action示例。

探针通过真实summary按钮展开折叠的demo回归区，导航/窄屏/普通LAN别名后同样处理，不通过强制点击隐藏元素。项目报告使用下载JSON核对完整6只读/2未执行，取消报告CANCELLED1/NOT_RUN7且terminal5与cancelConfirmed=true；有界DOM摘要不冒称完整下载。manual项目自带1项图谱用例，清空选择才禁用运行；日志清空保留其自身OK条目。实际失败/修正/截图与输出见I66。
## 0.8.2 回放预览与交互核对

本轮真实录制入口、手动检查及退出步骤见[回放预览](replay-preview.md)，最终数据口径见[性能记录](performance.md)。`workbench_replay_probe.mjs` 已适配全局连接抽屉、三个页签和折叠控件，新增真实旋转/缩放、属性、图谱筛选/布局/缩放保持、IDL、单云开关、导航、图像滚动、暂停恢复和直接断开。使用真实控件，不通过内存直接更改业务状态。

最终 `final-02`：120秒满负载、2次回退、8类交互、无页面错误，所有流计数持续推进；暂停/断开后订阅和GL为0，原生runner自动清理所有自有进程。`layout-final`10组通过（含1280/768/390宽度，仍不是物理平板）。`graph-viewport-red3`真实复现缩放0.835669被页签切换重置为0.727838；`graph-viewport-green`验证保持，最终回放再次验证1.25倍在页签之间保持。`npm run check/graph/workbench`通过；`colcon test --packages-select ros2_web_link`及test-result为37检查、0错误、0失败、0跳过。

失败不覆盖：`baseline-01`是脚本匹配到两个summary，修正后`baseline-02`通过；`graph-viewport-red`未验证滚轮前置条件，不能作为保持视角的有效证据；`red2`因上游初始滚轮限幅未达5%前置要求，`red3`用三次真实滚轮才构成有效产品复现；workbench首次离线核对的假控件少了graph-layout，补齐fixture后通过。全部记录保留于`runs/ros2_web_replay_ui_20261102/`。

