# 大型数据传输与性能验收

0.8.3主工作区安装、无缓存共享服务预算和工作台订阅选项修复，以及两轮600秒无界面NVIDIA图形压测的完整参数与结果见[NX 16GB 本机浏览器压力测试](jetson-orin-nx-performance.md)。原0.8.2失败及本轮串行编码检查失败分别保留；目标机ARM64、统一内存与功耗模式尚未实测。下文为各版本历史记录，历史PASS不能替代当前专项结论。

0.8.0新工作台：native-ui-06桌面8组通过，树sumRSS峰1920.90MiB；native-peach-03完成120秒共享连接/4路云/8空间/11总显示，2次录制回退，6活动关节、12/12资产、21 TF、RGB-D/曲线，峰2175.23MiB。11个10秒采样窗中点数0..345248（0发生于设计要求的回退清空），JS堆采样峰116.68MiB，未报告渲染/着色器错误。单WebGL上下文通过真实getContext对象与最终isContextLost核对。

录制首云偏移3.319s，循环云间隔3.338s；测试以该离线时序+1秒余量核回退恢复，不把空窗当持续有云，也不变更10秒推进门或3秒RPC门。前两轮因测试错误要求零空窗/1500ms恢复而FAIL，原始数据与更正见testing.md和recorded-gaps.json。模型加载数据来自displays[].assets，共享renderer统计剔除空宿主的模型指标。

该轮是新版显示容量核对，额外SDK回调只计数和stamp，不新增网络订阅；不做GPU耗时或全部设备/FPS保证，也不替代下文0.7.1的1800秒通信负载。Android实机与历史内置浏览器问题保留原边界。


独立包通过标准rosbridge为其他Web工程提供Topic/Service/Action/参数/类型接口，显示组件按需复用。Peach、IVG是需求样本；本轮未迁移业务，不启动真机或Gazebo。以下结果仅对应记录的环境、数据和测试时长，不是任意终端/带宽/点数的性能保证。

## 当前证据与使用边界

0.7.1源码归档/离线重建及默认q64、cache64、shared、FastDDS、无XML/诊断的1800秒完成，2691RPC零失败，3个健康SDK+慢读者、两次客户端重建、30次clock回退、40次慢读暂停/恢复均覆盖，五代RPC成功p95约6.39–7.68ms。内层树sumRSS峰910.54MiB、Node自采168.16MiB；独立cgroup采样peak约1025.18MiB且存活观察时OOM0，口径不同，仍有内存压力及227条无CID/限频队列警告，不能称有宽裕容量/逐帧无损。30–90秒树中位621.56MiB、末60秒806.31MiB；正增长不是泄漏诊断，也不证明数小时容量。精确配置、时长、SHA与清理见[本版完整后处理](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/long-verification.json)。0.7.0先前2688RPC0失败的独立结果仍保留。最终开发CLI/文档补充不改本轮生产代码/54dist，绑定见I66。

独立Chrome152/RTX3090完成Peach录制10项只读、12/12模型资源、21TF、约8.6万显示点、RGB-D/曲线和真实输入。32MiB有界CDP三段均无已知丢失，首观测构造约186.6ms；关闭Tracing/CPUProfiler后另轮构造83ms、稳定CPU提交p95约1.6ms、交互1.5ms，稳定/交互阶段未报告>=50ms LongTask。该计时不是GPU/显示帧延迟，不能做精确profiler开销比值；重启仍有228ms长任务，窄屏调整55–60ms，冷启动不是零阻塞。原ProfileChunk有负timeDelta，仅按采样次数解读，不换算CPU占比。浏览器树峰1742.14MiB（无profiler）/1897.98MiB（捕获），不含独立显存，不与1GiB无浏览器通信门混算。

989×900、hasTouch桌面模拟从主机自身WiFi地址192.168.2.90访问，WS主机自动跟随、10项只读/3D/深度/断开通过；不证明远程设备防火墙/Android GPU。用户要求本轮暂不测平板实机。独立Chrome结果不能关闭内置浏览器历史秒级GL阻塞，整窗OOM仍未恢复Python脚本身份，I36另一配置彩图停顿机制仍未确证。具体原始轨迹、截图、失败、观察器限制和清理见development.md I64。

0.7.1真实ROS＋Chrome新增10组网页表单（含页面12项demo回归）通过，超限PNG头在Blob前拒绝、compressedDepth拒绝、合法JPEG/PNG恢复、加载中停止和多视图独立启停通过。3GiB/200秒功能守卫下树sumRSS峰1955.84MiB，总13.613秒且已清理；不是1GiB通信/长期性能证明，见I65。以下旧版本轮次原样保留其条件与结论。

## 闪退结论

内核确认2026-10-28 18:09:33发生global OOM，强制终止Python PID553423；anon-rss=25751756KiB，约24.6GiB。证据为`runs/ros2_web_transport_20261028/crash-evidence/kernel-oom.txt`。退出后现有日志不足以恢复其脚本/父进程，不能直接归因rosbridge、相机或网页，也不能宣称已修复该系统根因。两份录制全部5个SHA256校验项再次通过。

后续每轮使用独立DDS域、有期限的进程树RSS守卫和finally清理；resources.json保存PID、父PID、创建时间、命令和采样。这是测试保护，不是产品进程的操作系统级内存隔离，也不限制外部浏览器RSS。

2026-10-29复核：用户明确是整个Codex/浏览器窗口关闭。当前桌面主进程启动于2026-10-28 18:09:42，与上述OOM相隔9秒；旧桌面日志止于18:09:28。这支持该时段窗口退出与系统内存压力有关，但内核直接终止的是Python，尚无证据恢复其脚本身份或证明桌面进程被直接OOM终止。03:20与08:47复查均未发现新增内核OOM/段错误/GPU重置或桌面崩溃报告，08:47可用内存约24GiB；未给出的其他退出时刻不能据此判断。检查证据见`runs/ros2_web_transport_20261029/crash-check-0320/`。

12:15再次只读检查宿主机：同一桌面主进程PID561018仍在，没有新增内核OOM/段错误/GPU重置记录，可用内存约20GiB。12:16按PID与创建时间复核，本轮负载进程全部退出，8088/9090及本轮20288/21290均无监听。详见[复查原始记录](../../../runs/ros2_web_transport_20261029/crash-check-0320/recheck-20261029-1215.json)。用户尚未给出另一次窗口退出的确切时间，不能据此排除纯应用崩溃，也不能把已结束的测试或连接拒绝当成窗口退出原因。

14:38复查仍只有历史18:09:33的OOM，当前可用内存约18.6GiB；14:39桌面主进程PID/启动时间未变。见[本次内核复查](../../../runs/ros2_web_transport_20261029/crash-check-0320/recheck-20261029-1440.json)。本轮负载已因RPC期限失败后正常清理，不能据此声称历史整窗口退出已修复。

19:04补查用户journal取得直接应用组记录：18:09:33旧Codex所在`app-org.chromium.Chromium-11333.scope`有进程被OOM杀死，18:09:38 scope以`oom-kill`失败，chatgpt.desktop的GPU进程同时报告context lost，18:09:42.330新主进程启动。这支持系统内存耗尽关联整窗关闭；Python具体脚本和主进程直接终止信号仍未知，不宣称系统根因已修复。原始匹配见[精确时间窗记录](../../../runs/ros2_web_transport_20261029/crash-check-0320/recheck-20261029-1904.json)。

21:26重新核对同一历史窗口，内核还确认16GiB swap已用尽，当前桌面主进程身份仍未变；两份录制全部5个校验项通过，相关测试结束后PID/端口均清理。精简原始日志见[本次整窗退出复核](../../../runs/ros2_web_transport_20261029/crash-check-0320/recheck-20261029-2122.json)。这仍不能恢复被杀Python的脚本身份，也不将任何未给出时间的新退出自动归到该事件。

## 数据与环境

Ubuntu24.04/Jazzy，本机RTX3090/ANGLE桌面浏览器。相机已关闭，播放已录制Peach实际消息。`capture_before_camera_off/peach_rgbd_mock_60s`时长约59.47秒，包含640×480 BGR8/16UC1、21帧TF、URDF/6活动关节、Marker/Path及完整注册云；云约172372点、point_step=24、4136928字节。显示按maxPoints=100000采样，实际约8.6万点。16UC1深度单位显式为0.00025m。录制有丢帧记录，不是无损标定数据。

本机大流组合采用项目`peach_replay_large`显式Reliable；常规sensor默认未改变。源时间保留录制值，不能用回放断言当前设备在线。/clock回退清动态TF和曲线历史，静态TF保留。

## 已有验收与修复

所有原始证据位于`runs/ros2_web_transport_20261028/`，失败轮保留。

- `bounded-core/`：原生SDK16项、921600/2764800字节图像JSON发布→DDS→CBOR回传SHA256及桥退出/重连通过，进程组合峰值575.3MiB，退出无残留。
- `bounded-data/`：12秒JSON与12秒CBOR实际完整负载，云14/30帧、深度15/24帧、彩图各10帧，TF/URDF/6关节均通过。服务p95约339/8.7ms，Node峰值188/154MiB，组合733.3MiB。采样区间不同，不将帧数差异直接当成编码压缩率。
- `bounded-native-fragments/`及`...-diagnosis/`：保留上游rosbridge2.7.1外层fragment id重复造成7/46片混流的失败。`bounded-native-fragments-fixed/`在自有桥安装版本限定UUID兼容后，15秒接收1063片，云17/深度21/彩图13帧解码与完整TF通过，服务p95约112ms，组合658.8MiB。内层RPC/Action ID不改。
- `bounded-ui/rpc-fragments.log`：3条真实rosapi响应强制512字节分片，共16片，ID和重组正确，退出缓存归零。
- `bounded-ui/`：实际项目10项只读PASS，模型12项资产、21TF、33Marker、约8.6万点、800路径点、640×480深度及曲线显示。停止/断开后订阅、等待、画布均0。240秒启动树峰值472.3MiB，退出无残留；该RSS不含浏览器。
- SDK传输使用roslib公开Transport与cbor2，补完整帧/分片字节、ID/片数/固定期限、发送积压预算和安全路由；36组wire故障探针覆盖异常和清理。原生证据与协议故障注入分别核对，不能互相替代。

## 五分钟多客户端负载

`sustained-three-plus-slow/`为300秒、3个独立SDK会话加1个慢读WebSocket，循环实际完整录制数据。每个SDK同时解码彩色/深度/云、接收URDF/JointState/TF，并每约2秒查询rosapi。25%和60%时分别销毁并重建客户端，重新获得完整TF；慢读者每45秒中的约15秒暂停，累计7次。

原始轮次结果PASS（尚未加入逐流停滞/恢复进度判据），检测5次/clock回退，慢读者恢复后累计2379帧、约3.15GB载荷，5代会话查询p95约17–35ms，无服务超时、解码/连接错误。Node自采峰值RSS176.4MiB、堆21.1MiB；守卫采样Node峰值177.9MiB、桥286.7MiB，启动进程组合峰值748.3MiB。不同采样器峰值不会完全一致。30–90秒组合RSS中位数638.2MiB，末60秒671.3MiB；5分钟未触及预算，不能据此声称长期无泄漏。每代订阅/发布者/等待/监听器均归零，resources.json的remainingPids=[]。

原始轮次浏览器直到临近结束才连接，未在300秒负载的完整时段运行渲染。审查补充了逐流10秒空窗、慢读者提前关闭与每次resume后8秒内帧增量判据，见load_progress_probe。sustained-checked-progress保留启动期6秒TF门失败；sustained-startup-diagnosis的30秒复现记录初始发现/共同动态时间建立：约4秒16帧、6秒21帧但相机链尚不可查，随后可用。因此首次完整TF发现预算独立设为10秒；一旦就绪，运行期丢失仍按6秒门失败，不改SDK外推规则。加强判据的sustained-progress-final完整300秒通过：3个SDK、两次重建、5次时钟回退，逐流空窗均未超10秒；慢读者2380帧/约3.14GB、7次暂停与7次恢复增量确认、未提前关闭。5代会话RPC p95约27–45ms，错误为空；Node自采峰值RSS177.5MiB/堆20.4MiB，启动树组合峰值761.4MiB。浏览器在部分时段额外执行10/10只读验收及真实模型、深度、曲线显示；这仍不等同于3个浏览器GPU容量测试。所有会话资源归零，remainingPids=[]，网页订阅/等待/画布0。真实网络、数小时运行与Android仍待验收。

## 渲染资源与首载

持续绘制曾因点云/Path材质重建和MarkerArray的DELETEALL→ADD反复销毁共享shader：`render-baseline/metrics.json`CPU render p95约3559ms。修复为更新GPU属性、按批次最终集合复用Marker、替换帧之后有界释放旧对象，并依据RViz支持namespace范围DELETEALL。`render-marker-batch/metrics.json`稳定p95约4.5ms，最新默认模式约5.8ms。这是最近最多120帧的CPU提交时间，不是GPU执行时间或承诺帧率。

首载专项`startup-phases/`区分脚本加载、WebGL初始化、构造、首次绘制、URDF解析和资产请求。可见面板中模块约25ms、URDF解析约4ms，但Three成功程序的getProgramInfoLog/getShaderInfoLog查询累计数秒。不能将面板状态/缓存差异单独当成卡顿原因。

v0.6增加默认errors-only、可切换verbose，以及显式profileShaders计时。errors-only每个新程序仍检查LINK_STATUS，失败取日志并报告；不静默关闭错误检查。`render-shader-checks/profiled.json`实际10程序、成功日志查询0次、链接错误0，但getProgramParameter累计约2946ms，构造2956ms，最大绘制1881ms。`default.json`未挂探针：构造1557ms，URDF解析2.7ms，12/12资产、完整场景有效，稳定CPU p95约5.8ms，最大绘制1963ms。最终构建的sustained-progress-final/scene.json构造进一步测到4915ms（WebGL初始化3921ms），稳定CPU p95约5.4ms、错误0、资产12/12。不同轮次有明显波动。因此同步GL查询和首载仍未解决；优化减少了查询，尚不能记流畅度通过。

Three compileAsync实验未稳定消除问题，改动已撤回；记录保留于`render-async/`、`render-final/async-experiment-metrics.json`。不继续堆叠未经证实的编译技巧。GL原生真实驱动失败注入已在下文补验：同一个createShaderChecks识别故意语法错误。SceneView的状态上报仍由受控renderer故障探针核对，未修改Three生成的着色器制造产品场景故障。

## 无ROS的原生GL对照

`gl-environment/results.json`在同一浏览器不加载ROS/模型/点云，分别测试default、low-power、high-performance；原生getContext约978/2975/973ms，主线程最大心跳间隔约1026/3026/1026ms。三者实际均为RTX3090/ANGLE，提示没有切换设备或稳定消除阻塞，未因此增加产品GPU选项。getParameter常规容量值多为缓存读取，而可选的设备名称查询约689/439/141ms；产品默认已关闭该设备查询。

`gl-environment/three-constructor.json`给Three0.186.1传入同一个已创建的公有WebGL2上下文并仅包装本上下文方法：native getContext约992ms；Three构造约485ms，其中EXT_color_buffer_float扩展获取约479ms，其他能力查询多数接近0。这里有预先创建上下文，不等同于产品完整构造耗时。证明当前环境本身的同步上下文/扩展查询可造成长任务，不应将其归因ROS带宽或URDF解析；还不能据此定位浏览器/GPU驱动内部根因。

KHR_parallel_shader_compile轮询就绪后，简单程序的LINK_STATUS及重复状态查询接近0ms。故意错误的片元源码在真实驱动返回链接失败，实际产品的createShaderChecks收集到“Fragment shader is not compiled”，错误路径通过。上述PASS仅表示功能断言正确，不表示耗时达标。诊断源码、三方许可证、文件SHA256与截图均保留；每轮只持有一个上下文，结束释放GL对象/loseContext并关闭页面，临时HTTP已清理。MDN的[同步查询说明](https://developer.mozilla.org/en-US/docs/Web/API/WebGL_API/WebGL_best_practices#avoid_blocking_api_calls_in_production)与[并行编译说明](https://developer.mozilla.org/en-US/docs/Web/API/KHR_parallel_shader_compile)是本次对照依据。

更长容量验证`long-30min/`目标1800秒，实际在638秒失败：client2.1的/rosapi/nodes查询超过3秒；桥同时报“Timeout exceeded while waiting for service response”。峰值进程树845.7MiB、Node自采178.7MiB/堆20.6MiB，未触内存门；慢读者14次暂停与14次恢复、10次时钟回退，最终资源归零。该轮不能记30分钟通过，也不能仅凭常规p95约13ms忽略单次超时。错误发生在桥等待ROS服务响应阶段，尚未定位具体根因，不以增加超时或自动重试掩盖。

新增显式`--trace-services`诊断：调用Jazzy公有configure_introspection(METADATA)，仅选定/rosapi/nodes，记录请求/响应阶段、GID与序号，不记录业务请求/响应内容；附一个持久的原生DDS客户端作为对照。默认关闭，不改变原服务回调或超时。30秒service-trace-smoke通过，原生16/16响应、p95约2.1ms；60秒service-trace-dense以100ms RPC周期完成1575次Web查询，原生31/31、p95约1.4ms，未复现长测超时。压力轮不替代原始2秒周期的长测。

短命客户端的introspection发布者可能在DDS发现前销毁；本机FastDDS同一交互的client/server阶段原始GID也不同。不能把原始GID+seq缺项直接解释为服务丢包，也不能从并发Web查询的时间接近就断言一一对应。保留原始metadata和rpc-trace，由标准阶段与原生对照共同分析。

独立审查修复了trace对rclpy公开关键字srv_type/srv_name的破坏，以及高频长测rpcCalls停留2000的计数问题；新探针先失败后通过。RPC总次数/失败次数独立累计，成功延迟保存全轮且最多36001样本/会话（1小时上限、循环至少100ms），不将最近窗口当作全程p95。trace模式下超过1秒的查询可请求限量Python线程栈，父进程仅向已记录且创建时间/命令仍匹配的两个自有包装进程发SIGUSR1；不对其他ROS进程发信号。

## 可重复运行

source Jazzy与工作区后，选择空DDS域和未占用端口：

```bash
python3 src/ros2_web_link/scripts/run_transport_benchmark.py \
  --bag /absolute/rosbag2_directory --project peach_replay_large \
  --asset-packages ros2_web_link,aubo_description \
  --evidence /absolute/fresh_evidence_directory --domain 97 --seconds 12
```

默认18288/19290。`--projects-config`可提供其他项目的数据契约，需含图像/深度/云/TF用例与scene配置。默认RSS上限1GiB、系统可用余量4GiB、Node堆512MiB；超限/超时失败并清理。每次使用新证据目录。

- 五分钟组合：加`--multi-client-seconds 300 --client-count 3`，30..3600秒、2..6客户端；与UI模式互斥。它替代双编码轮，使用CBOR。独立已启动桥可用`WEB_LINK_MULTI_SECONDS=300 WEB_LINK_MULTI_CLIENTS=3 npm run multi -- ws://host:port http://host:web_port/config.json`；此方式不提供服务器进程树守卫，须自行按launch清理。
- 浏览器：加`--ui-seconds 180`，最长600秒；不会运行Node负载，结束只记UI fixture stopped。证据目录创建stop.request可提前停止。不要把自动到时关闭当成崩溃。
- 原生JSON分片：`WEB_LINK_BENCH_ENCODINGS=none WEB_LINK_FRAGMENT_BYTES=131072`加在CLI前；只对探针自身订阅设标准fragment_size，不改产品API。
- 无硬件DDS图像往返：`npm run roundtrip -- ws://host:port`，仅向/web_link_demo/roundtrip_image发布合成图像，不能连接现场命令域。
- 纯协议/观测故障：`npm run wire`、`npm run observe`、`npm run audit`；官方ament lint单独执行，见testing.md。

当前TypeScript严格检查、构建、31观测/4定位/9资源/5着色器/4负载进展边界组通过。SDK核心的Service/Action/参数未在本补丁修改；Cyclone原生16项和双向哈希是本轮新执行证据，FastDDS对应核心证据见上述bounded-core，不混称同一次执行。独立审查指出的负载假通过和绘制错误漏报已用失败复现修正并复核。

## 后续取证与跨RMW核对

`service-trace-stack-smoke/`30秒通过，手工发起的一次栈通路检查只命中本轮两个包装进程，launch.log收录152个Python栈位置；这是人为诊断通路冒烟，不是实际慢RPC故障。I/O失败防护另用真实EISDIR先失败后修复，异步写入错误进入主流程，计数/finally只执行一次。

`fastdds-long-trace/`实际rmw_fastrtps_cpp，900秒通过：15次时钟回退、20次暂停与20次恢复，5代会话累计1341次Web RPC、0失败，p95约14–20ms；原生对照无超时。进程树峰值1003.3MiB，接近1GiB门。30–90秒组合RSS中位710.6MiB，末60秒917.3MiB；主要增长在bridge（203→308MiB）与rosbag播放器（132→224MiB），SDK Node中位140→153MiB。未据此认定泄漏或根因解决；30分钟原始失败仍保留。该轮在诊断文件写失败防护修正前启动，正常数据路径相同，保护分支由纯探针验证，后续完整验证将使用冻结的包。

本机已有rmw_cyclonedds_cpp2.2.4与rmw_fastrtps_cpp8.4.4。`cyclone-trace-smoke/`使用相同真实录制、QoS、CBOR和3客户端+慢读者，30秒通过，原生16/16、p95约1ms，组合峰值748MiB。`cyclone-core/`独立域下SDK核心16/16及921600/2764800字节JSON→DDS→CBOR哈希通过，峰值370.4MiB，资源归零。只对本轮子进程设RMW_IMPLEMENTATION，不改机器全局或Peach/IVG。

上游[FastDDS服务发现竞态报告](https://github.com/ros2/rmw_fastrtps/issues/842)与症状相近，但本轮缺少失败时的完整阶段/线程栈，尚不能据此认定同一根因。Cyclone短轮通过也不是FastDDS长测故障的修复证明。Jazzy对[RMW环境选择的原生规则](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/How-To-Guides/Working-with-multiple-RMW-implementations.rst)是本轮依据；仍继承部署方RMW，不在功能包中强制更换。

`cyclone-long-frozen/`已完成目标1800秒，使用独立本地安装的0.6.0冻结npm包（SHA256见cyclone-long-pack/frozen-package.json），trace关闭、2秒查询周期、3秒超时、相同真实录制和QoS。3个SDK与慢读者、2次客户端替换、30次时钟回退，5代会话累计2682次RPC零失败，p95约15.4–18.7ms。慢读者12675帧/17.83GB、40次暂停与40次恢复全部有进度且未提前关闭；数据流持续性与完整TF门通过。Node自采峰值RSS184.8MiB/堆19.0MiB，进程树峰值750.6MiB；30–90秒组合RSS中位549.2MiB，末60秒611.0MiB。最终每代订阅/发布者/等待/监听器0、remainingPids=[]，主机监听端口与测试进程均复查释放。该轮没有浏览器渲染负载，也不证明FastDDS原始超时或系统OOM根因已解决。

v0.6.1资源探针先复现JointState缓存无界：当前URDF只含已知关节，收到3000个无关名字后旧model字典达到3001项。修正为模型范围字典加1024项/1024字符的未匹配LRU，未知消息不再全量setJointValues。新探针核对部分位置/空position/NaN、无关名字/原型键/超长名、早到消息、模型替换和reset。独立审查还指出array-like name的伪造length可造成超长循环；先以length4复现缺少拒绝，再增加数组守卫，拒绝畸形name/position且保留CBOR Float64Array。带1e12 length getter的探针确认守卫根本不读取该属性。这些是输入/资源边界证据，不作为3D实际绘制性能证据。

`joint-cache-ui/`为0.6.1实际浏览器核对，最终前端文件与colcon安装SHA256一致，重载后通过真实URDF解析与12/12资产加载，15个URDF关节中6个活动值、21TF、33Marker、约8.6万显示点及800路径点；jointStateCache=6/0/1024。scene-final.json最近窗口CPU提交p95约6.1ms、绘制错误0，但构造4989ms（WebGL初始化3980ms）、首次完整绘制5205ms，首载问题继续保留。实际16UC1深度640×480和共享JointState曲线也在页面出现；media.json在fixture到时断开后记录25个曲线样本与“已断线，等待新图像”，不得将其当作在线截图。该轮300秒UI fixture按期停止（非崩溃），进程树峰值443.2MiB、不含浏览器；页面停止/断开后订阅、等待、画布0，remainingPids=[]。本轮未重跑项目10项或SDK16项，沿用各自独立证据。官方ament lint最新汇总17项、0错误/失败；31/4/9/5/4纯探针通过。

当前未完成：首载/GL同步阻塞的解决与复测、数小时容量、真实局域网及Android实机、系统OOM具体脚本来源。保留待验收，不将本机回放通过扩大为任意项目全量保证。

## 原生服务发现对照（2026-10-29）

新增显式`service_churn_probe.py`，使用已核对的Jazzy公有Node/Client/Future API，交替持久和每次新建/销毁的客户端，只调用`/rosapi/nodes`。默认无服务introspection或额外线程；它不走WebSocket。原生对照100ms周期是每组两次独立查询，不是Web长测的2秒周期，也不是重试失败查询。

- `native-churn-fastdds/`：保持录制播放器/桥/rosapi，但没有Web订阅。120秒，每模式1194/1194成功，p95约1.33/82.89ms（持久/新建），最大约5.22/93.14ms；原生进程峰值75.8MiB，服务启动树389.3MiB，两者采样范围不同。
- `native-churn-loaded-fastdds/`：另有3个SDK与慢读者接收真实完整点云/RGB-D；120秒每模式1185/1185成功，p95约1.21/82.95ms，最大19.23/99.56ms，原生峰值76.2MiB。同轮Web完整300秒通过，447次RPC零失败、p95约23.2–25.7ms；5次时钟回退、7次慢读暂停/恢复，启动树峰值791.1MiB，不含外部原生探针，remainingPids=[]。
- 以上两轮在绝对预算补强前启动；逐条核对其最大成功延迟均远小于3秒，不把迟到成功漏掉。`strict-gate-20s/`保留补测启动时底层fixture已到期的失败：零调用、服务不存在；这是编排时序问题，不作为DDS丢包证据。
- 新`--native-churn-seconds`模式把原生子进程纳入同一父守卫，避免该错位。`native-churn-owned-strict/`20秒通过（100ms默认原生周期，旧参数转发前）；最终`native-churn-owned-final/`显式转发2000ms周期，20秒每模式10/10成功、p95约1.06/1.15ms，组合峰值451.6MiB，remainingPids=[]。高频创建的约83ms开销在该低频短轮没有出现，不能据高频数据推断所有冷请求都有此延迟。

两个故障边界均先失败再修复：受控Future在3.1秒完成被旧探针误判PASS，现以发送+spin共用绝对3秒门并拒绝迟到完成；两个并发实例的先exists后write会覆盖证据，现ROS init之前独占创建JSONL、summary也拒覆盖，实测并发退出码由[1,1]变为[2,1]（一个缺服务FAIL所有者、一个初始化前拒绝）。原始失败和修复证据分别在`native-churn-output-race-red/green/`。这两项是诊断工具的正确性修复，不是产品ROS服务超时的修复。

`native-churn-negative/`的独立4秒服务实际触发3秒RESPONSE_TIMEOUT、exit1、calls=1/failed=1、无重试；故障服务最终exit0。纯deadline探针另覆盖迟到Future、发送消耗预算、pending移除/取消；不打开ROS上下文。官方ament lint当前19项0错误/失败。新模式与输出边界经只读审查，仍未确定原638秒故障根因；不改应用RMW、服务超时或执行重试策略。

## SDK请求绝对截止时间（v0.6.3）

`sdk-request-deadline/`以不可变0.6.2源码归档中的SDK作为旧实现，对同一组原生WebSocket事件故障先复现五项失败：本地5ms预算、主线程忙20ms后成功响应被接受；迟到Action反馈仍调用；响应处理跨过截止时间仍成功；相关桥错误覆盖已经过期的本地等待；真实握手的open回调延迟40ms后仍通过20ms连接门。旧实现仅依赖setTimeout先执行，事件循环忙碌时该假设不成立；依据[MDN的定时器延迟说明](https://developer.mozilla.org/en-US/docs/Web/API/Window/setTimeout#reasons_for_delays_longer_than_specified)。

新实现按单调时钟在接收、相关状态、成功完成及发送前检查绝对截止时间，完成/释放幂等。连接在创建前与观察到open时检查同一预算，超期走原失败/重连策略。浮点边界还复现(1.3+2147483647)-1.3向上取整成2147483648、被Node改为1ms，现共用取整后的合法范围钳制；最大合法等待可以继续等到显式Abort。仍保留定时器清理，不做远端自动取消或执行重试；主线程阻塞时无法按时回调，不将此修复称为实时保证或首载卡顿的修复。42组wire、31/4/9/5/4观测与资源组、20组audit通过。原始失败在red-1..5.log和red-max-timer.log、通过在wire-green.log；它们是协议边界证据，不证明原638秒FastDDS故障具有同一根因。

`sdk-request-deadline/large-data/`为请求截止修复后、连接截止补强前启动的原始实录回归：12秒JSON与12秒CBOR均无错误，云14/30、深度16/24、彩色10/11帧，URDF与6关节、相机/工具TF链有效。服务p95约369.8/9.6ms，Node RSS约193.3/152.9MiB，组合峰值716.0MiB，无残留。窗口及启动发现不同，不将两种编码的数值比当普遍性能比例。

`sdk-request-deadline/final-large-data/`在连接截止补强后执行，8秒JSON与8秒CBOR均通过：云5/16、深度7/16、彩色4/9帧，URDF/6关节及相机/工具TF链完整，服务p95约111.7/9.5ms、Node RSS153.9/145.4MiB，组合峰值666.5MiB，无残留。该轮使用正常秒级超时，最后的最大定时器取整边界由独立故障用例核对；这不是长期/Android/渲染首载的通过证据。

## 本地等待预算（v0.6.4）

Service/Action的未完成本地等待原先只有单项超时，没有总数上限；响应关联检查还会复制/遍历整个Set。现使用可配置的计数上限与Map查找，满额立即拒绝且不发送、不重试。该边界限制事件/定时器/回调的数量，不是进程RSS承诺，也不改变DDS执行的并发或取消语义。

本轮协议失败对照与原生链路结果存于runs/ros2_web_transport_20261029/pending-budget及source-rebuild-v064。错误Signal、运行期改预算、重入以及新增统计字段引起的多客户端探针兼容问题均单独复现。原638秒FastDDS超时、Python OOM源头、WebGL冷启动和Android实机验证仍保持未解决/未完成，不以本修复替代它们。

0.6.4冻结源码归档独立离线重建通过，51个dist文件逐字节一致；仅Jazzy的隔离ROS构建、17项原生SDK检查、921600/2764800字节图像JSON发布→DDS→CBOR接收精确哈希通过，无硬件示例峰值411.9MiB，清理无残留。51组wire、31/4/9/5/5观测与资源组、20组audit及19项官方lint通过。

配置相关结果必须同时保留：multi-client-30s使用常规peach_replay（sensor RGB-D、小型single_cloud）在约24秒因彩色流超过10秒无更新失败；其他流及RPC仍有进展，未见REQUEST_LIMIT或内存超限。随后multi-large-30s显式使用既有peach_replay_large（Reliable RGB-D与约4.1MB完整点云），3个SDK加慢读者30秒通过，发生2次客户端重建，45次RPC零失败、各代p95约6.6–15.7ms，组合峰值728.9MiB，慢读者1次暂停/恢复均有进展。最后3个活动客户端TF完整；首个被替换的会话仍处于初始TF预热，不能声称全部5代都完成TF验收。两轮remainingPids均为空。两个配置还改变了点云来源，不能据此把失败单独归因为QoS；常规sensor断流根因仍待对照。没有浏览器/GPU负载，不能将这些RSS或30秒结果推广为设备容量或长时保证。

## 本机大图QoS对照与表单修正（v0.6.5）

qos-controls的sensor-30s在同一录制与single_cloud下再次因彩色超过10秒无更新失败；仅把两项图像QoS从sensor改为reliable的reliable-rgbd-30s完成30秒、45次RPC零失败，进程树峰值547.1MiB。没有换点云来源。原生官方ros2 topic echo不启动Web/rosbridge/SDK：34秒内Best Effort depth5捕获2帧并报告至少35次message_lost，Reliable depth10捕获40帧；额外Reliable depth5也捕获40帧，排除仅靠加深队列解释差异。原生观察只有时间戳和丢失事件，未保留图像；它证明本机大图在SDK之外也会丢失，未定位到具体RTPS传输包/缓冲或内核原因。

ROS官方sensor预设允许丢帧；Reliable订阅只能匹配相应发布者。参考[Jazzy QoS兼容规则](https://repo.test.ros2.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)及[FastDDS大数据说明](https://fast-dds.docs.eprosima.com/en/v2.14.7/fastdds/use_cases/large_data/large_data.html)，分片丢失可使整条Best Effort大消息丢失，但本轮未抓包证明这一具体机制。因此仅让Peach录制项目的图像配置显式Reliable，不修改SDK通用sensor预设、全局RMW/XML或内核参数，不以此声称原638秒服务超时已修复。

浏览器还复现了独立的配置传递错误：Reliable图像用例“填入面板”后变成sensor，显式对象在Topic/Media中被替换成预设。0.6.5已保留完整QoS，结构化策略在可见下拉框中显示；缺省值与audit一致。CUA旧页失败和修复后的表单证据见ui-red.json/ui-green.json；实际数据显示与项目检查另行记录，不以填表成功代替DDS验收。

real-ui真实浏览器中，10项只读项目检查全部通过，显式Reliable depth7深度640×480实际绘制；原生端点图确认RELIABLE，History/Depth为UNKNOWN，深度值的外部发现验证不成立。模型12/12资产、6活动关节、21TF、33Marker、约6712显示点与800路径点到达。CPU提交p95约3.5ms、绘制错误0；当前为后台IAB，观测renderFps约4不能作为前台/GPU帧率结论。构造4045ms、首次完整绘制4900ms，WebGL首载阻塞继续保留。进程树峰值406.8MiB不含浏览器；fixture300秒正常停止，UI全部订阅/等待/canvas释放，自己的测试标签已关闭。

同轮修复manual项目切换残留：切换后Topic名称/类型清空，QoS为sensor、编码none、限频0，旧自定义选项删除。表单红/绿证据与本轮源码/安装产物一致；没有执行真实相机、机械臂或IO。

0.6.5冻结源码的FastDDS长测仍失败：目标1800秒，实际760.1秒因一次`/rosapi/nodes`响应超时退出，1125次调用中1次失败。守卫进程树峰值907.94MiB，未触1GiB门，数据流与TF仍有进展，最终remainingPids=[]。日志中的后续SIGINT与exit -2来自测试收尾，不能当作新的系统闪退。证据为`source-rebuild-v065/long-fastdds/`。本机rclpy的call_async与get_pending_request使用同一把锁，已排除“响应早于登记而无锁丢失Future”的源码假设；尚未定位此长测超时根因。

## 页面大结果与记录边界（v0.6.6）

页面原先先完整JSON序列化再截文本，操作记录还保留原始对象；大数组/参数可造成多余分配与保留。现统一先生成有界快照：总遍历256项、字符串预算8192字符、单字符串预览256字符、对象32字段、数组16项、深度6；二进制只显示类型/字节。结果面板最多18000字符，单条日志详情最多4096字符、最多200条，最近记录优先。循环/BigInt/读取异常不会破坏后续普通字段。项目摘要复用同一限制；Action质量警告直接读取结果degraded字段。SDK收到的原始值及项目完整报告导出保持原契约，不截断SDK响应。

`ui-preview/`保留原实现失败与14组显式纯格式探针通过。隔离Jazzy构建、TypeScript检查/打包、19项官方lint通过。`ui-preview/live/`实际CUA页面经原生只读参数服务读取200014字符，项目1/1通过，DOM报告摘要1253字符；页面下载的完整JSON保留全部200014字符，SHA256与原生发布端预期一致。没有通过隐藏页面变量或伪WebSocket获取报告。另一个约1MiB参数仅在fixture声明，本轮未读取，不计入通过。自己的fixture有360秒/1GiB守卫，峰值395.38MiB、remainingPids=[]，已关闭测试标签；RSS不含浏览器。该修复减少页面展示开销，不是原Python OOM或FastDDS服务超时的根因修复。

## 取消前等待状态诊断（v0.6.6）

`service-wait-trace/native-red`先保留原代码缺少取消前证据的失败；native-green/native-final用真实Jazzy Trigger服务及原生Future/Event验证正常响应与故意延迟超时，快照为done=false/cancelled=false/event=false，随后才取消。native-final清除了刻意继承的METADATA环境，峰值及进程清理见guard-result.json。该故障是受控fixture，不是760秒长测超时的根因复现。

独立审查指出并补验两个问题：新增分支误关旧METADATA模式，冻结旧安装源码的modes-red.log复现，12种模式/继承环境组合修复后通过；诊断文件及stderr同时失败会覆盖原异常，flush失败还会无限重写，sink-write-red.log/sink-flush-red.log分别保留失败，现IO前消耗最多8次尝试且二级日志失败隔离。受控Future完成/通知未执行、采样不分类、write/flush四种边界通过，weakref确认没有由历史环保留Future。它们核对观察器行为，不是长时性能、真实故障或DDS投递证明。默认通信与SDK没有增加诊断hook；自有桥仅在显式开启时安装版本限定观察器。

冻结0.6.6的`source-rebuild-v066/long-fastdds-waits`目标1800秒，603.6秒失败：891次调用中2次TIMEOUT（client1.2、2.1），各代成功延迟p95约25.5–29.3ms，完整数据流与末次TF仍有进展。10次时钟回退、慢读者14次暂停/13次恢复（失败发生于下一次恢复前，不能记恢复全通过），进程树峰值852.67MiB、Node峰值182.37MiB/堆21.50MiB，未触内存门；每代活动资源及所有进程归零。此轮wait诊断已安装但输出为空，桥未在收尾前记录响应等待错误。核对发现负载探针的失败后5秒窗口只识别旧METADATA环境，新模式立即收尾；先以实际失败处理代码复现grace-red.log，再补为两种诊断均保留5秒，grace-green.log通过。该改动仅完善失败取证，不修复RPC超时，空文件也不能证明故障不在ROS/桥端。

`source-rebuild-v066-grace/long-fastdds-waits`在527.2秒收尾为FAIL：771次RPC中2次TIMEOUT，p95约11.3–15.3ms，进程树峰值860.46MiB、Node181.77MiB/堆21.24MiB；8次时钟回退，慢读者12次暂停/11次恢复，失败中断了后续恢复验证。两条快照CID与桥错误精确相同，waitFalse后且原取消前均为futureDone=false/futureCancelled=false/eventSet=false；从桥调用进入到等待约1.1–1.5ms，同周期另一请求约5ms完成。对这两次调用，排除Future完成但通知未执行的候选；仍不能区分请求到达、服务处理、RMW响应匹配或取响应阶段。set时间在取消之后，不能当作响应完成时间。末次TF完整、所有会话资源与进程归零，未触内存守卫。

## 服务端标准事件与被动观察（v0.6.7）

本机tracetools8.2.6可用，但其已安装事件定义不含所需的逐RPC请求/响应阶段，因此未把client-init/C++回调跟踪当作本故障证据，也未启动LTTng会话。新增组合诊断使用ROS标准服务METADATA，仅服务端启用、被动collector不查询。它依然会增加DDS实体/改变时序；后续修复须关闭诊断复测。

`service-server-trace/native-smoke`使用真实Trigger服务，2次接收/2次发送、无客户端0/3阶段，native.calls=0，峰值422.51MiB、remainingPids=[]。`loaded-smoke`为真实完整点云/RGB-D、3SDK加慢读者30秒，45RPC零失败、服务端接收/发送各45条、无主动对照查询，峰值794.5MiB、全部清理。角色工厂与32种CLI/继承组合通过。审查发现RPC诊断日志写失败会覆盖原TIMEOUT，先复现request/response/error三阶段失败，现独立记录诊断错误、保留RPC结果和计数，三组通过。24项官方lint通过。这些是观测链路正确性证据，不是原长测故障修复或30分钟通过。

## 前台真实项目可视化核对（v0.6.7）

`runs/ros2_web_transport_20261029/visual-v067/`使用冻结0.6.7运行时、完整真实Peach录制、隔离域154，实际IAB前台可见；不启动相机、机械臂或Gazebo。10项只读检查全部通过。完整源点云约17万点/4.1MB，3D显示约8.6万采样点、21 TF帧、33 Marker、6活动关节、800轨迹点及12/12模型资产。实际拖动、滚轮、点云图层、视角复位和固定坐标系切换均有页面证据。640×480彩色图、16UC1深度（0.00025米/单位）和关节曲线可与3D同时运行，曲线保持600样本上限。

两次3D构造约260/308ms，首次完整绘制约266/311ms；并发截图时产品renderFps约57.84，CPU提交p95约1.8ms，渲染/着色器错误为0。该帧率来自产品RAF/提交计数，不是GPU计时或输入延迟；此次浏览器/GPU已运行，不能替代冷启动验收，也不能据此抹去此前1.6–4.9秒首载阻塞。停止后订阅/等待/Canvas均为0，再启恢复模型和TF；最终217秒收尾，ROS子进程树峰值453.23MiB（不含浏览器）、remainingPids为空、19788/20790端口释放，测试标签已关闭。完整报告本轮未下载；10/10来自CUA可见验收卡片。Android/局域网设备、历史OOM脚本与FastDDS长测超时仍未验收。

## 服务端被动事件长测失败（0.6.7，2026-10-29）

`source-rebuild-v067/long-fastdds-server-waits/`使用同一冻结运行时、完整4.1MB点云/RGB-D、3SDK和慢读者、2秒查询/3秒本地门，追加服务端METADATA与无额外ROS实体的WAIT观察。目标1800秒，540.69秒FAIL：795次RPC中client0.1与1.2各1次TIMEOUT；四代成功p95约17.66/18.09/11.87/19.09ms。9次时钟回退，慢读者4147帧/约5.36GB、12次暂停及12次恢复确认；未到计划的第二次客户端替换，不能计五代全通过。进程树峰值937.11MiB，Node181.84MiB/堆21.32MiB，未触1GiB门；最终资源和进程归零。

服务端接收/发送各795条，按同一服务端GID+seq可配795对；最后3次并发查询窗口内有3对事件，接收至发送均小于1ms。被动观察器native.calls=0。桥端wait失败文件为空、无服务等待错误；两名失败客户端各有1条ignoredResponses。该计数仅证明某响应ID在处理时已无本地等待，不足以确认就是该次超时或确定到达时间，因此不能直接宣布WebSocket排队、DDS丢包或迟到响应是根因。也不能用空wait日志断言在SDK截止时Future已经完成。此轮没有正常桥调用的最后时间线及客户端原始响应CID/时间；后续0.6.8补充见下文。此轮和前一轮futureDone=false证据分别保留，不互相覆盖。

## 0.6.8时间线取证正确性

`service-response-timeline/`记录正常调用退出环和原生WS事件诊断。首版同步会话标签失效、同步writer制造50ms请求超时均先保存失败，使用标准AsyncLocalStorage与有界结束后flush修正。原始SDK/观测代码未改，49个dist文件与0.6.7相同，版本锁副本和HTML版本文字变化。受控退出4种、原有Future/Event4种、WS5组与慢sink、模式32组通过；官方lint首次失败与修正后29项零错误/失败均保留。

native-smoke使用真实Trigger，成功/超时两个CID进入最终快照，峰值424.75MiB、remainingPids=[]。loaded-smoke为完整点云/RGB-D、三SDK+慢读者30秒，45RPC零失败、135条元数据、45个CID与桥完成记录精确对应，最大桥调用88.63ms，1次慢读暂停/恢复、0次时钟循环，组合峰值777.60MiB；最终诊断缓存及进程全部归零。该轮只证明观测链可用于下一次长测，不证明原超时修复；原生event时刻不等于网络包到达。

后续`loaded-buffered-smoke`还移除了旧rpcTrace的同步落盘，stack旗标异步合并并在判定前等待。完整负载30秒45RPC零失败，135wire/90rpc元数据、45CID与桥完成快照完全相同，最大桥调用84.85ms，组合峰值805.72MiB，1次慢读暂停/恢复、0次clock循环；两缓存和全部进程归零。新候选与首个source-rebuild-v068（在该修正前形成）分开记录，不把任一短测当原长测已修复。

## 原生响应与桥等待的CID时间线（0.6.8，仍失败）

`source-rebuild-v068-buffered-rpc/long-fastdds-server-waits/`绑定归档SHA256 `1af780fd0a2f4abd331baa77a646137a8fce8dbfcd06b5d406b2f392c498e638`，使用FastDDS、同一完整录制、三个SDK及慢读者，2秒查询周期、3秒等待预算。目标1800秒，在最后资源采样988.19秒时FAIL：1458次RPC中2次TIMEOUT，成功调用的四代p95约13.52/34.77/34.55/26.57ms。16次clock回退、22次慢读暂停及22次恢复确认；慢读者7144帧、9373258945字节。仅运行到四代，未到1080秒第二次替换。进程树峰值995.41MiB、Node自采187.95MiB/堆21.17MiB，未触1GiB门；各代最后TF完整，本地资源与全部进程归零。该峰值不包含外部浏览器。

两次失败CID尾号1456（client0.1）与1458（client2.1）均有原生WS send/sendReturn/receive及桥完成记录。桥进入约1.4–2.6ms后开始等待，3秒后wait=false；取消前Future未完成、未取消、Event未置位。两条原生receive均为同CID的`service_response result=false`，约在SDK本地超时6ms后执行。因此这两次ignoredResponses各1可确定是已超时后的错误回复，不能称作迟到的成功结果；取消触发的set也不是响应完成。原生事件时间不是网卡时间，两个进程的单调时钟不能直接相减。

服务端REQUEST_RECEIVED与RESPONSE_SENT各1458条，同侧GID/sequence配成1458对，最大事件stamp间隔约1.118ms，最后并发窗口三对均存在；主动原生对照调用为0。rcl9.2.11源码确认RESPONSE_SENT只在rmw_send_response返回OK后发布；它不证明DDS交付成功。rmw_fastrtps8.4.4中客户端被判GONE时也可能返回OK而不写，但本轮没有证明走过此分支。现有证据尚不能区分RMW接收、原生Client取样及pending关联，不能直接修改系统RMW或引入客户端缓存作为已验证修复。

wire共4374条/813129字节，rpc共2916条/237479字节，结束后两缓存为0，sinkErrors=0、budgetExceeded=false。写盘在请求窗口之外；仍需承认显式诊断改变时序。完整CID、原始字段、同侧事件窗口及清理见[本轮失败摘要](../../../runs/ros2_web_transport_20261029/source-rebuild-v068-buffered-rpc/long-fastdds-server-waits/summary.json)，源码语义见[source.md](source.md)。此结果不追溯解释0.6.7缺少CID的ignoredResponses，也不代表桌面闪退再次发生或根因已修复。

## 原生Client取样观察的短测（0.6.9）

`service-native-boundary`先完成默认关闭/版本守卫、返回/异常保真、隐私及有界退出记录的受控检查，并用真实Jazzy Trigger补验；最终fixture峰值430.83MiB，无进程残留。父工具原子拒绝旧目录，缺证原因独立记录，静态审查及原始红/绿输出见该目录与development.md I24。

`loaded-smoke`以相同完整录制和三SDK/慢读者运行30秒：45RPC零失败，45个CID在native sendReturn→同Client标识/sequence的成功takeReturn、WS和桥退出快照精确匹配。共288条native记录，含空取样；无drop、metadataError或超预算。进程树峰值785.00MiB；慢读者173帧/236621210字节，暂停/恢复各1、未提前关闭；0次clock回退，资源和进程归零。短测不证明历史超时修复，也不将空取样数当作丢包率；取样是executor调用边界，不是网卡或DDS接收时间。该候选后续长测未通过，见下文。

## 0.6.9长测：原生成功取样在超时后出现

冻结0.6.9的`source-rebuild-v069/long-fastdds-native`在637.04秒FAIL：939RPC中client0.1一次TIMEOUT，四代成功p95约12.22/13.28/12.38/12.40ms；峰值920.45MiB、Node自采/堆见原始multi-client.json，未触1GiB门。10次clock回退，慢读者4696帧/6013089088字节，暂停和恢复确认各14次。末次TF完整、每代计数归零、进程及20488/21690端口全部清理；仅四代，第二次替换未到。

服务端939对接收/发送事件，原生send各939、take各2295，其中1356次为空。6468条native元数据、零丢记录/错误/超预算，diagnosticFailure为空。失败CID尾号937的sequence=1被对应Client成功取到，但发生于native send返回后3000.883ms、waitFalse后0.238ms、取消引起的Event.set返回后0.213ms。没有保存DDS来源/接收时间，不能仅凭“紧接取消”判定唤醒原因，也不能宣布丢包。完整事件和界限见[终态摘要](../../../runs/ros2_web_transport_20261029/source-rebuild-v069/long-fastdds-native/summary.json)。

## 完整合成点云的字节一致性

`payload-integrity/native-final`在独立空域、仅两个自测话题下完成两图与4136928字节完整PointCloud2的JSON发布→原生DDS→JSON/CBOR接收。完整哈希、layout及首/中/末显示采样点XYZ/RGB正确，约6.17秒、峰值623.21MiB、无残留。早两轮测试脚本TDZ错误及并行构建dist竞争均保留为FAIL，详见development.md I25。合成云保留非零padding，哈希覆盖全部原始字节；该数据与真实Peach尺寸相同，但不是相机原帧，不能作为真实相机每帧无损证明或长期性能结果。

0.6.10成功take额外保存RMW来源/接收纳秒字符串，真实受控fixture已显示字段有效，约0.160ms写入→reader历史、0.208ms历史→观察到take；长测中的迟到/迟取仍待该版本捕获。此阶段不延长原3秒门、不换RMW、不自动重试。

`native-reception-time/loaded-smoke`随后以完整录制/三SDK/慢读者30秒通过：45RPC零失败、45CID跨WS/native/桥精确对应，266条metadata无缺证。写入→历史中位0.0187ms/最大0.494ms，历史→观察到take中位约0.381ms/最大约2.642ms；后一差值使用wall浮点锚点，不是纳秒精度保证。峰值801.04MiB、慢读暂停/恢复各1、0次clock循环，进程资源全部归零。这证明观察链正常，不替代原1800秒门。

## 0.6.10长测缺少失败调用的take

`source-rebuild-v0610/long-fastdds-reception`在687.27秒FAIL，1014RPC中client2.1一次TIMEOUT，峰值955.12MiB，11次clock回退、慢读暂停/恢复各15次；无诊断缺证/超预算，进程/20688与22290端口全部清理。1014对服务端接收/发送事件，最后并发三对事件均在请求后数毫秒内。失败CID只记录到原生send，原reader销毁前没有take，不能取得该次RMW接收时间或用正常调用的时间补推。完整数据见[该轮摘要](../../../runs/ros2_web_transport_20261029/source-rebuild-v0610/long-fastdds-reception/summary.json)。

0.6.11的显式post-timeout reader保留只用于填这项证据缺口，原超时仍失败；它改变超时后的生命周期，不能作为原生命周期通过的证明。`service-late-client/native`受控延迟服务保持网页result=false，另观察到约0.301秒后迟到take；2秒后提交原executor清理、峰值430.64MiB、无残留。`loaded-smoke`完整录制30秒45RPC零失败，峰值791.01MiB、慢读暂停/恢复各1，未超时所以late记录/Timer均0。普通请求立即清理，长测故障仍未修复。

## 0.6.11捕获到接近3秒的reader历史插入延迟

`source-rebuild-v0611/long-fastdds-late-reader`最终1753.02秒FAIL，2595RPC中1次TIMEOUT，5代客户端、29次clock回退、慢读暂停/恢复各39次，峰值1009.31MiB，系统可用最小17659.48MiB。2595对服务端事件、17534条native元数据完整，无drop/error/超预算。自有进程和20888/22590端口全部清理，见[摘要与精确时间线](../../../runs/ros2_web_transport_20261029/source-rebuild-v0611/long-fastdds-late-reader/summary.json)及[清理核对](../../../runs/ros2_web_transport_20261029/source-rebuild-v0611/long-fastdds-late-reader/cleanup-verified.json)。

失败CID尾号2593的RMW source至reader历史插入相差2999.312593ms，历史插入至观察takeReturn约0.126ms；后者含wall浮点采样误差，跨字段计算以同机时钟未跳变为前提。该次桥wait返回true、没有启动late保留；WebSDK先到3秒期限，约3ms后收到result=true，仍记FAIL。不能把迟到成功当成通过，也不能据此直接断言丢包、发现竞态或心跳恢复。此轮支持延迟发生在writer源时间至reader历史可见阶段，尚不是修复；真正修复后仍需关闭诊断、按原生命周期和相同负载复测。

## 单响应心跳对照仍因内存门失败

`heartbeat-profile/long-fastdds-100ms`只通过标准XML把`rr/rosapi/nodesReply`写入心跳改为100ms，其余负载/诊断/3秒判据/1GiB门保持。安装的公有profile API及RMW转换先证明只有心跳字段变化；30秒45RPC冒烟通过。长测1098.25秒因RSS=1042.06MiB失败，系统可用仍至少16251.86MiB；Node被停止前未输出最终报告，不能记完整RPC/TF/慢读通过。进程与端口已清理，见[终态摘要](../../../runs/ros2_web_transport_20261029/heartbeat-profile/long-fastdds-100ms/summary.json)。

1635次native成功take的source→reader历史最大100.794ms、p95=99.479ms；3秒基线最大2999.313ms、p95=4.693ms。最大值缩短同时p95变差，两者均保留，不能只以单一尾值宣布修复。比较仅为native阶段，并非WebSDK端到端耗时/失败率；见[两轮时间分布](../../../runs/ros2_web_transport_20261029/heartbeat-profile/timing-comparison.json)。下一步核对按帧数限制的出站队列与慢读期间保留字节，实验XML尚不进入默认部署。

## 0.6.12修正上下文丢失的绘制误报

Three在丢失上下文时正常返回，原来会被计作有效绘制。新增公有isContextLost前后检查，先失败复现再修复，4组边界、完整observe/wire/audit/preview和33项官方lint通过。实际IAB使用当前SDK/SceneView、仅本页网格执行丢失与恢复，成功帧13→13→14、累计错误保留1、首次CPU提交85.2ms保持；停止后画布0/SDK disposed/定时器停止，HTTP进程及标签清理。见[实际页面四阶段](../../../runs/ros2_web_transport_20261029/scene-context-loss/browser-stages.json)和[恢复截图](../../../runs/ros2_web_transport_20261029/scene-context-loss/browser-restored.png)。它不包含ROS或完整Peach场景，也不是冷GPU或GPU执行耗时证明。


## 0.6.13队列16长测仍失败

`outgoing-queue/long-queue16-100ms`在1339.29秒失败：1989RPC/1TIMEOUT、峰值911.27MiB、5代客户端最终TF及资源清理完整。失败调用source→reader历史0.402113ms，桥wait约8.37ms返回true、调用约41.89ms返回，记录窗口内无同CID WebSocket响应；同段队列满警告不含连接/CID，不能直接证明该响应被丢弃。不得以RSS降低宣布成功或改默认64。详见development.md I30及[原始终态](../../../runs/ros2_web_transport_20261029/outgoing-queue/long-queue16-100ms/summary.json)。

## 0.6.14页面离散事件观测

默认关闭的profileInteractions通过标准PerformanceObserver报告页面级离散事件，七组边界检查通过。实际IAB网格页真实点击isTrusted=true、记录41事件，最大同步处理265.1ms、已报告duration p95=288ms，包含启动构造负载，不是80ms按钮独立耗时。停止后canvas=0、client=disposed，再次点击计数不变，服务器/标签已清理。完整[阶段数据](../../../runs/ros2_web_transport_20261029/event-timing/browser-stages.json)与[页面截图](../../../runs/ros2_web_transport_20261029/event-timing/browser-disposed.png)。这只验证指标接入和释放，不证明完整Peach负载、连续拖动、INP、冷GPU或Android性能；冷GL初始化问题仍保留。


## 0.6.15双连接短测

同一桥上使用分离RPC/观测的现有RosClient，完整录制90秒135RPC零失败、5代、TF/双侧资源清理完整；data约2.725GB、control28242B且接收帧恰等于RPC数，峰值约733.3MiB。不选新模式/队列/XML的默认30秒45RPC零失败、峰值约720MiB，仍单连接。分别见[双连接核对](../../../runs/ros2_web_transport_20261029/separate-control/smoke90/verification.json)与development.md I32。当前只证明短测接线及释放，不能宣称长期故障修复；后续需同负载长测和关闭诊断的原生命周期验收。


## 0.6.16原始图像应用层复制

真实ImageView的受控DOM探针先记录640×480 RGBA二次分配，修改后只分配原解码结果，1228800B额外克隆不再发生；带padding BGR、大端16UC1像素未变，四组及完整observe通过。没有GPU、FPS、RSS或INP改善的测量；Canvas内部拷贝不在该计数范围。真实浏览器复核等待正在运行的冻结0.6.15通信长测终态，详见development.md I33。


0.6.15显式双连接长测随后1800秒通过：2688RPC零失败、最大135.620ms、逐代p95约105–107ms，慢读暂停/恢复40次、30次clock回退，峰值954.44MiB、所有资源/进程/端口清理。仅适用于队列16+100ms XML+诊断模式；不是默认配置或关闭诊断已通过。见[长测终态](../../../runs/ros2_web_transport_20261029/separate-control/long-queue16-100ms/summary.json)。

0.6.16实际录制三消息的6次JSON/CBOR DDS往返、独立全量显示参照和实际浏览器两图Canvas全像素哈希均通过；一次应用层RGBA分配已真实确认。峰值615.51MiB不含浏览器；首轮清单uint8误写bool的失败原样保留。详见[原生/浏览器结果](../../../runs/ros2_web_transport_20261029/image-copy/native-canvas-typed/verification.json)及development.md I33–I34。不能据此推断持续吞吐、GPU时间或同步重建正确性。

## 原生命周期失败与0.6.17功能范围

冻结0.6.16关闭诊断后，同一完整录制/双连接/队列16/100ms XML/3秒RPC及10秒逐流门的1800秒目标在777.54秒FAIL：client0.1彩图超过10秒未推进。1158RPC零失败、峰值847.58MiB不是整体通过；慢读在失败时仍暂停。源彩图最大内部间隔3.724秒、跨循环4.199秒，不能单独解释停顿；仍需区分ROS供给、节流、出站队列与消费。见[原生命周期终态](../../../runs/ros2_web_transport_20261029/separate-control/long-original-lifecycle/summary.json)和development.md I36。

0.6.17把可选双连接接入通用audit/单页，默认仍共用连接。真实demo共用/分离模式均6项只读通过，独立demo12项及Action取消终态5通过；最终握手修复后服务返回7、六项只读通过、订阅/等待0，启动树峰值333.57MiB不含浏览器。见[最终页面验证](../../../runs/ros2_web_transport_20261029/ui-channels/demo-ui-handshake-fixed/verification.json)。示例400点/96×48 PNG与后台CPU提交测量不能代替完整Peach、GPU/前台交互、持续大流、LAN或Android验收。


## 数据流诊断与0.6.18真实页面范围

冻结0.6.17加工作区可选v2 metadata观察器完成1800秒：2685RPC零失败、慢读40次暂停/恢复、30次时钟回退，峰值841.05MiB，健康彩图各代写完与Node计数一致；无诊断溢出或未关联。健康连接仍有一次tf_static/clock队列拒绝。原777.54秒停顿没有复现，带观察器通过不能当成关闭诊断的原生命周期验收或根因修复。详见development.md I37与[终态诊断](../../../runs/ros2_web_transport_20261029/stream-flow/long-v2/diagnostic-summary.json)。

0.6.18在实际安装/rosbridge/浏览器完成Clock初始零、暂停、精确到期、大秒值1ns回退和系统模式重建，demo 6项只读及12项通信自测通过。当前Peach完整录制页面10项只读通过，12/12模型资源、21TF、6活动关节、33Marker、约8.6万显示点、800Path点及两幅640×480 RGB-D/JointState曲线通过；旋转缩放、图层、固定帧错误/恢复、停止重启与资源释放另有真实页面证据。子树峰值465.47MiB不含浏览器，PID与22488/24090清理。详见[当前真实项目页面核对](../../../runs/ros2_web_transport_20261029/scene-clock/peach-ui/verification.json)。

该页面CPU提交p95约1.4–1.6ms，最大绘制仍2000ms、最大页面长任务2001ms；后台浏览器/截图不等于前台FPS或GPU计时。初始CPU提交68.7ms不表示全部模型或屏幕呈现已完成。未验收项仍为：原生命周期持续彩图停顿、WebGL秒级阻塞、LAN/Android实机及历史OOM具体脚本。时钟修复不能替代这些问题的关闭。


## 0.6.19 图像边界与显式断开

轻量彩图诊断long-v1已取得1800秒PASS：2688RPC零失败、5代p95约104.00–106.08ms、慢读10678帧/13957377286B、40次暂停及恢复，峰值867.35MiB不含浏览器。16行元数据无溢出，5代彩图publish/admitted/Node数1637/510/1025/1117/608一致，无彩图准入拒绝。它仍使用冻结0.6.17及显式观察器，不是无诊断验收，I36原777.54秒彩图断流保持开放。新基准仅在关闭报告保存已有color/depth/cloud接收时间与开始/关闭时间，不增加热路径时钟调用，不改变原门或输入。

image-headers/demo-native验证0.6.19共享/分离连接，6项只读和12项通信自测通过；PNG96×48、空format真实JPEG4×3显示成功，33B/5000×5000头及compressedDepth预解码拒绝，恢复PNG后重连/直接断开11→0。DOM容器及标准图片src清理，控制台无warn/error，子树峰值344.86MiB。该轮max render约1043ms、页面最长任务1465ms，不把合法图像预算门当渲染卡顿修复。

image-headers/peach-ui的10项只读、模型12/12、21TF、33Marker、约8.6万显示点、800Path、640×480RGB-D和曲线通过。实际资源已释放，但立即显示的订阅数仍9，500ms统计延迟使汇总为PARTIAL；保留原失败。随后同步刷新修复及第12组CLI通过，新普通隔离安装/35项官方检查通过，再在metrics-fixed/peach-ui窄复验完整观察和直接断开：计数立即9→0、等待0/512，后续三容器0/src空，峰值453.38MiB、7条Z-UP提醒、0error，主机PID身份/端口及临时标签均清理。这一轮未重跑10项卡片，不能把两个版本合成同一份报告。

本轮Peach预断开窗口CPU提交p95约1.3ms、renderMax约62.6ms，同时页面最大长任务2029ms。数据来自已有公有页面统计，未做GPU计时、堆回收、前台输入延迟或冷上下文控制；模型热启动变快不能覆盖旧秒级首屏问题。PNG/JPEG头库会复制编码数据一次，未测得压缩大流RSS/FPS改善，不宣称零复制。所有上述子树RSS均不包含外部Codex浏览器。


## 0.6.19 当前冻结版无观察器长测终态

source-rebuild-v0619/long-no-observer已完成1800秒PASS，使用归档SHA ac8b468765e80a1c77829d695fda226c82819d428639458dc9f7fb73bf16b9e3离线重建产物，所有服务/全流/彩图诊断关闭，队列16/100ms实验响应XML及原全部门保留。3逻辑双连接+慢读7WS、2685RPC零失败，5代p95=11.06/104.64/105.82/106.07/106.19ms；30次Clock回退，慢读10572帧/13.829GB、40暂停/40恢复。子树峰值841.61MiB不含浏览器，Node195.88MiB/堆21.96MiB，最小可用22,597.13MiB；最终全部TF、资源及主机PID/端口清理通过。

新增报告字段实际验证：5代彩图最后年龄204/860/408/407/405ms，深度490/138/19/24/21ms，点云607/329/1231/1223/1225ms。字段是Node回调接收时间，不是ROS输入时间；退休代与最终代各自在关闭时统计，不能把它们当同一时刻的端到端延迟。通过未解释原777.54秒故障，没有修复提交可将其确定归因或关闭；保持I36开放，也不覆盖0.6.20、冷GL或设备LAN/Android。


## 0.6.20 图像字段修复与当前首载分段

未修改的0.6.20普通安装在raw-image-validation/peach-native完成完整Peach10项只读、12/12资产/21TF/33Marker/约8.6万点/800Path、640×480RGB-D、曲线及单像素encoding拒绝→合法rgb8恢复；直接断开即时0订阅/0等待和三容器0。子树峰值482.02MiB，主机PID/端口/标签清理。15组纯图像边界/12组页面函数/完整CLI、独立审查及35项官方检查通过，仍不证明历史OOM或长任务根因修复。原页面CPU提交p95约1.3ms、renderMax49ms，但页面长任务2179ms。

临时canvas-phases副本以另一独立域191测量，不进入npm产品。暖Image/Plot/Scene构造最大0.3/2.4/48.7ms，模块22.7ms、WebGL2 getContext17.8ms、扩展float4ms、2D putImageData最大1.6ms，图像回调6.3ms、点云5.2ms、Marker6.1ms。20行无溢出/缺钩，原安装哈希不变。早期页面长任务1466ms而renderMax47.9ms，后期renderMax也达2040.3ms、页面2459ms；两种现象均保留，不选快窗口覆盖旧失败。尚未计时shader compile/link/status/draw及全RAF/Plot异步绘制，不能宣称已找到长任务位置。子树452.90MiB不含浏览器，控制台7Z-UP/0error，退出0资源、PID/22788/24390及标签清理。详细阶段原文和工具失败/修复见development.md I43及canvas-phases。


0.6.20源码闭包已单独验证（163文件、28离线依赖、四公共入口、54dist一致、纯Jazzy35官方检查）。临时诊断不进归档；功能原生测试、无观察器0.6.19长测和诊断计时仍分别记证，不汇成总体性能PASS。源码SHA及包内旧索引边界见development.md最终provenance，当前工作区索引在输出关闭后更新。


## 0.6.20 现有profileShaders的实际阻塞调用

10月30日render-profile/peach-native使用冻结20和公有profileShaders=true。纯Scene的程序查询最大7.3ms、renderMax44.2ms、页面longtask0；加入RGB后getProgramParameter最大1003.7ms、renderMax1033.5ms/longtask1035ms。第三份保留计数program52/pageLongTask2228ms是在480秒守卫自动退出后保存，不算在线第三阶段；纯Scene/RGB两份在守卫前采集。此方法聚合不区分LINK_STATUS/ACTIVE_UNIFORMS等pname，不能据此断言具体枚举或所有页面长任务已定位。阶段顺序/暖GPU与设备探针均限制对照含义，结果不是优化。

无着色器/绘制错误或缺钩，子树峰值586.29MiB不含浏览器，PID/22888/24490及临时页清理。全Scene异步revision门在产品实现前因连续更新活性、透明双面及pending句柄边界被设计审查否决；保留证据与原计划，产品仍20，后续先精确参数诊断。

后续精确查询诊断已完成：正常 profileShaders=false 的临时副本在中段快照捕获 ACTIVE_UNIFORMS1575.3ms、同阶段 renderMax1576.9ms/pageLongTask1771ms；LINK_STATUS和ACTIVE_ATTRIBUTES最大0ms，其他已计时调用最大17.6ms。33行无溢出/缺钩，控制台7提醒0error。它支持定位到新program首次uniform读取等待，但调用计时包含调度暂停，尚不区分具体材质或GPU独占成本，也不证明所有长任务同因。产品未加入异步准备，仍0.6.20。子树峰值551.81MiB不含浏览器，PID/22988/24590及临时标签已清理；末快照CONNECTED不代替独立RPC存活或人工断开验证。原始阶段与时序边界见[精确查询终态](../../../runs/ros2_web_transport_20261030/program-query-trace/peach-native/verification.json)。

10月30日01:19的宿主机只读复查：Codex主进程自历史OOM后的启动时刻未变，可用内存23.28GiB，17个相关测试端口均释放。00:14以来所选内核/用户日志查询无匹配、baseline可读；不能据此排除没有匹配日志的其他应用退出。保留[查询原始状态](../../../runs/ros2_web_transport_20261029/crash-check-0320/recheck-20261030-0119.json)，历史Python脚本来源与系统OOM根因仍开放。

## I45：root预编译隔离候选，尚不进入默认路径

固定Three公有compile(root,camera,mainScene)+KHR完成轮询的白盒副本，11组受控检查/独立审查后使用完整Peach回放。10项只读、全URDF/TF/Marker/约8.6万显示点/800Path、RGB后切深度640×480、曲线600及图层/缺TF恢复/停止重启通过。首Scene uniform42次最大0ms、renderMax8.2ms/CPU p95约1.8ms；重启后累计59次最大0.1ms、renderMax8.7ms/CPU p95约1.5ms。旧1575.3ms未复现，仍没有配对A/B或冷GPU因果证明。

页面长任务仍2456ms/重启1311ms；候选准备passes67148（9864次提交）、重启23711（3337次提交），待准备root在不同快照为0/1/6/38，不把部分画面提交当作全画面就绪，也不把prepare调用都计成新program编译。高pass数来源仍待区分，白盒字段/逐帧扫描器不作为产品API。固定光照、无自定义材质回调是实验边界；原errors-only checker被候选完成检查替换，观察器和顺序效应明确保留。性能结论PARTIAL，产品仍0.6.20。

477.95秒人工收尾、峰值464.33MiB子树（不含浏览器），最小可用23009.30MiB；直接断开0订阅/0等待/三容器0，5PID创建时间与23088/24690清理，临时页关闭，原200文件哈希不变。完整反例/修订/真实阶段/局限见[本次终态](../../../runs/ros2_web_transport_20261030/root-precompile/peach-native/verification.json)和development.md I45；不替代原I36、历史OOM、设备LAN/Android验收。

## I46：codec毫秒级，仍有未归因阻塞

原生LoAF+三处实际adapter计时的203文件副本，在完整Peach上10项只读通过。字节计数最大2.3ms、CBOR0.8ms、JSON2.4ms、dispatch9.1ms、renderMax10.3ms；本次codec未出现秒级调用。45秒无CUA操作窗口仍出现约2.01秒帧，仅animate5.7ms被归因，最大保留blocking约1.70秒。页面visible且约55FPS；不能推断背景节流、没有JS或GC/GPU/工具独占因果，后台自动化活动仍可能存在。原生归因仅是平台入口，非最深函数。

守卫480.46秒正常退出，末快照已在结束后，longtask2.229秒、LoAF2.488秒/blocking2.178秒且scripts为空；这些不是在线末阶段或已修复证明。108帧已检查、0预算跳帧，保留8帧；平台buffer丢失未量测。峰值551.31MiB子树不含浏览器，余量最小23136.31MiB，PID/23188/24790/标签已清理；事后本地断开0订阅/0等待/三容器0且LoAF停止。状态PARTIAL，产品仍20，下一步HTTP-only无视图对照。见[实际归因与时序](../../../runs/ros2_web_transport_20261030/frame-attribution/peach-native/verification.json)，源码/失败/修复过程见development.md I46。

## I47：空载没有复现已观察到的秒级帧

同一203文件页只运行HTTP，未连接ROS、未创建图形组件，0Canvas/订阅/等待，wire记录为空，未加载3D/media包。两段45秒无页面操作窗口内，47个已检查LoAF最大687.5ms，0预算跳帧；保留top8的blocking为0，不能当作全部帧blocking统计。有限观察没有>=1秒交付帧，尚无正对照或平台buffer丢失证明；这不是根因修复，也不证明数据或图形造成此前全部卡顿。

服务360秒自动截止，期限内三快照保留，峰值24.05MiB仅HTTP、不含浏览器，可用最小23651.92MiB；PID/23288/24890/标签已清理，observer停止。原app定时器仍在，非无脚本环境。下一步只接收同一完整点云、无Canvas分组；不同流集合和DOM预览工作量必须明确，不将子集通过替代完整Peach验收。见[空载报告](../../../runs/ros2_web_transport_20261030/http-only-control/verification.json)与development.md I47。

## I48–I49：上下文恢复修正与0.6.21冻结

真实静态Scene在旧版恢复后保持成功帧2/geometry0、空白，新版原生恢复自动到3/geometry2；丢失不增加成功帧，真实历史保留。该修正清旧pending并请求重绘，未改变ROS协议或默认预热策略。独立新源地址验证，第一次缓存命中失败原样保留，不作新版验收。完整Peach正常回归10项只读、12资产、21TF、约8.6万显示点、RGB-D及曲线、3D停启与显式断开通过，268.68秒人工结束，非耐久门；五PID/四端口/临时标签全部清理。

首Scene RGB renderMax1011.9ms、深度1036.4ms，CPU提交p95约1.3–1.4ms；第二Scene renderMax50.9ms但页面longtask2364ms。秒级阻塞仍可见，不能据平均或p95宣布流畅完成。子树峰值465.66MiB不含浏览器，可用最小22604.33MiB，未验证完整模型的context故障注入、Android/LAN或整窗退出修复。源码包21已独立离线重建：97运行文件/54产物字节一致、4入口与35官方lint通过；历史19无观察器1800秒PASS仍不当作21长测。

下一步仍为同一完整点云的无Canvas接收对照，然后与图形/更多流分组比较；当前静态恢复修正没有替代这一步。原始失败、修复、两次审查、缓存例外、完整Peach及归档证明见development.md I48–I49与[本轮范围终态](../../../runs/ros2_web_transport_20261030/context-recovery/peach-native/verification.json)。

## I50：接收点云与完整观测的同页对照

冻结21只附加临时原生观察器，正对照80ms已交付。空闲连接70.316秒与两段完整点云无Canvas（69.976/68.164秒）均0报告longtask，已检查LoAF最大977.7/746.3/899.5ms且blocking0；完整Scene+RGB+Plot的84.728秒则3longtask/max1371ms、LoAF1554.7ms/blocking1317.9。两段点云均实际超过4MB每帧且计数推进，不是只读元数据假负载。该对照仍有不同流集合/回放相位/缓存/工具影响，不证明点云任意条件快速，未作逐帧视口/输入延迟门。

本次q16日志5条写队列溢出，有1条与完整阶段长任务同窗，无topic/client ID，不能反推丢弃对象或因果；默认64本轮未测试。普通Topic长度1加100ms节流会合并突发，录制116条中27个相邻接收间隔<100ms，不能按平均Hz小于10就承诺全帧。0lastError也不覆盖上游丢弃。具体计数/字节与阴性边界见[范围报告](../../../runs/ros2_web_transport_20261030/data-only-phases/peach-native/verification.json)。

548.08秒人工结束、五PID/端口/标签/观察器全清，峰值511.65MiB子树不含浏览器。状态PARTIAL，产品仍21。下一实验固定同九路流与组件，3D显示→隐藏→显示后再判断绘制/解码的差异；当前未修复秒级卡顿、默认配置流门或I36。完整过程与审查见development.md I50。

## I51：同九路数据隐藏几何后仍有阻塞

全显示→五层全隐藏→恢复显示，订阅始终9、Canvas3且尺寸一致，WS sentFrames恒9/服务器一次连接，实际drawCalls101→2→101、triangles150277→0→150277。数据/资源/图像/曲线继续更新，不是断流来降低绘制。74.086/72.116/157.941秒分别报告3/2/14个longtask，最大2163/1688/2001ms；对应LoAF最大2401.7/1797.7/2091.9ms。隐藏几何未消除秒级阻塞，不能只按三角形数量解释，未归因部分不命名为GPU/GC/浏览器问题。

六个端点累计Scene.renderMax均43.4ms，不能把页面longtask当成已捕获的render阻塞。窗口长度不同、回放/缓存/数据更替/后台工具与未测原生buffer仍有混杂。q16共36条节流写队列溢出日志，默认64未验、无话题归属，不作无损或改善百分比结论。

509.95秒人工收尾，五PID/两端口/观察器/25标签均清；峰值559.76MiB子树不含浏览器。状态PARTIAL，产品仍21。现有浏览器无Trace能力，独立Chromium/Playwright选项等待用户明确选择；先补公开task名称/容器归属，不能回填旧数据。详见development.md I51与[原始终态](../../../runs/ros2_web_transport_20261030/layer-control/peach-native/verification.json)。

## I52：公开归属仅收窄到当前页面上下文

冻结21临时observer增加LongTask的name及有界attribution，14组/语法与独立复核通过，产品运行源码不变。实际80ms按钮正对照与完整九路/三Canvas的8条在线长任务均报告self、unknown/window；正式端点间隔90.8431秒、包含一次45秒无CUA窗口，最大1847ms。18LoAF最大2001.9ms/blocking1794.8ms，最大三条scripts为空，Scene累计renderMax两端均53.1ms。不能从self推断最深脚本、GPU、GC或工具，页面自报UA亦非二进制版本证明。

360.17秒守卫结束，第二计划窗口未执行，不重启补齐；最终阶段包含后端退出与重连，只保留时间线，不能写成在线通过。0订阅/0等待/0Canvas/observer0，26标签及五PID/两端口全清；子树峰值445.23MiB不含浏览器，q16有12条写队列警告且不是精确丢弃数。仍PARTIAL，默认64与当前版本长测未验收。SeriesPlot每100ms执行dirty/setData且不在Scene计时内，可临时单测该缺口；不根据时间间隔直接改产品。过程及边界见development.md I52和[分开的在线/守卫后结果](../../../runs/ros2_web_transport_20261030/task-attribution/peach-native/verification.json)。

固定uPlot源码已核对setData采用microtask提交，主要重绘发生在后续_commit；公开draw钩子也不包含前面的比例/尺寸计算。后续计时须包含实际提交，不能仅凭setData同步耗时阴性就排除曲线，也不改用同步batch来测不同调度。

## I53：曲线实际提交很短，页面秒级长任务仍在

固定uPlot1.6.32临时bundle只给setData/setSize/_commit三体加try/finally，原微任务队列不变。8组与独立复核含真实queueMicrotask/Promise fallback的原/补对照；原版语句、调用/结果相同，无额外timer/RAF。source map对临时改动失配明确保留，不当作产品构建，产品仍冻结21。

完整Peach九路/三Canvas在线capture间隔62.3488秒，含45秒无CUA。实际setData与commit各增加484次，最大0.6/1.2ms、invalid0；setSize0表示未覆盖resize。同期新增10条longtask，最大2346ms完整落在两个在线端点内；累计22LoAF最大2605.6ms/blocking2295.4、最大帧scripts=[]。Scene累计renderMax35.7→45.8ms，实际曲线提交体没有对应秒级调用。这个范围不能排除返回后的浏览器布局/合成/GPU及其他未计时工作，也不能从未知部分定因。

139.17秒人工结束、observer/plot trace停、断开0订阅/0等待/0Canvas、27标签及五PID/两端口全清。峰值441.52MiB子树不含浏览器；q16有14条写队列警告，不是精确丢弃数。当前版本默认q64和默认RMW无XML多客户端持续流尚需补验，旧v19实验不替代。完整方法、失败与覆盖边界见development.md I53和[范围终态](../../../runs/ros2_web_transport_20261030/plot-commit/peach-native/verification.json)。

## I54：默认64/共用连接/无XML持续流仍失败

冻结0.6.21原普通安装，默认FastDDS、write_queue_size64、共用连接、无任何诊断或DDS XML，3健康SDK加慢读者共4WS。1800秒目标保持2秒RPC/3秒截止、10秒流门和1GiB子树守卫；381.95秒FAIL，567RPC中3超时、三个客户端各一次，成功p95约21–25ms不能作为通过。桥同CID响应等待超时日志在SIGINT前，缺native时间不能细分DDS/executor/WS，不加超时/重试或默认XML掩盖。

失败时三大流回调仍新鲜、TF完整；Node自身峰值173.71MiB、子树峰值749.67MiB不含桌面/浏览器、可用最小22367.20MiB，未触内存门。9次慢读暂停/8次恢复，第9次恢复与450秒首次重建均未达到；6次时钟回退、57条节流队列满警告非精确丢弃数。SDK释放、六PID/两端口全部清理。原195安装文件/97源/归档SHA不变，配置来源、校验误报修正及[FAIL结果](../../../runs/ros2_web_transport_20261030/default-core/long-shared-default/verification.json)详见development.md I54。

下一轮用同负载与门限的既有WAIT/被动服务端/native元数据诊断，作为显式诊断负载另列。当前默认配置没有30分钟通过，I36与浏览器秒级长任务等原门继续开放。

## I55：服务延迟阶段已收窄，仍未修复

相同q64/共享4WS/默认FastDDS/无XML，追加WAIT/native/被动server诊断，无主动对照或late-reader。RPC约458.509秒失败，原5秒取证宽限后守卫采样463.28秒；681RPC/2TIMEOUT，首次重建到达，第二次/1800未达。4520native记录零drop/error/超预算，681服务端事件对，所有自有进程和端口已清理。原97源/原195安装文件不变，额外Python缓存和输入检查编排偏离详列于development.md I55，不能当默认无观察器通过。

精确CID679/680及原生key/sequence生命周期匹配后，writer→reader history分别2999.555773/2999.193907ms，history→takeReturn wall约0.113/0.337ms；Node在3秒截止TIMEOUT，wire约3003.637/3003.616ms才收到成功回复。下面只表示这两个已取样响应的阶段，不是网卡抓包：

```mermaid
flowchart LR
  W[FastDDS writer写时间戳] -->|约2999 ms| H[reader history插入时间戳]
  H -->|约0.11至0.34 ms| T[native takeReturn]
  T --> F[Future完成与Event通知]
  F --> R[WebSocket迟到成功回复]
```

原生wall−mono变化3.185µs；同轮681的SDK约4.671ms、writer→history0.012600ms，是有限对照。客户端key全轮有复用，选中两例无歧义；Python/Node mono单位不同不混减，wall尾部为近似，server GID不按时间硬配CID。只定位主要延迟阶段，没证明发现/心跳/丢包/重传分支。证据、独立复核及下一步标准Client复用窄实验见development.md I55与[时间线分析](../../../runs/ros2_web_transport_20261030/default-service-native/long-shared-native/timing-analysis.json)。

子树峰值806.06MiB不含桌面/浏览器，66条节流队列警告非精确丢弃数。10次慢读恢复通过、7次clock回退不替代RPC失败；所有默认持续流、I36、浏览器卡顿与设备验收门仍开放。

## I56：单原生Client生命周期对照通过，仍非产品修复

临时普通安装副本只增加入口加载与固定/rosapi/nodes复用helper；q64/shared/default FastDDS/no XML、原录制/SDK/3秒截止及WAIT/native/被动server保持。正式1800秒单调循环完成，2688RPC零失败，native1key/序号1..2688，创建1/复用2687/逐调用保留2688/Node真实关闭1、pending0。native send→take p95/max约1.584/18.742ms，writer→reader history约0.260/2.059ms，wire约8.533/104.799ms；本轮没有I55两例近3秒段，但变量同时改变多个实体生命周期因素，不能据此确定发现根因。

两次客户端重建、40暂停/40恢复、30次clock回退覆盖，全部数据/最终TF/SDK释放通过，八PID身份及两端口清。资源最后样本1799.87仅为下界，完整时长由冻结Node循环证明。子树峰1007.10MiB距1024守卫仅16.90MiB，前期bridge/回放增长、后窗口大致趋平，未证明长期有界；211条节流队列警告不是精确丢弃数。产品97源/原安装/0.6.21归档未改，临时诊断通过不能替代无诊断默认配置、浏览器或Android验收。CPU13组、真实假服务短测、分析器失败修正、范围和原始证据见development.md I56与[终态](../../../runs/ros2_web_transport_20261030/native-client-reuse/long-shared-native/verification.json)。

## 独立 Chrome 原生性能录制（I58，PARTIAL）

用户已授权独立Chrome＋Playwright/CDP；实际Chrome152/Playwright1.62.1、1440×1000/DPR1、RTX3090/ANGLE，仍只加载录制Peach、q16/shared/default FastDDS/noXML。原冻结0.6.21的97运行源、54产物不变；主机重启后从同SHA归档恢复普通安装。新profile不等于冷OS/GPU缓存，跨IAB/主机重启/版本/采样分类的差别不能直接确定因果。

全分类32MiB trace触2GiB浏览器守卫，保留FAIL；4MiB三窗虽交互/清理完成，但全部loss=true，保留PARTIAL。进一步聚焦后的第三轮浏览器132.514s正常退出/峰1860.97MiB，暖运行90s后继续交互；12/12模型资源、21TF、约8.6万有界显示点、RGB-D/曲线、9订阅/3canvas以及停/重启/断开资源归零有效。页面最大长任务261ms，重启174ms，限定该窗口未复现秒级卡顿；启动及稳定原生窗口完整、交互仍有损。最长已采集scope不是完整最长任务；稳定26.503ms的FunctionCall含24.801ms V8原生interrupt边界，不能全归传输业务CPU。ProfileChunk有负delta，旧权重解释废止，只保留节点采样计数。

第三轮后端守卫在会话中断时丢失且没有最终resources，孤儿launch已按身份补救清理。因此浏览器正常终态不构成整轮PASS，当前[前端分析](../../../runs/ros2_web_transport_20261030/playwright-native/full-page-v3/frontend-analysis.json)为PARTIAL，通用报告FAIL_EVIDENCE。具体失败、源码/轨迹/资源/原始图片、审查与勘误见development.md I58及[计划](../../../runs/ros2_web_transport_20261030/playwright-native/plan.md)。历史IAB同步GL阻塞、默认持续流、内存、Android/LAN均未因此验收。

## I61：Service受管集成的功能结果，非持续流性能验收

当前开发树通用池已接入，尚未冻结新版本。最新普通安装的demo/SDK/数据往返15.74秒，子进程树采样峰723.48MiB；cache0原始对照15.73秒/724.36MiB，两个工作负载都是功能短测，不能据接近数值证明内存、延迟或吞吐无差异。它们包含SDK17项、五组实际观测/九参数写读恢复、两图与4.14MB云四个精确往返。没有真实浏览器renderer，不支持FPS/GPU或真实Peach效果结论。

初始Node18失败、cache0重复SIGINT的收尾FAIL、metadata夹具模式冲突、指纹漏识别及修复均保留于development.md I61。最终18个真实启动/参数Node用例、37官方lint、metadata四阶段/WAIT/native与cache0迟到reader通过；安装核心源码核对与未完成项见[汇总](../../../runs/ros2_web_transport_20261030/service-client-design/task2-current-verification.json)。新3秒wire/关闭竞态、Peach回放CLI透传、新版本冻结及原默认1800秒尚待完成；原I54/I36/浏览器轨迹/冷绘制/OOM/Android/LAN门仍开放。


## I62关闭根因与通信功能补验（未冻结新版本）

I61两轮“最终无残留”被误写成正常退出，原日志均5秒后TERM，graceful声明已撤回。原生栈与单发布者复现确认：ROS main返回后仍等上游默认10秒非守护注销Timer，进程约9.93秒可自行exit0。使用公有launch宽限15秒及性能CLI根PID单次INT/等18秒后，真实发布者launch约9.94秒正常退出，无escalation；原3秒RPC、RMW、QoS、q64保持。完整失败、更正、源码参考、红绿及边界见[development.md I62](development.md)，不能据此宣称任意关闭问题或历史OOM已修复。

正式timeout:3约3002.784ms得到同CID false、后续fast恢复、远端slow继续；排队/无限响应SIGINT均约0.142秒退出。新普通安装demo17项、9参数恢复、实际数据流/精确大帧及37项官方lint通过。录制回放三轮实际参数64/0/默认64、q64、JSON/CBOR各约10秒、TF完整，RPC无失败；外守卫子树峰726.62/676.14/714.01MiB。6组真实信号回归无外层kill；167个已观测PID身份与8端口最终全清。这是功能/退出短测，不等于默认1800秒、浏览器流畅或Android/LAN容量通过。

## 最终随包浏览器CLI

修正旧探针没有打开details#demo-tools的问题后，21/21真实浏览器场景通过；增加当前项目选择/清空、6只读与完整JSON、Action取消终态5及NOT_RUN7、填入表单、清空旧日志保留本次OK条目、390/768布局和普通LAN HTTP。该轮3GiB/300秒功能守卫下30.461秒、树峰2033.42MiB，无页面错误且全部清理；不是1GiB通信容量测或Android物理测试。两次新增探针断言错误和原CLI隐藏按钮FAIL均保留，见development.md I66。
# 0.8.2 工作台录制预览（2026-11-02）

最终 `runs/ros2_web_replay_ui_20261102/final-02/` 使用普通安装、独立有窗口 Chrome 152、1440×1000/DPR1、RTX3090 ANGLE/OpenGL，真实 Peach 录制、共享连接、默认桥队列/缓存64。120秒容量阶段覆盖4个同源云/8路空间/11路总显示、RGB-D、模型/关节/TF/Marker/Path/曲线，2次时钟回退和8类交互（含结束后的恢复），页面错误0。输入与录制哈希不变，末尾资源/端口清理通过；不是1800秒、Android、IAB或4路独立相机验收。

| 最终采样项 | 结果与范围 |
|---|---|
| 整套受测进程 RSS 峰值 | 2270.32 MiB，约2.22 GiB；进程RSS求和含共享页重复，不是浏览器独占或GPU显存 |
| JS heap 采样峰值 | 115.60 MiB，约每10秒采样，不是内存分配总量 |
| 点数采样峰值 | 344828；4个同Topic逻辑显示共享网络订阅，配置总预算400000 |
| CPU绘制提交 p95 | 每次快照最近120次提交为1.3..2.5ms；不是GPU耗时 |
| 离散事件报告 | interactionId>0共158项，已报告样本p95=64ms、最大312ms（重启显示）；阈值16ms、8ms量化，不是INP |
| 稳定阶段长任务 | 1项77ms；对应LoAF的WebSocket回调为66.6ms，不据入口归因细分解码/显示/GC |
| 稳定阶段长动画帧 | 35项，最大99.4ms；原生报告阈值50ms，不是35次业务失败 |
| 首次/恢复3D | 主线程任务97ms / 292ms；恢复时离散事件312ms。仍有初始化停顿 |

0.8.1对照 `baseline-02`、中间布局 `final-01` 和最终 `final-02` 均跑完120秒/2次回退，RSS分别2328.38、2339.68、2270.32MiB。主机/GPU缓存、回放相位与操作时序不同，不能用这些小差异声称优化了渲染或通信性能。本轮修的是图谱可读性和视角保持；未改SDK、ROS桥、图像/云解码、限额或依赖版本。

原生Event/LoAF/LongTask条目按开始时间归入阶段，最多6000条，最终三个通道均无探针容量丢弃；跨阶段项另计。不存在条目不等于GPU无卡顿。操作脚本的wallMs含Playwright等待和截图，不能替代输入延迟。浏览器检查期间未看到崩溃/持续断流，但没有关闭历史IAB/OOM或长期内存问题。记录见 [最终汇总](../../../runs/ros2_web_replay_ui_20261102/final-02/analysis.json)、[原始浏览器指标](../../../runs/ros2_web_replay_ui_20261102/final-02/browser/browser-performance.json)、[启动与手动检查](replay-preview.md)。

