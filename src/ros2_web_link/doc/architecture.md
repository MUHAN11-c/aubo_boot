# 架构与当前验证范围

## 当前源码入口

`package.json` 的六个 exports 分别对应 `sdk/`、`scene/`、`media/`、`audit/`、
`graph/` 和 `workbench/`；`web/` 是复用这些入口的独立工作台。
`scripts/build.mjs` 生成 ESM、声明、浏览器 bundle 和第三方许可，`CMakeLists.txt`
在已有 node/esbuild 时重建并安装 `dist`，不会隐式下载 npm 依赖。

`launch/web.launch.py` 组合 `bridge_node.py`、`rosapi_node.py`、`serve.py` 与
可选 demo；`replay.launch.py` 组合原生 rosbag2 观测回放。`serve.py` 使用 Python
`ThreadingHTTPServer` 提供静态文件、配置和白名单包资源，不是 Peach 的 8090
FastAPI 服务。Peach 只消费本包 SDK/bridge，自行定义业务映射；其接线见
[集成说明](integration.md#peach-当前接线)。

`colcon test` 注册官方 ament lint，`npm run check` 仅做 TypeScript 静态检查；
显式脚本各自覆盖的行为与运行条件见 [testing.md](testing.md)。下文带版本的
通过/失败记录保留原时间和负载范围，不能代替当前源码的新验收。

公有SDK声明中的传输预算位于无上游依赖的`sdk/limits.ts`；内部`BoundedTransport`继续使用roslib公有Transport API。六个exports的声明不向消费端暴露roslib内部类型，严格NodeNext/Bundler与浏览器打包由显式`npm run api`核对。此拆分不改变预算值、WebSocket协议或ROS行为。

DDS配置选择补充：检测到当前目录的`DEFAULT_FASTRTPS_PROFILES.xml`时，不注入本包默认XML，由上游处理已有隐式/显式配置；此时需要部署者自行加入所需恢复周期。已有显式环境配置同样优先。

0.8.3模型材质遵循RViz2：URDF visual显式声明材质时覆盖DAE/GLTF/OBJ自带材质；未声明时保留嵌入材质。显式alpha（含半透明）与具名材质仍生效，只对RobotModel应用，不改变Marker。替换时释放不再使用的旧材质/纹理，保留共享资源与几何；可用`npm run observe`中的材质资源检查及真实URDF浏览器用例核对。

0.8.3：桥默认使用上游逐调用Service客户端（cache0），不依赖缓存路径的x86原生指纹。web.launch仅给自有rosapi/demo服务进程设置config/fastdds_services.xml，按标准service profile把响应writer可靠心跳设为100ms；replay.launch仅给观测播放器设置config/fastdds_replay.xml，将播放器默认writer恢复周期设为100ms。ROS可靠性/持久性/队列深度仍取ROS/录制配置，不设置RMW_FASTRTPS_USE_QOS_FROM_XML，不切换RMW。已有FASTDDS_DEFAULT_PROFILES_FILE或FASTRTPS_DEFAULT_PROFILES_FILE优先，部署者需自行合并所需配置；空环境值沿用上游行为。其他RMW忽略Fast DDS XML。精确原生缓存校验作为显式非零缓存的限制保留，未增加重试、延长RPC或云恢复截止。实测范围与NX待测项见[性能参数](jetson-orin-nx-performance.md)。

0.8.2 增加图谱显示的布局/缩放控制，直接使用既有 Cytoscape 公有 API，不增加依赖。ResizeObserver 只在首次获得可见尺寸时适应画布，普通页签隐藏/恢复不重置视角；图谱筛选、快照和布局变化仍重新布局。长 ROS 名称支持字符换行。SDK、桥、场景/图像解码和 ROS 参数未变；回放测量只在探针里启用，见[回放预览](replay-preview.md)。

0.8.1只调整浏览器表现与关系图的视图筛选：`web/shell.js`以 URL hash 切换现有面板，全局连接面板始终可访问；`WorkbenchView`保留控制器与显示实例，用三个可键盘操作的页签组织空间、图谱和接口。切换视图不改变 ROS 会话、不自动暂停或恢复订阅。显示管理与数据视图分别滚动。图谱搜索保留匹配 Topic 的直接发布/订阅节点，或匹配节点的 Topic 与对端，不扩展无关 Topic。没有新增运行依赖，也未改变 SDK、桥、ROS 契约、默认 QoS、限额或执行权限。使用见[workbench.md](workbench.md)，核对记录见[testing.md](testing.md#081-界面核对)。

0.8.0新增graph与workbench两个独立入口：图谱采集/关系视图与显示配置/注册/生命周期/UI分开。保留原SDK、scene、media、audit，Python桥和核心SDK源码本轮未改；不用Fastify WS代理。共享Scene拥有唯一renderer/camera/TF/clock，逻辑显示拥有各自资源与订阅；禁用释放、导入后暂停、跨连接旧图带来源标记。内置8类显示，未知消息用raw或本地插件。使用与限制见[workbench.md](workbench.md)，接口见[io.md](io.md)。

0.8.0以真实ROS/桌面Chromium核对工作台8组、SDK17项、经典页21项，以及Peach录制120秒、4路云/8路空间显示、2次clock回退；37项官方lint通过。CPU/fresh-review复现与全部成功/失败原始证据在runs/ros2_web_workbench_20261102。分发重建报告单列于工作区artifacts/ros2_web_link/，不据旧版1800秒结果对新显示作长期性能保证。


0.7.1冻结源码已离线重建，四入口/普通安装/37项官方检查通过；本机默认q64/cache64/shared/FastDDS/noXML、无诊断的1800秒完整Peach录制负载通过，2691RPC零失败、两次客户端重建、40次慢读恢复、30次clock回退。标准预览共用ImageView既有预算并隔离迟到停止；10组真实表单/12项demo按钮/SDK17项及随包CLI21项分别通过。桌面Chrome真实项目10只读、模型/图像/曲线交互、无损轨迹及关闭profiler对照/LAN布局另列I64；Android本轮按用户指示暂不测。0.7.0原始证据保留，0.7.1生产代码之后只修开发探针/文档，交付绑定明确版本与SHA。冷/重启仍有长任务，不称GPU帧时、任意设备或历史IAB/OOM根因已修复。当前结果、内存口径和限制见performance.md及development.md I63–I66。

0.6.21已冻结源码包，修复WebGL上下文恢复与着色器诊断，静态真实WebGL恢复及完整Peach正常数据回归通过；后者没有对完整模型做context故障注入。97个运行源码/54个dist离线重建一致，35项官方lint通过。秒级长任务与整体性能验收仍开放；冻结证明见development.md I48–I49，后续诊断I50–I53未改产品运行代码，也不将局部阴性当作性能修复。

0.6.22冻结归档仅改变测试CLI及版本元数据，没有ServiceClientPool。0.7.0在I61–I62接入下述池；I62另修退出宽限和重复INT：bridge Node等15秒适配上游默认10秒发布者Timer，性能CLI根PID单次INT/等18秒，保留组兜底/原子报告和原上游主循环。SIGKILL/OOM/断电不能由Pythonfinally保证，独立测试另套有期限/内存的cgroup。先前只凭无残留的graceful声明已更正，见development.md I62。

I54的0.6.21默认负载在381.95秒失败：567RPC中3次超时，历史FAIL保留。I55收窄到writer→reader history延迟，I56临时复用实验通过；0.7.0现已有通用生命周期与无诊断默认1800秒产品证据，未宣称普适DDS机制根因已证明，也不关闭I36另一配置的彩图停顿机制。详见development.md I54–I56/I63。

网页显式断开先调用既有本地观测收尾：Scene/ImageView/SeriesPlot dispose、Topic与标准数据off、撤销标准图片Blob并清src、失效异步optional启动，再关闭控制/观测连接并同步刷新统计栏。实际页面函数12组CLI、只读审查及真实demo/Peach回放核对通过；直接断开后活动订阅0、图形/图像/曲线容器0。SDK disconnect的订阅注册保留及意外断线恢复契约不变，不发送远端Action取消。CLI与DOM证明不等于GPU堆测量。

压缩PNG/JPEG在创建Blob/HTMLImage前做编码字节和头尺寸预算检查，复用image-dimensions 2.6.0，固定magic门先于通用解析器；不支持compressedDepth/tiff。默认4百万像素与16百万编码字节，各组件预算不是浏览器整体内存限额。头读取库会复制一次编码数据，不能称零复制；它也不能验证全部像素内容或替代浏览器解码。详见io.md与开发记录I41。

0.6.20工作区原始编码表改用Map，拒绝constructor/__proto__等继承属性；数值数组逐项要求自身属性、整数0..255，拒绝稀疏或隐式截断。不改变TypedArray/Base64路径、合法编码或预算。15组图像边界、独立审查、35项官方检查及0.6.20真实Peach10项只读/单像素拒绝与恢复通过；0.6.19的无观察器长测仍单独列证，不能当作此修复的原生长测。

显示时间显式选择：SceneOptions.useSimTime默认false；true才使用/clock，首帧前为零，暂停不外推。独立TfBuffer的clockTopic默认false，需显式选择仿真时钟。Scene的时钟/Marker截止均用整数纳秒；只在显示时转秒，stats另给十进制clockNanoseconds。两份Peach回放配置显式开启，在线相机/demo关闭；这不会更改ROS节点参数，应用应同步配置原生use_sim_time。相对0.6.17不再自动跟随任意/clock，见development.md I38。

默认共用一个RosClient。页面可显式勾选“分离观测连接”，在同一rosbridge URL上用第二个现有RosClient承载Topic/TF/3D/图像/曲线；Service、参数、Action和图谱仍在控制侧。audit通过RunOptions.observationClient接受调用者拥有的观测连接，不创建或dispose它；mockGuard仍从控制侧验证。双侧任一断线终止当前用例，取消/参数恢复继续走控制侧并确认结果。页面在首次双侧握手完成前禁止新发送，保留取消和断开。每连接预算独立，总计不是新的全局上限。该组合不隔离同一桥的事件循环/DDS，也不保证完整大数据流持续交付，参见development.md I35–I36。

SceneOptions.profileInteractions默认false；启用时用标准PerformanceObserver观测页面级离散事件，只保留120个数值样本。performanceStats().eventTiming明确报告支持状态、阈值、样本范围和局限，dispose断开observer。它不影响ROS连接，也不代表INP、连续拖动或GPU计时。

3D计数代表CPU绘制提交，不能证明GPU执行或屏幕呈现。SceneView在提交前后检查公有WebGL isContextLost；丢失时记录绘制错误，不增加成功帧数或首次提交时间。恢复后继续使用原Three恢复机制；0.6.21监听lost/restored，清旧pending并触发静态重绘，错误历史保留，dispose移除监听。详见development.md I29/I48。

回放launch的bridge_write_queue_size与性能CLI的--write-queue-size可显式透传上游write_queue_size。省略时不遮蔽web.launch原默认64或部署方已有的同名全局配置。该限额按消息数，非字节保证；同一连接各类回复共享队列，不能把减小观测缓存等同于控制回复交付保证。容量实验与默认部署分别记录，详见development.md I30。

```text
任意业务页 / 本包的单一自测页
  ├─ RosClient + roslibjs → WebSocket → rosbridge + rosapi → ROS 2 DDS
  ├─ 可选 audit：项目数据契约 → 所选用例 → 原生接口证据 / 可取消交互与收尾
  ├─ 可选 media：ImageView / SeriesPlot → 观测 RosClient（默认共用）
  └─ 可选 scene：SceneView / TfBuffer → 观测 RosClient（默认共用）
Python HTTP：静态文件、/config.json、只读 ROS 包资源 /assets/<package>/...
```

核心通信模块不导入 DOM、Three 或 uPlot。3D 和图像组件由页面按需加载，各自拥有订阅、定时器和图形资源，`dispose()` 释放。业务页可以仅使用核心 SDK。没有 Fastify/WebSocket 代理、运行期 Node 服务、CDN 或云依赖。Node 22+ 用于构建和显式CLI验收；依赖安装为显式 `npm ci`，此后 `colcon build` 自动重建本地网页。

请求生命周期使用单调时钟截止时间与幂等完成函数；定时器仅负责唤醒清理。主线程忙碌后收到的过期响应/反馈或处理期间跨过截止时间的结果不能成功完成。本地等待结束不影响远端任务的实际运行，远端取消仍需显式请求并等原生终态。

每个客户端以Map关联请求ID，Service/Action共用maxPendingRequests预算（默认256、范围1..4096、构造后不可变）。满额在事件/定时器登记和发送之前拒绝，不排队；响应查找不再复制/遍历所有等待。读取外部选项后检查容量，注册异常与所有结束路径均回收名额。计数预算、线帧字节预算、观测组件缓存分别生效，不代表进程总内存保证。

## 组合与权限

`web.launch.py` 是统一入口；`self_test.launch.py` 等价于 demo=true/profile=demo，`bridge.launch.py` 是兼容转发入口。launch 仅组合上游桥、rosapi、静态 HTTP 和可选无硬件示例；任何已启动的关键进程退出都会关闭该组。默认 demo=false、localhost，不固定 ROS_DOMAIN_ID、不连接硬件、不发送目标。应用组合时通过 config/profile 和资源白名单选择观测接口，不引入 Peach/IVG 业务依赖。具体参数见 [launch.md](launch.md)。

服务/发布/Action 白名单默认只允许示例命名空间；订阅读取默认开放。上游会向 services_glob 自动追加 /rosapi/*，本包固定 rosapi params_glob=[]，拒绝旧参数入口，标准参数仍走节点自己的 rcl_interfaces 服务。白名单不能代替应用节点的执行权限门。普通 HTTP/WS 供受信局域网操作，地址由操作员显式选择。

## 核心通信

roslibjs 2.1.0 负责Topic及原生操作事件；其公开Transport接口接native WebSocket与同版cbor2成熟解码器。sdk/transport.ts仅补上游私有分片Map缺少的TTL/字节预算、发送积压拒绝与安全路由，限制完整帧/分片内容/ID/片数；不改rosbridge协议或自写CBOR/CDR。SDK另补显式QoS、请求清理、原始Action终态、ROS 2参数、图谱/接口定义及原生生命周期服务。每次连接创建新的ROSLIB.Ros；断线只恢复仍登记的订阅，绝不补发命令。超时/AbortSignal只终止本地等待，远端结果可能未知。预算与错误码见[io.md](io.md)。

本版本需保留窄兼容点：通过公开 `callForSubscribeAndAdvertise` 钩子追加 Topic QoS；清理上游 advertise 在 reconnect_on_close=false 时遗留的匿名 close 监听器。Service/Action 使用公开收发事件保留关联 ID、完整数值终态。`bridge_node.py` 在进程内复用上游 octet→int 转换，修正 Jazzy rosbridge 2.7.1 非空 byte-array 参数响应无法 JSON 序列化的问题，随后 runpy 执行原桥。rosapi_node.py 另将本机不可用的Action图谱Node方法调用替换为公开rclpy.action.graph API，防止action_type查询使rosapi退出。无系统包修改或第二个协议服务器；升级时复核并删除已修复兼容点。

## 观测

自测页的Topic/图像表单沿用项目TopicOptions；结构化QoS通过有界的可见select选项保留到SDK，未指定时与audit的sensor/none/0一致。组件的独立默认选项保持原值。Peach录制图像在项目数据中显式Reliable，仅针对已验证录制与本机环境，不改变通用sensor或在线Peach配置。

- Three 负责 WebGL/OrbitControls；URDFLoader 负责 URDF 拓扑、网格与关节变换。JointState 更新测量值，不写关节命令。米为网格单位，Z 向上。资源走静态服务器包白名单；支持 STL/DAE/OBJ/glTF/GLB，不在浏览器执行 xacro。
- 每个 Scene 的资源失败摘要按最近顺序去重，最多 32 项、每项最多 1024 字符；新错误替换最旧摘要，状态文本明确记录范围。模型 generation 隔离与资源释放路径保持，错误诊断不会随不同失败 URL 无限增长。
- JointState位置缓存只保留当前URDF的自有关节；模型前到达或属于其他模型的名称单独以LRU保留最多1024项、单名最多1024字符。替换模型时过滤旧名并应用匹配的早到值；空position或非有限值不覆盖测量位置，断线/停止清缓存。该策略限制长期多模型共用话题的内存与每次setJointValues遍历，不改变ROS消息或发布任何关节命令。
- 着色器默认errors-only：关闭Three成功路径的详细日志查询，每个新链接程序仍检查LINK_STATUS，失败日志通过onStatus报告并保留到组件销毁；ROS数据reset不会清除仍被网格/坐标轴使用的程序错误。verbose保留Three详细日志。上下文方法不能挂接时退回verbose，错误由Three控制台报告；不修改全局WebGL原型。profileShaders显式开启调用计时，默认不查询GPU设备信息，dispose恢复本上下文方法。详见io.md；同步GL查询导致的首载长任务仍未解决。
- 静态TF采用reliable/transient_local、depth100接收多发布者；JointState与曲线统一sensor QoS以共享订阅。TF 树、组合和插值复用 Lichtblick v1.30.0 的 7 个必要源码文件（保留 MPL-2.0）。适配层负责订阅、10秒历史、最多256帧/每帧500样本；动态数据按精确时间查询，latest 使用共同可用时间；缺失 TF 时隐藏对象并显示原因。显示边界的零时间戳用latest；非锁定对象首次定位后保留fixedFrame位置，避免历史淘汰使永久Marker消失。/clock 回退清动态历史并保留静态，断线清全部会话数据；乱序 TF 不等同于时间回退。
- MarkerArray 处理类型0..11、namespace/id、ADD/DELETE/DELETEALL、lifetime、frame_locked；最多1000个 Marker、每条10万顶点。基础颜色/透明度及列表 RGB 已支持；逐点 alpha、嵌入材质选项等尚不等同于 RViz 全显示语义。Path 是 nav_msgs/Path 的位置折线，使用 Path 共同 header；不是 MoveIt 轨迹规划/关节轨迹回放。
- PointCloud2 按字段类型、offset、row_step、大小端解析；支持 packed/分离 RGB，跳过 NaN，最多16MB输入/10万显示点。有界采样，不把完整深度点云当作浏览器控制链。
- 图像支持 PNG/JPEG 和 rgb8/bgr8/rgba8/bgra8/mono8/8UC1/16UC1/32FC1，遵守 step 与大小端。16UC1 单位必须由调用者明确给出；32FC1 按米。无效深度为黑色，默认最多400万像素/16MB。compressedDepth 特殊编码不支持。
- uPlot 负责曲线；最多8字段、默认600样本，100ms合并绘制，header时间优先，否则标记浏览器接收时间；时间倒退清曲线。不执行字符串表达式。

图像/点云/曲线为观测，无运动授权或硬件急停功能。当前是轻量自测与复用组件，不是完整 RViz/rqt/Lichtblick 插件平台。生产相机带宽、Android触控/GPU、特定DAE/GLTF模型材质仍需应用现场验收。

## 示例与来源

示例使用 rclpy Node +原生 LifecycleNode + MultiThreadedExecutor；长服务/Action 使用 ReentrantCallbackGroup，取消能并发处理。所有观测与 TF 在 /web_link_demo 下；自生成PNG、深度、点云、两关节模型，无硬件、numpy、相机、规划器依赖。示例生命周期不承担生产 bond 管理。

参考本机 Jazzy launch/rclpy/rosapi 与标准消息源码、[rosbridge 协议](https://github.com/RobotWebTools/rosbridge_suite/blob/2.7.1/ROSBRIDGE_PROTOCOL.md)、[URDFLoader](https://github.com/gkjohnson/urdf-loaders/tree/v0.13.1/javascript)、[Lichtblick](https://github.com/lichtblick-suite/lichtblick/tree/v1.30.0)；采用 RViz/rqt 的标准接口、按需订阅与独立显示组件思路。Rebot-Arm 的模型/关节/资源加载功能用于核对；不复制其旧协议客户端与运动入口。


## 项目验收与录制数据

`audit/index.ts`独立导出选择式检查器，与页面共用RosClient。项目差异在config/projects.yaml：Peach在线17只读+3交互，Peach回放10只读；未执行/取消/失败独立计数。项目配置不import业务实现；在线mock动作先验证真实robot_description及应用使能，finally确认恢复。参数往返同样恢复并读回。

每项验收使用 `performance.now()` 的绝对截止时间；等待返回、完成判定与新命令发送前复核，避免消息解码阻塞主线程后，过期证据先于定时器回调而被判通过。Action 的 afterSetup 完成及 Goal 数据替换后再次检查，超期不发送新 Goal。取消确认、参数恢复和 cleanup 仍使用各自既有的 5 秒收尾预算；验收截止不取消这些必要收尾，也不表示实时停止远端任务。

`replay.launch.py`复用Jazzy rosbag2播放器、原Web launch和项目数据契约；不启动相机、机械臂驱动、Peach业务节点或demo。仅播放项目topic检查及view引用的观测话题，发布/clock，静态TF保留depth100；桥禁止发布和Action，服务仅rosapi图谱。必须使用独立ROS_DOMAIN_ID，不能与现场TF/JointState发布者混用。

录制源时间保持原值。TopicSpec.timestampMode=recorded只验证消息到达、数据结构/解码和段内频率，并在报告中注明不检查在线新鲜度；默认live的时钟/新鲜度门不变。/clock回退清动态TF、曲线按header回退清样本。回放不能证明原设备当前在线、服务可用、参数写入成功、Action或运动执行。


## 大型数据与可移植分发

v0.4仍使用同一roslib/rosbridge标准协议；Image和PointCloud2通过CBOR保留二进制uint8视图，减少Base64转换和主线程拷贝。限频/编码是每个组件的显式TopicOptions，项目配置可以覆盖。SDK、显示、audit和项目数据保持分层；未迁移IVG/Peach业务。IVG的maxHz与发送缓冲限制用于审查参考，不复制其项目协议映射或FK/IK路径。

npm包显式包含dist、类型声明、自有源码、供应商许可证和ROS配置源码；可以本地npm pack后安装到其他Web工程。私有标志只防止误发布，离线文件安装不受影响。ROS端仍用colcon安装本包，静态资源与bridge直接组合。

## 可选标准服务诊断

性能CLI的--trace-services只在隔离只读测试中启用Jazzy服务METADATA，并加入原生DDS对照；默认不改Node工厂、不注册诊断信号、不产生额外请求。记录服务阶段与有界线程栈，失败仍保留未知结果、不自动重试；不增加HTTP业务接口。RMW由部署方标准环境决定，本包不强制FastDDS或Cyclone。当前默认长测与历史失败分别列证，见performance.md。

互斥的--trace-waits不增加ROS实体：自有桥仅在显式WEB_LINK_WAIT_SERVICE下，为一个服务记录取消前的Future/Event状态。上游源码哈希不匹配则拒绝专用诊断，不修改默认通信；不持有原生对象或业务载荷。历史环64条、失败最多8次写入、每条128KiB；写盘及二级日志失败不覆盖原调用结果。状态采样非原子，取消本身也会通知Event；只据此缩小故障阶段，不当作DDS丢包或根因的证明。使用与边界见testing.md。

--trace-waits可追加--trace-server：仅rosapi使用标准METADATA，collector被动订阅，不增加主动对照查询或客户端introspection发布者。它会增加服务端事件发布者与观测节点，属于另一种显式诊断负载。客户端/服务端标识不强行配对，服务端发送事件不代表客户端交付；失败保留原判定和5秒取证收尾。默认模式清除未选择的诊断环境。

0.6.8的WAIT正常退出另保存最后64条完成调用时间线（128KiB/一次尝试），获取环锁50ms失败明确标记不可用；文件IO仍依赖父进程收尾。CLI原生WebSocket诊断只保留小型服务头，不替换原生实现，使用Node AsyncLocalStorage关联异步会话。24000条/8MiB元数据先缓存，SDK释放后flush，避免同步写盘进入RPC截止窗口；默认不启用，不进入浏览器/公共SDK。信号/系统中止且未执行finally时缓存可能丢失。详见development.md I23。

0.6.9可再显式追加--trace-native，观察精确本机rclpy绑定的send_request/take_response边界，复用WAIT线程局部CID。只记录原生Client标识、序号、头是否存在及时间，不读原生内存、增实体或回调；独立24000条/8MiB缓存、退出一次写，缺证/丢记录使诊断验收失败。观察面为版本限定私有API，默认路径不依赖它。所有benchmark必须原子创建新证据目录，拒绝复用，避免旧快照掩盖本轮缺证。详见testing.md和development.md I24。

0.6.10在成功take元数据中加入RMW的source/received纳秒字符串；FastDDS的received指reader历史插入，不能当作网卡时刻。0/负值和未知时钟不解读为交付时间，取样仍不改变原生结果。这里只补取证，不修改默认RMW、QoS、超时或executor。详见development.md I26。

历史0.6.11曾有默认关闭的--trace-late-client诊断（0.8.3已归档并在启动前拒绝）：只在原waitFalse/取消后，把原reader再保留2秒以观察迟到响应，本进程最多3次。正常清理立即直通，延期后仍经原executor调度，不直接操作handle；Timer期间临时保留Node/Client引用，退出有界取消/记录，原3秒判失败和不重试不变。该模式改变超时后的生命周期，不能当作原行为的性能验收或修复；原WAIT/native模式仍不保留Client。见development.md I27。


显式性能CLI可用--separate-control以两个现有RosClient隔离每连接的RPC/观测出站队列；同一rosbridge、同一事件循环，未新增SDK门面或产品UI。默认单连接不变，短测不等于长期可靠性修复，见development.md I32。

原始ImageView将decodeImage新建的普通ArrayBuffer RGBA视图直接交给标准ImageData，不再额外克隆整帧；不复用ROS输入缓冲或已提交的像素数组。这是应用层一次复制的移除，不是Canvas内部零复制保证。

## Service 调用预算与可选客户端缓存（0.8.3）

`bridge_node.py` 复用已核 rosbridge 2.7.1 的主循环、Tornado、协议、权限检查和消息转换。受管节点的 `call_service` 在创建工作线程之前经过同一个 `ServiceCallBudget`；所有 WebSocket 连接共享 `service_client_max_in_flight`，默认 128。超额以原生 `service_response/result=false` 明确拒绝，不排队、不重试。普通 ROS Node 不受该适配影响，Action 也不计入这一 Service 限额。

`service_client_cache_size` 默认 0。该路径逐调用创建客户端，复用上游类型/消息转换函数，通过公开 rclpy `executor.create_task` 在所属执行器创建、销毁实体。创建等待采用独立单调时钟期限，默认 5 秒；已超时但尚未消费的任务继续占名额，执行时跳过创建。已经进入工厂的迟到结果在所属执行器销毁；工厂可能已经注册实体后才抛错，或销毁未能确认时，保留隔离名额，直至正常停止并重建节点。预算是逻辑工作与排队实体操作的上界，不是进程内存或所有原生分配器的硬上界。

非零缓存仍使用原 `ServiceClientPool` 和原生兼容门；有效缓存容量为 `min(service_client_cache_size, service_client_max_in_flight)`。缓存实体/待消费票有独立槽位上限；公共工作线程准入与缓存槽位是两个限制，不承诺二者数量相加仍小于单个限额。默认 64/128 的显式缓存配置保持原有效容量，64/2 的配置最多使用两个缓存槽位。

Node 关闭先关闭预算和池、唤醒创建/响应等待者，再执行标准 `Node.destroy_node`。每次调用清理自己的 Future，不取消远端服务执行；响应 timeout<=0 沿用无响应期限语义，仍占共享名额且可被本地 Node 关闭唤醒。缓存路径保持原互斥回调组、owner 退役确认及工厂异常处理，不绕过已审 rclpy 7.1.12 源码/二进制指纹。无缓存路径使用公开 API，不依赖该 x86 原生缓存指纹；rosbridge 源码布局仍限定为已核 2.7.1。

WAIT/native 元数据诊断识别缓存、无缓存两个响应等待位置，默认关闭。旧 `--trace-late-client` 会刻意保留超时 reader，属于归档复现路径；0.8.3 在启动前明确拒绝该模式，避免将改变寿命的实验混入当前预算验收。历史证据与脚本保留，不将其当作当前性能证明。

边界探针为 `scripts/service_budget_probe.py`；真实多连接准入探针为 `scripts/service_budget_wire_probe.mjs`。它们是显式行为检查，不加入 `colcon test`。本轮安装、长测与资源结果见[性能参数文档](jetson-orin-nx-performance.md)。
