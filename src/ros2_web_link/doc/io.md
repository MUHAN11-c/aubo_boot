# 公有接口与语义

0.8.3模型材质遵循RViz2：URDF visual显式声明材质时覆盖DAE/GLTF/OBJ自带材质；未声明时保留嵌入材质。显式alpha（含半透明）与具名材质仍生效，只对RobotModel应用，不改变Marker。替换时释放不再使用的旧材质/纹理，保留共享资源与几何；可用`npm run observe`中的材质资源检查及真实URDF浏览器用例核对。

0.8.2：`GraphFilter.layout` 可选 `breadthfirst`（默认）或 `grid`；`GraphView.setZoom(level)` 接受有限正数，限制为 0.05..4，围绕可见画布中心缩放。`stats()` 增加 `zoom/layout`；构造选项 `onViewportChange(zoom)` 可接收缩放变化，dispose 解除监听。只影响本地图形，不发 ROS 指令。

0.8.1 的界面整理没有新增 ROS 接口或改动通信参数。全局连接设置复用原连接，页面/页签切换不发命令；图谱过滤只影响已采集快照的显示，不修改节点发现结果或导出内容。

0.6.19自测页“断开”先释放本地观察组件/订阅/标准图像Blob，再关闭双连接并同步刷新计数，状态显示停止；不会调用远端Goal取消。SDK公有disconnect仍保留订阅注册，意外断线恢复行为不变。12组页面函数、独立构建35项官方检查及真实demo/Peach回放直接断开验证通过，范围见development.md I40。

## 浏览器 SDK

0.8.0 在包根 SDK、scene、media、audit 之外增加 graph 和 workbench；核心 SDK 不引图形依赖。完整操作与复用示例见 [workbench.md](workbench.md)。

- `new RosClient({url, reconnect=true, retryMs=1000, connectTimeoutMs=5000, maxPendingRequests=256, transportLimits={}})`；`connect()` 等待 WebSocket 就绪。
- connectTimeoutMs按单调时钟检查打开事件被SDK观察到的时间；过期事件或过期后尚未创建的连接尝试走原失败/重连策略，不能误报连接成功。
- `onState(callback)`、`onStatus(callback)` 返回注销函数。`connected` 是连接状态；`stats()` 提供活动订阅数、发布者数、等待请求数与监听器数，以及配置值pendingLimit；清理时只检查资源计数归零，不要求配置值变为零。
- `disconnect()` 停止自动重连，保留订阅登记以供手动 reconnect；`dispose()` 永久释放实例。
- `subscribe<T>(absoluteName, messageType, callback, options)` 返回幂等的取消函数。callback 接收 `(message, {receivedAt})`。支持 `qos`、非负 `throttleMs`、`compression: 'none'|'cbor'`。同一客户端同一话题只允许一套相同类型和选项。
- `publisher<T>(absoluteName, messageType, options)` 返回 `{publish(message), dispose()}`。在线时声明发布者；DDS 发现需要时间，首次 publish 前应等待接收方发现。publish 成功表示交给 WebSocket，并不是接收方处理确认；可靠业务命令优先 Service/Action。
- `callService<T>(name, serviceType, request={}, {timeoutMs=5000, signal}={})` 返回响应 Promise；长服务应显式提高 timeoutMs。桥端与客户端都设置超时，单位 API 为毫秒，协议为秒。
- `sendGoal<G,F,R>(name, actionType, goal, {timeoutMs=60000, signal, onFeedback})` 返回 `{id,result,cancel}`；result 为 `{status,successful,value,error?}`。`cancel()` 仅请求取消当前会话内的这一目标；不提供危险的 cancelAll。终态 4=成功，5=取消，6=中止；0 是未知/拒绝/桥接失败，error 保留上游原因。发送 Goal 不等于已被接受。
- `parameters(node)` 提供标准 `list(prefixes=[],depth=0)`、`describe(names)`、`get(names)`、`set(items)`、`setAtomically(items)`。写入项 `{name,type,value}` 使用 ParameterType ID；写入结果逐项返回，不把 successful=false 当成成功。不跨节点事务、不自动写 YAML。
- `interfaceType(kind,absoluteName)` 读取 topic/service/action 实际类型；`describeInterface(type)` 返回 message 或 request/response、goal/feedback/result 的完整 rosapi typedefs。桥环境必须已安装类型。
- `lifecycle(node)` 提供 `getState/availableTransitions/change(id)`，直接使用 lifecycle_msgs 标准服务，支持 RequestOptions；转换顺序和授权由应用负责。
- `watchParameterEvents(node,callback)` 订阅并筛选 /parameter_events，返回注销函数，不自动写回。
- `graph()` 通过 rosapi 按需查询 topics/types、services、nodes、action servers，不轮询图谱。任意标准/自定义接口可直接使用通用 Topic、Service、Action API，无静态消息生成依赖；接口类型必须已安装在 rosbridge 的 ROS 环境。

SDK 使用 ROS 2 规范名称 `/包/节点或接口`，类型采用 `pkg/msg/Type`、`pkg/srv/Type`、`pkg/action/Type`。不做节点内相对名称、`~` 或 namespace 隐式解析。

## QoS 与参数边界

四种内置 QoS：`reliable` = keep_last/10/reliable/volatile；`sensor` = keep_last/5/best_effort/volatile；`latched` = keep_last/1/reliable/transient_local。允许显式 `{history:'keep_last',depth,reliability,durability}`；不暴露 keep_all、deadline、lifespan 等未经本包验证的策略。传感流选 sensor；动态 TF 选 volatile；静态 TF选 staticTf（keep_last/100/reliable/transient_local，多发布者历史），持久配置选latched。桥接 QoS 是 ROS 订阅端的配置，WebSocket 仍是 TCP。跨客户端共享的桥接订阅仍受上游合并策略约束。

参数 ID：0 NOT_SET，1 bool，2 integer，3 double，4 string，5 byte[]，6 bool[]，7 integer[]，8 double[]，9 string[]。byte[] 的 Base64 由 codec 还原。NOT_SET 使用 undefined；声明的非动态参数可能拒绝删除，由节点决定。类型已知的整数参数只允许 JavaScript safe integer 范围，禁止默默舍入；有限大 double（例如 1e20）仍合法。通用消息/服务的 JSON 数字遵循 JavaScript 精度，无法在没有字段 schema 时区分 double 与 int64；应用不得依赖通用 JSON 接口保真传递超出安全范围的 int64，若需要应另行扩展编解码并验证。

## 失败与释放

ClientOptions.maxPendingRequests默认256，可设整数1..4096，构造后只读。Service和Action共用当前客户端的本地等待名额；每个请求/目标占一项，反馈和发出cancel请求不释放名额，收到终态、超时、Abort、发送失败或断线才释放。满额报REQUEST_LIMIT且未发送：callService返回拒绝的Promise，sendGoal与离线/非法参数一样同步抛错，不返回GoalHandle。没有请求队列、重试或预留控制名额；该上限不是整个进程的字节内存上限。stats()返回pending/pendingLimit与原有资源计数。graph()内部并发四个请求，需要四个空闲名额；describeInterface的srv/action查询分别需要两个/三个。复合查询中已发送的请求不会因其他请求满额而自动取消。

signal读取/形状校验在登记前完成，避免把AbortController误当Signal或属性getter抛错后遗留等待。监听器注册失败会回滚；清理方法异常也先移除关联ID和ROS事件，再完成Promise。使用真实AbortSignal，清理/超时不发送远端取消。

Service/Action的timeoutMs是本地请求等待预算，v0.6.3使用performance.now单调时钟的绝对截止时间。接收响应/反馈/相关错误时及成功完成前都检查截止时间，避免GL初始化、解码或同步回调阻塞主线程后，迟到消息先于超时定时器执行而误报成功。超时之后不再投递该等待的反馈，晚结果忽略并计数；仍不重试、不自动取消远端。浏览器忙碌时无法保证准时运行超时回调，这不是实时停止通道。

`DISCONNECTED`：未发送或断线后结果未知；`REQUEST_LIMIT`：本地等待满额，本次未发送；`TIMEOUT`：本地等待结束，远端结果未知；`WAIT_ABORTED`：AbortSignal 结束本地等待，未取消远端任务；`SERVICE_FAILED` / `BRIDGE_ERROR`：上游错误；`DISPOSED`：资源已释放；`SESSION_CHANGED`：新会话不能取消旧 Goal。

Service 没有通用取消协议，Action 的 signal 也只结束本地等待。退出页面不自动取消目标；业务应用应在重连后通过自己的状态接口核对当前任务。SDK 不自动重发任何发布、服务、参数写入或 Action。订阅恢复与请求事件清理分别处理，迟到响应不会覆盖已结束的等待。

## 传输预算与故障隔离（v0.5）

`TransportLimits`按字段覆盖，默认值也由`DEFAULT_TRANSPORT_LIMITS`导出：

- `maxMessageBytes=16000000`：单个JSON UTF-8或CBOR帧，以及重组JSON内容的最大字节数。
- `maxFragmentBytes=32000000`、`maxFragmentMessages=8`、`maxFragmentsPerMessage=4096`：全部未完成分片内容、消息ID数和每条片数上限。
- `fragmentTimeoutMs=30000`：从首片起的固定截止时间；重复片、其他ID和主线程定时器延迟均不能延长。
- `maxSendBufferedBytes=16000000`：native WebSocket bufferedAmount加本次序列化消息必须在预算内；超限不发送、不另建队列、不自动重试。
- `maxCborDepth=64`：cbor2解码的嵌套深度。字节预算至多允许单消息/发送256MiB、分片合计512MiB；ID至多1024、每条片数65536、超时300秒、CBOR深度512。非整数、未知字段和无效对象均在连接前拒绝。

`transportStats()`返回当前或最近会话的收发帧数/字节数、忽略的未知响应数、分片ID数/字节数、socket发送积压、实际预算和lastError。网页“传输预算与连接统计”展示同一公有API；预算不是整个浏览器或进程的内存上限。浏览器在onmessage之前已经接收并物化帧，SDK不能代替服务端限频/队列或OS级资源隔离；CBOR解码及场景/图像还有各自资源上限。

普通Topic当前通过roslibjs使用rosbridge `queue_length=1`；transient_local TF单独取所选QoS depth以保留静态突发。该服务端订阅队列和DDS history是两层：配合throttleMs可合并突发，Reliable只表达DDS层可靠性，不承诺浏览器逐帧无损。rosbridge全连接write_queue_size满时还可丢任何共享回复；2.7.1仅节流日志，不带topic/client ID，SDK transportStats.lastError没有这项计数。测试应同时保留服务端日志与源录制节拍/逐帧证据，不能以平均Hz、消息数增长或lastError为空替代连续交付验收。I50实测q16出现同窗警告，不代表本轮测试过默认64。

收到超大帧、非法分片/路由、超分片预算或过期缓存，会报告`MESSAGE_TOO_LARGE`、`INVALID_FRAGMENT`、`INVALID_FRAME`、`FRAGMENT_LIMIT`、`FRAGMENT_TIMEOUT`等并关闭会话；在途请求失败且远端结果未知。丢片超时可按reconnect策略恢复订阅；确定性协议/预算错误停止自动重连，修正配置后手动connect。未知或已结束请求的响应被忽略并计数，不会路由成Topic或连接事件。

本地发送拒绝报`SEND_BUFFER_FULL`或`MESSAGE_TOO_LARGE`，未发送该消息。订阅/发布者初次发送拒绝时回滚其登记；取消订阅/发布者释放若因积压无法发送注销，仍释放本地资源，报告`CLEANUP_SEND_FAILED`并断开本会话以结束桥端连接资源。用户Topic/state/feedback回调即使抛出null等值，也只报告`CALLBACK_ERROR`，其他消费者继续运行；调用方仍应避免耗时同步回调。

标准JSON和CBOR复用JSON.parse/cbor2 2.4.1，保留ROS字节与数值TypedArray；分片按标准JSON协议重组。协议层PNG包装和BSON不属于本SDK支持的编码；sensor_msgs/CompressedImage的PNG/JPEG内容不受此限制。上游roslib的私有无TTL分片Map不再用于此客户端。

## Launch 与示例接口

统一 `web.launch.py` 参数（`bridge.launch.py`兼容转发）：address=127.0.0.1、web_port=8088、ws_port=9090、web=true；topics_pub_glob=/web_link_demo/*、topics_sub_glob=*、actions_glob=/web_link_demo/*。services_glob 显式配置 /web_link_demo/* 与四个 SDK 图谱查询，但上游仍会自动追加 /rosapi/*。rosapi params_glob 固定为空列表，在该节点中拒绝旧参数入口读写（返回 successful=false）。glob 使用上游单引号列表字符串，例如 `services_glob:="['/rosapi/topics', '/my_node/*']"`；当前上游解析器只去除单引号，不能改写成 JSON 双引号列表。服务端限制必须在桥及 rosapi 节点中配置，不能只隐藏按钮。

`self_test.launch.py` 在上述基础上启动标准示例，支持同样的 launch 参数：

- Topic：input/echo/ticks（std_msgs/String），sensor（Float64），retained（String，仅启动时发一次），均以 `/web_link_demo/` 为前缀。
- 标准观测：image/compressed（sensor_msgs/CompressedImage）、depth（Image，16UC1毫米）、robot_description（String，transient_local）、joint_states、points（PointCloud2，20×20 RGB）、tf/tf_static（TFMessage）、markers（MarkerArray）、path（Path）、diagnostics（DiagnosticArray）。
- Service：add_two_ints（example_interfaces/AddTwoInts）、slow（std_srvs/Trigger，约 1.2 秒）。
- Action：fibonacci（example_interfaces/Fibonacci），order 1..30 接受；13 模拟 abort，范围外拒绝。
- 参数节点 `/web_link_demo/demo`，gain 为 double，范围 0..10；read_only 只读，包含全部九类具体参数样本。
- 生命周期节点 `/web_link_demo/lifecycle`，使用标准 get_state/get_available_transitions/change_state。

## 可选项目验收入口

`audit/index.ts`导出`validateProject`、`runProjectChecks`及ProjectSpec/ProjectReport类型。调用方提供控制RosClient、项目数据配置和selectedIds；RunOptions.observationClient可选传同一URL字符串的第二个已连接RosClient，省略则共用。Topic/image/cloud/TF证据走观测侧；图谱、类型、Service、参数、Action和mockGuard走控制侧。引擎不dispose调用者客户端；任一连接中断终止用例，取消/恢复仍须经控制侧确认。ProjectReport.connectionMode是兼容旧报告的可选字段，新报告总返回shared/separate。可通过AbortSignal停止，通过onResult/onProgress接收结果。结果始终区分PASS/FAIL/CANCELLED/NOT_RUN；远端未知不能标成功。完整字段与示例见[project-testing.md](project-testing.md)。

TopicSpec.timestampMode默认为live（检查源时间与浏览器时钟的新鲜度），recorded显式保留录制时间、检查到达/结构/解码/段内频率，并报告liveFreshnessChecked=false；不能据回放判断当前设备状态。Peach回放项目只含10项观测用例，无参数/Service/Action写入。

## 标准图像预算入口（0.7.1）

`@local/ros2-web-link/media`额外导出`compressedImagePayload(message, {maxPixels?}) -> {bytes, mime}`。它复用ImageView既有的16MB压缩字节检查和PNG/JPEG头尺寸检查，默认4M像素，拒绝compressedDepth封装；不做像素解码，也不替调用方创建/回收Blob。调用方若需要URL，必须负责替换、停止、卸载时revoke。网页标准数据预览使用该入口及项目image.maxPixels，使用请求序号抑制迟到模块/旧订阅回调；普通ImageView、raw解码与曲线、3D的资源所有权保持原接口。预算不代表整个浏览器内存隔离或完整图像安全沙箱。

## 可选组件入口

0.8.0 的 `SceneOptions.layers?:SceneLayer[]` 只规定初始订阅集合，省略保持原五类订阅；旧 `setLayer()` 仍只切可见性，包括在 start 前调用。已启动根 Scene 可用 `createDisplay({layers,...options})` 创建逻辑显示，子实例共享 renderer / 相机 / TF / clock，不拥有公共资源或单独 RAF。启停订阅通过实例 start/dispose 管理；最后一个工作台空间实例退出后释放根画布。共享绘制后同时释放各子实例被替换的 Marker 资源。

SceneOptions 增加 robotDescriptionOptions、markersOptions、pathOptions；默认保留原有 QoS，可像点云与关节一样显式覆盖 TopicOptions。`assetStats()` 返回所属逻辑实例的 URDF 解析、资产进度及关节缓存。SeriesPlot 增加 subscription:TopicOptions 与 stats()，旧默认 sensor / 600 点不变。

`@local/ros2-web-link/graph` 导出 `discoverGraph` / `graphElements` 与 `GraphView`。采集输入仅要求 client 的 callService / onState / connected；快照含 status、receivedAt、elapsedMs、声明边、服务提供者、Action服务端名称、errors、scope、capabilities。支持 signal、timeoutMs、concurrency、maxNodes/maxTopics/maxEdges；缺省10秒/4并发/256/512/2048，单 RPC 最大3秒。取消只释放本地等待，不取消已进入 rosapi 的短查询。GraphView 提供 setSnapshot/filter/focus/fit/stats/dispose；节点投影截断单独报告，不改变原快照的声明事实。

`@local/ros2-web-link/workbench` 导出 WorkspaceSpec / DisplaySpec / TopicBinding、validateWorkspace / parseWorkspace / workspaceFromProfile、DisplayRegistry / createDefaultRegistry、WorkbenchController、WorkbenchView 和有界 previewMessage。工作区 JSON schemaVersion=1，解析前1MiB门、结构深度16/20000节点门，枚举必须是真字符串，不做数组到字符串转换。注册工厂是应用本地可信代码，JSON不能装载插件。DisplayInstance 实现 start/dispose，可提供 stats 和 assetStats。Controller 维护 active 实例与一个共享 Scene，add/remove/update/setEnabled/replace/snapshot/stats/resume/pause/dispose 为公有操作；replace 校验成功后才暂停替换且不自动恢复。

WorkbenchView 的 options 注入 getControlClient/getObservationClient/beforeResume/profile/storageKey/onStatus；提供 syncConnection/setProfile/pause/stats/dispose。stats 中 renderer 仅含共用渲染器指标，displays[].assets 才是对应模型加载进度。graphSource 显示脱敏桥地址，graphStale 标明上一会话；跨桥/断线后旧快照和端点信息保持历史标识，重新采集前禁止使用旧列表进行定义读取。下载图谱同样带 source/stale；不把“完整快照”扩展为端点QoS或消息到达证明。

`scene/index.ts` 导出 `SceneView(root,client,options)`、`TfBuffer`、`decodePointCloud`。SceneOptions 必填字段 fixedFrame、robotDescription、jointStates、tf、tfStatic、markers、pointCloud、path 均为显式配置（详见 config/profiles.yaml），可选 maxPoints=100000、maxMarkers=1000、assetBase=/assets/、pointCloudOptions/jointStatesOptions:TopicOptions、onStatus。方法 start、setLayer(robot|tf|markers|cloud|path,bool)、setFixedFrame、resetCamera、stats、performanceStats、dispose。root 必须有可见宽高；3D 需要 WebGL2。

0.6.18新增SceneOptions.useSimTime:boolean，默认false使用系统时间，不订阅/clock；true时Scene和内置TfBuffer共同使用/clock，首帧前为零，暂停不外推、回退清旧Marker/云/Path。寿命与回退用BigInt纳秒比较，到期边界包含相等；零寿命永久，重复Marker刷新期限。stats包含useSimTime、clockReady（仅表示初始化，不证明正在推进/在线）、clockSeconds（显示数值）和clockNanoseconds（精确十进制字符串，JSON安全）。墙时仍只有Date.now的毫秒分辨率。参数非法在WebGL构造前拒绝；配置useSimTime/profileInteractions/profileShaders严格为布尔值。

`performanceStats()`返回CPU渲染提交p95（最近最多120次）、累计render/RAF次数与平均频率、最大RAF间隔、页面longtask、URDF解析/资产加载耗时与数量、渲染设备、shader程序创建数、draw calls/triangles/几何体/纹理数。后台限频会影响频率，CPU提交时间不等于GPU执行时间；页面longtask含该页面其他工作。没有PerformanceObserver longtask支持时计数为0，不表示所有任务都快速。网页“3D性能与资源证据”展示此公有结果。MarkerArray按顺序得到最终可见集合后更新；带namespace的DELETEALL仅清对应namespace；替换材质在替换帧后释放，避免共享shader反复销毁。

SceneOptions还支持`shaderDiagnostics:'errors-only'|'verbose'`（默认errors-only）和`profileShaders:boolean`（默认false），profile YAML与网页SceneOptions JSON使用同一字段。errors-only仍验证每个新链接程序的LINK_STATUS，仅实际false时取链接日志；上下文丢失不当作链接失败，存活context返回null则显示无法验证。Scene监听原生lost/restored，两事件均丢弃旧pending并标记重绘，Three负责恢复资源；这覆盖两事件都先于下一flush的情况。dispose先移除监听，已销毁实例不响应晚事件；错误最多保留4项、每项2000字符，经onStatus显示并在ROS数据reset后保留，组件dispose后清除。verbose另外保留Three的编译/链接详细日志；若无法挂接上下文linkProgram则自动回退verbose，此时通过Three控制台诊断。`ShaderDiagnostics`作为类型从`/scene`导出。

新增指标`rendererInitMs/constructorMs/firstRenderMs/firstRenderCompleteAtMs/renderMaxMs`区分WebGL初始化、组件创建与绘制；`renderErrors/lastRenderError/shaderErrors/shaderDiagnostics`报告绘制和链接失败；绘制错误保留到下一次成功绘制或销毁，firstRenderCompleteAtMs仅在成功帧写入。`profileShaders:true`时，shaderCalls记录10个公有GL调用的次数/CPU总耗时/最大值；仅包装已有调用，不主动重复查询。默认不挂计时钩子或查询GPU设备信息。网页scene-bootstrap显示模块加载/构造/start/总耗时；模块加载包括脚本解析执行，不能等同网络耗时。该诊断不能测量GPU执行时间，也不能保证首载无阻塞。

`TfBuffer(client,{tfTopic,tfStaticTopic,clockTopic,historySeconds,maxFrames,maxSamplesPerFrame})`：start/dispose、ingest、ingestClock、lookup(target,source,stamp?)、frames、stats、clear。0.6.18起clockTopic默认false，仿真/回放显式传'/clock'或自定义Clock话题；不同于0.6.17自动订阅。省略stamp查询latest common time；精确动态范围外返回undefined；静态对任意时刻有效。底层几何/插值使用上游。decodePointCloud(message,{maxPoints,maxBytes})返回positions、colors?、count，Float32Array，colors为[0,1]RGB。

`media/index.ts` 导出 ImageView(root,client,{topic,type,depthScale?,maxDepth=5,maxPixels=4000000,subscription?:TopicOptions,onStatus?})、SeriesPlot(root,client,{topic,type,fields,maxPoints=600,onStatus?})；两者 start/dispose 幂等，dispose后不可重用。ImageView默认sensor/CBOR/500ms（2Hz）；subscription可显式覆盖QoS、compression和throttleMs；SeriesPlot 100ms合并绘图。`decodeImage` 与 `imageBytes` 可独立复用；SeriesPlot 的字段使用 data/position.0 等普通路径，最多8个。

0.6.19的maxPixels与imageBytes第二参数maxBytes（默认16000000）均要求正安全整数。imageBytes仅接受Base64字符串、数值数组或真实Uint8Array；拒绝number分配长度、其他TypedArray及伪造类型标签。跨realm Uint8Array可规范为共享原缓冲的本地视图。普通CompressedImage仅支持PNG/JPEG：先校验magic与format一致，再以image-dimensions 2.6.0读取尺寸并检查像素预算，之后才创建Blob/HTMLImage。空/未知普通format按Jazzy消息约定以JPEG解释，实际magic仍须为JPEG；compressedDepth、tiff和歧义codec明确拒绝。头预检不验证全部像素内容，库读取会复制编码字节一次；未承诺解码峰值、压缩流FPS或整浏览器内存上限。

0.6.20进一步要求数值数组索引为自身属性且值为整数0..255，拒绝稀疏数组（包括继承的数字索引）、NaN/Infinity、小数、负数、超范围及字符串等，不再依赖Uint8Array隐式转换。原始图像encoding用Map精确匹配既有支持集，constructor/__proto__等未知字符串明确报Unsupported image encoding；不增加新的图像格式或改变合法RGBA结果。

组件共享同一个 RosClient；同话题必须使用一致的 QoS/throttle/compression（见前述约束）。Scene 动态TF/Marker/Path reliable，静态TF staticTf、URDF latched，JointState/点云与/clock sensor；Image sensor+500ms throttle，曲线sensor。组件可与业务页面分别挂载，卸载必须dispose。标准包资源路由、路径安全和 profile 见 [launch.md](launch.md)；显示能力与明确限制见 [architecture.md](architecture.md)。

2026-10-28 真实相机+Peach mock联调：JointState显示与曲线统一sensor QoS，允许同一客户端共享订阅。静态TF使用depth100，避免多静态发布者在depth1下丢失机器人整批静态坐标；该问题已通过原生rclpy深度1/100对拍复现。


v0.4大型数据：Scene点云默认sensor/CBOR/100ms，pointCloudOptions可覆盖；JointState默认sensor/0ms，以兼容同客户端曲线订阅，jointStatesOptions可覆盖但共享订阅须保持选项一致。Marker/TF默认不丢帧限频，因为消息可能是差量更新，不能对整条TF任意抽样而假设所有子帧都保留。URDF重复字符串不重载，资源仍走本地只读白名单。

v0.6.1的performanceStats().jointStateCache报告modelValues、unmappedValues、unmappedLimit。modelValues只含当前URDF中的已知位置；未匹配值最多1024项/每名1024字符，淘汰较旧值，不将它们传入URDFLoader。新URDF消费匹配缓存并过滤旧模型关节，reset/dispose清空。position可按标准消息为空，不因此把当前模型位置归零；velocity/effort不代替位置。

同页标准数据观察跟随选中project，不再固定demo。原始消息预览仅保留有界摘要，不逐帧JSON化整幅图像或点云。SDK订阅回调仍接收完整原生消息；摘要只是自测页的显示策略。

## 自测表单与项目QoS

项目“填入面板”保留TopicSpec的QoS、编码与限频；缺省按验收引擎的sensor/none/0处理，图像组件自身的独立默认值仍为sensor/CBOR/500ms。显式QoS对象在Topic或图像下拉框中显示“自定义 · reliability · durability · depth”，原JSON作为该选项值保留，执行时还原对象交给同一SDK验证。切换项目/用例时替换旧自定义选项，最多保留一个；用户可改选已有预设。没有自动改QoS或静默回退。

切换到没有Topic工具配置的项目时，Topic名称/类型清空、QoS恢复sensor、编码none、限频0；不沿用上个项目的接口或自定义策略。

## Service客户端缓存部署参数

三个参数均属于rosbridge_websocket受管Node，使用标准ParameterDescriptor.read_only：service_client_cache_size默认0（0.8.3起），整数0..1024，0走有界逐调用客户端路径；service_client_max_in_flight默认128，整数1..4096；service_client_creation_timeout默认5.0，有限浮点(0,60]秒。max_in_flight在创建线程之前限制同一桥的所有Service连接，cache0与非零缓存均生效；超额立即result=false，不排队。cache0未消费的创建/清理任务保留名额，creation_timeout约束创建等待。非零缓存的独立槽位上限为min(cache_size,max_in_flight)。这些限制不涵盖Action或整机内存。参数启动期约束在监听前校验，不能经运行时参数写入切换。它们不是ROS服务名或业务重建开关。

标准call_service的service/args/id/timeout/result与服务权限保持，服务发现、参数、Lifecycle等通用服务均可复用；Action/Topic不改协议。response timeout<=0仍表示不设响应截止，本地Node关闭可唤醒，不能解释为取消服务端执行。创建/退役调度超期与cache/call/type/retirement busy通过原result=false回包，未增加重试或HTTP代理。

stats仅私有诊断：unconsumedCreationTickets为本池未确认票据，关闭后未消费可非零；pendingRetirements等待owner确认；unconfirmedCreations累计工厂失败次数（含找回已注册项），quarantinedEntries当前隔离项；nativeEntityHighWater只含已确认创建，不将未知项称零。无显式诊断/WEB_LINK_EVIDENCE时不写性能文件。


I62关闭行为复核：标准Service timeout:3仍按3秒失败，远端服务不会被自动取消；Node关闭会解除本地无限等待和未执行创建等待。上游发布者默认注销保留10秒可延后解释器退出，web.launch的bridge退出宽限为15秒，不是RPC超时。使用默认10秒Timer的真实退出测试约9.94秒、无TERM升级；更长手工上游保留期需要调用方同步适配launch预算。公有TS入口/参数/TF/观测协议没有因此新增接口，见[开发记录I62](development.md)。

配置边界：SDK直接构造ImageView/调用compressedImagePayload时可传maxPixels；标准预览也读取自定义projects_config中project.view.image.maxPixels。当前profile YAML的image schema只接受topic/type/depthScale/subscription，`config:=views.yaml`不接受maxPixels；普通图像表单没有像素预算输入，仍使用ImageView默认值。没有将CPU注入runtimeConfig的小预算案例冒充profile YAML端到端验证。

工作台模板的 `workspaceFromProfile` 保留 `scene.robotDescriptionOptions`、`markersOptions`、`pathOptions`、`jointStatesOptions`、`pointCloudOptions` 和 `plot.subscription` / 图像 subscription 的 QoS、compression、throttleMs。缺省值保持原值，所有模板显示仍默认关闭。profile YAML 对新增订阅字段采用与既有图像/点云相同的 TopicOptions 校验。
