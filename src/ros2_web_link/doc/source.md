# 依赖版本、本地源码与修改

0.8.0 唯一新增运行依赖是 Cytoscape 3.34.4（MIT，锁定完整性，自带 TypeScript 类型）。其完整 npm 源码在 `node_modules/cytoscape/src`，官方 API 为 https://js.cytoscape.org/；layout/事件/resize/destroy 直接使用该库，不自研图布局。适配器在 graph/，SDK没有引入该依赖。若需修改，复制到独立本地依赖后用 npm file: 更新锁文件并重建，不能只修改 node_modules 后交付。实际打包的 LICENSE 由既有构建脚本收进 THIRD_PARTY_NOTICES.txt。

本轮先核 RViz Jazzy 的 display/display_group/ros_topic_display 头文件、rqt_graph ROS 2 源码、本机 rosapi 2.7.1 的 NodeDetails.srv/proxy.py，以及 ROS 2 接口文档。共享显示保留 RViz 的订阅实例寿命思路，节点关系用 rqt_graph 同类的标准图谱事实；不装载其 C++/Qt插件。原始来源、apt版本及文件SHA见 `runs/ros2_web_workbench_20261102/source-review.json`；设计/API/实现分别在 doc/universal-workbench-design.md、doc/io.md 和 graph/workbench 源码。

2026-10-30补核原生性能API：[W3C Long Tasks API](https://www.w3.org/TR/longtasks-1/)当前标记为2026-10-20 Discontinued Draft；它不是最终推荐标准。当前Chrome仍实际交付longtask与LoAF，源码按supportedEntryTypes判断。LongTask的name/容器归属可区分部分事件循环上下文，但不等于最深函数；先前I50/I51未采这部分字段，不能倒推旧数据。I52临时公开观察器采到self和unknown/window，只能说明浏览上下文。对应[Chromium官方IDL](https://raw.githubusercontent.com/chromium/chromium/main/third_party/blink/renderer/core/timing/performance_long_task_timing.idl)及[容器归属IDL](https://raw.githubusercontent.com/chromium/chromium/main/third_party/blink/renderer/core/timing/task_attribution_timing.idl)为main，非本IAB精确构建证明。[NavigatorID](https://html.spec.whatwg.org/multipage/system-state.html#dom-navigator-useragent-dev)自报UA也不证明二进制构建版本。当时内置浏览器工具仅开放显示/视口而非Trace。用户随后已明确选择独立Chromium＋Playwright；I58已采原生轨迹，当时交互段有损与后端中断未通过；I64随后完成独立无损轨迹和无profiler对照，不据此更改产品默认协议或再次请求相同授权。

I53按固定[uPlot1.6.32声明](https://raw.githubusercontent.com/leeoniya/uPlot/1.6.32/dist/uPlot.d.ts)及[同tag源码](https://raw.githubusercontent.com/leeoniya/uPlot/1.6.32/src/uPlot.js)、本机esm/bundle核对微任务提交。setData同步返回不包含后续_commit；公开drawClear/draw只覆盖绘制子段。临时诊断bundle包三函数真实正文，未调用同步batch改调度；原/补在原生queueMicrotask与Promise fallback受控对照通过，实际Peach调用计数也非零。修改仅存在临时副本，map失配明确，不污染产品上游库或新发版；详见development.md I53。

I55重新核对[rmw_fastrtps8.4.4响应实现](https://raw.githubusercontent.com/ros2/rmw_fastrtps/8.4.4/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)及[FastDDS2.14.7 SampleInfo](https://fast-dds.docs.eprosima.com/en/v2.14.7/fastdds/api_reference/dds_pim/subscriber/sampleinfo.html)：成功take暴露的source/received字段对应writer写入与reader历史插入，不是NIC时间。两个精确失败CID主要延迟约2999ms在这两字段之间，独立复核同机时间/单位/生命周期配对；不据近3秒认定心跳或重传。[rmw_fastrtps#842](https://github.com/ros2/rmw_fastrtps/issues/842)、[rmw#372](https://github.com/ros2/rmw/issues/372)、[rclpy#1720](https://github.com/ros2/rclpy/issues/1720)只是相关候选报告，其环境/复现与本轮不同，不直接套用KEEP_ALL或将issue描述当本机根因。

I56按固定[rclpy7.1.12 Client源码](https://github.com/ros2/rclpy/blob/7.1.12/rclpy/rclpy/client.py)、本机Node及[rosbridge2.7.1 services](https://github.com/RobotWebTools/rosbridge_suite/blob/2.7.1/rosbridge_library/src/rosbridge_library/internal/services.py)核对Node拥有实体、独立Future/sequence和原executor创建/销毁。临时单endpoint复用保留上游call_service正文；SDK协议明确timeout:3会覆盖后台默认5秒。CPU与真实rclpy短测、30分钟原负载及边界见development.md I56，未修改系统库或产品源码，也未把原生实体复用当作已证明的DDS机制修复。

0.6.21按[Khronos WebGL上下文事件规范（当前编辑草案）](https://registry.khronos.org/webgl/specs/latest/1.0/)与本机Three0.186.1的onContextLost/onContextRestore核对：GL对象在丢失后失效，Three重建资源，Scene只在lost/restored清旧pending并请求下一帧；未修改Three源码或增加恢复timer。真实静态红绿、完整Peach普通回归与离线源码冻结分别见development.md I48–I49，不宣称编辑草案是最终推荐标准或本机验证等于所有设备。

0.6.12的上下文丢失处理核对了本机Three0.186.1 `WebGLRenderer.render`的正常提前返回分支，以及[Khronos WebGL上下文丢失语义](https://registry.khronos.org/webgl/specs/latest/1.0/#5.14.2)。只在SceneView调用公有isContextLost，不修改Three源码；CPU提交计数与GPU/呈现完成分别解释。compileAsync相关源码也已审查，但尚未接入或声称解决冷启动。

0.6.13只透传rosbridge2.7.1现有write_queue_size，核对[同tag参数定义](https://github.com/RobotWebTools/rosbridge_suite/blob/2.7.1/rosbridge_server/scripts/rosbridge_websocket.py)与[队列/写出实现](https://github.com/RobotWebTools/rosbridge_suite/blob/2.7.1/rosbridge_server/src/rosbridge_server/websocket_handler.py)，以及本机Jazzy IncludeLaunchDescription/DeclareLaunchArgument/ParameterValue。参数按消息数而非字节数，同连接回复共享队列；未复制或修改上游缓冲实现。可选回放参数另命名，避免共享作用域空值遮蔽下游默认。

本包自有 TypeScript/JS/Python/launch/构建配置均为可修改源码，不需要闭源服务。网页 JS 在本机打包，SDK 和浏览器 bundle 保留 source map；运行不联网下载脚本。npm 安装锁定传递依赖的完整性；离线重新构建需要预先准备 npm 缓存或镜像。

自有代码 Apache-2.0；TF复用文件保留MPL-2.0；上游 roslibjs BSD-2-Clause，rosbridge_suite 采用各子包声明的 BSD 许可。构建生成的 THIRD_PARTY_NOTICES.txt 收录实际进入 bundle 的依赖 LICENSE/NOTICE 全文，不依赖上游 JS 是否保留许可证注释。不要删除上游版权。

用于对照的上游完整源码保存在本包的本地 `.upstream/`（隐藏目录避免 ament 扫描上游测试与源码）：

```bash
mkdir -p .upstream
touch .upstream/COLCON_IGNORE
git clone --depth 1 --branch 2.1.0 https://github.com/RobotWebTools/roslibjs.git .upstream/roslibjs
git clone --depth 1 --branch 2.7.1 https://github.com/RobotWebTools/rosbridge_suite.git .upstream/rosbridge_suite
```

.upstream 是本地源码缓存，未跟踪，避免复制 Git 仓库或让 colcon 扫描第二份桥节点。已核对 tag/commit：roslibjs 2.1.0 = 5b60f744e45dc2e7ca9dace74727c27eddb93603；rosbridge_suite 2.7.1 = 94ac3a3a92b4e603df0e26441ce0152b484c9d57。运行版本可用 npm ls 和 rosbridge_server/package.xml 核对。本机 apt 安装的 rosbridge Python 实现也可直接阅读 `/opt/ros/jazzy/lib/python3.12/site-packages/rosbridge_library/`。

修改 roslibjs 时，按该版本上游 README 构建，再将本包 package.json 的 roslib 依赖改成对应本地构建包路径（npm file:），重生成锁文件并重新构建验证。修改 rosbridge 时，推荐放入独立 ROS overlay 工作区构建并 source 后再启动；不要直接修改 /opt/ros 或让两份相同包同时进入本工作区 colcon 扫描。

升级检查重点：ROS Topic QoS钩子与发布者close监听器、离线队列、Action终态、参数byte[]/int64、桥glob/timeout，以及Transport公开契约。`scripts/bridge_node.py`在自有进程内安装下文记录的octet、静态TF与版本限定分片ID兼容；rosapi_node.py单独处理Action图谱API差异。均不修改系统包。升级必须重新运行README与testing.md的实际链路核对。

## v0.2 观测依赖

本轮锁定的运行依赖：roslib2.1.0、three0.186.1、urdf-loader0.13.1、uplot1.6.32、gl-matrix3.4.4。Three/gl-matrix/uPlot为MIT，URDFLoader为Apache-2.0。源码已随npm安装在node_modules各包的src或dist目录；可直接阅读调试。要长期修改应fork或复制至独立本地依赖，使用npm file:路径并更新锁文件，不直接把node_modules临时改动当作可复现交付。

TF实现直接收录于 third_party/lichtblick_transforms，固定 Lichtblick v1.30.0（fcfc7d559d8e76db36c9730e78049a60f976c2d2），仅调整导入路径，详见该目录README；保留上游版权、完整MPL-2.0与文件级许可。修改这些MPL文件后分发时必须继续提供其相应源码。本包安装时连同全部自有适配源码一起复制至share/ros2_web_link/source，浏览器bundle也附许可证全文和source map。本地对照仓库位于 .upstream/lichtblick，获取：

```bash
git clone --depth 1 --branch v1.30.0 https://github.com/lichtblick-suite/lichtblick.git .upstream/lichtblick
```

Jazzy主机还需 rosapi_node.py 窄启动适配：本机 rosapi2.7.1 导入的ros2action函数调用不存在的 Node.get_action_names_and_types，使 /rosapi/action_type 查询直接杀死rosapi。仅在该Node方法不存在时换用公开 rclpy.action.graph.get_action_names_and_types，再runpy原rosapi可执行文件。未改系统库或自建图谱服务；原生类型查询探针覆盖此问题。升级时核对Node API并移除不再需要的适配。

## v0.5 传输边界与原生分片兼容

SDK通过roslib公开ITransport/AbstractTransport事件契约接native WebSocket；JSON使用标准解析，CBOR使用与roslib锁文件相同的cbor2 2.4.1（MIT，node_modules/cbor2/lib完整JS与类型）。sdk/transport.ts只实现上游缺少的有界分片缓存、发送预算与安全分发，原生操作/Topic仍由roslib处理；未复制CBOR或CDR解码器。升级必须复核这些公开类型和cbor2 TypedArray/深度选项。

本机rosbridge-library2.7.1的Protocol.send每条消息重新构造Fragmentation，导致实例seed反复从0开始，真实深度/完整云并发分片发生ID混流。bridge_node.install_fragment_id_compat仅此版本生效，幂等包装上游fragment，为每条物理消息生成独立外层UUID；serialize的cid在该版本仅用于日志，不修改消息内部RPC/Action ID。fragmentation_probe.py核对精确重组与内层ID；真实128KiB分片及512字节RPC分片分别由transport_probe与rpc_fragment_probe验收。该修复仅在自有桥进程内安装，不修改apt库，升级时重新验证并删除不再需要的包装。

URDFLoader0.13.1 README中的旧loadMeshCb例子只有3参数，本机真实源码/d.ts为(url,manager,material,onLoad)四参数；本包按实际源码实现。包目录回调不加末尾斜杠，因为上游自行追加分隔符；服务器仍严格拒绝空路径片段。

## v0.6 渲染诊断与负载探针

依据[Three WebGLRenderer文档](https://threejs.org/docs/pages/WebGLRenderer.html)及锁定0.186.1源码`WebGLProgram.onFirstUse`，成功程序的详细日志查询也会触发同步GL等待。scene/shaders.ts使用公开WebGL linkProgram/LINK_STATUS和Three debug.checkShaderErrors，提供errors-only与verbose；不访问私有program缓存、不静默放过失败链接。每个上下文只允许一个检查器，销毁恢复原方法；只读/忽略赋值的上下文回退上游详细检查。shader_probe核对故障边界，实际浏览器正常链接另行验证；另在gl-environment原生WebGL2对照中已用故意错误的片元源码核对相同checker的真实驱动失败路径；未对Three生成的着色器做故障注入。首载仍有同步查询长任务，不能以移除日志查询宣称首载已解决。

multi_client_probe.mjs复用同一个SDK、TfBuffer及标准图像/点云解码器；使用ws公开pause/resume模拟慢读者。run_transport_benchmark.py组合原生replay launch，记录每个自有子进程的PID/命令/RSS并finally清理。它们是可读可改的显式行为工具，不加入ament test，不操作业务节点或硬件。

## 静态TF多客户端兼容

本机rosbridge2.7.1的MultiSubscriber为晚加入客户端创建临时DDS订阅，但只回放首条保留消息就销毁它，多静态writer的剩余样本会丢失。static_tf_compat.py仅对TRANSIENT_LOCAL TFMessage使用每个晚客户端独立DDS reader，复用上游发送与executor销毁，不缓存TF、不广播新TF；其他话题保持上游路径。还需SDK为此类型将queue_length与QoS depth保持一致（100），避免Web队列1再次覆盖突发回放。重复订阅更新路由，旧回调token隔离，最后释放reader。升级上游需重跑scripts/static_tf_probe.mjs并评估移除此适配。

service_wait_trace.py为默认关闭的诊断，不是通信兼容修复。显式开启时核对已安装2.7.1的internal.services.call_service与CallService.call_service源码SHA256，仅读取wait失败后、原取消前的Future状态；升级必须重新阅读调用/线程语义，不能直接更新哈希绕过检查。当前不改变上游函数体、系统文件或默认RMW。原生服务与受控调度/写盘故障分别核对，详见testing.md。

## 可选服务metadata诊断

service_trace.py只在WEB_LINK_TRACE_SERVICE显式设置时启用，两个自有启动包装器调用它；使用Node.create_client/create_service及configure_introspection公有API，保留Jazzy关键字参数，并仅作用于解析后匹配的端点。METADATA不含应用字段；不改上游服务回调和请求重试策略。其SIGUSR1处理器采用Python标准faulthandler，仅输出线程位置到本轮stderr。关闭进程结束诊断，不提供运行期重新定向。未设置变量时不替换Node方法或注册信号。依据[Jazzy原生示例](https://github.com/ros2/demos/blob/jazzy/demo_nodes_py/demo_nodes_py/services/introspection.py)与本机rclpy service/client实现；临时事件发布者有发现窗口，原始GID不能直接跨阶段匹配。

服务端模式由rosapi包装器的WEB_LINK_TRACE_SERVER_SERVICE显式选择，调用install_service_trace(endpoint,trace_clients=False)，只替换该进程的服务工厂；桥端不启用METADATA。collector --passive用rosidl_runtime_py.get_service(...).Event与标准QoS订阅，只保存ServiceEventInfo，不读写业务字段或发起对照查询。完整与服务端模式不能在同一进程混用或运行期扩权。相关ROS依赖显式列入package.xml，不安装同名pip包。

服务端GID/sequence可在同一服务端关联REQUEST_RECEIVED与RESPONSE_SENT；不能直接用它与Web CID或另一侧的原始GID配对。[rmw问题357](https://github.com/ros2/rmw/issues/357)和Jazzy服务内省示例均显示部分RMW的客户端/服务端GID不同。本轮同时核对本机公开rmw_get_gid_for_client及rcl_client_get_rmw_handle声明，但未据它们推导不存在的跨侧标识映射，也未新增native内存读取。0.6.7长测阶段结论仅采用同侧事件及并发时间窗，具体限制见performance.md。

2026-10-29的dpkg完整性检查对本机rclpy7.1.12、rcl9.2.11、rmw_fastrtps_cpp/shared_cpp8.4.4未报告修改；rosbridge-library为2.7.1。该检查排除本机这些包文件被修改，不证明库本身没有缺陷，也不为其他机器版本作保证。

## 原生服务诊断源码补充

`scripts/service_churn_probe.py`仅用于显式只读性能对照，使用Jazzy的`Node.create_client`、`Client.wait_for_service/call_async/remove_pending_request`、`rclpy.spin_until_future_complete`和`Future.done/cancel/exception/result`。源码已与本机`/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/{node,client,task}.py`和[Jazzy分支Client](https://raw.githubusercontent.com/ros2/rclpy/jazzy/rclpy/rclpy/client.py)核对：不能在回调中同步call；本探针在主线程发异步请求并spin，短命客户端逐次销毁。时限使用monotonic，与回放/clock无关；不改apt或桥/业务回调。

新探针在ROS初始化前用Python独占文件创建占用证据目录，不使用自研锁服务；3秒响应门包括发送耗时。`run_transport_benchmark.py --native-churn-seconds`复用既有进程守卫与清理，把原生探针作为其直接子进程。详细命令与适用范围见testing.md，失败和不同查询周期的解释见performance.md。

## 归档源码重建

v0.6.2将tsconfig.json纳入npm files，并由build.mjs在删除旧dist前读取根package-lock.json，原样保存到dist/build-package-lock.json。依据[npm官方files规则](https://docs.npmjs.com/cli/v11/configuring-npm/package-json/#files)，根package-lock无法直接进入tgz；收件人恢复该副本后执行npm ci，依赖锁仍只有根文件一个权威，不引入另一套版本配置。tgz也含构建脚本、自有TS/JS/Python、launch/配置、CMake/package.xml和三方许可证。重建步骤见integration.md；在业务中只消费四个dist入口时无需开发依赖或重建。


## SDK绝对截止时间

v0.6.3复用已用于分片的performance.now单调时钟，分别记录连接尝试与Service/Action请求的截止时间；消息回调即使先于迟到的setTimeout执行，也不能使过期等待成功或交付反馈。成功完成前再检查一次，覆盖SDK处理阶段跨期；finish只清理/完成一次。connect保留原重连策略，普通请求不重试、不自动取消远端。该修复使用标准浏览器时钟/事件接口，不补丁roslib或ROS节点；模拟事件故障与真实DDS回归分开记录。

## 未完成请求资源

v0.6.4对照Jazzy本机rclcpp/client.hpp的async_send_request/remove_pending_request说明及锁定roslibjs2.1.0的Service/Action事件实现：无响应等待须主动清理。SDK只补浏览器本地并发预算与Map关联，不引入新服务协议或远端取消。上限256是本SDK默认值，不是ROS规定。AbortSignal使用跨realm可用的结构校验，捕获引用/操作后登记；注册和清理异常均不能留下请求ID。新增预算属性独立设为不可写/不可配置，保留旧连接选项的兼容性。load_progress的资源释放检查逐项看计数，配置元数据不能误算成泄漏；多客户端收尾逐个尝试并保留失败报告。

## 表单QoS保真

v0.6.5的fillQosSelect/readQosSelect只做DOM表单编码：以安全textContent显示策略，JSON保留对象，执行时交给原SDK处理。没有eval、隐藏项目模型调用、协议或参数转换。项目填表的缺省值对照audit的TopicOptions保持一致，项目tools与图像view也使用同一选择器。录制项目的Reliable例外依据本机官方CLI及固定负载对照，通用SDK与在线项目的QoS保持原值。

## 0.6.8完成调用与原生WebSocket取证

最终时间线复用Python atexit/threading.Lock公开API，仍核对同一rosbridge2.7.1源码哈希。原生事件观察仅用于显式Node CLI，查阅[Node22.22.3 AsyncLocalStorage文档](https://github.com/nodejs/node/blob/v22.22.3/doc/api/async_context.md)、本机run实现和锁定ws的事件适配源码，采用标准run/getStore传播两个数值标签。没有在SDK新增Node依赖或把Node原生WebSocket换成ws。同步写盘的审查缺口已有慢sink红/绿；改为有界缓存及结束后flush，不以捕获异常代替时延隔离。具体字段/预算/丢失边界见testing.md和development.md I23。

旧rpcTrace的同步IO也由共享有界缓冲代替；stack旗标使用已核对的[Node22.22.3 fsPromises.writeFile](https://github.com/nodejs/node/blob/v22.22.3/doc/api/fs.md#fspromiseswritefilefile-data-options)与本机实现，遵守同文件前一Promise完成前不再写的限制。Node26页面只用于定位入口，实际实现核对以22.22.3版本为准。异步错误必须在最终判定前drain；审查、受控迟到IO与新的30秒记录分别保留。

## 服务事件的原生边界（本机精确版本）

核对[rcl service.c 9.2.11](https://github.com/ros2/rcl/blob/9.2.11/rcl/src/rcl/service.c)：REQUEST_RECEIVED在rmw_take_request成功后发布，RESPONSE_SENT在rmw_send_response返回OK后发布。事件只能界定该接口阶段，不能证明客户端收到响应。[rcl client.c](https://github.com/ros2/rcl/blob/9.2.11/rcl/src/rcl/client.c)中REQUEST_SENT也在发送返回后产生；客户端GID取自rmw_get_gid_for_client，不强行与服务端request_header.writer_guid配对。

[rmw_response.cpp 8.4.4](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)有client_present_t::GONE时返回RMW_RET_OK而不写响应的路径。安装的custom_service_info.hpp以互斥保护的端点映射及匹配集合判断存在；未记录实际分支，不能把此路径当作本机超时根因。[rmw_wait.cpp](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/rmw_wait.cpp)等待前及ready判断检查get_first_untaken_info，不支持“单次不匹配取样使后续队列永久不唤醒”的简单推断。rclpy7.1.12 executor取响应后按sequence查pending、设置executor再完成Future；call_async登记与get_pending_request共用Client锁。

rosbridge2.7.1的internal.services.call_service从服务图查实际IDL并新建短命Client；传入envelope的type字段未在该路径另行验证。SDK发送类型名不能表述为额外的IDL一致性门。该项是源码审计，尚未做真实错误类型请求试验；需要时可使用已有interfaceType/describeInterface核对，本轮没有为每次RPC增加发现请求或引入另一套服务协议。逐项审计原稿与故障执行路径的区别保留在[原始记录](../../../runs/ros2_web_transport_20261029/source-rebuild-v068-buffered-rpc/source-audit-rcl-9.2.11.md)。

0.6.9继续读取精确[rclpy7.1.12 Client绑定](https://github.com/ros2/rclpy/blob/7.1.12/rclpy/src/rclpy/client.cpp)和[Executor实现](https://github.com/ros2/rclpy/blob/7.1.12/rclpy/rclpy/executors.py)：send_request返回序号，take_response在RCL_RET_CLIENT_TAKE_FAILED时返回(None,None)，成功返回(header,response)。Executor随后从header.request_id.sequence_number查pending。Jazzy在线Client文档本次被Anubis拒绝访问，因此按已安装公开Client文档字符串及精确版本源码核对，未拿其他发行版API补齐。私有pybind面仅用于可拒绝的诊断，不对外承诺通用ABI。

`service_native_trace.py`只在WAIT明确请求时安装；本机原生绑定SHA256 `53efd7622c109ae6f9ff7ffafc3bffac4856700d39ca51d8689ecaa617a596bd`和两个Python调用面的源码哈希都须相同。版本号一致但本地构建不同也拒绝。原方法仍接收原参数、返回原对象、抛出原异常；只复制标识/序号/时间，不修改系统文件或访问rcl_client_t内存。具体预算、缺证判定和标识复用限制见testing.md。

0.6.10审阅精确[service_info.cpp绑定](https://github.com/ros2/rclpy/blob/7.1.12/rclpy/src/rclpy/service_info.cpp)及本机rmw/types.h：source_timestamp与received_timestamp作为只读整数暴露。Web缓存不能读取时只读下载官方原稿，保存于`runs/ros2_web_transport_20261029/native-reception-time/upstream-service_info.*`，未把它们当作修改过的系统源码。[rmw_fastrtps8.4.4响应代码](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)从SampleInfo复制这两个值；[FastDDS2.14.7文档](https://fast-dds.docs.eprosima.com/en/v2.14.7/fastdds/api_reference/dds_pim/subscriber/sampleinfo.html)定义来源为DataWriter写时刻、接收为DataReader加入历史时刻，后者不是网卡时刻。仅成功take能看到这些值，0/负值或不明时钟不能解读；跨进程/不同主机比较还依赖时钟对齐。以十进制字符串保存纳秒，不经过JS Number。

0.6.11只在新诊断模式包装同一已审阅的_destroy_client_async（SHA256见development.md I27），仍调用上游schedule_on_executor，不从Timer线程直接销毁handle。已读取Python3.12 threading.Timer的run/cancel与线程引用生命周期，以及本机Context.ok的锁/原生context使用。Timer构造/启动失败回退原清理，正常调用直通。此模式是原WAIT不保留Client边界的显式例外，只在超时判定后短暂保留，不影响原Future取消/失败；没有复制上游服务函数或改系统文件。


## 0.6.14 可选 Event Timing

[W3C Event Timing](https://www.w3.org/TR/event-timing/)核对了PerformanceObserver event、durationThreshold最小16ms、duration的8ms量化、受信离散事件、连续事件排除和buffered历史104ms门槛。本包buffered=false，只记录启用后的报告；不保留Entry/target，不按单个事件p95宣称INP。浏览器可不支持，能力与无样本状态必须可见。TS7.0.2已含PerformanceEventTiming，但PerformanceObserverInit缺durationThreshold，使用规范字段的局部交集类型。见scene/event-timing.ts及development.md I31。


## 0.6.16 ImageData缓冲所有权

[WHATWG HTML：ImageData interface](https://html.spec.whatwg.org/multipage/imagebitmap-and-animations.html#the-imagedata-interface)的data构造步骤与initialize source步骤明确保留传入TypedArray对象。对照本机TS7.0.2的lib.dom.d.ts：ImageDataArray=Uint8ClampedArray<ArrayBuffer>。decodeImage本来就新建普通ArrayBuffer，保留准确返回类型后直接传入即可去掉额外应用层复制；putImageData仍由浏览器完成像素提交，不能称整条显示链路零复制。


## 实际记录的独立参照与消息类型

只读依据[Jazzy rosbag2_py reader绑定](https://github.com/ros2/rosbag2/blob/jazzy/rosbag2_py/src/rosbag2_py/_reader.cpp)及本机_reader.pyi，使用SequentialReader.open/set_filter/read_next/close；不调用rclpy.init。本机sensor_msgs/Image.msg的is_bigendian为uint8，PointCloud2.msg为bool，清单必须保留差别。独立Python只用struct/hashlib逐字节/Float32小端参考，不依赖JS解码器。浏览器用标准[Web Crypto digest](https://www.w3.org/TR/webcrypto/#SubtleCrypto-method-digest)核对全画布像素，不用截图替代像素证据。


I37工作区数据流诊断重新阅读的直接来源：

- [rosbridge 2.7.1 WebSocket实际队列实现](https://raw.githubusercontent.com/RobotWebTools/rosbridge_suite/2.7.1/rosbridge_server/src/rosbridge_server/websocket_handler.py)，对照本机Jazzy apt模块；源码SHA以stream-ready/observer-v2-source清单为准，未使用ros2分支最新内容替代本机实现。
- [上游协议订阅/节流与队列语义](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md)。当前分支仅作协议参考；实际行为同时核对本机capabilities.subscribe、subscription_modifiers与Protocol.send。
- [Python3.12 Semaphore](https://docs.python.org/3.12/library/threading.html#semaphore-objects)，实际观察原acquire返回，不再次试取槽位；线程上下文与锁也使用标准库。
- [Tornado WebSocketHandler接口](https://www.tornadoweb.org/en/stable/websocket.html#tornado.websocket.WebSocketHandler.write_message)，本机实际继承实现/usr/lib/python3/dist-packages/tornado/websocket.py纳入SHA限定。open完成先于on_message，write_message返回原Future；完成不等于消费者收帧。


## 显式显示时钟与Marker寿命

0.6.18对照[ROS 2 Clock and Time](https://design.ros2.org/articles/clock_and_time.html)、[Jazzy RViz MarkerBase源码](https://github.com/ros2/rviz/blob/jazzy/rviz_default_plugins/src/rviz_default_plugins/displays/marker/markers/marker_base.cpp)和本机visualization_msgs/Marker.msg：ROS零时间不替换为墙时，暂停不外推，回退清旧显示；寿命以当前显示时间加Duration，并在当前值大于等于截止时移除。未宣称复刻RViz对所有极短TTL的阈值策略。

沿用TfBuffer既有规范纳秒转换，Scene内部BigInt比较，公开JSON统计输出十进制字符串。Scene.useSimTime和独立TfBuffer.clockTopic均默认关闭，回放显式开启；不设置远端节点的use_sim_time，不改变RPC的单调截止时间，也不另广播Clock。profiles文件仅接纳明确布尔项，未知字段仍严格拒绝。CLI红绿、只读审查、实际浏览器与真实Peach范围见development.md I38。

## 轻量彩图诊断与异步编译评估

stream-color属于工作区工具，不进入发布包。核对[rosbridge 2.7.1 WebSocket队列源码](https://raw.githubusercontent.com/RobotWebTools/rosbridge_suite/2.7.1/rosbridge_server/src/rosbridge_server/websocket_handler.py)与[同tag协议](https://raw.githubusercontent.com/RobotWebTools/rosbridge_suite/2.7.1/ROSBRIDGE_PROTOCOL.md)：共享写槽的成功acquire只说明准入，不能当作入队/网络/消费完成。只用实际Subscription/Subscribe/解析RosbridgeProtocol.send同步路径的TLS关联，不替换协议或队列；所有源门先于方法替换。P2红绿、只读审查和实际范围见development.md I39。

本轮另读锁定Three0.186.1 WebGLRenderer.compileAsync/compile、WebGLProgram.isReady/onFirstUse及[Three公有文档](https://threejs.org/docs/pages/WebGLRenderer.html)、[KHR_parallel_shader_compile规范](https://registry.khronos.org/webgl/extensions/KHR_parallel_shader_compile/)。其compileAsync用公有COMPLETION_STATUS_KHR轮询，材质已就绪才完成；缺扩展时只延迟检查，不能保证首次使用不阻塞。它不会解决创建WebGL上下文或扩展查询本身的秒级同步等待。旧gl-environment原生记录parallel=true，也记录getContext/扩展查询的独立长任务；仍需当前真实场景对照，不将读源码或预期效果当成优化完成。本轮尚未接入compileAsync、worker或WebGPU，当前2秒绘制阻塞仍开放。

## 0.6.19 压缩图像头读取

采用[image-dimensions 2.6.0](https://github.com/sindresorhus/image-dimensions/tree/v2.6.0)的公开imageDimensionsFromData浏览器入口。写代码前读取README、[JPEG解析](https://raw.githubusercontent.com/sindresorhus/image-dimensions/v2.6.0/types/jpeg.js)、[PNG解析](https://raw.githubusercontent.com/sindresorhus/image-dimensions/v2.6.0/types/png.js)、[utilities](https://raw.githubusercontent.com/sindresorhus/image-dimensions/v2.6.0/utilities.js)及[index.d.ts](https://raw.githubusercontent.com/sindresorhus/image-dimensions/v2.6.0/index.d.ts)，并解包实际npm发布物核对限制/复制/格式解析路径。MIT、无运行依赖，精确锁定2.6.0；npm最新tag查询落盘，不用主分支源码代替发布实现。发布tgz SHA256为`cef85c891e5fe1c7ff33fe676e3dbda9064dacd81215baf8f93fa4b59b400906`，原始下载/元数据/完整源码位于image-headers/upstream；产品分发附MIT全文。

该库入口会复制编码字节，且支持本组件不接纳的其他格式，故在调用前加固定PNG/JPEG magic门，不把最终type检查误当解析前限制。JPEG逐段offset读取和PNG头信息仅用于尺寸预算，不是内容完整性或全解码内存保证。另审阅image-size 2.0.4的官方npm源码/许可证而未安装该候选；其仓库网页不可达时依发布源码核对，不假称已读网页。

普通format的JPEG回退及compressedDepth特殊封装依据本机`/opt/ros/jazzy/share/sensor_msgs/msg/CompressedImage.msg`。PNG/JPEG以外明确拒绝，不自行解释tiff/rvl。TypedArray输入使用ECMAScript标准内建品牌getter鉴别真实Uint8Array，不能只信可伪造的Object.prototype.toString。原生验证用本机apt Pillow10.2生成极小JPEG，按实际Image.new/Image.save docstring及JpegImagePlugin._save源码核对；它只用于工作区夹具，不成为产品依赖。


## 0.6.20 原始图像字段校验

读取本机Jazzy Image.msg、image_encodings.hpp及[common_interfaces/jazzy Image.msg](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/sensor_msgs/msg/Image.msg)，编码采用受支持集精确匹配，数值数组保持标准uint8[]范围。docs.ros.org的相关API页面被Anubis拒绝，没有宣称读到其正文。标准[Object.hasOwn](https://tc39.es/ecma262/multipage/fundamental-objects.html#sec-object.hasown)调用HasOwnProperty而不查继承链；项目已有ES2022目标，本次没有增加polyfill或运行依赖。严格校验仅适用于数值数组，现有CBOR Uint8路径不增加逐字节扫描。

另查阅上游[历史吞吐下降报告930](https://github.com/RobotWebTools/rosbridge_suite/issues/930)与[旧大数据内存讨论212](https://github.com/RobotWebTools/rosbridge_suite/issues/212)，仅作假设来源：前者是Humble/1.3.2、后者来自2016，不是本机2.7.1 I36的根因证据或现成修复。实际Subscription/QueueMessageHandler/MultiSubscriber及共享有界写槽均以当前apt源码核对，保持原测试门，不据旧issue更换协议/依赖或关闭故障。


I43诊断读取[WHATWG Canvas getContext](https://html.spec.whatwg.org/multipage/canvas.html#dom-canvas-getcontext)及[Khronos WebGL2上下文规范](https://registry.khronos.org/webgl/specs/latest/2.0/)，并对照已有gl-environment/probe.js的公有方法委托；后者当前页面标明Editor’s Draft，未将其当作新的已发布产品依赖。包装仅观察原本调用，不增加同步GL查询；原this/参数/返回/异常保持与能力/清理边界由替身检查、独立只读复核确认，实际GPU/浏览器证据另列。


I44进一步核对固定Three0.186.1 WebGLRenderer.prepareMaterial/getProgram/compileAsync及WebGLProgram.onFirstUse/isReady：公有[compileAsync文档](https://threejs.org/docs/pages/WebGLRenderer.html#compileAsync)建议利用KHR并行编译，但本版无扩展固定10ms、透明DoubleSide只等待最后currentProgram。库的“减少不必要stall”不能扩大为所有program/持续变更/取消语义的保证；原全Scene revision设计因独立审查缺口而撤回，未改apt/npm源码或产品。程序查询pname由WebGL标准数值枚举细分，只包装实际调用，不添加GL检查。

I45再次读取[Three公有compile](https://threejs.org/docs/pages/WebGLRenderer.html#compile)和[KHR_parallel_shader_compile规范](https://registry.khronos.org/webgl/extensions/KHR_parallel_shader_compile/)。前者允许单Object3D与目标主Scene，但要求先配置光照/环境；后者只保证completion轮询非阻塞，完成不等于成功，context lost时completion会无条件true，无扩展则无法保证非阻塞状态读取。固定本地WebGLRenderer.js的prepareMaterial、renderObject会对透明DoubleSide两次needsUpdate，不能简单把version自增都当外部shader变更。隔离helper因此补共享材质/缓存失败/丢失/异常反例；它有意读取编译后Scene私有字段且替换checker，仅用于工作区诊断，不导出为公有适配层或修改上游。

I46读取[Chrome原生长动画帧说明](https://developer.chrome.com/docs/web-platform/long-animation-frames)与[W3C FPWD](https://www.w3.org/TR/long-animation-frames/)；后者是2026-04-28工作草案，非已完成Recommendation。能力检测后观察，裁剪原生Frame/Script标量，不保留Window引用。归因指平台入口，长帧总时间、blocking、脚本执行/样式布局时间意义不同；没有脚本条目不能推出没有JS工作。SDK只在诊断bundle现有BoundedTransport表达式上加计时，根源码/依赖不变；因bundle已改，不能拿原始sourcemap映射修改后的字符偏移。

I58获得用户独立Chrome/Playwright授权后读[BrowserType.launch](https://playwright.dev/docs/api/class-browsertype#browser-type-launch)、[CDPSession](https://playwright.dev/docs/api/class-cdpsession)与[Tracing协议](https://chromedevtools.github.io/devtools-protocol/tot/Tracing/)、[IO协议](https://chromedevtools.github.io/devtools-protocol/tot/IO/)，并核本机Playwright1.62.1 coreBundle原生start/stopTracing及protocol.d.ts，SHA见[tool-source-review](../../../runs/ros2_web_transport_20261030/playwright-native/tool-source-review.json)。浏览器newCDPSession/Tracing.start→end/tracingComplete/IO.read顺序64KiB写盘，显式缓冲/文件/RSS/时长上限，不积累原始ROS载荷。Chrome非bundled可执行组合不当作官方兼容保证；设备、版本和实际成功另测。dataLossOccurred或缺阶段mark只作PARTIAL；未采完整调度类别不能把已采集scope当全浏览器最长任务。CPU ProfileChunk负delta当前语义未核清，旧毫秒权重弃用，仅保留节点采样计数；没有据此修改产品GL/传输实现。I58后端未知杀父信号和SIGTERM控制复现分别记录，不互相替代。

I59查[Python3.12 signal](https://docs.python.org/3.12/library/signal.html#execution-of-python-signal-handlers)及本机signal.py、Jazzy launch_service.py（SHA记在本轮ledger）：Python handler在主线程解释器检查点运行，任意raise可能打断资源登记；Jazzy launch的SIGINT走shutdown，SIGTERM分支明确提示可能留孤儿。因此只登记轻量终止请求，由主路径检查并继续现成Popen/进程组清理；不改launch或系统库。subprocess.run仍拥有并回收预检子进程，保持10秒timeout。外层本机临时systemd service经实际/proc/PID/cgroup、memory.max=1073741824、memory.swap.max=0验证独占组与限额；stdout MemoryPeak与内核memory.peak不一致时保留两者并采用明确内核量，不混称RSS。用户业务集成不因此引入systemd SDK或自研进程管理框架。

## I60–I61 Service寿命与受管适配来源

写实现前复核本机Jazzy rclpy7.1.12 Client/Node/Executor/Task/Future/CallbackGroup及rosbridge2.7.1 services/executor_helpers/websocket，逐函数与原生二进制指纹见service-client-design/task2-source-review.json和源码常量。远端同tag与本机队列布局不同，不以版本号单独冒称兼容。标准Node.clients识别注册后wake异常；公共remove_pending_request清自身Future；互斥组+已审owner任务边界处理Destroyable的延迟退役。官方文档站当前Anubis拒绝时读官方raw及本机apt源码，原记录见native-plan.md。

Python3.12 inspect.getsource可接受code object，getsourcelines对function会自动unwrap。本轮实际装饰器反例证实单看被展开函数会误认替代代码，因此运行布局、WAIT、native指纹均核实际__code__源码；没有放宽原哈希白名单。适配仅支持已验证布局，未知包装明确拒绝。参考[Python3.12 inspect](https://docs.python.org/3.12/library/inspect.html#inspect.getsource)、[rclpy Destroyable](https://raw.githubusercontent.com/ros2/rclpy/7.1.12/rclpy/src/rclpy/destroyable.cpp)、[CallbackGroup](https://raw.githubusercontent.com/ros2/rclpy/jazzy/rclpy/rclpy/callback_groups.py)。


I62按本机上游发布者/线程/launch实际实现追溯退出：外部tag与apt源码的逐行布局不同，以[source-review-i62.json](../../../runs/ros2_web_transport_20261030/managed-lifecycle/source-review-i62.json)保存的本机SHA和原生行为为准。[rosbridge2.7.1 publishers](https://raw.githubusercontent.com/RobotWebTools/rosbridge_suite/2.7.1/rosbridge_library/src/rosbridge_library/internal/publishers.py)的默认10秒注销Timer继承[Python3.12线程daemon](https://docs.python.org/3.12/library/threading.html#threading.Timer)；[Jazzy ExecuteLocal](https://raw.githubusercontent.com/ros2/launch/jazzy/launch/launch/actions/execute_local.py)由noninteractive决定收到根INT时是否转发。只使用Node公有sigterm_timeout=15及DEVNULL/单根INT/父18秒等待，未修改系统库或引入计时器框架；复现、红绿及限制见development.md I62。

## 0.7.1标准预览复用

审查确认标准面板原先直接Blob而绕过ImageView已有预算。修复只导出既有compressedImage校验的薄入口compressedImagePayload，web消费此公有入口及项目maxPixels；不新增图片解析算法/依赖，不改decodeImage或ImageView。原image-dimensions2.6.0源码/API、CompressedImage的format与特殊compressedDepth约束依据沿用本文件已有阅读。新入口类型与资源所有权见io.md。

原生Chrome性能捕获依据CDP Tracing公有协议，有界32MiB/文件128MiB且检查dataLossOccurred及同文档标记；原4MiB丢失轨迹保留。实际X显示为:1，修正仅测试guard；Xauthority只交给原生客户端，未输出cookie。公开LongTask/LoAF与CPU提交时间不等于GPU呈现；ProfileChunk负timeDelta仅按样本次数解读。所有层次与源码映射/记录见development.md I64–I65，不声称读遍RViz/rqt所有源码或完整GitHub机器人项目。

I66仅修开发browser_probe.mjs的实际DOM展开，并增加项目/完整下载/清空与布局验收；生产SDK、Python、web、Scene/Media正文和54dist与0.7.1正式长测完全一致。最终源码归档的差异绑定明确列出README/doc与这一个探针，不将后续文档写入原已验归档，也不覆盖旧失败。
# 0.8.2 回放交互核对来源

本轮先核 [W3C Event Timing](https://w3c.github.io/event-timing/)、[Long Tasks](https://w3c.github.io/longtasks/) 与 [Chrome Long Animation Frames](https://developer.chrome.com/docs/web-platform/long-animation-frames)：原生观察器只作为有界诊断；未来事件阈值 16ms，duration 8ms 量化，连续移动/滚轮不计入 Event Timing，不把离散样本 p95 写成 INP。探针按 entry.startTime 分阶段，记录跨界项；最多保留 6000 个标量条目，LoAF 仅保留最长 4 个脚本摘要，不保留 DOM/Window/payload。

图谱使用 [Cytoscape 视口与布局文档](https://js.cytoscape.org/)，并核本地锁定 3.34.4 的 `index.d.ts`、`src/core/viewport.mjs`、`src/extensions/layout/grid.mjs`、`breadthfirst.mjs` 与 renderer `coord-ele-math/labels.mjs`。沿用内置 grid/层级布局、set zoom 与 renderedPosition 中心，不新增图算法或运行依赖。原 ResizeObserver 每次 fit 是页签恢复重置缩放的根因；原 text-overflow-wrap=whitespace 不能折断 ROS 路径，改用官方 anywhere 和标签高度。前三次 wheel 有上游限幅，原生复现使用多次真实滚轮并先检查缩放确实发生。

