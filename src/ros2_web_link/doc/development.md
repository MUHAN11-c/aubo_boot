# 开发、参考、问题与验证记录

整理日期：2026-10-30，Asia/Shanghai。当前源码0.7.1：默认q64/cache64/shared/FastDDS/noXML、无诊断的1800秒完整Peach录制负载通过，2691RPC零失败；SDK17项、真实表单10组、页面demo12项与修正后的随包浏览器CLI21项分别通过。桌面原生性能轨迹及无profiler对照见I64；本版预算修复/冻结/长测见I65，开发工具与最终交付见I66。旧版本、失败和撤回方案全部保留；Android本轮用户暂不测，历史IAB冷GL与OOM脚本身份仍未知，不称任意设备/网络或无限时长均已验收。本文按用户要求汇集整个独立包的实现和验证细节，原始输出位于工作区runs/ros2_web_*；目录名包含fixed/final/green不自动代表通过，以各报告实际范围与终态为准。

本记录根据当前源码与已落盘证据整理。没有留存的早期逐行差异、执行步骤或结果不能从当前实现反推，不补写成已经发生的事实。后续修改应同步补充问题条目、复测和证据索引。

## 阅读入口与证据层次

- [architecture.md](architecture.md)：当前运行结构、组件边界、权限与资源所有权。
- [io.md](io.md)：公有API、类型、QoS、截止时间、错误及预算。
- [capabilities.md](capabilities.md)：需求映射、固定技术版本、实现和未实现范围。
- [source.md](source.md)：官方/上游源码来源、许可证及窄兼容点的复核依据。
- [launch.md](launch.md)：构建产物、组合启动、配置、白名单和资源路由。
- [integration.md](integration.md)：其他ROS/Web项目的接入及本地源码重建。
- [testing.md](testing.md)：通信与故障用例、执行命令、判据、收尾。
- [project-testing.md](project-testing.md)：项目选择、Peach在线/录制数据及操作方法。
- [performance.md](performance.md)：原始性能数值、失败、对照、适用范围和未完成门。
- [evidence-index.md](evidence-index.md)：本次整理时存在的所有相关证据文件与源码文件索引。
- [evidence-catalog.json](evidence-catalog.json)：上述文件的大小、可用哈希、报告原有状态和索引生成边界，可供脚本复核。

当前快照描述现在的行为，本文的问题条目解释修改原因，原始运行目录提供可复查结果。报告中的PASS仅覆盖该报告声明的版本、环境、时长和断言。官方lint、受控协议探针、原生DDS、真实浏览器、真机/Android分别保留覆盖范围，不能相互替代。

## 目标与已确认约束

独立包为不同项目提供标准ROS 2通信与可选观测组件。Peach、IVG和Rebot-Arm用于提出需求；暂不迁移Peach/IVG业务，不在通用SDK内实现采摘状态机、重建算法、机械臂SDK或新的运动权限框架。IVG硬件SDK的FK/IK只读对比分析路径按用户意见保留。

Web默认直接连接rosbridge；Peach原HTTP对照入口保留在其业务项目，本包不替换它。本包HTTP只提供静态页、配置及允许的模型资源，不增加ROS业务HTTP代理或Fastify中转。业务仍通过Topic、Service、Action、节点参数和Lifecycle表达原生接口。

所有运行依赖本地提供、版本锁定、源码可读可改；自有代码Apache-2.0，复用文件保留原许可。Node仅构建时使用。页面采用单一自测工作台和按需加载组件；没有另造一个完整Lichtblick/RViz插件平台。

测试不执行真机运动、SetIO或dashboard。驱动栈只读；Gazebo按2026-10-28用户裁定暂停，不作为联调入口。允许mock和已录制数据。相机关闭前已录制，之后的回放不能证明相机或业务节点当前在线。测试结束清理本轮进程，不停止其他操作员的程序。

## 技术与参考的具体落点

版本、职责和许可的当前唯一核对表位于[capabilities.md](capabilities.md)。运行依赖为roslib2.1.0、cbor2 2.4.1、Three0.186.1、URDFLoader0.13.1、uPlot1.6.32及gl-matrix3.4.4；TF收录Lichtblick v1.30.0的必要源码。ROS端使用Jazzy apt的rosbridge/rosapi 2.7.1及rclpy，不重复pip安装同名ROS库，不改变numpy1.26.4约束。

采用的模式及原因：

1. Jazzy的Topic/Service/Action分工：流数据、短请求、可取消长任务分别使用对应原生接口，避免所有操作都变成HTTP请求或参数开关。
2. ROS标准QoS：普通传感器可丢帧，命令和服务可靠，静态TF为transient_local。具体大流必须核对发布端和DDS环境，不能由名称推定可靠性。
3. roslib公开Transport和成熟CBOR库：保留标准协议，只补预算、分片生命周期和响应关联边界，不自写CDR或第二套协议服务器。
4. RViz/rqt的标准消息与按需观测思路：URDF、JointState、TF、Marker、PointCloud2、Image、参数、Lifecycle继续使用ROS原生契约。当前并未完整实现这些工具的全部显示语义，也不宣称已经逐文件审计所有RViz/rqt或机器人GitHub仓库。
5. Lichtblick的TF树、插值与几何复用：只留订阅、缓存预算、时钟回退及显示适配，避免在Web端再造积分算法的TF权威。
6. 原生rosbag2：录制/循环回放/clock由ROS工具负责，网页不重写MCAP存储与播放器。
7. IVG参考用于限频、发送缓冲和资源加载的需求核对；旧项目现有可用性不能直接作为新包性能证明，不复制其项目协议和FK/IK路径。

上游完整源码缓存与固定提交见[source.md](source.md)。ROS官方入口包括[Topic/Service/Action](https://docs.ros.org/en/jazzy/How-To-Guides/Topics-Services-Actions.html)、[QoS](https://repo.test.ros2.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)、[服务内省](https://repo.test.ros2.org/en/jazzy/Tutorials/Demos/Service-Introspection.html)。文档与本机发行版行为不同，以锁定源码和实际行为核对，不凭另一发行版记忆替换API。

## 编程结构和资源责任

核心通信不依赖DOM、Three或uPlot。四个npm入口为根SDK、`/scene`、`/media`和`/audit`；单一页面复用它们，不复制通信实现。

### 核心SDK

- `sdk/index.ts`：RosClient连接、状态、订阅共享、发布、Service、Action、图谱、类型及Lifecycle。每次连接创建新ROSLIB.Ros，断线恢复仍登记的订阅，命令不补发。Service/Action用Map关联ID，共享可配置等待预算。
- `sdk/transport.ts`：公开Transport的WebSocket适配、CBOR解码、JSON分片与字节/数量/时限预算、发送积压拒绝、响应安全路由和统计。对没有pending ID的响应计入ignoredResponses；该计数本身不携带具体CID/到达时间。
- `sdk/parameters.ts`：节点标准参数服务、九类值、描述/范围/只读信息、逐项与原子写入；超出JavaScript安全整数范围明确拒绝，不悄悄舍入。
- `sdk/errors.ts`：稳定的错误类型及安全错误文本。

默认完整帧16MB、分片内容总量32MB、8个在途分片ID、每条4096片、30秒固定TTL、发送积压16MB、CBOR深度64。本地Service/Action等待默认256、范围1..4096、构造后不可变。这些是不同层的边界，不是整个进程RSS上限。完整字段和错误码见[io.md](io.md)。

### 观测与项目验收

- `scene/tf.ts`：Lichtblick适配，10秒动态历史、最多256帧/每帧500样本；精确时间、共同latest时间、静态TF和/clock回退。
- `scene/pointcloud.ts`：按PointField、大小端、offset、row_step解析PointCloud2，处理RGB与NaN，输入/采样有界。
- `scene/index.ts`：Three、URDFLoader、OrbitControls、模型/测量关节、TF轴、Marker0..11、点云及Path；实例拥有订阅、RAF、观察器、GL对象与加载世代，dispose回收。
- `scene/shaders.ts`：本上下文的链接错误检查与详细日志模式，销毁恢复方法；不修改全局WebGL原型。
- `media/image.ts`：原始/压缩图像解码、stride/大小端/深度单位和有效像素边界。
- `media/index.ts`：ImageView订阅和canvas/图片资源所有权；`media/plot.ts`用uPlot维护有限字段和样本，合并重绘、时钟回退清历史。
- `audit/index.ts`：项目数据契约、选择式只读/交互用例、前置条件、取消与finally收尾、报告。交互不因页面连接自动执行；回放显式recorded，不做在线新鲜度断言。

### 页面与ROS组合

- `web/index.html/style.css`：一个工作台、导航、项目验收卡片及通信/观测面板；平板用同一HTTP/WS页面，布局模拟不等于Android实机。
- `web/app.js`：连接与手动面板、按需组件、操作记录、有界预览。组件共享同一客户端；项目验收期间会暂停手动观测以避免混用统计。
- `web/project.js`：项目选择、用例勾选、执行状态、QoS填表保真、摘要和完整报告导出。
- `scripts/serve.py`：静态文件、`/config.json`和ament包资源白名单；真实路径检查拒绝symlink逃逸，不提供任意文件或ROS业务代理。
- `scripts/bridge_node.py`：执行原上游桥，仅在自有进程安装下述窄兼容点；`rosapi_node.py`执行上游rosapi并适配本机Action图谱API差异。
- `scripts/demo_node.py`：无硬件原生Node/Lifecycle/Service/Action和合成观测示例，不代表Peach业务已经验证。
- `launch/web.launch.py`为组合入口，`self_test.launch.py`选择demo，`bridge.launch.py`兼容转发，`replay.launch.py`组合原生播放器。关键进程退出关闭本组，默认localhost/demo=false，不固定现场DDS域。
- `config/profiles.yaml/projects.yaml/replay_qos.yaml`分别声明视图/预算、项目数据契约和回放QoS。业务名不进入SDK源码；项目差异可用数据配置表达。
- `scripts/build.mjs`生成ESM、声明、浏览器脚本/source map、许可证及锁副本；CMake在依赖已安装时构建网页，绝不在launch隐式npm安装。

完整源码文件与SHA256见证据目录。其余`scripts/*probe*`及负载工具的用途按测试分层归纳如下，逐个文件也纳入索引。

## 重构与问题修复账目

每项列出触发、修改、复测与边界。已修复项只表示对应问题的证据成立，不替代后面仍失败的总体门。

### I01：示例绿灯不足以代表真实项目

最初demo通信页面不能回答Peach实际消息/观测是否可用。增加独立audit入口、项目数据契约、可选择只读和交互用例、单项证据及未执行状态；在线Peach17只读+3默认不选交互，回放10只读，demo6只读+2交互。新增原生回放入口与recorded时间模式；不引入业务依赖。`ros2_web_live_camera_mock_20261028`保存真实相机/mock及关闭前录制，`bounded-ui`、`visual-v067`保存真实录制项目的10/10页面结果。回放不能证明在线服务/Action或真机动作。

### I02：URDF资源/加载回调与实际库签名不符

URDFLoader旧README例子的loadMeshCb参数少于实际0.13.1源码/d.ts；按四参数实现，package回调不再额外追加分隔符。HTTP严格保留资源包/路径白名单，不为加载模型开放任意磁盘路径。实际Peach12/12资产、6活动关节的证据见`bounded-ui`、`joint-cache-ui`及`visual-v067`。未验证所有DAE/glTF材质变体。

### I03：rosapi Action图谱调用导致进程退出

本机rosapi调用不存在的Node.get_action_names_and_types。`rosapi_node.py`仅在缺少该方法时使用公开rclpy.action.graph API，随后执行原节点；源头和升级复核见source.md。原生SDK探针覆盖Action类型/字段发现，不修改apt源码。

### I04：非空byte-array参数响应不能JSON编码

Jazzy octet序列表示为list[bytes]，上游快捷转换绕过标量octet→int。自有桥仅对octet列表复用上游标量转换，其余类型不变。九类参数与原生SDK探针分别核对；标准参数仍调用节点服务，不启用rosapi旧参数接口。

### I05：并发分片外层ID重复、混合大图/点云

上游2.7.1每条消息新建Fragmentation，局部seed重置；原生失败保留在`bounded-native-fragments`和`bounded-native-fragments-diagnosis`。`install_fragment_id_compat`只在该版本为每个物理消息指定UUID，内部RPC/Action ID不改。`bounded-native-fragments-fixed`与`bounded-ui/rpc-fragments.log`分别核对真实大数据和512字节RPC分片；SDK异常重组边界由wire探针核对。

### I06：晚加入客户端缺少多个静态TF writer

上游临时订阅仅回放首条保留消息，剩余writer样本未到即销毁。`static_tf_compat.py`只对transient_local TFMessage为晚客户端保留独立reader，复用上游发送和executor销毁，不广播新TF、不另缓存一棵权威树。SDK静态TF的queue_length与QoS depth都设100。`static_tf_probe.mjs`及多次客户端重建核对晚订阅、重复订阅和释放。

### I07：时间语义、缓存与回放循环

TF计算复用Lichtblick；动态按指定stamp、latest按共同有效时间，缺TF隐藏并说明原因。/clock倒退清动态历史、保留静态；乱序包不当作全局时钟回退。零stamp显示允许latest，非锁定Marker定位后保留fixedFrame位置。曲线按header回退清历史。受控观测探针和实际回放循环分别核对；Web观测不能替代ROS重建侧tf2积分契约。

### I08：大帧、分片、发送与响应路由无完整边界

加入独立传输预算、固定分片截止时间和安全响应ID过滤；CBOR复用cbor2，不自写二进制协议。异常帧/丢片/乱序/重复/超限/重入/回调异常与清理由wire故障探针核对；原生大图JSON发布→DDS→CBOR回传以SHA256核对921600和2764800字节，不仅比较显示尺寸。当前SDK边界不等于桥write_queue的字节预算，详见I21。

### I09：忙主线程下迟到请求仍被判成功

只依赖setTimeout时，成功回调可能先于迟到定时器执行。`sdk-request-deadline`保存旧0.6.2的五项失败：迟到Service、Action反馈、处理中跨期、桥错误及延迟open。0.6.3改为单调绝对截止时间并在关键完成点检查，保留定时器清理和幂等finish；最大定时器的浮点取整溢出也先失败后钳制。没有重试或自动取消远端；忙线程不能准时回调，不宣称实时保证。

### I10：本地Service/Action等待总数无界

0.6.4增加共享请求数预算、Map关联、发送前拒绝和资源清理。`pending-budget`保存畸形Signal、不可变预算、重入、名额回收及统计兼容的红/绿；满额不排队也不发出命令。等待数量是计数预算，不是总内存或DDS执行并发限额。

### I11：持续点云/Path/Marker更新反复重建shader

`render-baseline`曾测得CPU提交p95约3559ms。改为复用材质、更新GPU属性、按整批Marker最终集合处理DELETEALL/ADD、替换帧后释放旧对象，并支持namespace删除。`render-marker-batch`稳定p95约4.5ms。资源/位置语义探针和真实渲染独立核对；不把CPU提交值称为GPU耗时，也不把这项修复当作首载已解决。

### I12：共享JointState带来无界名称缓存

旧实现收到3000个其他模型名后字典达到3001项。0.6.1将已知关节限制在当前URDF，早到/未知名称放入1024项、单名1024字符的LRU；拒绝伪array-like、超长名和非有限位置。模型替换、早到值、空position、CBOR Float64Array及reset由资源探针核对；`joint-cache-ui`实际模型缓存6/0/1024。没有改变ROS消息或发布关节指令。

### I13：项目QoS填入面板后丢失

Reliable项目配置被表单默认sensor覆盖，结构化策略也被替换。0.6.5增加统一fillQosSelect/readQosSelect，可见下拉项保留完整对象；Topic/Media/audit缺省值一致。`qos-controls/ui-red.json`、`ui-green.json`及真实深度画面保留修复前后。manual项目切换清理旧名称、类型、限频和自定义选项。外部端点图只能确认RELIABLE，History/Depth为UNKNOWN，未声称外部验证depth7。

### I14：Best Effort大图长时间无帧

常规回放多客户端约24秒因彩色10秒空窗失败。相同数据/单独QoS的官方CLI对照显示BE少帧/丢失，Reliable持续到帧；仅Peach录制项目显式Reliable，通用SDK sensor默认不变。证据为`qos-controls`。未抓包证明具体RTPS分片丢失机制，不把此项当作Service超时根因。

### I15：展示大结果前完整序列化、日志保留原对象

0.6.6改成先生成有界快照：总遍历256项/8192字符、字符串256、对象32字段、数组16项、深度6；二进制显示元数据。面板18000字符、单日志4096、最多200条。项目摘要有界，SDK值与完整项目导出保持原契约。`ui-preview`14组纯探针及真实200014字符参数读取/页面完整JSON下载哈希通过；另一个约1MiB参数未读取，明确不计通过。这不是24.6GiB Python OOM的根因修复。

### I16：原生服务对照工具误判和证据覆盖

旧探针可能将3.1秒才完成的Future判PASS；现请求提交+等待共用绝对3秒门。两个进程的先exists后write竞态改为ROS init前独占JSONL，summary也拒覆盖。`native-churn-output-race-red/green`、`native-churn-negative`和deadline探针分别验证。持久/短命原生客户端120秒对照是逐次查询，不等于三个并发桥请求，也未复现原长测故障。

### I17：METADATA诊断破坏关键字与统计窗口

trace包装须保留rclpy公开srv_type/srv_name关键字；角色工厂探针保留失败与修复。RPC次数与成功延迟分开累计，最多36001样本/会话，不用2000项滚动窗口冒充全轮p95。默认关闭诊断，不新增HTTP或业务调用；高频短测只验证观察链，不替代2秒周期长测。

### I18：WAIT失败后缺少取证时间

最初5秒收尾只识别旧METADATA模式，WAIT超时后立即结束，导致原生错误来不及落盘。`service_failure_grace_probe.mjs`先复现后补两种模式；原3秒SDK门和失败判定不变。`source-rebuild-v066`的空文件仍保留为取证不足，不用修复后的说明覆盖旧结果。

### I19：诊断IO覆盖原错误或无限写入

WAIT文件/flush与stderr同时失败可能替换原调用错误，flush失败还可能无限重试。现IO前消耗最多8次尝试、二级日志隔离；真实Future/Event受控探针验证完成但通知未执行、采样不分类、write/flush失败和weakref无保留。RPC trace三阶段写失败同样隔离，原TIMEOUT和统计不变。它们修复诊断可靠性，不修复产品RPC超时。

### I20：WebGL首载阻塞，未解决

稳定绘制改进后仍曾有1.6–4.9秒构造、最长约5.2秒完整首帧。原生WebGL无ROS对照也有约1–3秒getContext，Three扩展获取也可能同步等待。errors-only减少成功日志查询，仍保留LINK_STATUS与失败日志；真实错误shader可被识别。compileAsync实验未稳定奏效，已撤回，`render-async`证据保留。

`visual-v067`前台实际约57.8提交帧/秒、并发CPU p95约1.8ms、两次构造260/308ms；这次GPU/浏览器已运行，不构成冷启动修复。尚无实际输入延迟或GPU计时验收，Android也未测试。不能仅加loading文案就宣称主线程阻塞已解决。

### I21：FastDDS长测RPC超时，未解决

完整负载为真实约4.1MB点云、RGB-D、TF/URDF/JointState、三个SDK及慢读者，每约2秒并发查询rosapi、3秒本地等待。多轮目标1800秒，但分别约638、760、603.6、527.2、540.69、988.19秒失败。成功p95较低不能抹去单次超时。失败轮各自的版本、参数、RSS、清理和原始输出见performance.md及证据索引。

527秒WAIT轮的两条CID与桥错误相同，取消前futureDone=false/cancelled=false/eventSet=false；已排除这两次是“Future已完成但通知未执行”。仍不能区分请求/服务处理/RMW匹配/取响应。

0.6.7服务端被动METADATA轮795次查询、2次失败，服务端接收/发送各795条，最后并发窗口三对事件均完成；桥wait失败文件为空。两名失败客户端各有ignoredResponses=1，只说明某响应处理时没有pending ID，尚未保存其CID及原始接收时刻，不能直接定为迟到、错配或丢包。

0.6.8第二候选的原时限长测在988.19秒FAIL：1458次RPC中2次TIMEOUT，峰值995.41MiB，未触1GiB门。两次CID尾号1456/1458均匹配桥wait=false与取消前Future未完成，原生WS最后收到同CID的result=false错误回复。因此这两次不是迟到的成功结果；服务端1458对接收/发送事件也不证明客户端交付。随后0.6.9的637秒失败已取得原生逐调用证据：该次成功take在waitFalse之后0.238ms返回，尚缺DDS历史插入时间以区分迟到/迟取，详见I24/I26。不能追溯推断0.6.7缺失的CID或其他轮次。禁止先延长超时、重试或替换应用默认RMW当作修复。

Cyclone已有1800秒2682次RPC零失败的证据；诊断开关会改变实体/时序，某轮METADATA通过也不能替代关闭诊断的FastDDS失败。当前没有任意RMW、数小时、真实LAN或多浏览器GPU容量保证。

### I22：整个Codex/浏览器窗口退出，已确认同组OOM，Python脚本身份未知

2026-10-28 18:09:33内核global OOM杀死Python PID553423，匿名RSS约24.6GiB；Codex主进程18:09:42启动，时间相差9秒。该时间关系支持内存压力相关推断，但无法恢复Python脚本/父进程，也不能证明桌面被直接OOM杀死。后续复查无新增内核崩溃记录，不能用“未找到日志”证明从未退出。保留`crash-evidence`和`crash-check-0320`。

2026-10-29 12:15只读复查无新增内核OOM/段错误/GPU重置，桌面主进程PID561018未变，可用内存约20GiB；本轮长测进程按PID/创建时间全部退出，8088/9090/20288/21290无监听。新原始记录为`crash-check-0320/recheck-20261029-1215.json`。该结果不排除未提供确切时间的纯应用退出；长测RPC超时、守卫收尾与历史整个窗口关闭分别记录。

14:38再查内核，自2026-10-28 17:50起仍仅上述18:09:33的global OOM，没有新增匹配的OOM/段错误/GPU重置；系统可用约18.6GiB。14:39桌面主进程仍为PID561018，名称ChatGPT、启动时间18:09:42.330。被杀Python的脚本身份及桌面直接退出原因继续标为未知；不能因为时间接近而断言Codex被内核直接杀死。见`crash-check-0320/recheck-20261029-1440.json`。

后续负载由有时间/RSS/可用内存门的父进程拥有，finally停止自己创建的组；这不是操作系统级cgroup隔离，也不限制外部浏览器RSS。实际被守卫终止/按时退出、ROS进程exit -2、页面连接拒绝与桌面闪退分别记录。

2026-10-29 19:04精确复查历史用户journal，补齐此前缺失的应用组证据：18:09:33 systemd记录`app-org.chromium.Chromium-11333.scope`内进程被OOM killer杀死；18:09:38该scope以`oom-kill`失败，同一时刻`chatgpt.desktop`的GPU进程报告不可恢复错误/context lost。旧Codex主进程为11333，新进程561018于18:09:42.330启动。因此“该次整窗关闭与系统内存耗尽相关”已有同应用组记录支持，不再仅凭9秒时间差推测；仍不能恢复Python553423的具体脚本，或断言内核直接选中Codex主进程作为牺牲者。用户未提供另一次退出的确切时间，不能将本次历史结论自动套用到任何新退出。当前主进程仍未变，可用内存23.49GiB；`coredumpctl`未安装，不宣称检查过全部core dump。见`crash-check-0320/recheck-20261029-1904.json`。首次journal解析遇到MESSAGE为字节数组，修正只读解析后完成；不涉及产品代码。

### I23：缺少成功调用及原生WS事件时间线，诊断也可能制造超时

0.6.7服务端轮出现795/2失败、空wait失败文件及未匹配响应计数，无法区分桥内成功等待、发送排队和客户端处理。0.6.8在WAIT模式正常退出时额外写`service-wait-final-PID.json`，最后64条完成调用、128KiB、单次尝试，包含CID、进入/等待/通知/返回时间及wall锚点。获取环锁最多50ms，忙时明确snapshotAvailable=false；文件open/write/close本身没有50ms保证，依赖父守卫最终终止。元数据不保留Future/Client/帧/响应值。

`scripts/service_wire_trace.mjs`只在显式CLI诊断时子类化原有globalThis.WebSocket，不把原生Node WebSocket换成ws实现；后者仍用于既有慢读者。仅检查不超过16384字符、前256字符含候选操作的小型JSON，保存send/sendReturn/receive、服务、CID、slot/generation与时间，不记录args/values。receive表示原生message事件执行，不能称网卡接收时间。Python单调时间为秒；Node performance.now为本进程起点的毫秒，不能直接相减跨进程比较；CID与各自wall/单调锚点共同解释。

首版临时同步标签在真实SDK异步创建transport时丢失，`wire-context-red.log`保留失败，现使用Node标准AsyncLocalStorage.run传播两个数字，三路并发连接标签核对通过。独立审查另发现同步writer可能在send或SDK onmessage前消耗请求预算；50ms本地RPC配100ms慢sink实际复现TIMEOUT，见`wire-slow-sink-red.log`。改为只缓存有界JSONL，关闭SDK并恢复全局WebSocket后显式flush。总24000条/8MiB预算不随flush重置，失败不重试、最多8次sink错误；预算/最终IO错误保留整轮FAIL。进程在flush前被信号或系统中止、未执行finally时，缓存可能丢失，不能宣称总能得到记录。

证据均在`service-response-timeline/`。受控final成功/写失败/close-flush失败/真实锁被占四种通过，既有Future/Event四种边界通过；实际原生Node WS+本地ws服务器及真实SDK覆盖标签、透传、隐私、条数/UTF-8字节预算、IO/二级报告失败，慢sink修复后50ms请求通过。早期32模式探针先因沙箱socket权限、再因未source包环境而未到预期检查，补齐执行环境后32组通过；这些是探针编排限制，不计产品故障。首次官方lint指出导入顺序、docstring语态及长行，修正后29项0错误/失败。

`native-smoke`实际Trigger成功与故意超时对应两条退出快照（约490/102ms），被动服务端接收/发送各2条、主动对照0、峰值424.75MiB、无残留。它不是FastDDS历史故障复现。`loaded-smoke`完整4.1MB点云/RGB-D、三SDK与慢读者30秒通过：45RPC、135条原生记录、45个CID与桥退出上下文精确匹配，最大桥调用约88.63ms，慢读暂停/恢复各1，峰值777.60MiB，缓冲和全部进程归零；未发生回放时钟循环。不能将此短测当1800秒门通过。49个dist运行文件与冻结0.6.7逐字节相同，另两项只是版本锁副本/HTML版本文字；本轮没有修改SDK或观测代码。

## 测试方法、工具与完整使用边界

### 官方构建与静态检查

`npm ci`使用根锁，`npm run check/build`验证类型/产物；`colcon build/test`只使用官方ament lint。具体数量随新增Python文件变化，不将早期17/19项套到最新24项。最新源码冻结重建独立记录；不是所有旧浏览器/原生测试都在每个新版本重跑。

### 无ROS的明确故障核对

`wire_fault_probe.mjs`核对传输、截止时间和pending边界；`observation_probe.mjs`核对图像/点云/TF；`scene_probe.mjs`核对位置/Marker语义；`scene_resources_probe.mjs`核对图形和关节资源；`shader_probe.mjs`核对公开GL边界；`load_progress_probe.mjs`核对逐流/慢读恢复门；`audit_probe.mjs`核对报告和交互finally；`preview_probe.mjs`核对格式预算。`service_*probe*`分别核对诊断角色、模式、失败取证、IO和原生等待边界。受控替代依赖明确记录，不能当作真实DDS或GPU性能结果。

### 原生ROS/DDS与端到端

`probe.mjs`连接真实示例，`roundtrip_probe.mjs`核对大图精确哈希，`fault_probe.mjs`只拥有自己的临时桥，`fragmentation_probe.py/rpc_fragment_probe.mjs`核对实际分片，`static_tf_probe.mjs`核对多writer晚订阅。`transport_probe.mjs`处理实际项目组合；`multi_client_probe.mjs`维护独立客户端、逐流进度、替换、TF和慢读恢复。`service_churn_probe.py`为顺序原生对照，不等于并发负载。

`run_transport_benchmark.py`是这些显式工具的父守卫：启动前检查端口和隔离域，记录PID/命令/创建时间/RSS，运行中1GiB组合门和至少4GiB可用内存门，结束停止本轮组并核对remainingPids。UI模式2..600秒，多客户端30..3600秒。不得同时在同域加入未声明探针并仍声称负载相同。

### 真实页面与交互

历史browser_probe保存全部操作组、截图及布局模拟；当前实际UI核对使用CUA读取DOM/可见画面并操作按钮、拖动、滚轮。没有从隐藏页面变量直接取报告冒充界面验证。下载完整报告与页面摘要分开；某轮未下载就明确写未下载。

`visual-v067`执行10只读、完整点云、12资产、21TF、6活动关节、33Marker、Path、彩色/深度与曲线、旋转/缩放/图层/固定坐标系/停止重启。记录前台visibility、产品RAF计数、CPU提交、长任务和清理；没有GPU计时、精确输入延迟、冷GPU或Android结论。217秒结束后进程/端口和自己的标签都清理。

### 真实数据来源

`capture_before_camera_off/peach_rgbd_mock_60s`约59.47秒、1.1GiB，含42话题/23214消息及完整云；另一个Reliable30秒包约434.7MiB、29.24秒、17话题/9466消息，但不含完整云。录制丢帧分别有原始记录，不能当无损标定集。已有5项校验和核对；索引对大二进制只记大小及已有校验文件，不重新假装完成逐文件哈希。源时间保留，深度单位来自实际配置而不是猜测毫米。

复现实操命令以testing.md、project-testing.md、integration.md为准；运行目录中的run-config/owned/resources进一步固定该轮参数。自测页面手工功能与自动探针覆盖是两条证据，不以一个绿灯代替所有功能。

## 版本、源码分发与可复现性

早期通信、可选观测、项目audit、实际回放、大帧/分片及渲染重用逐步落地；后续0.6.1关节缓存、0.6.2归档源码闭包、0.6.3绝对截止、0.6.4pending预算、0.6.5表单QoS、0.6.6有界预览/WAIT、0.6.7服务端被动METADATA、0.6.8完成调用/原生WS事件取证分别保留独立证据。版本相近不代表全部行为相同，测试应绑定归档SHA256或声明逐字节不变的模块。

`source-rebuild-v062`至`v067`的artifact/verification记录本地tgz、源目录和重建。0.6.7归档SHA256为`c81ed19fd07cec47ddbedde33e40506c703c155b0a3a13374c5501e6b3319c06`，139文件、离线缓存安装27依赖、51个dist文件重建逐字节相同、仅Jazzy的隔离ROS构建通过。相对0.6.6-grace的49个运行文件未变，另外是版本锁副本和HTML版本文字。该归档形成于本次新增文档之前；它不会自动包含后来写入的开发记录，也未通过FastDDS长测。

npm按官方规则不打包根package-lock；构建将原样副本放入dist/build-package-lock.json，收件人恢复到根后npm ci，再check/build。四个dist入口可直接在其他Web工程本地安装，不要求生产端运行Node。源码、类型、launch/配置、CMake、许可及source map随包分发；大型runs证据不塞进npm运行包，需随验收资料另行保留。

不得直接改node_modules或/opt/ros然后把临时状态当可复现交付；上游修改使用明确本地依赖/overlay，并重跑对应兼容探针。没有创造虚假的历史提交或把未保存的旧实现当作已可复现。

## 当前未完成与下一轮记录要求

1. FastDDS长测失败的精确阶段与根因、修复后关闭诊断的相同负载复测。
2. WebGL冷启动同步阻塞及真实输入延迟；当前前台短测不能代替。
3. 真实LAN、Android/平板触控与GPU、长时容量；没有设备证据就保留未验收。
4. 历史OOM脚本身份/桌面退出直接原因；缺失历史证据不能靠猜补齐。
5. 新文档和后续修改完成后的分发归档/证据目录一致性复核。

后续每次修改记录：触发与最小复现、查阅的本发行版文档/源码、选择的最小改动及文件、原始失败、修复后覆盖/未覆盖、实际命令与版本/配置、资源/退出检查。撤回的方案保留撤回原因和证据；没有执行的方案写“待实施”。不得因报告更整齐删掉失败、缩短原目标或扩大PASS含义。

## 证据索引维护

本机维护源码位于`runs/ros2_web_transport_20261029/documentation-audit/refresh_catalog.py`，也被原始文件索引记录。测试全部退出、日志文件关闭后执行该脚本，会从当前源文件及所有runs/ros2_web*重建目录并核对相对链接；它不执行测试、不生成总体PASS、不重写原始运行报告。不要把stdout重定向到正在被索引的runs文件，否则该文件在采样后继续增长会立刻使快照过期。归档生成后可以再次更新目录；归档内目录的生成时间可能早于归档本身，不在归档内自引用其哈希。

### I23后续：旧诊断IO、异步收尾与第二候选

独立复核继续定位到旧rpcTrace和full METADATA的1秒stack.request也会同步写盘。rpc-buffer-red和stack-async-red分别复现实际query/旗标代码仍有2次同步写。现wire/rpc复用同一薄缓存工具，各自独立24000条/8MiB总预算，flush不重置；stack用Node22 fs/promises.writeFile，合并为最多一个在途Promise，SDK释放/全局恢复后先await，再检查errors，避免晚ENOSPC漏进PASS。受控延迟失败、三阶段RPC计数与跨flush预算均通过，最终只读审查无新实际问题。

loaded-buffered-smoke另以完整实际录制30秒验证45RPC、135wire及90rpc元数据，45CID与桥最终记录相同；最大桥调用84.85ms、峰值805.72MiB、两缓存与进程清理归零、慢读暂停/恢复各1、未经历clock循环。这是新候选短测，不追溯改写首次loaded-smoke。source-rebuild-v068首候选归档221d634b97be9ba4495733fcf397cadc807d6ba254949fcbaf87d09d3ccd40e3及51dist/隔离构建结果保留，其形成于本次旧RPC日志修正之前，不能当新CLI的冻结证据。后续使用独立buffered-rpc候选目录。

第二候选`source-rebuild-v068-buffered-rpc`归档含146文件，SHA256=`1af780fd0a2f4abd331baa77a646137a8fce8dbfcd06b5d406b2f392c498e638`。离线缓存安装27依赖、51dist逐字节重建一致、四个公有入口导入和仅Jazzy隔离构建通过。该结果证明源码闭包，不是1800秒运行门；长测以此候选源码和安装前缀执行，新增输出形成后再次刷新目录。

### I24：原生Client取样证据缺口与旧快照误接收（0.6.9）

988秒失败已按CID确认错误回复及未完成Future，尚缺native send/take阶段。新增`service_native_trace.py`与显式`--trace-native`，复用WAIT的线程局部CID，包装精确审阅的rclpy7.1.12绑定；默认通信、SDK、显示组件和RMW不改。原生地址只作标识，不读内存。模块、CMake安装、父守卫及真实fixture同步接入；源码依据/精确哈希见source.md，操作与限制见testing.md。

先保存native-red（缺少模块）与modes-red（未清继承native变量），再实现记录与显式环境选择。受控basic/budget/sink/guard四组核对原返回/异常、空取样、CID/序号、Client弱引用释放、隐私、容量、退出写盘失败及拒绝未知构建；80种模式组合通过。既有final成功/写盘/flush/忙锁四组和Future/Event四组回归通过。首次回归误用了旧case名称notify，CLI拒绝后改用其真实done/sampling选项，原输出保留，不计产品故障。首次官方lint有7项长行/缩进错误，修正后31项零错误/失败。

`native-smoke`实际Jazzy Trigger两次查询：成功CID的sendReturn与takeReturn同Client标识/序号，故意超时有原生send，非目标服务不记录。共6条metadata、零drop/error，服务端接收/发送各2，主动原生查询0；进程树峰值432.86MiB、无残留。故意超时不是历史FastDDS故障的复现，不据此宣布修复。

独立只读审查指出三处证据风险并修正：fixture先写guard-result再验native会漏报，现清理后先验并记录再抛错；主异常与缺证并存时缺证原因丢失，现resources独立diagnosticFailure；父工具复用目录时旧快照可能冒充本轮，现启动前原子拒绝任何既有目录。stale-evidence-red-checked在原代码确实到达图查询，修复后80组合加旧目录拒绝通过且旧文件不改。最初stale-evidence-red受MagicMock.stdout真值干扰报domain占用，保留并明确不作为该产品缺陷的复现。审查复核无新实际缺陷；仅静态审查，不能代替最终版本真实fixture/完整负载验证。

最终版本`native-final`真实fixture另目录通过，峰值430.83MiB、diagnosticFailure为空、无残留。`loaded-smoke`用完整实际录制、三SDK加慢读者30秒通过：45RPC零失败，45个CID在原生send/成功take、原生WS及桥完成记录精确匹配；288条native元数据包含空取样，没有drop/error/预算超限。峰值785.00MiB、慢读暂停/恢复各1、0次clock循环，本地和所有进程归零。空取样为原生API允许返回，不按其数量推导丢包。两项诊断报告问题修正后官方31项仍零错误/失败。

0.6.9冻结归档148文件，SHA256 `9fe7a0e3699a69c249df994744a44114754132c07318206f29aa0e8b65356494`，目录`source-rebuild-v069`。首次临时npm缓存缺ws8.22.0而ENOTCACHED，未联网补装；改用本机已有`/home/mu/.npm`离线缓存后安装27依赖、check/build、51dist逐字节重建、四公有入口导入及仅Jazzy隔离构建通过。8个关键脚本与工作区逐字节一致。相对上一候选49个dist运行文件未变，另两项为锁文件副本与HTML版本文字；不是新一次浏览器/Android验收。

该冻结候选在独立域163/20488与21690端口运行原定1800秒长测，使用原2秒RPC周期/3秒门、完整录制、3SDK及慢读者，追加WAIT/服务端被动/native诊断。初次与619秒检查均确认同一组进程存活，随后637.04秒终态FAIL：939次RPC中1次TIMEOUT，峰值920.45MiB、10次clock回退、慢读暂停/恢复各14次，未到1080秒第二次替换。原生记录6468条、无drop/error/超预算，服务端939对事件；所有资源和进程清理，端口释放，见该轮summary/cleanup-verified。运行阶段文件不改成终态结论，也没有因为工具等待无输出重启测试。

失败CID尾号937在native send返回后3000.883ms成功取到同Client/sequence响应；该take比Event.wait返回False晚0.238ms，比取消触发的Event.set返回晚0.213ms。因此只能排除“一直没有原生成功取样”，还不能区分DDS较晚加入reader历史与executor较晚取样。此轮没有保存RMW source/received时间，不推测其未观测的到达时刻，不将时间接近取消当作唤醒根因证据。后续取证见I26。

后续仍须完成原定长测，使用native标识/序号收窄历史超时阶段；没有取样事件不直接判网络丢包。原1800秒门、WebGL冷启动、真实终端和历史OOM均未因此完成。

### I25：完整点云字节往返检查不足

原roundtrip_probe只对921600/2764800字节合成图像做完整SHA256；实际项目点云探针验证结构、采样解码、TF及流进度，不能据此断言完整原始云逐字节一致。沿用同一显式工具，增加172372点×24=4136928字节确定性PointCloud2，大小参照真实Peach注册云，填入有限XYZ、packed RGB及非零padding；数据本身不是相机原帧。只向两个`/web_link_demo/roundtrip_*`话题发布，JSON/Base64→DDS后分别JSON和CBOR接收，核对全数据SHA256、完整布局、首/中/末显示采样点XYZ/RGB及释放计数。

只读审查先发现“sampled geometry”没有数值断言、publisher创建位于timer清理try之外。补数值核对，将subscribe/advertise纳入同一try/finally，立即观察Promise拒绝而保留原await结果。实际首次运行又暴露新增const topic的TDZ遮蔽，未进入原图像检查，修为cloudTopic。第二次与构建同时读取被build.mjs删除重建的dist，报模块缺失；这是本轮编排错误，不是DDS失败，后续必须顺序执行或用不可变冻结目录。两轮原始FAIL及清理结果保留于`payload-integrity/native`与`native-fixed`。

构建完成后顺序执行的`payload-integrity/native-final`通过：两图及完整点云两种接收编码的SHA256正确，云哈希为`178fc69fddaaac973481f3cd0cb00a2d2e20f7a7bece8372498657ba325677ca`，几何/RGB采样正确；约6.17秒、峰值623.21MiB、无残留。父守卫源码也保留在该目录上级，45秒/1GiB/至少4GiB可用内存门、空域/空端口、新证据目录、严格两个话题白名单，不启用机器人/相机/Gazebo。此项不证明真实相机逐帧哈希、持续容量或GPU性能。

### I26：DDS历史插入时间与原生取样时间未分开（0.6.10）

为解释637秒失败，先审阅精确rclpy7.1.12 service_info.cpp/.hpp只读绑定、rmw types头、rmw_fastrtps8.4.4的响应取样代码及FastDDS2.14.7 SampleInfo文档。Web缓存无法读取绑定文件后，仅只读下载官方精确版本原稿到`native-reception-time/upstream-*`，保留Apache许可。来源/语义回链见source.md。

新增两个takeReturn元数据字段：sourceTimestampNs、receivedTimestampNs，以十进制字符串保留完整纳秒，避免JS安全整数截断；空header为null，0/负值原样保留但不当作有效交付时间。Fake header高精度断言先因缺字段失败，再修改原观察器；basic/budget/sink/guard及0/负值检查通过。未新增原生内存读取、实体、Future回调、payload保留或请求窗口IO，默认通信与SDK不变。31项官方lint通过；静态审查未发现实际缺陷，强调时钟不倒退/同域比较限制。

`native-reception-time/native`真实Jazzy fixture通过，峰值430.77MiB、无残留。成功响应的FastDDS写入→reader历史插入约0.160ms，历史插入→观察到take返回约0.208ms。它证明此环境两个时间字段可用，不是历史长测故障复现；后续仍须在原负载中捕捉失败，不据此断言executor或DDS根因已确定。

同目录`loaded-smoke`完整实际录制30秒通过：45RPC零失败，45CID跨原生WS、native send/成功take、桥完成记录精确对应，所有正纳秒字段有效；266条native元数据、零drop/error/超预算。写入→reader历史中位0.0187ms/最大0.494ms，历史→观察到take中位约0.381ms/最大约2.642ms（后者含浮点wall锚点精度界限）。进程树峰值801.04MiB、慢读暂停/恢复各1、0次clock循环，资源与进程归零。原长测失败门仍开放。

已只读查阅rmw_fastrtps8.4.4 README与FastDDS2.14.7扩展QoS/ROS配置：Writer heartbeat默认3秒，支持对特定服务回复使用XML profile。637秒失败的约3秒现象不能单独证明此机制；本轮未写入/应用XML、改全局RMW/QoS或增加周期唤醒，先取得失败时来源/接收时间。避免把可能的配置改动先当作修复。

0.6.10冻结归档`source-rebuild-v0610/local-ros2-web-link-0.6.10.tgz`含148文件，SHA256 `091d0ea9bff22c66375834a9eb21c9a0dc4b8adfe0336dcf476250bd89ee4646`。本机已有缓存离线安装、类型检查/构建、51dist逐字节重建、四公有入口导入和仅Jazzy隔离ROS构建均通过，9个关键脚本与工作区一致。49个SDK/显示运行dist与0.6.9相同，另两项为版本锁副本/HTML文字；本轮没有新的浏览器/GPU通过结论。后续长测必须使用该冻结源码/安装前缀，并保持原负载与3秒门。

该冻结候选的`long-fastdds-reception`在687.27秒终态FAIL：1014RPC中client2.1一次TIMEOUT，峰值955.12MiB、11次clock回退、慢读暂停/恢复各15次，未到第二次客户端替换。1014对服务端事件、6894条native记录无drop/error/超预算；失败CID尾号1014只有native send，没有成功take，因而没有该调用的RMW接收时间。正常调用时间不能补推失败调用。最后一轮服务端三对事件均在请求后数毫秒内，未观察到3秒服务端处理；但同侧GID仍不强行逐条配Web CID。所有进程/20688与22290端口均清理，详见该轮summary/cleanup-verified。原阶段RUNNING检查文件保留为历史状态，没有用工具无输出作为重启理由。

即时取消/销毁的reader可能在迟到响应可观察之前消失，因此继续重复同一观察器不能保证取得该时间字段。后续I27增加只在超时判定后的短暂reader保留；它不改变原3秒失败结果，但改变超时后的生命周期，必须作为另一种诊断负载单列，不能用于宣称原生命周期验收通过。

### I27：超时即销毁后无法观察迟到回执（0.6.11，诊断模式）

新增默认关闭的`--trace-late-client`，依赖native+WAIT且端点相同。先核对rosbridge2.7.1的call_service/_destroy_client_async和原executor调度器，并固定新增helper源码哈希`6b046376b027e2a3de5e5a513673bb9a830ee3cd0c1a4d8d1b708d50afd8f846`；读取Python3.12 Timer实现和rclpy Context.ok线程模型。服务仍按原期限返回超时并取消Future，不重试请求。只有waitFalse且Future未完成时标记当前Client id，在原销毁请求处用标准Timer多保留2秒，再调用原executor清理调度入口，不直接销毁原生handle。

本进程最多3次保留；标记阶段只存id/CID，Timer期间临时保留Node/Client引用，线程完成后释放。这是对原WAIT“不保留Client”边界的显式例外，不应隐藏成普通观察模式。无新DDS实体或Future回调；元数据只在退出写一次、最多3条/8KiB。timer构造/启动失败回退原清理，预算/提交错误、退出取消、缺证或未提交均使诊断验收失败。cleanupSubmittedAt只说明已调用调度入口，实际销毁还要看执行器/终态清理。

native旧场景缺模块、继承late变量未清理的失败先保留；实现后192模式/继承环境加旧目录拒绝通过。受控normal/constructor/budget/exit/sink五组通过：正常清理立即直通、仅已标记超时延后、弱引用释放、最多3次、退出取消不算完整证据、IO/二级报告失败单次隔离。Timer构造错误最初会逃逸并阻止原清理，先复现constructor-red后扩大try范围修正。原WAIT/native的12个边界场景回归通过。官方lint初次指出2项缩进/长行，修正后33项零错误/失败。只读审查无新增实际缺陷，但明确模式改变超时后生命周期、不是修复。

`service-late-client/native`真实延迟Trigger验证：网页仍返回timeoutRejected=true，native观察到失败CID的迟到成功take，约在wait返回后0.301秒；2秒后仅提交原清理，Timer/pending均0、无取消/错误/超预算，峰值430.64MiB，进程全部清理。它是受控延迟场景，不是历史FastDDS故障复现。

`loaded-smoke`完整录制/三SDK/慢读者30秒45RPC零失败，峰值791.01MiB、慢读暂停/恢复各1、0次clock循环。late记录为空、Timer/pending为0，证实正常请求未触发保留，资源和进程全部归零。一次本地摘要读取直接访问成功报告未定义的failure字段而KeyError，改为get后仅重新读取已有终态，没有重跑测试或改原报告。原1800秒失败门仍开放，后续使用本模式捕获迟到回执，并在真正修复后关闭所有诊断按原生命周期复测。

0.6.11冻结归档`source-rebuild-v0611/local-ros2-web-link-0.6.11.tgz`含150文件，SHA256 `5f67a8516f8ab3705d12adba9042a54c62bbf58920a8a990679cd4386cf0d1bf`；离线依赖安装/类型检查/构建、51dist逐字节重建、四公有入口导入、仅Jazzy隔离构建均通过，10个关键脚本与工作区一致。该归档含诊断模式，不是超时修复或原生命周期通过证据。

该冻结前缀的`long-fastdds-late-reader`使用域172/20888与22590端口，目标1800秒，原完整录制/三SDK/慢读者/2秒查询及3秒判定不变，仅显式开启超时后2秒保留。14:09核对同一组自有进程存活的`initial-live-check.json`保留为阶段证据；最终在1753.02秒FAIL：2595RPC中client0.1一次TIMEOUT，5代客户端、29次clock回退、慢读暂停/恢复各39次，峰值1009.31MiB，系统可用始终至少17659.48MiB。不是1800秒通过，也不是窗口崩溃。所有进程、20888/22590端口均清理，见该轮`summary.json`与`cleanup-verified.json`。

失败CID尾号2593在native send后约3秒取得成功回执；RMW原始字符串时间为source=`1793255649944644372`ns、reader历史插入=`1793255652943956965`ns，差2999.312593ms，插入至观测takeReturn约0.126ms。这里的接收时间是FastDDS reader历史插入，不是网卡到达；同机墙钟未跳变是跨字段相减的前提。这条证据将本次主要延迟定位到writer源时间至reader历史可见阶段，不能据此直接判定丢包、发现竞态或心跳恢复。

该次bridge Event在自己的wait截止前置位，waitReturned=true，没有触发late保留，退出时records/Timer/pending均0；SDK先到3秒期限，约3ms后收到同CID的result=true，原判定仍是FAIL。2595对服务端事件、17534条native元数据无drop/error/超预算，最后一批服务端三对请求/响应均在数毫秒内；不将并发GID强行逐条映射Web CID。原始诊断包装器始终安装，因此仍须在实际修复后关闭诊断按原生命周期复测。该轮未改变超时、未自动重试，也未应用FastDDS XML调参。

### I28：检验可靠响应的3秒周期是否控制迟到尾部（对照实验）

I27在失败调用上首次观察到source至reader历史插入2999.312593ms、随后take约0.126ms。以此提出单变量假设：若延迟尾部受默认可靠写入周期控制，缩短该周期应缩短reader历史可见尾部，而无需放宽网页3秒期限。这仍是假设，不以数值相近直接证明丢包、发现竞态或重传机制。

先读rmw_fastrtps 8.4.4的`rmw_service.cpp`、`qos.cpp`与README及Fast DDS 2.14.7的RTPSReliableWriterQos文档，再核对本机Publisher/DomainParticipantFactory头文件和WriterTimes默认构造。服务响应按完整DDS名选择writer profile：本例`rr/rosapi/nodesReply`，没有全局service/default profile；RMW随后覆盖ROS可靠性/深度等，不覆盖heartbeatPeriod。保持`RMW_FASTRTPS_USE_QOS_FROM_XML`未设，避免改变正常内存/发布模式与data sharing。

`heartbeat-profile/inspect_writer_qos.cpp`是显式本地检查器，不装入功能包：用Fast DDS公有profile API加载，再调用已安装RMW的服务QoS转换，对完整DataWriterQos做仅心跳差异核对。只短暂建无topic/writer/reader的participant/publisher，退出删除。编译先后暴露缺少传递include、全局dds别名冲突和auto推导返回枚举问题；原失败日志保留，修正后编译通过，不能把这些编译错误当作行为红灯。默认值对100ms断言的`baseline-red.json`确为FAIL且实际3000ms、清理成功；新增仅该响应profile的XML后`profile-green.json`为100ms，完整QoS归一后相等、其他响应/请求profile均未变。它证明解析/转换，不是读取现有rosapi writer内部状态。

实验XML仅在`runs/.../heartbeat-profile/rosapi-nodes-100ms.xml`，SHA256 `3b5a7616857ec5145ab581d32b71bc18c6829c53a1c6bd7d1318fd175da8f5c6`；功能包默认配置、源码归档与超时均未修改。完整录制/三SDK/慢读者/原诊断模式的30秒冒烟45RPC零失败、峰值785.85MiB、一次慢读暂停/恢复，进程与端口清理。首次读取进程环境已在短测结束之后，空结果明确不当作继承证明。

14:50使用同一0.6.11冻结前缀启动`heartbeat-profile/long-fastdds-100ms`：域175、21088/22790端口、1800秒、相同诊断/负载/3秒门/1GiB守卫，唯一配置变量为该响应心跳100ms。`long-live-environment.json`已从两个真实运行进程的白名单环境键核对XML路径、RMW与域；未设USE_QOS_FROM_XML/发布模式覆盖。该初始运行状态保留，终态如下。

1098.25秒因原1GiB组合RSS门FAIL，峰值1042.06MiB，系统可用最小16251.86MiB；不是系统OOM或RPC判据通过。守卫停止了Node，未执行到其finally，缺少multi-client.json及缓冲wire/rpc落盘，不能宣称1635次WebRPC零失败或完整TF/慢读收尾通过。1635对服务端事件与1635次native成功take可核对；11154条native无drop/error/超预算，late保留为空。PID身份和21088/22790均清理，见summary/cleanup-verified。

native source至reader历史插入的100ms试验中位3.373ms、p95=99.479ms、最大100.794ms，618次超过50ms、没有超过250ms；3秒基线为中位2.988ms、p95=4.693ms、最大2999.313ms、仅1次超过50ms。这支持周期控制延迟尾部的解释，也暴露p95明显变差，不能只挑最大值称优化完成。不同终止原因/时长不比较总体失败率，精确丢失/发现机制仍未确定，实验XML继续不作为产品默认。

RSS采样中桥由约178MiB上升/波动至401MiB，播放器约124至223MiB，Node约151至188MiB；这些数据未区分活对象与分配器保留，不能直接定性泄漏。本机rosbridge2.7.1源代码的出站队列按消息数限制，本包web.launch默认64，单帧可达数MiB；慢读暂停期间可保留大量字节。这是下一步容量核对方向，尚未增加字节适配器、减少负载、放宽1GiB门或改产品默认。

### I29：WebGL上下文丢失被误记为绘制成功（0.6.12）

静态审查确认Three0.186.1的WebGLRenderer.render在`_isContextLost`时正常返回，原SceneView只据“未抛异常”增加renderCount并设置首次提交时间，可能把丢失上下文当作有效绘制。查阅本机Three完整相关分支与Khronos WebGL的isContextLost语义后，先新增显式`scene_context_probe.mjs`调用真实renderFrame、仅替代GL/ROS边界。旧dist的`scene-context-loss/red.log`实际失败：already lost时计数为1而非0。

最小修复在renderer.render前后查询公有isContextLost；已丢失则不提交，提交期间丢失则不计成功，走既有renderErrors/lastRenderError和状态显示路径。恢复后下一次dirty可正常绘制，历史错误计数和原首次时间保留；dispose后不再查询或绘制。未增加后台预热、GPU计时、worker或compileAsync，也未声称冷上下文创建问题已解决。首次完成字段仍是CPU提交返回时间，不是GPU执行/屏幕呈现完成。

4组边界核对、完整observe套件、51组wire、20组audit及14组preview通过。首次修正已有renderer测试替身时使用了错误工作目录下的相对路径，未改到文件；随后改用绝对路径并完整执行。第一次wire被沙箱回环监听EPERM阻止，保存原日志后在宿主权限下重新执行通过；两项均为编排失败，不计产品故障。新scene guard构建在工作区dist，运行中的FastDDS对照继续读取独立冻结目录，没有替换其文件或启动额外ROS实体。Jazzy-only隔离构建与官方33项lint零错误/失败；独立审查无新增实质缺陷，指出受控disposed标志只证明入口守卫，不等于完整资源释放集成验证。

随后在真实IAB通过临时可见按钮驱动本页WEBGL_lose_context，页面加载当前实际SDK/SceneView bundle，显示网格但从未client.connect，不创建ROS实体。运行/丢失/恢复/停止四阶段均实际核对：成功帧13→13→14，丢失后错误计数1并在恢复后保留，首次CPU提交时间始终85.2ms；恢复事件后明确点击重置视角触发下一dirty。停止后canvas=0、client=disposed、采样定时器停止，自己的标签及21188服务器均清理。原始browser-stages/console、恢复截图与进程核对在scene-context-loss。此轮只证明本机网格上下文的真实丢失/恢复，不能替代新版完整Peach画面、冷GPU、Android、GPU栅栏或输入延迟测试。

0.6.12冻结归档`source-rebuild-v0612/local-ros2-web-link-0.6.12.tgz`为151文件，SHA256 `07d244ab1cc3d325bc897bb0a41308379a2321990b4f3f0c33231d8e54e33a3f`。本机缓存离线安装27依赖、check/build、51dist逐字节重建、四公有入口和清洁Jazzy-only隔离构建通过。相对0.6.11，45个dist未变，6项变化是scene的ESM/browser代码及map、构建锁副本和HTML版本；没有改核心通信SDK。初次默认cache的npm pack返回226，未保留stderr，具体原因不回推；显式/tmp cache加offline后成功，无依赖变更或联网。新归档形成在本段及后续证据目录刷新前，不包含自己的哈希或后来新增的原始文件。

### I30：出站队列容量缺少可复现回放入口（0.6.13）

I28在1GiB门失败后，核对本机rosbridge2.7.1与同tag源码：每连接write_slots按write_queue_size计数，满时丢弃新出站消息，drain等待每次write_message完成。本包web.launch原值64，一条完整云约4.1MiB，按帧数限额不能直接当作字节预算。先只开放已有上游参数的容量对照，不自研队列、不改默认64、全帧尺寸/速率/客户端数/超时或RSS门。

性能CLI新增可选--write-queue-size，范围1..4096、缺省不追加覆盖，选择自动进入run-config。replay使用独立的bridge_write_queue_size可选空参数，非空规范化为整数并明确传子write_queue_size。最初同名空参数方案在20组Include字典检查中通过，但独立审查指出共享launch配置会用空值遮蔽子默认64，导致ParameterValue求int('')失败。补执行实际web参数声明、SetLaunchConfiguration和ParameterValue.evaluate，inherited-default-red真实复现；改成独立名称后，默认64、显式边界/16、已有全局32保持及非法输入共25组通过。这里的P1是本轮尚未部署候选引入，不能藏在“测试优化”中。

探针构造LaunchContext先触发默认~/.ros/log写入，被沙箱拒绝；第二次只在后续_start中设ROS_LOG_DIR仍然太晚。随后在Context构造前设置临时日志目录，才进入正确的转发/继承红灯。192组原模式回归也先后遇到socket权限与未source安装前缀，完整环境下通过；这些编排失败的日志都保留。官方lint首次指出import顺序和100字符行，修正后隔离构建、34项lint零错误/失败。独立复核确认P1修正后未发现新增实际缺陷。

queue16-smoke在原100ms实验XML下使用完整录制/三SDK/慢读者90秒：135RPC零失败，一次clock回退，慢读605帧/804958844B、两次暂停/恢复且未提前断开，峰值765.16MiB，诊断无缺证、进程/端口清理。保存了实际桥启动参数文件write_queue_size=16。期间首次额外只读参数探针使用自建Context却省略executor，触发默认未初始化Context错误；finally已关闭，修正后的第二次尝试因90秒测试已正常结束而无服务，不能当产品故障或重启理由。

另起core16无硬件示例，使用匹配的默认Context/Executor读回桥参数type=2/value=16，再执行已有SDK原生功能17/17：九类参数、图谱/类型、发布订阅/QoS/晚订阅、服务成功失败超时、CBOR/参数事件、白名单、Action反馈/成功/拒绝/取消/中止、Lifecycle、pending预算和断线清理。峰值410.59MiB，进程及21388/22990端口清理；不是慢网拥塞下命令交付证明，也不覆盖真实硬件。

缩小队列也可能丢静态TF/Service/Action消息，不能仅凭RSS降低改通用默认。后续同负载长测仍须同时检查逐流进展、晚加入完整TF、SDK回复和慢读恢复，再关闭诊断复测；100ms XML仍属实验，不能用容量参数掩盖原RPC故障。

default-regression不带队列覆盖或实验XML，实际默认回放启动并完成各12秒JSON/CBOR完整数据核对，峰值729.04MiB，进程及21488/23090清理；保存的桥启动参数仍为write_queue_size=64。这是修正同名空值回归后的实际默认启动证据，不仅是Include字典检查。

0.6.13冻结归档`source-rebuild-v0613/local-ros2-web-link-0.6.13.tgz`含152文件，SHA256 `8113df0e7d86b8f8b09856d394ea3ffd5834cfe0e3af38d363b27d4459d3f389`。离线缓存安装27依赖、check/build、51dist重建逐字节相同、四公有入口、清洁Jazzy-only隔离构建和12个关键源码一致性通过。相对0.6.12，49dist未变，仅构建锁副本/HTML版本变化；没有改核心SDK或3D运行代码。归档不包含之后写入的本段和运行中长测文件。

15:54该冻结前缀开始`outgoing-queue/long-queue16-100ms`，域179/21588与23190、1800秒、队列16，维持I28同一100ms XML、原三SDK/慢读者/完整数据/2秒查询/3秒失败判据/1GiB门/诊断模式。与I28容量对照的唯一设置变化为64→16；运行dist除版本文字外保持一致。`long-live-config.json`从真实自有进程核对参数文件中的16、固定XML SHA256及未设USE_QOS_FROM_XML/发布模式覆盖，不增加ROS实体或请求。初始RUNNING只代表阶段状态，随后按同一进程取得终态。

终态1339.29秒FAIL：1989次RPC中client2.2一次TIMEOUT，5代客户端、22次clock回退，慢读8174帧/10724680007字节、暂停/恢复各30次，峰值911.27MiB。原生13812条记录无丢记录/错误/超预算，1989对服务端事件，late保留未触发；5代最终TF完整、SDK资源均释放，进程及21588/23190端口清理。失败CID尾号1989的source→reader历史仅0.402113ms，bridge wait约8.37ms即true，调用约41.89ms返回，已不同于I27的3秒DDS阶段延迟；记录的约5秒宽限内没有同CID的WS service_response。311条出站队列满警告包含一条邻近失败时刻，但上游警告没有连接/CID，不能证明该条回复确被丢弃。降低RSS而丢失回复仍是失败；默认64及实验XML的部署结论均不变。下一项对照将复用既有两个RosClient分开RPC/大数据，仍为同一rosbridge与单一UI，不先增加自研调度器或放宽期限。

### I31：缺少标准页面离散事件耗时观测（0.6.14）

按W3C Event Timing核对受信离散事件、16ms请求阈值、8ms duration量化、buffered历史阈值及连续移动/滚轮不在报告范围。SceneOptions新增默认false的profileInteractions；只在明确启用时创建PerformanceObserver，未来事件buffered=false，不注册额外鼠标监听，不保留PerformanceEntry或DOM target。只保存最近120个duration数值，累计有效/忽略计数和最大input delay/同步handler处理/duration。p95仅为保留的已报告事件，不是所有事件p95、INP、连续拖动延迟或GPU耗时；未采样为null，不支持时supported=false。

新增显式event_timing_probe，缺模块红灯后实现。首次check发现TS7.0.2的PerformanceObserverInit缺durationThreshold成员，按规范使用本地类型交集，未使用any或修改依赖。六组核对后，又以缺supportedEntryTypes的旧API替身复现TypeError，改成可选链；七组及完整observe通过，check/build通过。初始化放在constructorMs记录之前，独立审查指出的P3计时遗漏已修复，复核无新增实质缺陷。Jazzy-only隔离构建和官方34项lint零错误/失败；它们不证明真实输入或ROS负载性能。

实际IAB以当前bundle、无ROS连接的网格页核对：真实点击isTrusted=true、supported=true，计数41；最大同步处理265.1ms、已报告duration p95为288ms，包含启动点击的构造负载，不能把它误写成单独80ms按钮耗时。80ms按钮明确人为忙等，只验证报告能力。停止后observer.disposed=true、canvas=0、client=disposed，再次受信点击后计数仍41。browser-stages.json、无控制台错误的browser-console.json及browser-disposed.png保存在event-timing；临时标签和21688服务器已关闭。此轮不证明完整Peach画面、冷GPU、连续拖动、Android或局域网性能，也未修复历史桌面闪退。

0.6.14冻结归档`source-rebuild-v0614/local-ros2-web-link-0.6.14.tgz`含157文件，SHA256 `4c61bde85f049c1d5305892bd8b9b7a67314836de68e117664f64ad61ef6a7d5`。离线缓存安装27依赖、check/build、完整observe、54dist逐字节重建、四公有入口及清洁Jazzy-only隔离构建/官方34项lint通过，12个关键源码一致。它不是长测容量通过；归档形成在本段及后续索引刷新前，不含自身哈希和稍后的原始记录。

归档后复核首次colcon test发现遗漏--base-paths，输出实际是0 packages/0 tests，不能当通过。原日志保留；补显式提取源码路径后才真实运行官方34项并零错误/失败，verification.json已改为只认修正结果。冻结包在此过程未改写。

### I32：控制回复与观测流共享出站队列的对照（0.6.15）

I30显示RPC原生阶段已完成而网页缺回复。再次读取rosbridge2.7.1的send_message/_drain_write_queue，确认同连接所有出站类型共享计数队列。这里只增加显式--separate-control性能模式，复用两个公开RosClient分别承载观测/TF和只读RPC，仍接同一rosbridge、共用事件循环和DDS；不是第二个网关或自研控制调度器。默认false，单UI及通用SDK默认行为不变；新模式每逻辑客户端2个socket，3客户端加慢读者共7个连接，不能说连接数量不变。

真实CLI选择探针先因不认识参数失败，再实现可选参数、multi-only前置校验及明确0/1环境覆盖，7组通过；保留192旧模式/环境与25队列回归。只改显式性能脚本，没有增加核心SDK API、重试、超时放宽、QoS/帧尺寸/数据限速或业务迁移。会话在首次connect前登记两侧所有权，部分连接失败由原finally释放；报告保存data/control各自清理与传输统计，RPC只走control。独立只读审查无实际缺陷，指出仍共用桥和事件循环，不能以短测宣布修复。

在不变的完整真实录制、100ms实验XML、队列16、3秒RPC门、1GiB进程树门下，smoke90通过135RPC/0失败、5代客户端、1次clock回退、慢读604帧/800817730B、暂停/恢复各2。峰值约733.3MiB；data共2725320669B，control仅28242B且每代接收帧数恰等于RPC数，双侧订阅/发布/等待/监听均0，端口21688/23290及进程清理。这是接线/释放短测，不证明修复1339秒故障，也不以单次峰值差宣称容量改善。

default30省略新开关、队列覆盖及实验XML，仍单连接/默认队列64，完成45RPC/0失败及5代客户端收尾，峰值约720MiB；21788/23390与进程清理。另以本地标准WebSocket服务器受控拒绝第二条握手，实际multi CLI保持FAIL并退出1，两侧RosClient资源为0、服务器连接0；验证部分连接的所有权，不是ROS故障复现。首次短测摘要读取误用了不存在的guard-result.json，改读实际resources.json后只核对已有终态，没有重跑测试。长测及关闭诊断后原生命周期验收仍待执行，XML与网页部署默认未修改。

补充default30范围：首代client1.1在7.5秒按计划替换，最后TF查询仍缺两个相机光学frame，处于10秒初始预算内，不能说该30秒测试全部历史代TF均完整。终场在场三代TF完整、5代资源均释放。首次额外摘要用了更强的“所有历史代完整”断言而失败，保留缺失清单后按真实范围记录，没有重跑或放宽CLI判据。

0.6.15冻结归档`source-rebuild-v0615/local-ros2-web-link-0.6.15.tgz`为157文件，SHA256 `6ee952861dd9ab480cb1a0d3293b3a4b35dacfaf494f6a0463bffcdb0b3ee9ad`。缓存离线安装、check/build、54dist字节重建、四公有入口、Jazzy-only隔离构建与官方34项lint零错误/失败；相对0.6.14有52dist未变，仅锁副本与HTML版本变化，核心SDK/图形运行代码不变。新脚本已在归档中，归档后的本段、长测输出及后续目录不在归档内。

后续以此冻结前缀启动separate-control/long-queue16-100ms，域182/21888与23490，目标1800秒；保持原数据、3客户端/慢读者、2秒查询、3秒期限、队列16、100ms实验XML、1GiB门和诊断。相对I30仅RPC另走同桥的独立现有RosClient，WS总数从4到7是此组合的明确代价。初始启动不构成通过，必须等待同一组终态；测试期间不改其dist、不追加GPU负载、不在日志增长期间刷新目录。


I32冻结长测终态补记：1800秒PASS，2688RPC零失败、每代p95约105.06–106.72ms、最大135.62ms，2688个CID各一次成功回复；data45465533753B、control568749B，慢读11175帧/14537365875B，暂停/恢复各40次、30次clock循环。5代最后TF检查完整、双侧资源0，峰值954.44MiB、系统可用最低11489.16MiB，所有自有进程及21888/23490释放。18584条native元数据无丢记录/错误/超预算，wire8064/rpc5376条结束后flush且buffer0；late未触发、Timer/pending0。此为队列16+100ms实验XML+分连接+诊断包装的首次完整30分钟通过，不能推广为默认64/默认RMW参数或原生命周期无诊断已通过；部署默认和产品网页尚未改变。

### I33：原始图像显示重复复制完整RGBA（0.6.16）

读取ImageView实际raw分支，发现decodeImage已分配RGBA后又new Uint8ClampedArray(decoded.pixels)。先查WHATWG HTML ImageData构造器：使用传入的原TypedArray对象；本机TS7.0.2 lib.dom的ImageDataArray要求ArrayBuffer而不是ArrayBufferLike。这里解码器本来就新建普通ArrayBuffer，因返回类型省略泛型而丢失了该类型事实，不需要靠复制恢复类型。

新增显式image_view_probe，调用真实ImageView.start与订阅回调，只替换DOM/Canvas/ImageData边界，Proxy记录实际Uint8ClampedArray构造。旧dist红灯为2次而非1次RGBA分配。最小修改将decodeImage返回pixels明确为Uint8ClampedArray<ArrayBuffer>，直接交给ImageData，并注释为何不克隆；不共享原ROS输入缓冲、不复用已提交的RGBA、不改变颜色/深度/步长/单位。四组绿色覆盖带padding的BGR、640×480完整BGR像素哈希、大端16UC1及重复start/dispose的单次订阅/监听释放。check/build和完整observe通过。

640×480每帧确定减少一次1228800B应用层RGBA复制；原始解码输出仍新分配一份，Canvas内部上传/复制不在此结论内。没有据此声称帧率、RSS或输入延迟已改善。该探针是受控DOM，不等于原生ImageData/Canvas验收；为保持正在运行的I32对照负载，不启动浏览器/GPU，真实浏览器复核留待该长测终态后。长测始终读取冻结0.6.15，与工作区新dist隔离，没有替换运行中文件。

I33原生浏览器复核现已完成：真实录制color/depth各640×480，通过实际RosClient发布至独立自测话题、原生DDS、CBOR接收、实际ImageView和Canvas2D读回。全部RGBA像素SHA256与独立Python参照一致，实际ImageData保留传入视图、每帧一次1228800B应用层RGBA分配；停止后订阅/发布/等待/监听/Canvas全0，控制台无错误，自有标签和进程均关闭。记录在image-copy/native-canvas-typed/browser-stages.json及color/depth截图。仍不证明FPS、RSS、GPU内复制或持续交互性能改善。

### I34：真实录制完整字节与独立显示参照

此前I25完整往返是同尺寸合成数据，不能代替实际相机帧。现在只读SequentialReader提取已保存bag中各自第一帧：depth614400B、color921600B、cloud4136928B；不初始化ROS、不建立DDS实体，reader在finally.close。三帧并不同步，不当作RGB-D/TF积分组。保存原header、bag接收纳秒、完整CDR及payload SHA256、布局；Python独立生成两图全量RGBA与按100k上限抽取的86186点全部XYZ/RGB Float32小端哈希。JS离线解码全量哈希一致，不再只查首/中/末点。官方Jazzy阅读教程网页不可访问、所试rst路径404，改核对Jazzy上游_reader.cpp、安装pyi和本仓frozen_camera_replay现有调用；未凭其他发行版API实现。

第一次native-canvas失败：提取清单将两种消息的is_bigendian都转bool，但Image定义为uint8、PointCloud2才是bool，桥明确拒绝发布depth。保留FAIL、空deliveries和410.49MiB峰值及清理，不改SDK容错或重试。修提取器保留原IDL类型，重新读取到新recorded-frames-typed目录，三payload哈希不变。Node检查也改为严格类型/布局相等，避免bool规范化遮蔽错误。

修正后的6次实际记录JSON/Base64→原生DDS→JSON/CBOR（两图和完整点云各两次）均通过全payload、布局和独立显示参照，未放宽5秒工具门或重试。随后实际浏览器color/depth分别走同一DDS链路至真实Canvas2D并核对全像素，见I33。只发布两个固定/web_link_demo/roundtrip_*标准传感话题；test frame_id用nonce关联回复，原header留在清单，因此不是完整header/TF往返等价。所有证据源/辅助脚本在image-copy，最新native-canvas-typed峰值615.51MiB不含浏览器、进程及21988/23590/22688端口清理。不是实时相机新鲜度、同步重建、长期性能或跨浏览器结论。

当时安排的原生命周期核对现已执行：关闭所有WAIT/native/server/late诊断及Node原生wire包装，保持双连接、完整原录制、队列16、100ms XML、2秒查询/3秒门、1GiB门，目标1800秒。结果为I36的777.54秒彩图停顿FAIL；单次带诊断通过不能代替这一项。网页默认仍共用连接，未迁移两个业务项目。

0.6.16源码归档`source-rebuild-v0616/local-ros2-web-link-0.6.16.tgz`含158文件，SHA256 `241ddf21e580fe66862fa0348a0452a6d97ab83c4036f81867d00c556efc3d51`。缓存离线安装、check/build/完整observe、54dist逐字节重建、四公有入口、Jazzy-only隔离构建及官方34项lint通过；核心SDK与长测源码对0.6.15逐字节未变。独立只读复核未见实际功能/所有权/当前TS工具链兼容缺陷；没有扩大为其他浏览器/TS版本或持续性能结论。归档在本段和后续复测输出之前形成，不包含自身哈希与后来文件。


### I35：通用项目引擎与单页的可选观测连接（0.6.17，demo功能已验证）

将I32已证明的RPC/观测隔离接到实际消费者：RunOptions.observationClient可传同URL的第二个现有RosClient，默认复用原client。topic/image/cloud/TF走观测侧，Service/参数/Action/图谱/接口定义和mockGuard仍走控制侧；mock标识必须验证命令来源。两侧任一断线停止当前项目用例，控制连接仍可发送取消和参数恢复，清理后的观测验证另用独立预算。引擎只释放自己登记的订阅/监听，不dispose调用者的连接；报告新增可选connectionMode字段。不同URL在任何远端操作前拒绝，不将跨网关数据误当成同一项目证据。

单页新增默认未勾选的“分离观测连接”，始终一个地址/一个页面。选择后建立控制/观测两条WS；订阅表单、标准数据、SceneView、ImageView、SeriesPlot、demo数据自测和项目审查使用正确侧，操作仍留控制侧。两侧状态和字节预算分别显示，顶部资源为合计，各自维持原SDK预算。连接活跃时模式锁定，分连接首次部分失败会释放本轮两侧，重新连接释放旧实例；代次检查阻止旧回调/迟到配置重开已取消连接。断开和pagehide处理两侧；项目收尾完成前保留控制连接。默认单连接行为保留，不自动调整队列/XML，也未迁移业务项目。

受控SDK/DOM检查先红：观测仍落控制侧、数据断线不触发取消、跨URL未拒绝、缺模式报告；UI仍只建一条、部分连接不清理、迟到配置会重开。初版mockGuard探针误用/robot而非既有要求的/robot_description，触发原校验并出现未及时观察的Promise拒绝；修正测试输入和即时catch后保留真正红灯，不放宽产品校验。最终9组audit分工/失败/Action取消/参数恢复及旧20组通过，8组实际页面handler/HTML边界与旧14组preview通过；check/build、完整observe及官方34项lint通过。受控DOM结果不作为浏览器性能证据。

首轮真实浏览器发现错误按钮ID`topic-start`（实际为`topic-subscribe`），页面初始化报空对象赋值错误；`demo-ui`保留FAIL。修正ID并增加实际HTML元素存在性核对后，`demo-ui-fixed`在默认共用/显式分离模式均完成6项只读检查，独立demo按钮12项回归通过；3D/图像/曲线同时工作，Action停止取得原生终态5且cancelConfirmed=true，模式往返后订阅/等待/画布归零。该轮示例云400点、PNG96×48，非完整Peach性能验收。

只读审查再发现P2：控制已连、观测仍在首次握手时，新Service/Action曾可发送；随后观测握手失败会释放两侧，已发远端命令的结果可能未知。新增握手边界先复现`service-call enabled before paired handshake completed`，修为连接命令结束前禁止新发送/观测启动/项目运行，取消、停止等待、断开仍可用。最初测试仅等待微任务，未进入延迟握手就提前reject，得到unsettled top-level await；修测试调度后才取得产品红灯，不将编排错误当作产品证据。审查复核关闭P2，无新增实质缺陷。

最后实际浏览器`demo-ui-handshake-fixed`确认正常双侧握手后按钮重新可用、模式锁定，AddTwoInts手动调用返回7，6项只读验收通过且报告connectionMode=separate，结束订阅0/等待0/512。断开后模式可切换、新发送不可用，临时标签关闭；333.57MiB为启动树峰值、不含浏览器，PID身份及22188/23790均清理。截图只覆盖结果区，复选框flex修正没有单独的最终视觉截图，不能据此宣称其视觉验收。以上均不关闭I36图像流停顿、冷WebGL、真实LAN/Android或当前完整Peach负载的验收缺口。

### I36：无诊断原生命周期仍有图像流停顿

冻结0.6.16的separate-control/long-original-lifecycle在777.54秒FAIL，原因client0.1 color超过10秒未推进；1158RPC零失败、4代客户端、13次clock回退、TF最后检查均完整，峰值847.58MiB，非内存门。慢读4586帧/6030024878B，暂停18次、恢复确认17次，失败时仍暂停，不能写成慢读恢复完整通过。无诊断包装或额外元数据实体，进程及22088/23690全部清理。它否定“带诊断30分钟通过即可视为原生命周期通过”，不改历史PASS，也不把1158RPC零失败扩大成全流通过。

只读扫描同一bag的原始接收时间，color78条、平均1.312Hz、最大间隔3.724秒、跨循环间隔4.199秒；depth453条，cloud116条，均无超过5秒的源间隔。来源节拍不能单独解释10秒停顿。核对本机rosbridge节流使用monotonic而非ROS时间；通用满队列警告仍不能定位具体话题/连接，现报告也未保存逐流最后接收年龄。下步必须区分ROS供给、节流、出站排队和消费，未放宽10秒门、未缩小帧或降低输入负载，也没有盲目重跑。数据流问题保持开放，网页默认不自动改为已验证部署。

I35检查更正：新增restoreViaControl探针最初错误使用字符串type=string，而既有ParameterType要求ROS数值枚举（string为4），原校验拒绝且audit-nine-green实际失败。异步命令完成前提前将第9组写为通过是不准确的；原日志保留，修测试到枚举4后再明确重跑，不修改参数API。前8组结果不受影响，后续只以新日志为准。

0.6.17源码归档`source-rebuild-v0617/local-ros2-web-link-0.6.17.tgz`含160文件，SHA256 `438201118cfdcbdaf87fd9cdcd6ce712f6518a0c4f27f52d2b5f3c4b8c2c91a7`。缓存离线安装27依赖、check/build、audit20+9、preview14+8、完整observe及四公有入口通过，54dist逐字节重建一致；干净Jazzy环境隔离构建、官方34项lint通过。四个SDK源码和三个长测/线事件脚本相对0.6.16逐字节未变；本轮只新增可选audit/UI组合，不能将归档验证当I36修复。冻结工具曾误引不存在的sdk/client.ts，实际为sdk/index.ts；该元数据检查失败单独保留，未重打归档，修正路径后继续同一源码的隔离构建并取得上述终态。归档形成在本段及后续核对之前，不包含自己的哈希或后续输出；当前源目录文档增加本段不影响已冻结包内容。


### I37：逐连接/话题的数据流诊断（工作区工具，尚非修复）

重新阅读本机rosbridge2.7.1真实路径：Subscription.on_msg进入每连接节流处理，Subscribe.publish在Protocol.send之前已经生成CBOR；WebSocket以真实Semaphore.acquire(False)决定进入共享发送队列或直接丢弃。原警告既无话题也无连接，不能定位I36。没有改变产品默认或添加新的传输协议，而在stream-flow保存显式sitecustomize诊断，只对WEB_LINK_STREAM_TRACE=1且bridge_node.py入口安装，相关上游源码不符时在挂钩前拒绝。无额外ROS实体、替代队列或热路径文件写；线程上下文传话题，真实acquire仅调用一次，原返回/release/消息/Future对象保留。只保存连接UUID、话题、时间/计数及最多256个整数帧身份，不保留payload。128行×每行128事件、退出JSON4MiB，溢出/未关联/缺证不能作有效定位。

源输入、节流后候选、真实准入/丢弃、Tornado写入开始/完成分别记录。input指桥调用Subscription.on_msg，不是发布者或网卡时间；writeFinished是Tornado流控完成，不是浏览器收到或绘制。复制冻结0.6.17到/tmp，仅给Node最终报告增加接收/开始/关闭时间，判据/SDK/输入不变；client-report-only.diff及source.json记清差异。按已审阅的顺序建连和服务器open时间关联5代观测连接；不能将/clock误当成仅slot0的标识，TfBuffer也各自订阅它。分析器只在有完整时间环覆盖时才将最近10秒计数当作完整窗口；静态URDF接收年龄不作为连续流门。

原Semaphore无诊断先红（成功acquire无连接/话题计数）；最初4组边界通过，涵盖成功/拒绝/释放保持、线程/嵌套上下文、有界事件/身份和关闭、重复同一bytes身份FIFO关联。首版测试有3次真实准入但只消费1次，错误断言保留，修正测试而未丢弃关联。入口检查曾因实际Subscription误写Subscriber而拒启78，修正类名；随后上游导入正常打印capabilities使整段stdout相等断言失败，改为核对末行及退出文件。入口/源码拒绝2组通过，没有为测试创建ROS实体。

首轮完整录制smoke90通过：135RPC零失败、5代、慢读611帧/826259249B，暂停与恢复确认各2次；组合峰值666.86MiB，不含浏览器。诊断58行，overflow/pending身份/未关联写入均0。捕获慢读深度159次、点云46次真实拒绝；5个健康连接彩图准入/写完数分别100/19/62/81/38，与各自Node计数一致，没有彩图队列拒绝。它证明当前关联有效，未复现777秒故障，不能排除长期队列或DDS供给问题。进程身份及22288/23890均清理。首轮观察器未预先独立冻结，不把它当可重建版本证明；后续修后v2先冻结六份源码和SHA清单再运行。

只读审查发现两个P2，均先复现再修：关闭清理后已进入发送的线程可重新登记帧身份，导致假的pending；原源码守卫还未绑定实际overlay导入路径、Protocol.send及继承的Tornado写入方法。关闭标记与关联登记现在由同一锁协调，关闭后拒绝元数据登记但保留真实发送行为，记录packetRegisteredAfterClose；该标记只说明登记时点，不能证明实际acquire发生在关闭之后。closed集合也有界。源码守卫逐一校验五个实际模块的__file__/SHA，并核对七个实际实现经unwrap后的文件归属，避免检查旁边文件却修改另一模块。5组Semaphore/关闭边界、2组入口/SHA及5组实际模块/方法替换拒绝通过；审查复核关闭两个P2，无新增实质问题。原红灯、错误测试输入和撤回的守卫均留原日志。

修后smoke90-v2通过：135RPC零失败、5代、1次clock回退、慢读609帧/821507124B、暂停与恢复各2次，峰值664.81MiB。58行、无overflow/pending/未关联/关闭后登记；健康彩图无队列拒绝，慢读深度160次和点云47次拒绝。进程与端口清理；见stream-flow/smoke90-v2/diagnostic-summary.json及observer-v2-source.json。

随后使用同一冻结v2观察器启动long-v2：1800秒目标、原完整录制、3逻辑客户端/7WS、2秒RPC/3秒期限、10秒逐流门、队列16、100ms实验XML、1GiB/4GiB可用内存门均保留。其他五种服务诊断关闭。该轮已取得1800秒终态PASS，2685RPC零失败，5代p95为18.74–106.08ms，30次clock回退；慢读10719帧/13912016853B，40次暂停及40次恢复，未提前关闭。组合峰值841.05MiB、可用内存最小23067.95MiB；全部最后TF检查完整，PID身份和22388/23990清理。诊断58行、无overflow/pending/未关联或关闭后登记；健康彩图5代写完/Node计数一致（1639/510/1035/1131/609），无彩图队列拒绝。慢读深度2992次/点云720次拒绝；健康连接仍有一次tf_static及一次clock拒绝，不能宣称共享出站队列对所有消息可靠。data接收44103680857B、control492933B，ignoredResponses=0。原777.54秒故障没有复现，带观察器PASS不是I36根因修复或关闭诊断验收。旧失败与本轮均未见桥内Traceback/异常日志；通用警告仍不能追溯旧故障丢弃话题。运行期所用dist/冻结观察器未变，没有额外浏览器/GPU负载。I36、冷WebGL、真实LAN/Android与完整Peach当前页面负载仍开放。


### I38：显示时间显式选择、Marker寿命与纳秒回退（0.6.18）

对照ROS2 Clock and Time及Jazzy RViz MarkerBase：原Scene在首次/clock前以系统epoch计算寿命，之后自动改用ROS秒比较，可能永不过期；默认订阅任何/clock也会让真实项目受无关回放影响。原到期使用大于而非大于等于。先用实际Scene方法/受控渲染器复现，前6项中5项红；不创建WebGL或ROS实体。

SceneOptions.useSimTime现在默认false，显式true才订阅/clock，并让Scene内TfBuffer采用同一时钟策略；首帧前ROS时间为0，不与系统epoch混用，暂停不外推，回退清旧Marker/云/Path并保留静态TF。独立TfBuffer的clockTopic也改为默认false，回放须显式/clock或自定义Clock话题；这是相对0.6.17的默认行为变化，不隐藏为兼容修改。两个性能CLI显式补clockTopic=/clock，维持原实际订阅/输入；正在运行的旧长测使用独立冻结0.6.17，未受修改影响。两个Peach回放项目显式true，demo/在线相机/默认profile为false。不修改任何ROS节点的use_sim_time参数；应用需保持其与页面选择一致。RPC/transport继续使用单调截止时间。

只读审查捕获P1：profiles.yaml新增useSimTime但serve._strings白名单未接纳，默认HTTP服务会在配置加载时退出。config_probe.py用实际load_config和真实YAML先复现Unknown view fields，再同时接纳useSimTime和已有profileInteractions，三个布尔项拒绝字符串/整数等隐式转换，3组通过。没有为了开关放宽其他未知字段检查。

审查另发现P2：Scene用浮点秒比较而TF用整数纳秒，大秒值下1ns回退会让TF清历史但Scene保留旧对象，短寿命也会提前到期。新增实际TfBuffer+Scene回调组合先红，改为共用既有Time/Duration规范纳秒转换，Scene时钟/截止/比较全程BigInt；每帧取一次当前时间。stats提供useSimTime、clockReady、仅展示的clockSeconds及精确十进制clockNanoseconds，避免JSON BigInt；墙时输入仍是Date.now的毫秒分辨率。onStatus显示系统时间或ROS时间，未初始化明确等待/clock。未声称浏览器墙时具有原生纳秒分辨率，也不宣称完整RViz所有显示阈值一致。

最终10组时间检查覆盖：默认无Clock订阅、显式仿真、初始零、暂停、精确到期与资源释放、零TTL、回退、非法Clock、构造前布尔验证、大秒值1ns回退与TF/Marker/云/Path同代清理、静态TF保留、短寿命边界、同形Marker刷新期限/解除期限。用例分组数不等于覆盖项枚举数。扩展状态断言曾因夹具遗漏assetErrors集合失败，补夹具后重跑，原日志保留；另一次npm check误在工作区根运行，未执行到TS检查，改包目录重跑通过。默认TF开关新用例另有真实红灯。check/build/完整observe通过；审查复核P1/P2关闭，未发现新增实质问题。

原生配置检查不启动HTTP/ROS服务器；时间探针虽然组合实际TfBuffer，仍以原型夹具跳过Scene完整构造。随后通过隔离普通安装启动真实HTTP/rosbridge/demo。第一轮实际浏览器确认初始ROS零时间、首次Clock后寿命到期、墙时等待不推动暂停的ROS寿命及精确到期；480秒守卫先于纳秒命令结束，该命令在初始化ROS前拒绝，纳秒项在该轮标NOT_RUN。第二轮demo-nano真实发布int32最大秒值、1ns再回退至0ns，页面Marker从1变0；重建为useSimTime=false后显示系统时间，订阅由8变7。默认6项只读检查与12/12无硬件通信自测均通过，控制台warn/error为空。明确停止3D/图像/曲线再断开，三个容器子资源均0，SDK订阅/等待0，临时标签已关闭；自有PID与22188/23790主机复核清理，子树峰值344.92MiB不含浏览器。

GUI编排曾遇到重复导航定位、旧闭包仍引用已关闭标签，以及第一次时限不足；均保留实际范围，不当成产品故障或补报通过。验收汇总脚本首次误把AX差异中的10个新增勾与全部12项比较，改读最终完整AX后通过，无产品修改。另一轮在服务先退出后点击断开仍保留8个SDK订阅注册；SDK允许重连恢复，是否需要网页显式断开同时停用观测尚待单独分析，不宣称已修复。Jazzy官方lint先因新增配置脚本I100导入排序失败，修正后35项0失败。完整Peach当前页面验证继续单独记录；不把demo、10组CLI或0.6.17数据长测当成0.6.18全部端到端通过。证据集中在stream-flow与scene-clock；此阶段未冻结，不复用旧tgz作为当前交付；后续冻结证明见下段。

完整Peach当前页面随后在独立域188、22488/24090、普通安装的0.6.18完成：10项只读全部通过，真实URDF的15关节/6活动、21TF、33Marker、约8.6万显示点和800Path点，12/12模型资源；useSimTime=true且显示录制Clock。原始彩色/深度均640×480、深度单位0.00025m，JointState曲线收到样本。实际CUA旋转/滚轮、点云与Marker开关、视角复位、无效固定帧提示缺失TF/恢复base_link、停止/重启恢复完整模型均验证。先停止所有观测再断开，Canvas/图像/曲线容器0，临时标签关闭，PID身份与两端口主机复核为空；子进程树峰值465.47MiB不含浏览器。控制台有14条Collada Z-UP坐标提醒（两次模型加载），无error；不隐藏上游警告。采集到CPU提交p95约1.4–1.6ms，同时renderMax=2000ms、页面最大长任务2001ms，保留卡顿未解决。首个CPU提交68.7ms不能代替全部资源/GPU呈现完成；后台截图环境也不能当作前台FPS保证。见scene-clock/peach-ui/verification.json。DOM项目报告预览已截断，10/10来自完整AX实际结果，未下载完整报告；首个曲线点击与图像启动重叠未启动，观察到空容器后顺序重试收到样本。summary的DOM role定位一次未命中，刷新AX后原生点击成功。

随后冻结[0.6.18源码归档](../../../runs/ros2_web_transport_20261029/source-rebuild-v0618/local-ros2-web-link-0.6.18.tgz)，162文件，SHA256 `0f1f06a1a9d3fc1a44fbdec110e3bd6be7e60fd7f2915b83301932f070b32df1`。离线缓存安装、check/build、audit/preview/完整observe、四公共入口、原生配置检查均通过，重建54个dist与工作区逐字节一致。仅source Jazzy的普通隔离colcon构建1包；首次test未传base-paths，在隔离构建目录发现0包，结果0tests被汇总断言拒绝。保留原始日志及harness-discovery-failure.json，不把命令exit0当成验收；同一归档/构建补显式源码路径后35项官方lint零失败，未重打包或换实现。核心4个SDK源码与0.6.17一致；两个性能CLI补显式Clock选项，不能再声称基准源码逐字节不变，实际输入订阅保持。详见[离线重建证明](../../../runs/ros2_web_transport_20261029/source-rebuild-v0618/verification.json)。本段最终provenance和随后刷新的证据目录在冻结包之外；包内目录保留其生成时快照，不承诺包含后续运行文件。未把归档重建绿灯扩大为I36/冷GL/Android/全部目标完成。

### I39：轻量彩图边界诊断（工作区工具，不是修复）

全话题v2诊断在1800秒通过却未复现I36。新假设是所有话题加锁/帧身份/Future完成回调可能改变时序，因此单独制作stream-color，只保留真实彩图的Subscription.on_msg/inputReturned、节流后Subscribe.publish/publishReturned和原Semaphore准入/拒绝，另记连接开关以映射代次。非目标流不入元数据锁，不挂send_message/write_message/Future、不保留载荷或帧身份、不新增ROS实体。32行×128事件、1MiB退出JSON，安装前校验实际导入模块SHA及解析方法源码，显式环境变量且仅bridge入口生效，安装异常退出78。仍不能称为无扰动或无诊断验收。

六组显式CLI先对全量v2复现非彩图会建立元数据行，再对轻量工具通过：实际acquire恰好一次、参数/timeout/release不变、嵌套/线程TLS隔离、非目标不入锁、行/事件界限。独立审查捕获P2：只检查Protocol.send基类会漏掉实际RosbridgeProtocol.send覆盖；原open内部catch后正常返回时，观察器访问不存在write_slots新增异常。分别补实际解析方法反例和实际上游open受控构造失败（Node.__new__，无ROS初始化/实体）先红后修；关联的不完整close异常被client_id访问覆盖也先红后修。安装门/初始化/关闭/未启用/非桥入口等10组通过，审查复核三项关闭。上游源码以本机2.7.1为准，普通GitHub页面cache miss后读取同tag raw，不用其他分支替代；参考见source.md。

工具五份源码先冻结到stream-color/frozen-v1并保存SHA，发布包未变。90秒真实完整回放135RPC零失败、16行、5代彩图publish/admitted/Node计数分别100/19/62/81/38，全部相等且无彩图准入拒绝；输入115/20/72/95/43的减少与上游节流相符，不能叫数据丢失。最后10秒事件全部覆盖，峰值663.88MiB不含浏览器，PID身份与22588/24190清理。见[短测终态诊断](../../../runs/ros2_web_transport_20261029/stream-color/smoke90-v1/diagnostic-summary.json)。

随后long-v1按同一原1800秒门完成PASS：2688RPC零失败、5代p95约104.00–106.08ms、30次clock回退、慢读10678帧/13957377286B、40次暂停与恢复，未提前关闭。峰值867.35MiB，诊断16行无溢出，5代彩图publish/admitted/Node分别1637/510/1025/1117/608完全相等、无彩图准入拒绝，最后10秒覆盖完整。PID身份和22588/24190已清理；见stream-color/long-v1/verification.json。仍保留完整录制、3逻辑客户端/7WS、2秒RPC/3秒期限、10秒逐流门、队列16/100ms响应XML、1GiB/4GiB可用内存守卫，其他服务诊断关闭。原I36未复现，轻量观察器通过仍不是关闭诊断验收或根因修复。使用原冻结0.6.17 SDK/桥和报告字段补丁版基准，未混入0.6.19 WIP。没有额外浏览器/GPU负载。准入不是实际入队/写出/接收；input不是源相机或DDS历史插入，分析按此限定。目录在长测持续写入期不刷新，避免制造过期SHA。计划、红绿、审查、冻结清单、所有原始输出保留在stream-color。

### I40：网页显式断开的本地资源收尾（0.6.19）

0.6.18实际页面曾在显式断开后保留8个SDK订阅注册和Scene RAF。SDK的disconnect保留注册以便重连是公有契约，不修改它。页面处理函数原来直接断双连接，没有调用已有closeSubscriptions；CLI现在执行真实stopOptional/closeSubscriptions，而不是空stub，先复现6个本地资源释放计数均0。

显式断开改成先收尾Scene/ImageView/SeriesPlot、Topic和标准数据off、失效optional启动代数，再断控制/观测连接。closeSubscriptions同步撤销标准图片Blob并清src，显示各观察已停止。已有observe-stop自己的清理保持有效。不会调用远端goal.cancel，不把本地断开当作远端任务取消；正常意外断线的SDK/组件恢复策略不变。两项新CLI加上真实挂起optionalModule→断开→重连→旧模块续体不创建实例的正反对照，共11组通过，含重复断开只释放一次。控制资源采用替身，未直接执行真实RAF/GPU，因此不据CLI声称修后真实页面已通过。

只读审查未发现实质回归，核对了unsubscribe发送失败可能同步断线时的SDK/Scene状态回调顺序。初期为避免给独立长测增加构建/GPU负载，未启动浏览器，明确记录11组CLI而非原生通过；长测结束后才开始新版普通安装验证。首次多文件文档补丁因io标题不匹配被原子拒绝，核对标题后重试，无部分源码改动。原始红绿与计划见stream-color/ui-disconnect-*。

真实demo在共享模式、完整Scene/PNG/曲线/标准图像/Topic启用时11订阅，直接断开后0，三个视图容器0、标准图像src清除；之后重连为分离连接，JPEG/PNG及全观测恢复，再直接断开0。6项只读及12/12通信回归通过，控制台无warn/error，子树峰值344.86MiB不含浏览器，进程/端口/临时标签全部清理。真实Peach第一次新版页面10项只读通过、12/12资产、21TF、33Marker、约8.6万显示点、800Path、640×480彩色/深度和600曲线样本；直接断开实际资源归零，但紧接着截图仍显示9订阅，因为统计每500ms更新。该轮原断言拒绝，verification保留PARTIAL，不重写为全部通过。

针对计数延迟先加入不运行timer的实际页面函数反例，旧实现仍显示9；将原统计体原样抽成updateMetrics，保留500ms周期，并在显式断双连接后同步调用一次。第12组通过，只读审查复核无新增问题。重建新的独立普通安装，35项官方检查通过；再用完整Peach回放开启Scene、原始RGB/深度和曲线，9订阅直接断开后立即0、等待0/512，后续DOM三容器0、src空。完整模型21TF、12/12资产、33Marker和800Path；控制台7条Collada Z-UP提醒、0error；峰值453.38MiB不含浏览器，全部PID身份及22488/24090主机复核释放。见image-headers/metrics-fixed/peach-ui/verification.json。该窄轮未重跑10项业务只读卡片，沿用前一独立证据，不把CLI或DOM视图清除当GPU堆测量。工作流曾误将诊断脚本当作share/scripts已安装文件，Python在启动前退出；改为源码CLI并保留隔离overlay。首个DOM快照选择器误写scene-performance，改为实际scene-metrics；DOM限制不提供childElementCount，零计数与后续querySelectorAll零容器分别留证。

### I41：压缩图像头尺寸、数据类型与预算（0.6.19）

原raw图像有像素门，但压缩图像在Blob/浏览器解码前没有同等尺寸门。按Jazzy CompressedImage.msg约定复核普通jpeg/png/tiff与compressedDepth格式；本组件只支持PNG/JPEG，其他类型明确拒绝。调研官方npm发布源码后采用无运行依赖、MIT的image-dimensions 2.6.0，版本精确锁定。image-size 2.0.4仅下载审阅未安装，选择与JPEG offset读取、浏览器入口及当前用法适配相关；未以未测攻击性能作比较结论。

先用小型头夹具复现预解码门缺失，9组初始边界中8组红；增加压缩尺寸读取、正安全整数像素/字节预算校验、codec与magic核对后9组通过。独立审查再捕获两个P2：既有imageBytes把number当Uint8Array分配长度绕过.length预算，以及通用格式解析器可能在最终拒绝前处理ftyp非目标格式。分别用10B请求/1B预算和64B非目标头复现；添加固定PNG/JPEG magic前置门，data仅接纳字符串/数组/实际Uint8Array，标准TypedArray品牌getter避免伪造Symbol.toStringTag，允许跨realm Uint8Array而拒绝其他视图。捕获buffer/offset/length后再做instanceof规范视图，修复TS的never窄化错误。补超预算JPEG后共13组通过，复审关闭两个P2。库公开入口会复制一次编码数据，Blob也有自身成本，不宣称整链路零复制或压缩帧FPS/RSS改善。

合法普通format为空或未知按消息约定回退JPEG，仍须JPEG magic；含compressedDepth、tiff或歧义codec提前拒绝。ImageView构造前验证maxPixels，raw解码沿用0.6.16的RGBA所有权优化。头读取不是全部图像内容校验，也不替代浏览器原生解码；字节、像素门不是整浏览器内存上限，更不能据此宣称历史24.6GiB Python OOM根因已修。check/build/完整observe/preview/audit均通过；最初package.json编辑误在根目录执行而无文件，改绝对路径后纳入observe并重跑；保留各失败及修后日志。

真实demo采用专用域185无硬件发布者，发布前核对fixture PID/创建时间/角色；Pillow10.2只生成4×3、633B的JPEG，不是产品运行依赖。浏览器实际PNG96×48、空format JPEG4×3均解码显示；5000×5000的33B PNG头在创建Blob/HTMLImage前拒绝，compressedDepth同样拒绝，之后恢复合法PNG。超尺寸夹具没有分配真实超大像素。共享/分离断开和重连范围见I40；完整Peach原始RGB-D也通过。image-headers保留发布artifact、源码、原红绿/审查、原生截图/AX/进程清理证据，审阅tar包SHA与上游链接见source.md。

本轮multi_client_probe仅在最终报告复制原有receivedAt/start并记录close时间，未在收帧热路径增加时钟调用、改变RPC/逐流/内存门、输入或SDK。receivedAt当前仅含color/depth/cloud，不假设包含URDF/关节；它用于下一次关闭观察器的长测比较逐流最后年龄，添加字段本身不能关闭I36。0.6.17的带诊断长测不混称当前0.6.19原生验收。


0.6.19随后冻结[完整源码归档](../../../runs/ros2_web_transport_20261029/source-rebuild-v0619/local-ros2-web-link-0.6.19.tgz)，163文件，SHA256 `ac8b468765e80a1c77829d695fda226c82819d428639458dc9f7fb73bf16b9e3`。仅本地缓存离线安装28依赖，check/build/audit/preview/完整observe、配置检查和四公有入口通过，重建54个dist逐字节一致；纯Jazzy普通隔离colcon构建1包，显式base-paths的35项官方检查零失败。四个SDK源码与0.6.18一致，media/image与新增依赖、页面收尾/统计及报告字段变更分别按本节说明，不声称整个基准源码未变。见[离线重建证明](../../../runs/ros2_web_transport_20261029/source-rebuild-v0619/verification.json)。新依赖官方tgz先加入本地缓存，无网络或安装脚本。上述源码在所有原生页面关闭后构建；未替换正在运行的页面资产。当前这段最终provenance、README归档状态、集成文档版本和后续目录刷新在冻结包之外，包内保留形成时快照；不循环打包自己的哈希或将后续结果冒充包内内容。


### I42：原始编码表与数值字节数组的严格边界（0.6.20）

只读检查实际decodeImage发现编码字典继承Object.prototype，constructor和__proto__误作bpp，单像素得到黑色而未报未知编码。imageBytes数值数组[256]/[-1]/[1.5]/[NaN]则被Uint8Array隐式包装成0/255/1/0，与uint8语义及PointCloud既有严格校验不一致。先新增两个行为组，原13组仍通过而两组明确红灯；改编码表为私有Map，数组在一次写入循环中验证数值整数0..255，无新依赖、不改变TypedArray/Base64、RGBA所有权和字节/像素上限。

独立只读审查复跑15组通过，但指出低优先级边界：稀疏数组可从原型读取42而被接纳。以独立数组原型复现红灯，再在读取前Object.hasOwn校验；不依赖可能被覆盖的实例hasOwnProperty。复审关闭该边界、无新增发现，最新check/build/完整observe/audit/preview通过。数值数组额外逐项校验成本未做大流基准，正常CBOR Uint8路径不进入该循环。无大分配、ROS或GPU用于这些纯边界验证；审查期间独立冻结0.6.19长测继续，不受工作区重建影响。

Jazzy Image官方网页遭Anubis拒绝，以本机apt Image.msg/image_encodings.hpp及common_interfaces/jazzy原始源码核对encoding与uint8[]；按ECMAScript Object.hasOwn语义仅检查自身属性，项目既有目标ES2022可用。原始红绿、计划、复审与CLI日志见raw-image-validation。当前尚未将0.6.19的浏览器/归档当成此修复通过；后续原生、构建与冻结结果另补。


I42补验构建：新的普通隔离安装仅source Jazzy构建1包，35项官方检查零错误/失败，见raw-image-validation/colcon-result.log及isolated-build.json。未写入正在运行的冻结0.6.19路径；当前尚待0.6.20真实页面/源码冻结。

### I43：跨Canvas首载分段诊断准备（工作区工具，非产品修复）

现有Peach记录pageLongTaskMax2029ms而Scene renderMax仅62.6ms，读取源码后不能只归因着色器。准备canvas-phases临时副本，包裹实际Canvas.getContext、少数已存在的GL初始化查询、2D.putImageData、公开subscribe的Image/Cloud/Marker回调及三个组件构造；没有额外GL查询、上下文或ROS订阅，不访问Three私有program缓存。48行聚合/32慢调用/标签96字符，UI每秒输出摘要，不留载荷或历史context。时间是CPU经过时间，不是GPU完成、输入延迟或无扰动基准。

工具草稿先写再做极小VM边界，首三处红灯是null callback被改为函数、readonly SDK赋值抛错和context getter异常覆盖原返回；修后9组通过。独立只读复核再实际复现初始化依赖/DOM失败残留钩子、一个原型冻结令stop中断另一项恢复。先解析依赖/创建DOM再挂钩，timer/listener初始化失败回滚；stop各项独立捕获并允许重试，timer仅清一次。新三个组和补丁一并加入，未保留独立旧版red文件；复审12组通过，两处P2关闭。实例context代理停止后仍存在但active=false直接委托，页面退出恢复全局原型及timer；不承诺BFCache返回继续测量。

另复制0.6.20普通安装，206文件，仅app.js构造计时、index.html加载诊断script和新增canvas-timing.js三处不同，原四SDK/Scene/Media/桥及原始安装未改，copy.json保存哈希。此工具不进入src/npm包，所有源代码及复核在工作区证据目录。准备期间原无观察器0.6.19长测持续运行，尚未打开任何诊断页面；实际原生阶段与终态后补，不把准备/CLI当优化完成。


I36/I41后续无观察器终态：冻结0.6.19的source-rebuild-v0619/long-no-observer在同样1800秒完整录制、3逻辑双连接加慢读7WS、2秒RPC/3秒期限/10秒逐流/队列16/100ms实验响应XML/1GiB与4GiB余量门通过。启动实际桥环境核对所有观察器关闭，未混入stream-color/sitecustomize；运行无浏览器/GPU，工作区0.6.20未写入运行副本。2685RPC零失败，5代p95为11.06/104.64/105.82/106.07/106.19ms，30次Clock回退，慢读10572帧/13828776114B、40暂停/40恢复，未提前关闭。子树峰值841.61MiB（不含浏览器）、Node峰值195.88MiB/堆21.96MiB，最小可用22597.13MiB。所有代最终TF完整，元数据新字段实际有效，color/depth/cloud最后接收年龄均<1.24秒。PID创建时间及22688/24290主机复核清理。见[无观察器原门终态](../../../runs/ros2_web_transport_20261029/source-rebuild-v0619/long-no-observer/verification.json)。这仅证明当前冻结版本单次通过，I36旧777.54秒断流未复现/根因未明，不因一次PASS关闭间歇问题或历史系统OOM。


I42真实未修改页面随后通过：分离连接Peach完整回放10项只读全部PASS，12/12模型资产、15关节/6活动、21TF、33Marker、约8.6万显示点、800Path、640×480原始彩色及0.00025m/单位深度、600曲线样本。专用域188向独立/web_link_test/raw_image发布1×1、3B合法ROS Image，encoding=constructor明确拒绝且画布保持隐藏；之后rgb8恢复1×1显示。非法uint8数组未假称通过DDS（序列化本身会拒绝），仅以公开API边界CLI验证。直接断开立即0订阅/0等待、三容器0、标准图片src空；7条Z-UP提醒、0error。子树峰值482.02MiB不含浏览器；回放和两单像素发布者PID/创建时间及22488/24090均清理，临时标签关闭。CPU提交p95约1.3ms、renderMax49ms，同时页面最大长任务2179ms，性能问题保持开放。证据raw-image-validation/peach-native/verification.json，本轮未采集完整画布像素哈希，不用截图替代该证明。

I43真实诊断副本也已停止清理：普通副本仅三处文件变化，经主机确认服务前缀；先Image→Plot→Scene分段，后切深度，20行聚合、overflow0、unavailable空，原始安装所有哈希仍一致。暖启动Image构造max0.3ms、SeriesPlot2.4ms、Scene48.7ms，模块22.7ms、getContext:webgl2=17.8ms、EXT_color_buffer_float=4ms；2D getContext max0.7ms/putImageData1.6ms，图像回调含RGB/深度max6.3ms、点云5.2ms、Marker6.1ms。初始Scene renderMax约47.9ms但页面longtask1466ms，稍后最终renderMax也达到2040.3ms/页面2459ms；不能只挑早期窗口宣称绘制已快。没有覆盖shader compile/link/program status/draw、完整RAF或Plot异步绘制，尚不能精确归因；下步补相应实际调用边界，而不是据暖构造就更换协议/库。控制台7条Z-UP提醒、0error，子树峰值452.90MiB不含浏览器，直接断开资源0，PID/22788/24390和临时标签均清理。见canvas-phases/peach-native/verification.json。测试顺序与未修改页不同、无冷GPU控制，此PASS仅覆盖诊断工具运行/边界，不是性能改善；工具不进入产品。


0.6.20完成[源码冻结与离线重建](../../../runs/ros2_web_transport_20261029/source-rebuild-v0620/verification.json)：[源码tgz](../../../runs/ros2_web_transport_20261029/source-rebuild-v0620/local-ros2-web-link-0.6.20.tgz)含163文件，SHA256 `ccf3c4b52b0088e8459733856a3e8e98bbd52d5fcfa7a6cf466464b19f5144e2`。仅本地缓存离线安装28依赖，check/build/audit/preview/完整observe、配置及四公开入口通过；54个dist逐字节一致，纯Jazzy隔离普通colcon构建1包、35官方检查零失败。4个SDK及3个基准/线事件脚本与0.6.19逐字节一致；图像字段校验为本次变化，原生0.6.20与0.6.19长测分别留证，不将后者改称0.6.20长测。临时Canvas工具完全在工作区runs，不入归档。该最终provenance、README/集成版本及随后刷新索引在不可变归档之外；包内索引保留早期生成时间的0.6.18快照，不把旧manifest当0.6.20全量目录，应使用当前工作区最新evidence-catalog。归档本身未覆盖重打，旧版本/失败/诊断均保留。

00:18收尾复查：当前Codex主PID仍561018、创建于10月28日18:09:42.330，主进程RSS约517.8MiB，可用内存23.27GiB；所有本轮使用的10个HTTP/WS端口均释放。21:22之后内核/用户日志过滤未返回OOM/context lost匹配，query code1及可用baseline code0均原样保存，不把空匹配夸为完整崩溃审计。首汇总脚本误要求grep退出0而在空输出时拒绝，修为保存真实状态和独立可用性，不补报第一次成功。历史同应用组OOM整窗退出证据仍成立，Python具体脚本与直接主进程信号未知，未宣称系统根因修复。见crash-check-0320/recheck-20261030-0014.json。


### I44：精确定位GL程序查询与异步准备方案复核（0.6.20诊断，尚非修复）

在未修改的冻结0.6.20普通安装上，GUI显式profileShaders=true，域192/22888/24490、完整回放/q16/480秒UI与原内存守卫，先纯Scene，再RGB，再曲线。Scene阶段12资产/21TF/33Marker/约8.6万显示点，CPU renderMax44.2ms、longtask0、getProgramParameter max7.3ms；加入RGB后getProgramParameter单次1003.7ms、renderMax1033.5ms、同阶段页面longtask1035ms，programCreations30；第三份快照是在480秒守卫自动退出后保存的保留计数：program52、累计156次程序参数查询、最大仍1003.7ms、页面longtask2228ms；不能当作在线第三阶段。无着色器/绘制错误或缺钩，控制台7条Z-UP提醒、0error。该计时没细分pname，不能将所有长任务或某枚举直接归为根因，profile模式还多一次设备信息查询，不能冒充默认冷启动。

守卫先于最后快照退出；之后显式断开，本地订阅/等待/三容器0，临时页关闭，不能当作在线直接断开证明。子树峰值586.29MiB不含浏览器，PID身份与22888/24490主机核对释放。原始分阶段JSON和终态见runs/ros2_web_transport_20261030/render-profile/peach-native。一次使用未提供的locator.inputValue在变更前失败，改为只读DOM值再通过可见textarea填写，没有绕过UI执行隐藏对象方法。

随后阅读固定Three0.186.1 compile/prepareMaterial/getProgram/compileAsync及WebGLProgram.isReady，拟用公有异步准备避免首次uniform/LINK查询等待。已核对compile阶段不取uniform位置，KHR轮询每10ms，无扩展会固定延迟。原全Scene shaderRevision方案写入async-shaders/design.md/plan.md，在改任何产品代码前由独立builder check发现三处重要缺口：无KHR固定10ms或持续新program时严格最新revision门可能永远不绘制；透明DoubleSide编译两program却只等最后currentProgram；shaderChecks独立pending对淘汰/丢失句柄需清理。dispose的Three properties释放可使其内部poll最后收束，但不清自有pending。

原方案不采用，设计原文/审查/实施ruling保留，Task未标完成；没有把未验证全帧等待放进产品，也没有改版本号为0.6.21。技术细节由实现者解决，不以此要求用户重复授权。下步先program-query-trace复用已验证Canvas工具增加四种pname和draw/buffer/delete等实际调用的计时，不新增查询/ROS订阅，不保留句柄/载荷，仍48行/32慢调用。两个受控反例先红，14组全绿；普通诊断副本与原200文件安装仅三处前端差异。产品源码保持0.6.20，真实新工具结论另补，不能把工具绿灯当性能修复。

program-query-trace/peach-native 已取得终态：域193/22988/24590、默认 profileShaders=false、完整录制、临时三文件诊断副本、480秒/1GiB/4GiB守卫。中段在线快照捕获 `getProgramParameter(ACTIVE_UNIFORMS)` 单次1575.3ms；同阶段 renderMax1576.9ms，页面 longtask1771ms。52次各枚举查询中 LINK_STATUS/ACTIVE_ATTRIBUTES 最大均0ms，其他已计时调用最大17.6ms。33行、无溢出/缺钩，完整21TF/33Marker/约8.6万显示点/600曲线采样，控制台7条Collada Z-UP提醒、0error。调用经过时间包含调度暂停，未记录具体材质/program身份或精确时钟回退时点，不能宣称独占GPU耗时、每次回退均触发或已解释所有长任务。

Three源码对应的是新program首次使用时的uniform枚举；即使关闭成功日志检查仍会经过该调用。上述证据收窄了等待位置，尚非优化结论。最后快照的CONNECTED仅是客户端状态；已补存文件时刻与守卫窗口，不把它当作单独RPC存活证明或在线人工断开验收。子树峰值551.81MiB不含浏览器，480.43秒后守卫清理，PID身份/22988与24590宿主机复核释放，临时页已关闭，原200文件安装哈希保持一致。详见[精确查询终态](../../../runs/ros2_web_transport_20261030/program-query-trace/peach-native/verification.json)。

01:19再次只读复核整窗退出：Codex主PID仍561018，创建时刻10月28日18:09:42.330未变，主进程RSS526.61MiB、可用内存23.28GiB；17个历史/本轮测试端口均无监听。00:14以来内核及用户journal过滤均返回code1/`-- No entries --`，baseline读取code0，原始输出与空匹配分开保存；这不是完整崩溃审计。历史同应用组OOM关联整窗关闭仍成立，被杀Python脚本与主进程直接退出信号仍未知。未重启应用、未启动测试或扩大归因，见[整窗退出复查](../../../runs/ros2_web_transport_20261029/crash-check-0320/recheck-20261030-0119.json)。

第二次builder check已完成：按root固定任务比全Scene revision门更窄，但当前修订稿仍不宜直接实施。隐藏对象仍被compile遍历，失败ticket可能被后续整Scene准备再次纳入；固定完成集合还须覆盖取消任务留下的未完成缓存program，并把“完成”与“LINK成功/捕获完整”分开；context restore不能把机制改成false的visible重采样为目标可见性。当前retire只保留材质、立即移出旧对象，不等于保留旧画面。更小候选为公有compile(root,camera,mainScene)+现有RAF轮询，需先隔离验证收益；不新增不可取消Promise或第二timer链，不因审查建议就宣称修复。原稿、原计划、第二次审查及不实施裁定均保留在[实施记录](../../../runs/ros2_web_transport_20261030/async-shaders/ledger.md)，产品仍0.6.20。

### I45：公有root预编译的真实Peach隔离实验（产品仍0.6.20，性能PARTIAL）

沿I44第二次审查建议，先做工作区白盒实验，未改产品Scene或新增公有API。固定Three0.186.1以compile(root,camera,mainScene)收集主场景光照，一次有限cohort覆盖调用前pending及同步compile的新链接；KHR completion为true后才LINK检查，失败与完成分开。每次实际render期间临时隐藏未准备drawable，finally恢复，继续用原Scene RAF，无自有Promise/timer/RAF。每帧最多8次准备、2048 drawable、512 pending program、4条有界错误；无扩展明确不启用实验。固定灯光、无自定义材质回调修改是本次边界。白盒依赖编译后的Scene字段并替换原shader checker，绝不能直接作为产品集成API。

初始7组实际helper检查先红再绿。独立审查发现共享透明DoubleSide在Three compile/render中自行增加version，两个mesh会互相使签名过期、连续4帧全隐藏；另有失败缓存program被后续空集合放行。review-red保留两反例，修为仅吸收同步库调用自身的版本变化、真实LINK失败/捕获溢出标记实验invalid。根agent再复现render中contextlost令WeakMap换代后finally抛错，修为先完整恢复visible再更新仍有效元数据。审查补指出compile异常没有上游回滚保障，fatal-red→green后当次及后续禁止原render。11组及语法检查和独立复核通过后才启动GPU实验。全部过程见[root-precompile](../../../runs/ros2_web_transport_20261030/root-precompile/plan.md)及其审查/原始红绿日志。

从冻结20普通安装创建202文件副本，仅app/index及两个诊断script四项差异，原200文件哈希不变。域194/23088/24690、完整Peach录制、q16及480秒UI上限/1GiB子树/4GiB余量守卫。真实项目10项只读通过，完整URDF15关节/6活动、12资产、21TF、33Marker、约8.6万显示点、800Path、640×480彩色后切深度（0.00025m/unit）、曲线600。点云开关、视角重置、缺TF负例及base_link恢复、3D停止重启均实际操作。初启及重启均无绘制/链接错误，控制台14条Collada提醒、0error。

首Scene末快照：9864次部分/完整提交、准备passes67148、取消1050、81 ready/6 waiting，compileMax2.7ms；ACTIVE_UNIFORMS42次最大0ms、renderMax8.2ms、CPU提交p95约1.8ms，但页面longtask2456ms。重启后3337提交、passes23711、49 ready/38 waiting，累计uniform59次最大0.1ms、renderMax8.7ms/CPU p95约1.5ms，页面longtask1311ms。早期快照也记录87 ready/0 waiting。等待数随数据变化，不能据单张快照宣称永久饥饿，或把部分提交计数当成每个对象都已可见。每次prepare也可能只是缓存命中，不是实际新shader编译次数；高pass来源尚未区分输入对象更替与版本失效。

均有暖环境/观察器/顺序效应，没有配对随机化A/B，故uniform旧1575.3ms未复现只能算候选可行证据，不能认定独占GPU根因已修、所有长任务解决或冷首载通过。候选不合入默认，下一步先区分对象更替/签名失效开销与未定位的页面长任务。产品仍20；I36、OOM脚本及LAN/Android等开放项不变。

477.95秒由人工stop.request收尾，非完整480秒耐久PASS。直接断开即时订阅0/等待0、三个容器canvas均0，临时标签17关闭。子树峰值464.33MiB（不含浏览器）、可用最小23009.30MiB，守卫failure=null/remainingPids=[]；宿主机按5个PID创建时间与23088/24690复核全部退出，原安装哈希与候选script一致。详见[本次PARTIAL终态](../../../runs/ros2_web_transport_20261030/root-precompile/peach-native/verification.json)。测试期间未修改运行副本或刷新证据目录。

### I46：剩余长任务的原生归因与实际codec计时（诊断，产品仍20）

先读实际BoundedTransport与ImageView：raw解码是同步，已在既有Image回调分段内；没有凭猜测增加异步raw管线。工作区增加原生long-animation-frame观察器，仅取裁剪的标量/源码位置，不读entry.window/toJSON或载荷，无额外timer/RAF；每交付最多检查32帧，每帧最多看32script，从已检查数据保留top8帧/top4script，未覆盖量明确显示。原8组红绿后独立审查发现“前32帧子集却称全部最长”的口径错误，补第33个最慢9999ms反例红→绿，现9组。SDK诊断bundle只对实际bytes/JSON或CBOR decode/accept包同步measure，不改变协议/this/返回/异常；三处差异独立核对。副本203文件/6项差异，原安装manifest实际为200文件。先前201是单script诊断副本数，当前文档/元数据已纠正，不重写冻结归档。

域195/23188/24790、同一完整Peach回放/q16及480秒UI/1GiB子树/4GiB余量门，10张可见只读卡片通过；在线阶段模型15关节/6活动、21TF/33Marker/约8.6万显示点/800Path、640×480彩色与曲线继续运行。LoAF原生支持、无采集错误；最终108帧全检查、无预算跳帧，只保留8帧，平台buffer丢失未量测。实际wire最大：字节计数2.3ms、CBOR0.8ms、JSON2.4ms、分发9.1ms，renderMax10.3ms；它们没有复现同秒级阻塞，只代表本次数据/设备/观察器条件，不能关闭I36或外推任意输入时延。

45秒无CUA操作窗口的前后文件为02:20:18→02:21:33：seen73→91，出现新最大2009.7ms帧，仅animate5.7ms获脚本归因，保留帧最大blocking1696.9ms。未采timeOrigin，不能精确epoch对齐；后台浏览器/自动化工作没有被排除。页面实际visible、hidden=false、约55FPS，不能把全部长帧简单归为后台节流。早期约1秒帧的blockingDuration为0，证明帧duration不能直接等同主线程阻塞；空scripts也不等于无JS或已证明GC/GPU原因。

守卫480.46秒自动正常清理；02:22:58末快照在退出后，显示CONNECTING、模型/TF/数据/ROS时间清零，不能计在线末阶段。末页longtask2229ms，LoAF最大2487.8ms/blocking2178.1ms且无script归因。截图已查看并命名browser-after-guard.png，不是完整画面验收。随后停止LoAF并断开/取消重连，0订阅/0等待/三Canvas0，18标签关闭；5PID创建时间与23188/24790宿主机清理确认。峰值551.31MiB子树不含浏览器，可用最小23136.31MiB，guard failure=null。未因观察到重连重启fixture，完整终态见[本次归因报告](../../../runs/ros2_web_transport_20261030/frame-attribution/peach-native/verification.json)。

工具失败原样记录：npm roslib没有src路径，按package入口dist及本仓adapter重新查阅；readonly DOM代理不提供hasFocus，未绕过限制，仅读实际可用visibility/公开指标。首汇总因visibility快照无phase抛KeyError，未产成功报告；独立verify.py把缺wire留null、按空白匹配可见卡片再核对通过。性能仍PARTIAL，下一步先做仅HTTP未连接ROS/未创建视图对照，收窄未归因阻塞范围；不据当前结果编造根因修复或引入生产框架。

### I47：同页HTTP-only空载对照（非项目验收或根因证明）

沿用I46同一203文件普通安装，逐文件哈希完全一致，只启动原serve.py；预检无ROS进程，23288/24890均空闲，不启动bridge/rosapi/rosbag，不点击连接，不创建Scene/ImageView/SeriesPlot。复用既有HTTP守卫模式，360秒/HTTP128MiB/系统4GiB余量，未安装依赖或改产品。元数据写入也放在finally保护范围内，避免写失败后遗留服务。

02:39:58、02:41:05、02:43:41三个期限内快照均OFFLINE、0订阅、pending初始纯0、0Canvas，Canvas/wire计时rows为空。DOM script列表与HTTP日志均无可选3D/media或assets请求。两次等待前均用同一存活session确认，再各45秒不操作页面；实际保存间隔更长。LoAF已检查6→23→47帧，0预算跳帧，已检查帧的最大duration687.5ms，保留top8的blocking均0。后者不是全部47帧的blocking最大值；平台buffer丢失未量测，没有人为主线程正对照，也没有实例化Scene的单独LongTasks observer。

本次空载没有交付/检查到>=1秒原生帧，因而没有复现I46约2秒阻塞；只算有限对照，不能断言ROS数据或图形就是全部原因。原app指标/Canvas诊断定时器仍运行，不是无脚本控制。下一步按同一录制先只接收完整点云、无Canvas，再区分图形或更多流的影响。现有标准数据面板只接TF/JointState/点云/diagnostics，raw图像不会创建img；直接用该面板不能假称与全Scene+RGB+曲线等量。

HTTP守卫360秒正常自动截止，手动stop.request在其后，reason明确deadline；快照均在期限内，收尾页面为缓存脚本状态。峰值24.05MiB HTTP进程（不含浏览器），可用最小23651.92MiB，返回0/无failure。LoAF停止、19标签关闭；唯一PID创建时间与23288/24890宿主机核对释放。初汇总把从未连接的纯0误当0/256而断言失败，读原始快照后按实际语义修正验证器，没有改数据。一次sdk/observations.ts路径检索为空，实际逻辑在web/app.js，已按源码确认。详情见[空载有限终态](../../../runs/ros2_web_transport_20261030/http-only-control/verification.json)，状态PASS_WITH_SCOPE不构成整体通过。产品仍0.6.20。

### I48：WebGL丢失诊断与静态恢复重绘（0.6.21，冻结证明见I49）

按Khronos WebGL lost/restored语义和本机Three0.186.1源码核对：丢失时LINK_STATUS可返回null，旧context的GL对象恢复后失效；Three重建内部GL状态，但Scene的按需绘制仍需恢复事件触发。原shader逻辑把非true全部当链接失败，原Scene又没有context事件监听。新增受控反例保留真实红灯，null不可验证与实际false分开，丢失期间不查询/归因，真实失败历史保留，dispose各项独立收尾；不扩展SDK或ROS协议。

独立复核指出loss+restore都在flush之前的旧pending遗漏，补两事件同步失效。Scene只清pending并设dirty，无新增timer/RAF，Three继续负责资源恢复；dispose移除两个稳定监听，晚事件不能复活。12组shader、4组实际Scene帧边界、完整observe、audit20+9、preview12与类型/构建通过；普通Jazzy隔离构建和35项官方lint通过，SDK四个主要源文件仍与冻结20一致。CPU替身与原生验证分别记证。

真实IAB静态反例使用固定720×360根容器、无ROS数据的客户边界替身；未改视角/尺寸/dirty。冻结20两个稳定样本均成功帧2，原生lost/restored后仍2且geometries0、画面空白。修正后的独立源地址收到脚本200，恢复后成功帧2→3、geometries2、无shaderErrors，丢失期间renderErrors1并在恢复后保留，首次CPU提交时间未变；停止后画布/订阅/状态监听全0。第一次修正页复用23388命中旧脚本缓存，日志仅GET / 304，失败保留并排除新版验收，改用新23488源地址后通过。三轮HTTP与标签均清理。该证据是静态真实WebGL恢复，未对完整Peach模型做原生context故障注入。

完整Peach回放在0.6.21候选普通安装、新23588/24990与域196上回归，10项只读通过，12模型资源、15关节/6活动、21TF、33Marker、约8.6万显示点、800Path、640×480彩色与深度（0.00025m/unit）、600样本曲线可见。当前RGB阶段renderMax1011.9ms/pageLongTaskMax1774ms仍表明秒级阻塞未解决，不能按CPU p95约1.4ms宣称全部流畅。268.68秒人工stop.request结束，非480秒耐久通过。停止3D后容器清空，重启模型与12资产恢复；显式断开即0订阅/0等待/三个Canvas0。第二Scene renderMax50.9ms，但pageLongTaskMax2364ms仍在。14条Collada提醒、0控制台error；五个PID身份与四端口均清理，子树峰值465.66MiB（不含浏览器）、可用最小22604.33MiB。208个安装文件含14个运行缓存，194个非缓存文件运行前后哈希相同。页面报告JSON是有界摘要，并未导出完整原始报告；10项结果按可见卡片核对。见[Peach范围终态](../../../runs/ros2_web_transport_20261030/context-recovery/peach-native/verification.json)。

全部失败、路径/替身修正、两轮独立审查与引用在[本轮过程记录](../../../runs/ros2_web_transport_20261030/context-recovery/ledger.md)，静态原生核对见[static-verification.json](../../../runs/ros2_web_transport_20261030/context-recovery/static-verification.json)。本节先在21候选核对，随后按I49冻结；20归档不覆盖。I36、长任务、历史OOM脚本、Android/LAN仍开放。

### I49：0.6.21源码冻结与离线重建

归档163文件，SHA256=`c0b173e14a031f6108be5500d87260dac3d20b46b57d3e042c502734312d2939`，与旧20分别保存。全新目录从包内dist/build-package-lock.json恢复npm打包省略的根lock，已有本机缓存离线安装28依赖，不联网补装或升级。check/build、54个dist逐字节重建一致，四公有入口导入、配置CLI、observe/audit/preview与仅Jazzy普通隔离colcon构建/35项官方lint通过。97个运行源码/配置/锁文件与当前工作区逐字节一致；归档内活文档是形成最终证明之前的快照，I49和后续证据索引更新不追写进归档。

首次编排把CMake web重建与CLI同时调度，虽然均返回0，但它们会共享dist，因此不以该并发轮作为最终稳定目录证明。构建终态后顺序再跑observe/audit/preview/config/import全部通过，再核54dist哈希。原始日志及最终证明见[源码冻结终态](../../../runs/ros2_web_transport_20261030/source-rebuild-v0621/verification.json)。这一重建只证明本地源码可复现，不替代I48的原生静态恢复/完整Peach普通回归，也不证明完整模型的context故障注入、全版本长测、秒级卡顿或整窗退出修复。所有本轮测试和浏览器临时页已结束，未动驱动、真机IO、相机或Gazebo。

### I50：同页完整点云无Canvas与完整观测分阶段对照（产品仍21，性能PARTIAL）

沿I47计划，从冻结21普通安装复制195→196文件，只在index加一份工作区原生observer脚本，SDK/app/scene/media不改。两种PerformanceObserver分阶段观测longtask/LoAF，takeRecords先于disconnect，无历史buffer、旧阶段身份隔离、6阶段与条目/脚本/字符串/错误预算；无新增timer/RAF。缺模块、首版语法错误、第9组缺capture及review发现的UI快照覆盖均留红；独立live节点修正后10组/语法与只读复核通过。详情见[过程与审查](../../../runs/ros2_web_transport_20261030/data-only-phases/ledger.md)。

域197/23688/25090，完整Peach回放，q16/600秒UI/1GiB子树/4GiB余量，无XML或RMW覆盖。实际正对照longtask80ms/LoAF84.4ms且button脚本获归因。空闲连接70.316秒0订阅/Canvas，0longtask，8LoAF最大977.7ms/blocking0。完整点云第一段69.976秒，Reliable/CBOR/100ms、约17.2万点/4.13MB、1订阅/0Canvas，0longtask，4LoAF最大746.3ms/blocking0，UI计数+133。取消该订阅后启Scene/RGB/Plot，12资产齐全后测84.728秒：9订阅/3Canvas、3longtask最大1371ms，14LoAF最大1554.7ms/blocking1317.9。停三个组件，再测相同点云68.164秒：计数+85、0longtask、3LoAF最大899.5ms/blocking0。每段capture窗口至少45秒无CUA操作，visibility=visible；不是随机化，回放相位/缓存/不同流/后台工具仍未排除。最大完整阶段帧仅animate6.5ms获脚本归因，不能补推其余为GPU/GC/工具。停止后缓存的scene-metrics不算活实例，Canvas0与停止状态另证。

服务器5条写队列溢出警告，4条处于完整观测启动但未计量区间，1条按timeOrigin落在1371ms长任务内。warning每秒节流且没有topic/client字段，只能证明部分同窗，不能识别丢哪路或精确丢弃数，更不能定因果方向。q16为实验，默认64本轮未验；transportStats无lastError不证明服务端没有丢。两段点云计数率不同仍非逐帧门；离线116条录制接收metadata有27个相邻间隔<100ms，100ms贪心示例为93条，不能当实际排队模拟。普通Topic的queue_length1与订阅节流也可合并突发，Reliable DDS不等于Web端无损，队列源码SHA与离线原始数据均入本轮证据。

548.08秒人工停止，非600秒耐久通过。observer0、断开0订阅/0等待/0Canvas、24标签关闭，五PID身份和两端口宿主机全清。子树峰值511.65MiB不含浏览器，可用最小22671.19MiB；guard failure=null。原冻结普通安装195文件及产品运行源码不变，未新发版。没有运行项目10项用例，本轮只做性能分组；前轮10项证明不得冒充本轮重做。精确数值、限制、警告时间与清理见[PARTIAL终态](../../../runs/ros2_web_transport_20261030/data-only-phases/peach-native/verification.json)。

下一步保持同样九路和组件，只切换3D图层显示→隐藏→显示，先控制不同流集合，再决定针对哪条实际慢路径修正；不盲目扩队列或把当前阴性对照当根因关闭。I36、全版本默认配置持续流、冷GPU与Android/LAN等开放项不变。

### I51：固定九路流的五层显示/隐藏/显示（产品仍21，性能PARTIAL）

从冻结21普通安装复制195→196文件，只改index加载临时observer；observer相对I50仅加三个阶段名，新增Unknown phase红→绿，原10组加标签共11组/语法和独立协议复核通过。相同完整回放域198/23788/25190，q16/600秒/1GiB子树/4GiB余量，Scene/RGB/Plot只启一次，所有正式阶段维持9订阅/3Canvas/12资产。六个端点sentFrames均9、Canvas尺寸不变，服务端一次连接/一次断开，图层动作未发新ROS订阅或命令；仍不等于每路逐帧连续交付。

全显示74.086秒，101 drawCalls/150277 triangles，3longtask/max2163ms，LoAF2401.7ms/blocking2111.4；五层全隐藏72.116秒，实际网格截图及2 drawCalls/0 triangles验证，但仍2longtask/max1688ms，LoAF1797.7ms/blocking1635.8；恢复显示157.941秒，101/150277，14longtask/max2001ms，LoAF2091.9ms/blocking1953.2。各段收帧/字节及成功帧持续增加，隐藏不停止数据、资源更新或其余图像曲线，也仍有网格/轴绘制。每段至少45秒无CUA窗口，实际端点间隔73.178/71.362/157.176秒，不把较长第三段算作等长比较。

Scene累计renderMax六次采样均43.4ms，首段前已达到；相应原生秒级longtask未被该绘制提交计时捕获。隐藏几何没有消除阻塞，仅据三角形数量不足以解释当前全部现象；不能把未归因部分称为GPU/GC/工具或排除所有场景处理。回放相位/缓存/资源更替/非随机化/后台活动及原生buffer遗漏仍开放，未逐帧测视口曝光/输入延迟。全程q16日志36条写队列溢出，按阶段/过渡时间留证，不是精确丢弃数，无topic/client归属，默认64本轮未验。

509.95秒人工结束；observer0、断开0订阅/0等待/0Canvas、25标签关闭，五PID身份与两端口清理，guard failure=null；子树峰值559.76MiB不含浏览器，可用最小22590.43MiB，7条Collada提醒0浏览器error。产品SDK/app/scene/media未改，不新发版，也未重复项目10项用例。完整数据和限制见[固定流PARTIAL终态](../../../runs/ros2_web_transport_20261030/layer-control/peach-native/verification.json)，所有过程/审查/截图在[本轮记录](../../../runs/ros2_web_transport_20261030/layer-control/ledger.md)。

现有浏览器能力仅显示/视口，未开放原生Trace；已请求独立Chromium/Playwright技术对照选择，未在无答复时执行。后续先补公开LongTask名称/容器归属再定位，现有记录未采字段不倒推。Long Tasks规范当前标为2026-10-20 Discontinued Draft，实际能力必须feature detect，不当最终推荐标准。I36、默认配置持续流、冷启动和Android/LAN等原门保持开放。

### I52：原生长任务公开上下文归属（产品仍21，性能PARTIAL）

复用I51临时观察器，仅给longtask投影增加name、最多4项attribution的name/containerType、遗漏计数与快照深复制；不用containerSrc/Id/Name、Window或toJSON。显式capture增加有界页面自报UA，缺失null。受控字段缺失先红，原11组保留后14组/语法与独立只读复核通过；无新增timer/RAF/ROS实体。普通冻结安装195→196文件，两项差异仅index与observer，97个产品运行源码及21归档SHA不变。规范、源码版本、失败与审查详见[过程记录](../../../runs/ros2_web_transport_20261030/task-attribution/ledger.md)。

域199/23888/25290，完整录制回放、q16、默认RMW无XML，360秒/1GiB子树/4GiB可用内存守卫。80ms自有按钮报告self及unknown/window，LoAF85.1ms/blocking30.2且实际来源为按钮脚本。完整Scene/RGB/Plot正式在线端点间隔90.8431秒，包含一次45秒无CUA窗口，均9订阅/3Canvas、12资产、15关节/6活动、21TF、33Marker、86269点、800Path、RGB640×480；收帧+22457/字节+952515823、绘制+4897、发送计数均9。8条longtask全部检查且保留，最大1847ms，均self及unknown/window。18LoAF最大2001.9ms/blocking1794.8ms，最大三条scripts=[]；Scene累计renderMax两端均53.1ms。self仅指当前浏览上下文，不能定位产品函数、GC、GPU或工具责任。

第二个计划窗口尚未开始，分析期间360秒守卫已到，未重启补齐。后端360.17秒退出，最终421823.3ms结束的阶段包含断线重连，19longtask/max1874、92LoAF只作混合时间线保留，不算在线持续验收。终态observer0、断开0订阅/0等待/0Canvas、26标签关闭，五PID身份及两端口主机复核全清。峰值445.23MiB子树不含浏览器、可用最小22724.53MiB，guard failure=null；7条Collada提醒、0控制台error。q16全程12条写队列警告，不是精确丢弃数，默认64本轮未验。没有重复项目10项用例。在线和守卫后字段分开的[范围终态](../../../runs/ros2_web_transport_20261030/task-attribution/peach-native/verification.json)为PARTIAL。

页面自报Chrome156而主机静态Chrome152，前者不是二进制版本证明；没有执行独立Chrome/Playwright自动化，仍待明确选择。补看SeriesPlot有100ms的dirty/setData定时回调，Scene计时不含它，下一步宜临时测量实际调用，不根据长任务时间小数相近直接定因。I36、默认配置长流、冷GPU、Android/LAN及历史OOM脚本等原门不变。

预读固定uPlot1.6.32官方tag声明与本机dist后确认：setData通过commit排入microtask执行_commit，后者含比例/尺寸、布局、绘制、光标/图例；公开drawClear/draw只覆盖绘制子段。仅测setData同步返回会漏掉后续执行，batch则改变调度。下一步诊断须保持原队列并覆盖真实commit或明确子段限制；本轮没有实施第三方改动，参考原文链接见上述过程记录。

### I53：真实uPlot微任务提交体计时（产品仍21，性能PARTIAL）

基于I52已核对的uPlot1.6.32源码，在冻结普通安装的临时副本中对setData/setSize/_commit三函数体外加try/finally经过时间；原正文、commit/microTask调度和return/throw保留，无额外timer/RAF/ROS实体。helper固定3scope/top8/数值token/generation与深复制。首版probe语法红与缺helper红分别保存，独立复核指出替代队列组不足以证明真实调度；补用原bundle的microTask+commit，原/补在queueMicrotask和Promise fallback下两次setData均合并一次提交，结果相同。8组、语法及独立复核通过后才运行原生。所有方案、失败、补正及审查见[过程记录](../../../runs/ros2_web_transport_20261030/plot-commit/ledger.md)。

普通195→197文件四项差异仅index、media bundle、span helper与原phase observer；media source map已标记失配，仅诊断，不作为产品修改。域200/23988/25390、完整Peach录制、q16、默认RMW无XML、360秒/1GiB子树/4GiB可用守卫。一次启Scene/RGB/Plot，原生80ms按钮正对照后，12资产就绪重置generation1。正式native两个在线capture间隔62.3488秒、plot capture62.3446秒，包含一次45秒无CUA窗口。均9订阅/3Canvas、15关节/6活动、21TF、33Marker、约8.6万点、800Path、RGB640×480、曲线600采样；Canvas尺寸稳定、101 drawCalls/150277 triangles。

同步setData与实际_commit各100→584，窗口各增加484；最大setData0.6ms、commit1.2ms、invalid均0。setSize为0，不能宣称resize阴性。同期longtask count1→11（新增10），最大2346ms完整落在在线端点内；phase到after累计22LoAF，最大2605.6ms/blocking2295.4，最大帧scripts=[]；Scene累计renderMax35.7→45.8ms。收帧+14629/字节+607499427、绘制+2582，发送计数均9且服务端一次连接/一次断开。该窗口实际曲线提交体没有对应秒级调用，仍不能将函数返回后的布局/合成/GPU、其他JS、GC或工具定为根因；不是全面排除绘图工作。

139.17秒人工stop.request结束，非360秒耐久通过。观察器0、plot trace stopped、断开0订阅/0等待/0Canvas、27标签关闭，五PID身份/两端口主机全清，guard failure=null；峰值441.52MiB子树不含浏览器、可用最小22534.78MiB，7条Collada提醒0error。q16全程14条写队列警告，非精确丢弃数，默认64仍未测；本轮未运行项目10项用例、未调用独立Chrome/Playwright。原195文件、97个产品运行文件及21归档SHA保持一致。精确端点、计时覆盖、未覆盖resize、source map与清理边界见[PARTIAL终态](../../../runs/ros2_web_transport_20261030/plot-commit/peach-native/verification.json)。

下步优先补冻结21默认q64、默认RMW且无XML的多客户端持续流；旧v19/q16/XML的30分钟不能代替当前产品配置。浏览器秒级长任务、I36原间歇断流、冷GPU、Android/LAN与OOM脚本等原门仍开放，独立原生Trace技术选择仍待用户明确回复。

### I54：冻结21默认配置持续流复现服务超时（FAIL）

原195文件普通安装及97个运行源码保持冻结21，复用原性能脚本，不传队列或分连接覆盖、不启服务诊断、默认RMW/无XML。三个健康RosClient各单连接共享Topic/RPC，另加慢读者共4WS；与旧q16/XML/7WS不作A/B。域201/24088/25490、完整Peach录制、1800秒目标、2秒查询/3秒截止/10秒流门及原TF/恢复门不变，1GiB子树/4GiB余量。正式session58869从ready跟随至终态，没有因静默重启。方法、源码核对和失败详见[过程记录](../../../runs/ros2_web_transport_20261030/default-core/ledger.md)。

主机只读确认launch YAML队列64、实际加载rmw_fastrtps_cpp，无trace/RMW/XML覆盖；不是参数Service回读，也没有额外ROS观察节点。cwd两种默认XML均无。首次校验误把Jazzy的ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET当异常，保留校验FAIL，核对Jazzy dsv默认来源后只修正校验为PASS，不改变运行进程或配置。

381.95秒终态FAIL：567RPC中564成功/3TIMEOUT，三个初代客户端各189次/1失败；成功p95约20.882/22.498/24.829ms不能抵消超时。尚未到450秒首次重建，不算重建/1800秒通过。桥同CID565/566/567在SDK关闭附近、SIGINT之前也记录响应等待超时；无native send/take/source/received元数据，不能细分DDS/executor/WS或由断开相近推因果。下一步保持负载和3秒门，用既有显式WAIT/被动服务端/native诊断采服务链路，诊断负载另列，不加默认超时/XML/重试。

结束时三个color最近回调年龄248/250/247ms、depth100/76/89ms、cloud48/63/46ms，最后TF完整；这是Node回调年龄，不是源到屏幕延迟或全帧无损。clock回退6次；慢读2939帧/3890866241字节、9暂停/8已验证恢复，第9次暂停期间因RPC错误结束。峰值749.67MiB子树不含桌面/浏览器，Node173.71MiB/堆19.86MiB、可用最小22367.20MiB，未触内存门。57条节流队列满警告无topic/client归属，非精确丢弃数。

三个SDK订阅/发布/等待/监听/分片归零；4连接/4断开、六PID创建身份/两端口主机全清，remainingPids=[]。清理SIGINT的exit-2不当作另一次闪退。原195安装文件、97运行文件和归档SHA不变；[验证脚本与FAIL终态](../../../runs/ros2_web_transport_20261030/default-core/long-shared-default/verification.json)分开，脚本exit0不代表被测通过。architecture“候选未冻结”漂移已纠正，源码冻结与性能验收分开，未新发版。

当前默认配置长期稳定仍不通过。旧I36、秒级渲染/交互、Android/LAN和OOM脚本继续开放；未执行独立Chrome/Playwright。官方教程页面遇Anubis拒绝后改读官方jazzy仓库原文，远端rosbridge路径不可读则以精确本机apt源码为准；没有据单次Client示例推导缓存修复。一次含空hunk的文档补丁被拒绝，核对无部分写入后修正，不涉及产品或测试结果变更。

### I55：两个失败回复延迟收窄至writer→reader history（诊断FAIL，机制未定）

同冻结21/完整录制/q64/共享4WS/默认FastDDS/无XML，增加已有WAIT/native与被动服务端METADATA，不增加主动对照请求或late-reader，不改3秒截止/负载/产品代码。域202/24188/25590，session40699始终跟至终态。首次组合输入断言失败而编排仍启动的偏离完整保留：后验97源与原195文件SHA一致，额外6个Python缓存；未存首次分项/确切计数，不倒填201。后续依赖步骤显式检查退出码。原始输入、配置、偏离、源码语义及独立复核见[过程记录](../../../runs/ros2_web_transport_20261030/default-service-native/ledger.md)。

RPC约458.509秒失败，原5秒诊断宽限后守卫采样至463.28秒：681RPC/2TIMEOUT，分别slot0.1与重建后的1.2。首次450秒重建已发生，第二次1080秒及1800目标未到；慢读3729帧/4925725719字节、10暂停/10已验证恢复、clock回退7次。四代成功p95约23.321/22.092/6.280/18.654ms，其中1.2仅两份成功样本，不能把低p95当稳态通过。

4520 native记录零drop/error/超预算，wire2043条与rpc1362条退出缓存0、sink error0；被动服务端接收/发送各681、主动calls0。WAIT失败文件空符合行为：两条Event.wait最终均true，最终ring保留完成；不保留late reader也捕获成功take。Web已过3秒TIMEOUT后收到成功回复，不能改判PASS。详见[诊断与失败分开的终态](../../../runs/ros2_web_transport_20261030/default-service-native/long-shared-native/verification.json)。

按Web CID→native clientKey/sequence/最近生命周期时间窗配对，避免681次send/678个不同key的实际地址复用；本次CID679/680/681各key只一次send。失败679/680的writer时间戳→reader history精确2999.555773/2999.193907ms；history→takeReturn wall约0.113/0.337ms，native send→take约2999.891/2999.815ms，后续Event.set也很快。Node wire往返3003.637/3003.616ms才收到同CID result=true，约晚于Web错误4ms。同轮681成功约4.671ms，writer→history0.012600ms，是有限对照，不证明全系统正常。

Python mono为秒、Node wire mono为毫秒，只本域相减；DDS纳秒以整数差计算，wall尾部为近似。native wall−mono全程变化3.185µs，未见秒级宿主跳时，回放clock回退与宿主时钟不同。服务端681事件对独立统计最大间隔约0.792ms，不按时间硬配CID。精确rmw_fastrtps8.4.4与FastDDS2.14.7定义表明字段是writer写/reader入history，不是NIC；因此只确认这两回复的主要延迟在该阶段，尚不证明发现、心跳、丢包或重传分支。独立原始数据复算通过，见[review.md](../../../runs/ros2_web_transport_20261030/default-service-native/review.md)和[timing-analysis.json](../../../runs/ros2_web_transport_20261030/default-service-native/long-shared-native/timing-analysis.json)。

峰值806.06MiB子树不含桌面/浏览器，Node180.65MiB/堆21.32MiB、可用最小22478.66MiB，未触内存门；66条节流队列警告非精确丢弃数。5连接/5断开，SDK资源归零，七PID身份/两端口主机全清。原97源/195文件/21归档SHA不变，额外缓存单列，无新产品版；本轮不是默认无观察器验收或I54/I36同因证明。

下一步在临时副本复用一个标准/rosapi/nodes rclpy.Client，保留原call_service正文和截止、标准Node生命周期，检验短命原生实体是否参与现象。实体数/寿命/并发pending同时变化，须核对单实体、递增sequence、取消与退出清理，重复同负载后才判断；一次通过也不直接当发现根因或产品修复。ROS上游相关issue只作候选来源，不据描述直接改KEEP_ALL/超时/XML。独立复核认为该窄实验与证据相称，尚未实施；浏览器卡顿、Android/LAN和OOM脚本等原门继续开放。

### I56：临时单原生Client复用30分钟对照通过（产品修复仍未合入）

针对I55约2999ms writer→reader history段，保持冻结21/原录制/q64/共享4WS/默认FastDDS/无XML与原3秒协议和Web截止，只在普通安装临时副本复用一个Node拥有的/rosapi/nodes Client。标准Node创建/销毁及原call_service正文不变；这个变量也改变实体数量、寿命、并发pending与发现负载，不能单次归因发现/心跳机制。原安装201文件先完整取hash，旧195及额外6缓存分别列证，临时副本只有入口两行与新增V2 helper。97运行源、原安装和归档不变，无产品新版本、硬件/相机/IO/Gazebo或Peach/IVG迁移。全部源码、中间失败和限制见[本轮过程](../../../runs/ros2_web_transport_20261030/native-client-reuse/ledger.md)与[计划](../../../runs/ros2_web_transport_20261030/native-client-reuse/plan.md)。

V2锁外原生创建、弱owner/client缓存和token/epoch占位，guard50ms/创建等待0.5s，未知或超预算标诊断无效并原API直通，不能把ready.maxCachedClients=1冒充实际实体数。逐RPC只跳过缓存身份的异步销毁，最终Node真实销毁，pending只读有界计数，无额外Future回调/重试/payload保留。初版保留撤回，审查复现的锁内创建阻塞observer、旧D2销毁擦新C2均先红后绿；关键词/快照/取得锁后invalid复检等共13组CPU检查。独立审查与修正见[review.md](../../../runs/ros2_web_transport_20261030/native-client-reuse/review.md)。

先在独立204域用真实AddTwoInts假服务和原services.call_service短测：三并发成功，第四次人为250ms慢响应在50ms超时后pending0，第五次仍成功，take顺序1/2/3/5/4，Node关闭一次/线程退出/恢复hook。它只证明真实rclpy机制，不是rosapi、原桥executor或项目验收。runner最终glob误纳ready导致汇总断言，原FAIL和旧脚本保留，修正数字PID筛选后只对相同已退出证据重算，不重跑。smoke关闭函数的timeout不限制ThreadPool join，真实外围守卫为45秒/512MiB；本次drained=true，不能复制其失败时无条件销毁作生产模板。见[有限短测结果](../../../runs/ros2_web_transport_20261030/native-client-reuse/smoke-native/verification.json)。

正式域203/24288/25690，session46781同一handle正常exit0，冻结SDK单调循环完成1800秒。2688RPC/0失败，五代成功p95为11.690/10.034/8.426/7.701/7.729ms；native始终单key、sequence1..2688，创建1/复用2687/逐RPC跳过销毁2688/Node真实销毁1、销毁前pending0/无在途。native10752条、encoded2123145B，零observer drop/error/超预算和原生操作错误；wire8064条、RPC5376条实际JSONL与stats/每代SDK计数一致，server逐GID/sequence请求/回复各2688，主动calls0。37个离线门全部通过，见[终态](../../../runs/ros2_web_transport_20261030/native-client-reuse/long-shared-native/verification.json)及[逐请求分析](../../../runs/ros2_web_transport_20261030/native-client-reuse/long-shared-native/analysis.json)。

native send→take p95 1.583756ms/max18.742308ms；writer→reader history p95 0.260146ms/max2.058818ms；wire往返p95 8.532986ms/max104.798753ms。分别在各自时钟域相减，DDS整数纳秒先核正数/非负差；wire result只为协议结果，不推SDK截止前成功。I55两个迟到true却SDK TIMEOUT的真实反例已用于审查。Client生命周期变化与本轮较低延迟相伴，仍不是具体DDS机制或生产缓存正确性的因果证明。

450/1080秒两次网页客户端重建、40次慢读暂停及40次恢复、30次录制clock回退完整覆盖；慢读12437帧/16232030144B，未在清理前关闭。各代URDF/关节/RGB/深度/点云均非零、TF最终完整，SDK和fragment资源归零，六连接/六断开，八PID创建身份（含benchmark父）及两端口主机全清。rosapi在收尾SIGINT退出-2并由launch记ERROR，发生于负载结束，不能写成所有子节点exit0；bridge及回放正常结束。

半秒资源最后采样1799.87秒是结束时长下界，预声明以冻结单调循环完成/原PASS证明1800，不按结果降低阈值。子树峰值1007.1015625MiB，不含桌面/浏览器及benchmark父，仅剩16.8984375MiB守卫余量；Node峰189.421875MiB/堆24.151817MiB，可用最小22007.8828125MiB。前期bridge/回放RSS增长，最后几个窗口大致趋平，不证明长期有界或泄漏根因。211条节流队列满警告无topic/client，非精确丢弃数。3504采样/窗口及边界见[memory-analysis.json](../../../runs/ros2_web_transport_20261030/native-client-reuse/long-shared-native/memory-analysis.json)。

分析器第二轮审查修正了协议成功混淆、重复CID字典覆盖、server总数漏检、无效时间戳、遗漏native错误和真实JSONL计数；逐gate累计且保留transportStatus，不让真实失败在写报告前被assert遮蔽。外层异常另存exclusive FAIL/PARTIAL，缺resources的真实CPU故障检查通过并保留既有FAIL；analysis/verification独占写，旧verify_v1保留。这些门均在正式负载终态前固定，没有重跑/替换失败会话。

这是固定单Node/单endpoint的诊断对照，尚未进入产品。下一步评估上游/通用生命周期方案，并验证类型/服务消失重现/多服务/并发取消/关闭的完整契约，再做产品源码冻结和无诊断默认组合持续流；内存余量也必须收口。I54原默认失败、I36间歇断流、浏览器秒级卡顿/冷GPU、Android/LAN、OOM脚本身份仍开放，不由本轮绿灯抹去。

### I57：通用生命周期进入暂存实现，未装入产品

I56的有限通过不直接推广固定endpoint全局钩子。本轮重新读本机Jazzy/rclpy7.1.12/rosbridge2.7.1相关8份源码并记录SHA，核普通Service逐次实体与rosapi参数缓存差异；上游参数缓存不能直接复用到任意类型/QoS/owner。通用设计选择bridge Node拥有的有界LRU/独立Future/容量护栏/关闭唤醒，仅适配受管Node与上游短Service正文，保留原协议权限/转换/上游main；不改全局rclpy.Node，不添加TTL/timer、新IDL或HTTP协议。参数与完整验证门见[设计](../../../runs/ros2_web_transport_20261030/service-client-design/design.md)及[实施计划](../../../runs/ros2_web_transport_20261030/service-client-design/plan.md)，fresh builder check为nothing important left。

新增真实关闭语义证据：隔离205域的SingleThreadedExecutor受控callback尚未结束时，shutdown(timeout=.01)约0.046ms已返回true，spin线程仍活；release+实际join后才销毁Node/关闭context。6个资源采样峰67.61MiB，进程创建身份已清，无ROS库修改。不能用shutdown返回值独自证明排空，官方tag和本机相同条件之外的内部布局也不能假定字节相等。见[result.json](../../../runs/ros2_web_transport_20261030/service-client-design/shutdown-native/result.json)及[source-review.json](../../../runs/ros2_web_transport_20261030/service-client-design/source-review.json)。

暂存candidate/service_clients.py已经过10个不同CPU组：任意服务/别名、两服务隔离、独立Future并发/公共pending清理、非法图谱/字段、Node拥有关闭、预算、LRU空闲淘汰和真实FakeNode高水位；进一步先红后绿复现销毁异常错误复用slot、异常traceback保留弱Node owner，并核错误文本1024上限。第一次测试夹具create_task漏可变参数的问题单独记录，没有改产品去迁就fake。所有初版、失败和修正见[ledger](../../../runs/ros2_web_transport_20261030/service-client-design/ledger.md)。

候选仍不完整、未安装、未改当前97个产品运行源；创建/关闭/取消/类型等剩余竞争与真实多服务/WS/新产品无诊断长测必须继续。原生工厂失败只缓存有界文本，等待者新建RuntimeError，不承诺日志具体异常类仍等于原生类；转换/Future异常仍直接走原路径。新实现不能拿CPU fake或此关闭语义短测代替原生通用池验收，当前产品仍0.6.21，未发布0.7.0。

### I58：通用池竞争反例、获授权的独立 Chrome 原生轨迹

前轮为progress：暂存池修复与31组CPU产生新证据。重新核对[cpu-matrix.json](../../../runs/ros2_web_transport_20261030/service-client-design/cpu-matrix.json)及24份终态日志，7基础组的重复导入不重复计数；新增共享创建票据/全部busy拒绝、绝对创建期限、关闭排队/工厂/退役/发送窗口、原生失败后的显式新调用恢复、过期排队票据、ready通知间隙、类型变化、乱序响应/单请求超时/图谱消失/Future异常、调度失败、准入先于转换与primitive统计。红绿保留于[ledger](../../../runs/ros2_web_transport_20261030/service-client-design/ledger.md)，首次即通过的回归不称red。候选SHA与未安装状态见[checkpoint](../../../runs/ros2_web_transport_20261030/service-client-design/checkpoint.json)；真实多服务/实现审查/产品接入和默认长测仍待完成。

用户明确选择独立Chromium＋Playwright同页/CDP录制，只用录制数据。插件空白探测仅headless/SwiftShader，关闭并按创建身份核对；CLI使用bundled Playwright1.62.1配本地Chrome152.0.7977.82，实际1440×1000/DPR1、RTX3090/ANGLE OpenGL、WebGL enabled。首轮可选Browser.getBrowserCommandLine因未启enable-automation失败，finally关闭；改为记录接口不可用而不加浏览器flag。空白硬件探测峰1288.98MiB，因此预先给独立浏览器2048MiB预算，ROS原1024/free4096门不变。具体协议/源码SHA、授权、环境见[计划](../../../runs/ros2_web_transport_20261030/playwright-native/plan.md)。主机已重启且旧/tmp普通安装消失；从原0.6.21归档c0b173…2939恢复普通colcon安装，核97运行源和54个现成dist，未将安装现成产物写成npm重建。没有修改Three/uPlot/WebGL或传输正文，网页只注入有界原生PerformanceObserver摘要/标记。

第一轮32MiB全浏览器trace在导出期间触2048MiB门（2056.01MiB/16.87s），没有完整trace/result，按FAIL保留；TracingService分项增至339MiB，不能据此推产品泄漏。原回放正常退出，418.9MiB，18身份与两端口清。第二轮4MiB缓冲完成38.57s交互，峰1908.49MiB；9订阅/3canvas，RGB→深度、拖动/滚轮/图层、调整视口、停止/重启/断开均执行；三份trace均dataLossOccurred=true，分析正确为PARTIAL，即使功能/清理门通过也不生成整体性能PASS。回放425.6MiB并正常退出，创建身份/端口清。原失败、[v2分析](../../../runs/ros2_web_transport_20261030/playwright-native/full-page-v2/analysis.json)与有损分类回归均保留。

第三轮进一步聚焦JS/layout/CPU分类，模型ready后不录trace暖运行90s，再采稳定与交互；浏览器守卫132.514s正常退出、峰1860.97MiB。12/12资产、15关节（6活动）、21 TF、完整输入云的有界显示约8.6万点/33 Marker/800轨迹点，截图已人工查看；RGB及深度640×480显示、9订阅/3canvas、停止0/0→重启9/3→断开0/0。页面原生长任务最大261ms、重启174ms；该窗口未复现秒级长任务，不能推广到历史IAB、冷驱动、LAN或Android。startup/steady两份trace无已知丢失且双标记跨度4633.586/10037.761ms；interaction有损且缺end。当前分类没有RunTask/完整调度图，最长已采集主线程scope分别EventDispatch257.351ms、FunctionCall26.503ms；后者包含V8.InvokeApiInterruptCallbacks24.801ms，不能全算传输业务CPU，也不能仅凭名字确认采样开销根因。

ProfileChunk实际有负delta，首版权重解释已废止：当前[frontend-analysis](../../../runs/ros2_web_transport_20261030/playwright-native/full-page-v3/frontend-analysis.json)只计有效节点采样，不将采样数/间隔当CPU毫秒，见[勘误](../../../runs/ros2_web_transport_20261030/playwright-native/analysis-errata.json)。保留的旧v2权重、v3旧权重文件不作为结论。独立只读审查重算六份trace SHA/字节数与v3摘要一致，接受PARTIAL范围，见[review](../../../runs/ros2_web_transport_20261030/playwright-native/review.md)。

会话中断后第三轮后端守卫父进程丢失，launch被重托管，stop.request无人消费，且无resources终态；具体信号未知。按owned.json完整命令/独占组及PID创建身份核对后仅清本轮组，见[orphan-cleanup](../../../runs/ros2_web_transport_20261030/playwright-native/peach-native-v3/orphan-cleanup.json)。通用分析保持FAIL_EVIDENCE、前端补充保持PARTIAL，不补造后端峰值/守卫PASS；最终全部96个已记录身份及14端口均清，详见[host-cleanup](../../../runs/ros2_web_transport_20261030/playwright-native/host-cleanup-final.json)。下一步优先修守卫中断收尾并取得可靠终态，再拆短交互trace消除loss；不借本轮关闭I54默认长测、I36断流、通用池/内存、历史OOM或Android门。

### I59：0.6.22 测试守卫可处理信号收尾

先前未知杀父信号保持未知，单独受控复现SIGTERM：原冻结0.6.21真实录制回放守卫父进程退出-15，没有resources，5个孤儿由外层按确切身份/自有组清理，见[signal-red](../../../runs/ros2_web_transport_20261030/playwright-native/signal-red/probe.json)。写源码前读Python3.12 signal及Jazzy launch_service.py；后者SIGTERM明示可能留孤儿，因此不改上游，测试守卫只登记轻量终止请求、由预检/就绪/执行主路径抛InterruptedError，在finally继续原自有组清理。原3秒RPC截止、协议、QoS、队列/内存门均不变。

候选v1的TERM/INT/重复TERM/正常stop/启动期5组实际回放通过，但[只读审查](../../../runs/ros2_web_transport_20261030/playwright-native/interruption-review-v1.md)发现3项Important：预检仍先于handler、poll→killpg/kill竞争可跳过其他组和落盘、浏览器未采样detached后代可漏清。没有直接装入产品。v2将预检纳入保护，subprocess.run保持10秒上限并拥有/回收子进程；组消失容忍，其他清理错误隔离后继续，mandatory清理先于可选诊断，最外finally原子resources.tmp→replace并恢复原handler。子清理结束、提交报告前是完成边界，晚到信号不开始新操作。cleanupErrors非空或中断必非零退出，正常stop.request仍成功。

同SHA67dbcecd…c5c1f的6组真实回放顺序执行，包括真正ros2 node list预检TERM；均自身收尾、无外层兜底kill、无已观测孤儿，正常case exit0/其余守卫exit1并记录SIGINT/SIGTERM，见[signal-matrix-v2](../../../runs/ros2_web_transport_20261030/playwright-native/signal-matrix-v2.json)。AST注入直接执行候选stop/finally正文，组已消失/组PermissionError/子kill前消失/资源写失败4组验证继续收尾、失败状态及handler恢复；是CPU证据，不冒充原生进程。外层本机systemd临时service设置75秒/MemoryMax1G/Swap0/KillModecontrol-group，实际每个后代/proc cgroup一致、内核memory.max1073741824；前几轮内核memory.peak约204–206MiB，RSS和stdout MemoryPeak不是同一量，后者显示2.6M不采用为峰值。所有unit仅本轮临时创建，无系统服务安装或硬件调用。

浏览器临时守卫另用真实detached Node后代在首采样前发TERM，原remaining=[]但后代活的[red](../../../runs/ros2_web_transport_20261030/playwright-native/browser-race-red/result.json)保留；终止Node前再次发现/登记子树后[green](../../../runs/ros2_web_transport_20261030/playwright-native/browser-race-green/result.json)自身清理。元数据写失败只记录failure继续mandatory清理，后续实际浏览器还必须套独占外层cgroup；Node先自行退出、未发现后代、SIGKILL/OOM/断电不靠Python独自保证。该小反例不等于Chrome/GL全流程再次通过。

独立[复审](../../../runs/ros2_web_transport_20261030/playwright-native/interruption-review-v2.md)接受CLI在声明范围接入0.6.22。对旧97运行源只改变测试CLI与3个版本文件，93文件不变，通用池未安装。首轮官方lint唯一E501（101>99）使汇总报告2条失败，原日志保留；只换字符串行，AST去位置完全相同，现SHAf675421b…cc86b，4组AST检查重跑通过，不伪称旧6组是在新SHA重跑。`npm run check/build`及普通colcon build成功，最终[官方结果](../../../runs/ros2_web_transport_20261030/playwright-native/product-colcon-result.log)35项零错误/零失败。完整差异、预声明设计、运行源码、格式等价和中间失败在[interruption-plan](../../../runs/ros2_web_transport_20261030/playwright-native/interruption-plan.md)及本轮ledger。安装入口核对和分发快照单独列证；不以此patch宣称持续大流/浏览器秒级卡顿或OOM已修复。

普通安装使用其自身setup与share/source/scripts真实入口，终止与正常停止两组通过、源SHA完全相同，见[installed-matrix](../../../runs/ros2_web_transport_20261030/playwright-native/installed-matrix.json)。四个JS公有模块导入通过；54个dist中53个与冻结0.6.21相同，仅build-package-lock变版本。汇总[product-verification](../../../runs/ros2_web_transport_20261030/playwright-native/product-verification.json)仅PASS_PATCH_SCOPE；后续源码快照及索引不扩展这些行为覆盖。

### I60：通用Service池的真实原生契约与反例修复（仍未集成产品）

0.6.22运行源码未动。I57暂存候选的31个CPU目的不能证明native寿命/队列边界；本轮按本机rclpy7.1.12、rosbridge2.7.1及FastDDS源码重新测试，详见[修订计划与全部历史纠正](../../../runs/ros2_web_transport_20261030/service-client-design/native-plan.md)。早期不同Context仍在同进程共享participant，导致同名异型server创建失败；后来使用独立provider进程。全局服务图谱可包含cached Client，类型歧义时改用标准server-only按节点API，捕获明确NodeNameNonExistentError，真实多种server类型拒绝猜选。忙旧类型拒绝、空闲旧类型即使cache有余也先退旧；LRU改key仍保留旧endpoint所有权。

旧候选真实停转executor八次超时积压八任务；修正后一个未消费ticket占一个slot，超时/未知提交错误不能假定任务未接收，Task不强持Node，直到实际消费才释放。早期6个CPU edge“RED”实际因导入未设__main__门而KeyError，按[INVALID_FIXTURE](../../../runs/ros2_web_transport_20261030/service-client-design/invalid-edge-import.json)明确排除；修正导入后重新逐个核预期Assertion的真实红绿，不混用旧无效日志。

真实multi首次空闲换型仍失败：destroy_client(True)只请求退役，Node.clients移除后owner WaitSet或迟到take线程还可持有原生handle。分别用两个确定性[WaitSet反例](../../../runs/ros2_web_transport_20261030/service-client-design/native-retirement-red/result.json)和[迟到take反例](../../../runs/ros2_web_transport_20261030/service-client-design/native-late-retirement-red/result.json)复现。修正使用每Client标准MutuallyExclusiveCallbackGroup：退役前占组排除worker，忙则明确retirement busy、不destroy、不隔离；destroy后finally放组，Destroyable已禁止新借用，后续owner任务dispatch再确认旧WaitSet退出。确认前保留slot/旧key/计数，close/timeout/未知ack提交失败也不提前建替代项。无sleep重试、新Node/线程/计时器或私有队列修改；限定本机已审builtin布局，不宣称通用public FIFO保证。

池要求同Node同executor寿命。无executor构造/中途更换明确拒绝；旧inline/换executor测试被明确的新contract替代，原失败和旧脚本保留。独立审查再指出Node.create_client先append后wake，异常时本地赋值仍为空；[真实RED](../../../runs/ros2_web_transport_20261030/service-client-design/native-create-wake-red/factory-wake-boundary.json)确为actualClients1/slot0。现在任何native factory异常都隔离slot，本次唯一group通过公有Node.clients识别到实体才计入已确认native/created；无匹配/枚举失败也不伪称原子失败或零实体。恢复须正常关闭旧Node后重建，不在原池盲重试。unconfirmedCreations是累计工厂失败次数（含找到已注册项），quarantinedEntries是当前隔离slot数；未知项不纳入“已确认原生计数”证明。send失败的后续显式退役恢复另测仍通过。

最终候选SHA08b5f925…43ecb，[真实single10/multi12](../../../runs/ros2_web_transport_20261030/service-client-design/native-factory-verification.json)全部PASS；49条send逐序号查公开get_pending_request均KeyError，observer errors为空、每组实际spin join后Node.clients0。数学服务进程树RSS峰分别151.11/152.02MiB；各独占512MiB临时cgroup/90秒父守卫，resources正常退出且remaining空，stdout的systemd MemoryPeak不作进程树RSS。全部输入原样快照/SHA验证。CPU最终38个高级CLI入口及导入基础7组通过，导入重复与两个忙类型重叠不累加unique数，见[CPU矩阵](../../../runs/ros2_web_transport_20261030/service-client-design/cpu-factory-matrix.json)。CPU语义版本dc85…06b6的3个E501只折行到当前SHA，[AST等价](../../../runs/ros2_web_transport_20261030/service-client-design/format-equivalence.json)，候选flake8通过；不伪称CPU在新SHA另跑一遍。

[最终只读复审](../../../runs/ros2_web_transport_20261030/service-client-design/review-i60.md)解除Task1限定环境Important，未发现新Critical/Important。 [主机最终清理](../../../runs/ros2_web_transport_20261030/service-client-design/native-host-cleanup-final.json)按全部PID创建身份、fixture命令和测试端口复核，无残留；临时单元已清除。Task1候选完成且仍在runs；Task2的精确布局门、上游实际globals/参数/首请求惰性创建/close/标准WS/WAIT/cache0尚待接入，Task3新普通安装与默认q64/shared/noXML/3秒截止/1800秒/1GiB门仍未执行。上游constructor先监听、main随后add_node，已修计划为监听前核配置/布局、首请求在Node私有锁下创建池；不能在_handle_parameters就构造要求附着executor的池。短数学服务不关闭I54大流失败、I36断流、浏览器交互有损trace、历史OOM或Android/LAN门。

### I61：受管rosbridge集成、参数与真实协议功能（开发中，尚未完成Task2）

上一轮为progress：I60的08b5候选、原生22组与审查已结束。本轮重新查工作树/源SHA及宿主无残留后按计划接入ServiceClientPool，原生池仅改说明与docstring风格，与I60去docstring后AST相同。新增service_client_compat，bridge_node加载上游非main模块后只替换main真实globals中的薄子类，不全局替换rclpy Node；上游Tornado、protocol、Action/Topic路由、信号与spin_thread.join保留。只有该受管Node分派到池，普通Node/cache0走原函数。代码、许可、指纹与参数来源见[源审](../../../runs/ros2_web_transport_20261030/service-client-design/task2-source-review.json)、[设计](../../../runs/ros2_web_transport_20261030/service-client-design/design.md)。

因constructor在add_node前监听，三个readonly部署参数及布局检查放在监听前，池在首Service worker私有锁内惰性创建。默认cache64/inflight128/creation5.0，范围0..1024/1..4096/(0,60]，响应timeout及网页3秒不变。无executor构造/换executor拒绝，close同锁禁复活、finally仍清Node且不吞异常。缺适配器API先RED、实际runpy副本/真实globals/幂等/普通Node直通通过；曾被finally return吞掉close异常，独立RED→去return修复。真实参数Node18组覆盖只读、非法值/NaN/类型/Events/inline/late冲突的监听前拒绝、16并发getter单实例、close先到不建池、未知runtime与显式cache0；[最终原生结果](../../../runs/ros2_web_transport_20261030/service-client-design/compat-startup-code-guard/result.json)PASS。该夹具在核原上游后替换listener边界，不能当真WS结果。

WAIT原库指纹拒新dispatcher的RED保留；新实现分别核原库、确切dispatcher与实际response method的源码/code，响应Event仍services.Event，工厂票据使用独立threading.Event。timeout取证记录responseCaller，不把创建等待分类为响应。新增参数/动作/服务权限不绕上游services_glob，标准CID/result=false与转换保持。首轮官方lint指纹常量格式/长行及D213修正，不关闭lint；第一次colcon test默认发现本轮备份CMake导致重复包名而未跑，备份加COLCON_IGNORE并显式base-paths后通过。[最终正式结果](../../../runs/ros2_web_transport_20261030/service-client-design/task2-colcon-results-cli.log)37项0错误0失败0跳过。

首个真实demo轮[web-cache64](../../../runs/ros2_web_transport_20261030/service-client-design/web-cache64/resources.json)因systemd环境Node18不支持import attributes而失败，ROS服务本身已启动，不将其写成缓存失败或PASS。测试守卫改为预检Node>=22并显式使用本机22.22.3，不修改全局Node。随后默认cache64与cache0均通过SDK17项、五组实际流与四个大帧精确往返。cache0首轮功能通过、收尾FAIL：守卫向整个launch组SIGINT、launch再次转发，日志显示asyncio teardown期间重复中断并需终止兜底。仅改守卫先通知launch root、保持原8秒门后[同cache0重跑](../../../runs/ros2_web_transport_20261030/service-client-design/web-cache0-root-stop/resources.json)最终清理成功。**I62更正：当时“正常收尾”的判断只看根进程/残留，漏掉launch日志中的5秒后SIGTERM升级；该轮及web-current64的graceful结论撤回。** 原始日志与报告保留，[更正记录](../../../runs/ros2_web_transport_20261030/managed-lifecycle/i61-cleanup-errata.json)区分功能通过、最终清理和正常退出；重复信号不是解释全部退出延迟的唯一原因。

独立只读审查发现默认cache与旧METADATA包装的真实冲突：install_service_trace包装Node.create_client后，原指纹拒绝监听。CPU与真实Node的RED保留，改为精确核诊断wrapper及它的原方法，而非用cache0回避。复审再发现inspect.getsource(function)自动unwrap，可让functools.wraps包住的不同代码误认；[wrapped RED](../../../runs/ros2_web_transport_20261030/service-client-design/compat-wraps-red.log)后改核实际__code__的源码、缺失拒绝，WAIT/native同类校验也收紧，原哈希不放宽。known/unknown/wrapped及17个最终CPU入口通过；[复审](../../../runs/ros2_web_transport_20261030/service-client-design/review-i61.md)解除两个Important，未发现新Critical/Important，仍不裁决未完成验收。

真实METADATA首轮夹具把full与server-only模式传同一rosapi进程，按原互斥约束被拒，失败保留；修正为bridge full客户端、独立数学server采用server-only，不放宽rosapi限制。最终[cache64诊断](../../../runs/ros2_web_transport_20261030/service-client-design/wait-current64/guard-result.json)成功：四标准phase各两条、native成功CID/sequence已take、故意timeout时Future未done/未cancel、Event未set，实际responseCaller=managed-cache。另一轮[cache0迟到reader](../../../runs/ros2_web_transport_20261030/service-client-design/wait-current0-late/guard-result.json)成功、实际caller=upstream、2秒有限保留暴露迟到native take；两轮remainingPids空。诊断默认关闭，不把该0.1秒超时夹具当正式3秒预算验收。

CLI缺口补上--service-client-cache-size与replay的独立可选bridge_service_client_cache_size；未指定时不遮蔽web64默认，late-reader未指定时显式0、指定非0提前拒绝。实际CLI边界RED→GREEN，25个队列回归、192种诊断环境组合通过。旧队列fixture人工context漏新空值导致一次失败，补其声明环境后通过；诊断模式probe第一次沙箱禁止socket，宿主执行仅端口检查且mock进程边界后通过，不伪称启动了ROS。真实回放的该参数透传仍待测试。

以最后构建复测的[web-current64](../../../runs/ros2_web_transport_20261030/service-client-design/web-current64/resources.json)15.74秒/子树峰723.48MiB，SDK17项含参数/Lifecycle/Action反馈成功拒绝取消中止/权限拒绝/并发预算/断开清理。五组实际数据验证URDF主题与HTTP文件一致、mesh可取、关节、TF静动态查询、CBOR云、PNG字节/16UC1深度、Marker/Path/数值流；九类参数均写入/读回/恢复。640×480与1280×720 RGB，以及4,136,928字节云的JSON/Base64→ROS→none/CBOR返回逐字节SHA一致；客户端订阅/pending/listeners归零。未创建WebGL renderer或真实浏览器，不能当页面FPS、GPU、真实相机或Peach业务验收。默认/原始对照的旧版本短测不能代替当前冻结长测。

[本轮汇总](../../../runs/ros2_web_transport_20261030/service-client-design/task2-current-verification.json)核安装lib核心源码与当前字节一致、I60池去docstring后AST一致；Task2仍未完成：实际3秒wire与恢复、排队创建/无限响应SIGINT、生产benchmark收尾策略、真实回放cache透传仍待；之后才冻结0.7.0并执行默认q64/shared/noXML/原3秒2秒RPC/1800秒/1GiB门。I54/I36、浏览器交互轨迹损失与冷绘制、历史OOM脚本、Android/LAN均未关闭。当前package元数据仍基于0.6.22开发树，旧归档不覆写，不能将新增功能倒写成冻结0.6.22已经验证。 [宿主收尾](../../../runs/ros2_web_transport_20261030/service-client-design/task2-host-cleanup.json)复核104个PID创建身份、独占安装/夹具命令、测试端口，无存活项，临时单元0；诊断CLI单独提供remainingPids，未冒称它保存了全部历史创建身份。


### I62：正式3秒协议、关闭边界及上游发布者计时器

本轮继续I61未完成的Task2。所有数据只来自自建无硬件服务、demo与已录制Peach观测，不启动相机、真机、SetIO、dashboard或Gazebo。包清单仍为0.6.22开发树，未冻结0.7.0；旧归档与失败轮不覆盖。当前两个运行文件的前后完整源码/patch/普通安装匹配记录见[源审](../../../runs/ros2_web_transport_20261030/managed-lifecycle/source-review-i62.json)、[变更](../../../runs/ros2_web_transport_20261030/managed-lifecycle/runtime-change.patch)。Service池/受管Node/WAIT/native/协议本轮没有改算法。

真实独立Trigger server与原生WebSocket验证[deadline3-v3](../../../runs/ros2_web_transport_20261030/managed-lifecycle/deadline3-v3/wire.json)：标准call_service timeout=3在3002.784ms收到同CID的result=false和原超时文字，随后fast正常；远端slow继续至4秒完成，明确不是取消服务。观测8秒只用于等待证据，不扩大3秒协议门。首轮services_glob错传ROS数组导致启动失败，第二轮wire已通过但夹具误把已正常退出的wire当核心死亡，均保留FAIL；最终守卫及bridge退出0、remaining空。[无限等待关闭](../../../runs/ros2_web_transport_20261030/managed-lifecycle/infinite-response/result.json)在远端30秒服务已开始、timeout=0时发桥SIGINT，0.14155秒exit0、socket关闭/pending0，远端并未被自动取消。独立server的Event由自身收尾释放。

[排队创建关闭](../../../runs/ros2_web_transport_20261030/managed-lifecycle/queued-close/node-destroyed.json)用实际executor任务暂占owner，观察真实票据排队，created=0/Node.clients=0；仅在原executor.shutdown已置_is_shutdown后释放测试阻塞，原桥main/join不变。SIGINT至exit0约0.14198秒，调用已结束、pool.closed/active0/pendingCreations0、没有创建原生Client，server未收到fast。关闭统计仍诚实保留unconsumedCreationTickets=1，不能将它描述成任务已消费或原生实体泄漏。该夹具继承的通用remoteServiceNotCancelled字段不作为排队用例证据，实际结论是没有发出请求。hook/源码输入快照是测试观察器，不加入产品。

I61的“正常退出”复审发现漏判：web-current64与web-cache0-root-stop日志均在桥输出Exiting之后、5秒时升级SIGTERM并exit-15，尽管最终无残留。两轮graceful结论撤回，功能结果保留。第一次线程诊断prefix缺sys.path只是夹具启动失败；补齐后prefix/sitecustomize观察轮正常退出，未复现不当修复。最小PYTHONFAULTHANDLER观察的[原生栈](../../../runs/ros2_web_transport_20261030/managed-lifecycle/shutdown-fatal-stack/launch.log)复现主线程threading._shutdown等待两个Timer；为取栈在4秒发送SIGABRT，禁core dump，该轮明确FAIL而非新系统崩溃，所有已观测PID已清。

核本机[上游2.7.1发布者实现](https://raw.githubusercontent.com/RobotWebTools/rosbridge_suite/2.7.1/rosbridge_library/src/rosbridge_library/internal/publishers.py)、[Python3.12线程/Timer](https://docs.python.org/3.12/library/threading.html#threading.Timer)和[Jazzy launch信号转发](https://raw.githubusercontent.com/ros2/launch/jazzy/launch/launch/actions/execute_local.py)。真实“advertise/publish/subscribe后断开、不主动unadvertise”的[单因子复现](../../../runs/ros2_web_transport_20261030/managed-lifecycle/publisher-timer-cause/result.json)确认：ROS main已返回，PublisherManager._unregister_impl的10秒非守护计时器仍活，约9.931秒后进程自行exit0。Timer继承创建线程daemon状态，因此显式unadvertise与断开收尾的调度差异会改变是否阻止解释器结束；不是executor join或Service Future挂死的证据。

最小修复使用公有launch参数：只把bridge Node的sigterm_timeout设15秒，保留上游10秒保留语义/主循环，不取消其私有Timer或改daemon。性能CLI只给自己拥有的launch根PID初次SIGINT一次，stdin=DEVNULL确保noninteractive转发；等待18秒覆盖该15秒宽限，leaf仍组INT8秒，所有组TERM5/KILL3兜底保留，server=None使预检中断也可安全进入finally。没有改变RPC3秒、QoS、q64或默认RMW。范围只覆盖上游默认unregister_timeout=10；手工提高此上游参数必须同步配置更长退出预算，不能外推当前15秒支持任意值。

停止role探针先捕获缺关键字RED，最终8组核18/8/5/3秒实值、root/leaf、两级超时、初信号进程已消失与poll已退出；[实际finally AST4组](../../../runs/ros2_web_transport_20261030/managed-lifecycle/cleanup-race-results.json)覆盖组消失、PermissionError、kill前子消失、资源写失败，保留错误/继续组收尾/handler恢复。均是CPU证据，不代替原生清理。新普通安装[真实launch发布者退出](../../../runs/ros2_web_transport_20261030/managed-lifecycle/publisher-launch-grace-v2/result.json)9.937秒exit0、bridge clean0、日志无escalation；首轮产品行为相同但夹具结尾误用旧状态常量exit1，原始FAIL保留。新版[完整demo](../../../runs/ros2_web_transport_20261030/managed-lifecycle/web-grace64/resources.json)9.80秒/子树峰718.64MiB，SDK17项、5组实际流、9类参数写读恢复及两图/4.14MB云SHA通过；严格检查退出日志而非仅root状态。新普通构建37项官方lint零失败，XML原件在本轮目录；纯Python项目的compile_commands未使用警告保留。

三轮真实录制回放分别[显式64](../../../runs/ros2_web_transport_20261030/managed-lifecycle/replay-cache64-v2/result.json)、[显式0](../../../runs/ros2_web_transport_20261030/managed-lifecycle/replay-cache0/result.json)、[省略参数](../../../runs/ros2_web_transport_20261030/managed-lifecycle/replay-cache-default/result.json)，通过标准ros2 param get --no-daemon读取实际值64/0/64，实际队列均64；不能只凭argv断言透传。每轮JSON与CBOR各约10秒、完整颜色/深度/4.14MB云/URDF/关节及所需TF均有效，3秒RPC零失败，bridge clean0/无escalation/端口释放。外守卫子树峰分别726.62/676.14/714.01MiB；短测的不同编码帧数/p95不是缓存优化倍率或长期无泄漏证明。首个显式64回放功能通过，但参数CLI自动启动daemon被外守卫判FAIL并回收；改官方--no-daemon后重跑，未降低残留门。性能CLI按文档从源码scripts运行，ROS launch/静态资源来自新普通安装；share/source是审阅副本，不声称其中的相对dist探针可独立执行。

同一当前CLI SHA a13ce015…1765f6的[真实信号6组](../../../runs/ros2_web_transport_20261030/managed-lifecycle/signal-matrix.json)覆盖normal/INT/TERM/repeat/early/preflight，全部自身收尾、无孤儿/外层kill；中断组守卫故意非零并写明信号，不写成性能成功。此矩阵是q16/UI录制夹具，不冒充默认q64容量测试。每轮独占systemd限时/内存cgroup，守卫和内核memory.peak字段分别保存，不采用systemd-run摘要的几百KiB为整树峰值。

[宿主最终收尾](../../../runs/ros2_web_transport_20261030/managed-lifecycle/host-cleanup-i62.json)核167个PID创建身份、本轮安装/夹具命令、8个端口与临时单元，全部无残留。[完整汇总](../../../runs/ros2_web_transport_20261030/managed-lifecycle/verification-i62.json)、[独立只读复审](../../../runs/ros2_web_transport_20261030/managed-lifecycle/review-i62.md)无新Critical/Important，Task2限定范围完成，状态见[checkpoint](../../../runs/ros2_web_transport_20261030/service-client-design/checkpoint.json)。Task3仍须冻结0.7.0、离线重建、公有入口与默认q64/shared/noXML/3秒/2秒周期/1800秒/1GiB门。浏览器冷绘制、有损交互轨迹、I54/I36、历史OOM脚本身份和Android/LAN均继续开放，不由本轮通信/关闭短测替代。


### I63：0.7.0源码冻结与默认持续流（本机限定范围通过）

Task2限定补验/复审完成后，三个版本清单同步0.7.0；前端依赖和API没有升级，保持I62池/布局门/退出修复。当前工作区构建与归档包含完整可修改自有源码、类型、map和许可，原0.6.22归档不覆盖。冻结范围是源码，不自动判性能通过；进行中的默认1800秒以同一会话终态为准，不重启/延长预算，也不在该4连接负载上加浏览器。

[初次冻结](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/verification.json)明确FAIL：离线ci/check/build/audit/preview/observe/wire均通过，但我误把需要source ROS与运行demo的fault探针放进无ROS组，ros2cli元数据导入失败；不是产品通信反例或fault通过。[续验](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/verification-resumed.json)再次FAIL：独立build目录colcon test漏--base-paths，显示0包/0测试；验收要求非零而拒绝。仅修正验证流程，未改冻结归档/产品源，原日志保留。第三步显式source路径执行真正官方lint；colcon汇总37项=33个xunit用例加4组CTest，不能重复累加为独立功能覆盖。初始报告把xunit33字段标为汇总的版本亦保留，当前分别报告两计数。

[最终离线证明](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/verification-final.json)PASS_SOURCE_FREEZE_NOT_PERFORMANCE：归档SHA256 2756ab41c06df7f627ab35fac647fdb98cd2534117f47328608fc6f27a141c0d，165文件、99个运行源码/配置、54个dist离线逐字节一致，4公开入口导入、普通Jazzy构建、37项官方lint零失败。不硬编码旧源码数，不把npm pack省略的根lock当缺失：从dist/build-package-lock.json恢复与原字节相同的canonical lock，再npm ci --offline；归档所有文件与提取/重建后逐项SHA相同。最终验证新增说明/目录属于后续证据，不冒称被写入较早冻结归档。

从该归档的新普通安装执行[原生回归](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/native/resources.json)，独占DDS域227/1GiB/150秒：fault真实临时桥丢失令Service/Action等待结束，订阅重连恢复、离线发布拒绝、15轮rapid connect/disconnect通过；该探针明确模拟自己的桥丢失，不把强制故障动作称graceful。相同冻结源码SDK17项、9类参数写读恢复、实际URDF/mesh/关节/TF/点云/PNG/深度/Marker/Path、两图与4.14MB云精确SHA通过。总12.85秒，子树峰725.68MiB，守卫正常终态/remaining空，主launch无escalation。第一次被错放的fault失败不覆盖，本轮才是有效原生证据。

[默认长测输入](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/long-inputs.json)核归档/99运行/54dist/普通安装全文件，清RMW/XML/诊断环境，使用新的空工作目录避免隐式默认XML。只播放已录制Peach RGB-D/完整云/URDF/TF/关节；不启动相机/机器人/Gazebo。参数不传队列、缓存、分离或诊断覆盖，3个共享SDK加慢读者，2秒RPC周期/3秒截止/1800秒，原inner sumRSS1GiB/free4GiB/Node768MiB守卫保留；外层独占systemd MemoryMax1GiB/Swap0/1980秒/KillMode control-group。

[实际配置](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/running-configuration.json)只读核原生PID/已加载库与launch临时YAML，实际FastDDS/cache64/q64、未开启诊断；不发额外ROS请求或新增观察节点。进度只读采样保存于[long-progress.jsonl](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/long-progress.jsonl)，出现过进程树RSS约0.6–0.85GiB与内核max事件，采样时无OOM。memory.stat同时含匿名页、文件缓存及内核等；不能把cgroup memory.peak或systemd摘要当同一sumRSS，也不能仅“没OOM”宣称内存余量充分。[内核cgroup v2文档](https://www.kernel.org/doc/html/latest/admin-guide/cgroup-v2.html)说明memory.max可能短暂超过上限，max计数不等于OOM；本轮不重设/增加限额。所有结果仍须最终核对，尤其原日志的Write queue full无连接CID且受限频，警告数不是丢帧数或具体慢读者归属。

独立浏览器后续[夹具准备](../../../runs/ros2_web_transport_20261030/native-browser-v070/plan.json)尚未运行。旧I58三段轨迹均dataLossOccurred=true，显式4MiB recordUntilFull约97%录满；按[CDP Tracing协议](https://raw.githubusercontent.com/ChromeDevTools/devtools-protocol/master/json/browser_protocol.json)调整为有界32MiB/文件128MiB，保留原类别与采样，不把录制质量变化当产品速度改善。startup定义改为同一文档内首次观测启动，避免导航混淆标记；归档内Playwright1.64.1与实际Chrome/GPU/视口需重新记录。仅在默认核心负载终态后，以独立域/后端运行真实10项只读验收、模型/颜色深度/点云/曲线、拖动/缩放/层/尺寸、停止重启断开；每段无丢失/完整首尾标记才可解读。准备源码不是执行或流畅度PASS。


I63终态补记：[原始控制器](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/long-controller-terminal.json)exit0，同一会话30分3.638秒，无重启/延时；[完整后处理](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/long-verification.json)全部门通过。2688RPC零失败，五代成功p95约5.91–8.15ms；450/1080秒两次重建、30次clock回退，慢读者12253帧/16220975371B、40次暂停/40次恢复，未提前关闭；每代流有效/获得完整TF、SDK资源归零。TF在时钟回退后可短暂缺失并恢复，不把最终完整写成全程零缺失。6连接/6断开、桥clean0无escalation。内层server+Node sumRSS峰907.02MiB、Node自采峰167.03MiB、最小可用内存约14376MiB；采样cgroup peak1074896896B（约1025.10MiB），末次max141020/OOM0计数采于约1786秒，另有终态成功证据，不能冒称拿到了已销毁cgroup的最后计数。196条限频队列警告没有CID，不是196帧丢失或特定连接归因。

第一版后处理错误地要求半秒RSS最后采样>=1800，1799.95被误判；[原FAIL](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/long-verification-v1-halfsecond-sampling-error.json)保留。未放宽3秒/流门或重跑，用不可变Node monotonic循环完成分支+slot0实际1800.126秒证明，采样只作下界。99运行/54dist/210安装全SHA及归档不变。[宿主收尾](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/long-host-cleanup.json)20个PID创建身份/6端口/临时单元全清；[独立复审](../../../runs/ros2_web_transport_20261030/source-rebuild-v070/review-i63.md)未发现Critical/Important。该结果关闭本版本/本负载的默认长测缺口，不改写I54的旧FAIL、不宣称找到了普适DDS机制或I36别配置断流根因。后续浏览器/布局与使用说明另列I64。


### I64：独立Chrome真实项目交互、无损轨迹与LAN布局

I63默认核心负载终态/清理后才启动独立浏览器后端，继续只用真实Peach录制。用户已授权独立Chromium＋Playwright对照；本轮实际Chrome152.0.7977.82/Playwright1.64.1/RTX3090 ANGLE OpenGL，旧I58为1.62.1，不将版本/缓存差异当成产品优化倍数。三段轨迹按CDP协议32MiB/每文件128MiB，保留类别与采样；startup定义为同文档导航/验收后首次观测启动，记录成对同PID标记与dataLossOccurred。初次[capture](../../../runs/ros2_web_transport_20261030/native-browser-v070/capture/resources.json)在开页前失败：guard沿用DISPLAY=:0，本机实际X1；缺失显示不是网页错误。默认沙箱本地socket探测也EPERM；合法宿主只读预检与X1校正保存在display-correction.json，不输出Xauthority cookie。

[capture-v2分析](../../../runs/ros2_web_transport_20261030/native-browser-v070/capture-v2/analysis.json)18门通过；三段16615/约2.5万/3.6万事件均无已知丢失，原始文件大小/SHA与首尾标记见traces.json。真实10项Peach只读PASS、12/12资产、21TF、6活动/15总关节、约8.6万显示点/33Marker/800Path、两种640×480图和曲线有效。实际鼠标拖动、滚轮、图层开关、视口调整、深度切换、停止/重启/断开；最终订阅/pending/listeners/canvas0，无页面/渲染/shader错误。构造186.6ms/renderer178ms，稳定CPU提交p95约1.3ms；冷/重启仍有213–231ms长任务，不记零卡顿。浏览器树峰1897.98MiB、独立后端463.07MiB；两者不能替代核心1GiB门。后端本轮按300秒上限停止，后写stop.request，不称浏览器退出立即停止。[宿主清理](../../../runs/ros2_web_transport_20261030/native-browser-v070/capture-v2/host-cleanup.json)26个创建身份/两端口/单元均清。

原生CPU profile在部分30–40msmessage事件范围出现V8 profiler interrupt开销，并有ProfileChunk负timeDelta；仅按有效nodeID采样次数分析，绝不将这些delta当CPU时间权重或据此重写图像/云解码。另做[无profiler对照](../../../runs/ros2_web_transport_20261030/native-browser-v070/unprofiled/analysis.json)，不调用Tracing/CPUProfiler，保留公开LongTask/LoAF，17门通过：首构造83ms/renderer76.3ms，稳定提交p95约1.6ms/交互1.5ms；这两个阶段未报告>=50ms LongTask。重启构造218.2ms/LongTask228ms，989×900尺寸切换55–60ms，不能称GPU帧时或全部交互无延迟。OS/GPU缓存未清，时间不同，不能算精确profiler开销比值。旧result.traces=3字段只是阶段计数，明确nativeTraceCount=0。窄屏截图沿用滚动位置，不冒充聚焦3D；下一轮另补。浏览器峰1742.14MiB、后端457.09MiB；driver finally主动写stop.request，两个原执行handle均终态0、18个PID/端口/单元清理。

[LAN/触摸模拟](../../../runs/ros2_web_transport_20261030/native-browser-v070/lan/result.json)使用主机自身WiFiIP192.168.2.90、proxy关闭、989×900/hasTouch，真实HTTP/WS默认host自动跟随，两桥端口21588/21590绑定0.0.0.0，10项只读/12资产/21TF/深度/3D/断开通过。聚焦scene-989.png已人工视觉核对，无横向溢出，3D662.625×400可见；23个PID/端口/单元全清。3GiB/120秒功能门下组合树峰2068.94MiB，总17.309秒，不作容量测。它没有远程设备网络/防火墙/物理Android证据；用户最新明确“平板暂时不用测试”，本轮据此暂缓且保留未验收，不再询问/阻断本轮。独立Chrome不关闭内置浏览器历史秒级GL，也不定位旧OOM的Python脚本或I36别配置的彩图停顿机制。

### I65：标准预览共享预算、完整网页表单与0.7.1复核

全面控件审查发现标准数据observe-start直接创建Blob，绕过ImageView已有压缩字节/PNG-JPEG头尺寸门。这是实际缺口，不依据profiler噪声猜测解码算法。先用生产handler＋真实media导出的[CPU复现](../../../runs/ros2_web_transport_20261030/native-browser-v070/standard_preview_probe.mjs)：有效RED确认5000×5000小头与compressedDepth到达Blob、旧回调可在stop后复活；首个RED夹具漏callback info，原日志保留，不当产品反例。随后只导出compressedImagePayload薄入口，复用原校验并传image.maxPixels；web等待可选media加载时用独立observeRequest序号检查stop/close/连接变化，替换/停止回收URL。ImageView、raw decodeImage、TF/点云/曲线和后端Service池算法未改。profile小像素预算另先RED，最终7组CPU通过，preview14组/页面连接12组、完整observe、check/build通过。[只读复审](../../../runs/ros2_web_transport_20261030/native-browser-v070/review-preview.md)无Critical/Important，要求真实浏览器补证后才判功能完成。

新增无硬件真实rclpy小图片publisher，只发送33字节超限PNG头、compressedDepth标记或4×3合法JPEG，不分配25M像素图片，其他消息由原demo提供。独立Chrome通过实际DOM点击。首轮控制器错误假设merged install，hash预检就失败，尚无ROS/浏览器；[demo-forms/fixture-error.json](../../../runs/ros2_web_transport_20261030/native-browser-v070/demo-forms/fixture-error.json)保留。v2真实前8组通过，图谱预览被测试误当完整数组而TypeError；生产有界预览正确，修改测试对预览结构和公有SDK完整图谱分层验证，没有改产品或绕过预算。该轮FAIL与所有日志/清理保留。

[demo-forms-v3](../../../runs/ros2_web_transport_20261030/native-browser-v070/demo-forms-v3/result.json)10组PASS：实际延迟media脚本加载中stop后无订阅；PNG96×48显示→超限头/特殊depth拒绝→JPEG4×3显示→PNG恢复、stop移除src；页面完整demo按钮12项PASS；Topic表单原样回显；Service加法/非法JSON/100ms超时/本地停止等待；Action反馈/成功4/拒绝0/中止6/取消5；九类参数实际表单写读/非法gain拒绝/原子恢复；Lifecycle configure/activate/deactivate/cleanup；图谱刷新/接口识别与字段；标准/3D/ImageView/Plot并用，停标准保留独立图像、最终订阅/pending/listeners/canvas0。DOM参数/图谱预览有界，因此完整TypedArray读回用公有SDK额外验证，范围在JSON写明，不冒称DOM显示全部。无pageerror，服务端/图像publisher/browser返回0，3GiB/200秒功能门树峰1955.84MiB，总13.613秒，不能当1GiB长期门。[宿主清理](../../../runs/ros2_web_transport_20261030/native-browser-v070/forms-host-cleanup.json)54个精确创建身份/两端口/单元全清；首个systemctl观察缺用户DBus环境，修正只读观察环境后才出清理PASS。

三个版本清单同步0.7.1，普通开发安装已完成；0.7.0归档/长测输入未改，不能直接冒称覆盖新网页。接下来在source-rebuild-v071目录冻结完整源码、离线恢复lock/重建、四入口、普通安装与非零官方lint，再按原默认q64/cache64/shared/FastDDS/noXML/无诊断/3秒RPC/2秒周期/1800秒/1GiB门使用同一执行handle验证。当前总目标继续，平板实机按用户指示暂缓，Peach/IVG不迁移。


I65冻结与正式终态补记：[0.7.1离线冻结](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/verification-final.json)归档SHA6032705e8e88f3e7ecfcc77edc5e3ea38625322949f529692b2a0c28ef210a91，165文件/99运行源码配置/54dist、4公开入口、普通安装、37项官方检查（33xunit+4CTest）通过。第一次wire因默认沙箱禁止回环listen报EPERM，保留verification.json FAIL；仅将需要本地socket的探针及后续构建移到允许宿主回环的同一归档续验，不改源码/归档，也不把它写作协议错误。原生[回归](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/native/resources.json)22.85秒/树峰722.89MiB、fault/sdk17/streams/roundtrip全0，无残留、无launch升级。

[浏览器运行时绑定](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/browser-runtime-binding.json)核forms-v3所服务普通安装与冻结98个已安装运行文件（CMakeLists不安装）/54dist一致；[源差异绑定](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/runtime-diff-binding.json)确认SDK、Scene、所有Python后端、私有压缩解析正文及decodeImage/ImageView正文与0.7.0逐字一致。只增加标准预览入口/取消边界、相应页面夹具及版本元数据，不把较早浏览器证据悄悄改版。

同一[handle3222正式终态](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/long-controller-terminal.json)exit0/30分3.513秒；[后处理](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/long-verification.json)全部门通过。默认q64/cache64/shared/FastDDS/noXML/无诊断，2秒RPC周期/3秒截止，完整1800秒（冻结Node单调循环+slot0墙时1800.11秒；RSS采样仅下界），2691RPC零失败/五代p95约6.39–7.68ms；450/1080秒两次客户端重建、30clock回退、40慢读暂停/40恢复、13198帧/17449649782B，未提前关闭。每代流/TF有效、收尾资源0，6连接/6断开、bridge clean0无升级。仍允许clock回退后的短暂TF缺失/恢复，不称全程零缺帧。

内层树sumRSS峰910.54MiB、Node自采168.16MiB、最小可用内存18274.82MiB。采样cgroup peak1074974720B（约1025.18MiB）、末次max142485/OOM0是存活期间观察，不冒称销毁后的终态计数；最后一次observer在root自然退出后得到NoSuchProcess，单独记录observer-final-note.json，不发信号、不计产品失败。227条限频write queue警告无CID，不是丢帧数或慢读连接归因。[内存趋势](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/long-memory-trend.json)30–90秒树中位621.56MiB、末60秒806.31MiB；主要bridge与player各增加约87MiB，Node约18MiB。正增长不能据此诊断泄漏，也不能称数小时容量或内存余量宽裕。99运行/54dist/210安装SHA和归档均不变。[宿主收尾](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/long-host-cleanup.json)20个创建身份/5端口/两个单元全部清理。历史I54 FAIL与I36另一配置机制、旧OOM脚本身份、IAB环境根因保留；Android按用户本轮指示暂不测。后续仅开发浏览器探针/文档收尾另列I66，生产运行文件保持本轮SHA。


### I66：随包浏览器CLI适配项目页、控件补测与最终交付

完整长测已自然终态且清理后，审查发现随包browser_probe仍直接点击self-check，页面自项目化后按钮位于默认关闭的details#demo-tools。[原CLI真实RED](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/browser-cli-red/browser.log)在当前冻结网页复现：已连接后等待隐藏元素30秒TimeoutError；不是ROS通信失败。独立3GiB/330秒单元自动清理，原源脚本/输入SHA与日志保留。只修开发探针click助手：需要self-check时先通过真实summary点击展开，包含导航/窄屏/LAN场景；不强制点击隐藏按钮，不改生产页面或已验后端。

补齐当前项目控件/完整报告导出/停止按钮与日志清空断言。两轮新增夹具错误如实保留：第一轮误把manual项目当0用例，实际配置有manual.graph一项；第二轮期待清空后瞬间文案，但共享command logger按设计又写clear-log自身OK。产品行为无需修改，按真实契约改为清空选择后禁用运行，以及导出仅含一条clear-log记录。[最终CLI](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/browser-cli-green-v3/cli/summary.json)21/21通过，工具实际终态0/30.461秒，树sumRSS峰2033.42MiB；不是1GiB通信或Android实机门。两次FAIL的前序通过项不能冒称整轮成功，原始outputs与as-executed探针副本均保留。

21组包含离线/连接、demo12项、项目manual/demo切换与清空、6只读/2NOT_RUN完整下载、填入服务表单、单选项目Action并停止、CANCELLED1/NOT_RUN7且cancelConfirmed=true/terminal5的完整下载、图谱、Topic回显与释放、手动订阅后再次demo回归、Service成功/JSON错误/停止等待、Action取消、参数写读拒绝/原子、生命周期四转换、标准图像/TF/点云/诊断、Action类型、五图层/缺帧恢复/复位视角、深度单位/错误字段/曲线与停止、断开重连、操作下载/清空、768/390布局、非安全上下文LAN别名及无pageerror。截图scene.png已视觉核对模型、云、Marker、网格；图谱/参数完整值与有界预览的区别保留。[宿主收尾](../../../runs/ros2_web_transport_20261030/source-rebuild-v071/browser-host-cleanup.json)核77个创建身份/两端口/四单元，全部无存活项。

本轮之后产品源码不变，仅browser_probe.mjs与README/doc更新。最终本地tgz/普通安装在source-delivery-v071独立生成，原source-rebuild-v071归档SHA和正式长测不覆盖。交付绑定将分别核98个未变运行源码/配置与54dist，以及唯一开发探针差异；不把新探针回写成旧冻结包当时可运行，也不为文档/开发工具重复声称新的默认长测。全部输出关闭后再刷新catalog与SHA，归档内generated索引是打包时快照，工作区新索引包含最后证明；两者不做哈希自引用。用户平板暂不测、Peach/IVG不迁移；IAB/OOM/I36未知机制和明确容量边界不删除。

I66文档收尾：独立复审仅指出本文件页首还停留I59的0.6.22快照，现改为上述0.7.1当前证据入口。生产源码/依赖/54dist不变；final子目录中的文档终版归档与前一交付归档只允许doc/development.md差异，原归档909239d3…ba8564保留，完整离线重建/公有入口/37项检查继续由相同代码输入证明，不为纯文档重复长测或行为测试。最终索引在输出关闭后更新，工作区普通安装同步最新包级文档。


## I67 — 0.8.0 通用工作台、关系图与显示寿命（2026-11-02）

用户确认首轮完整实现节点—Topic发布/订阅，服务与Action另列；Peach/IVG只作需求样本、不迁移。先核Jazzy接口/rviz_common/rqt_graph/rosapi源码，新增Cytoscape3.34.4（MIT）并锁定；既有SDK与Python桥无本轮改动。graph明确有界allSettled发现和node_details事实，主方法graph()保持兼容；独立graph/workbench入口与原四入口共存。workbench公开配置/注册/控制/视图，8类内置显示、同类多实例、Topic/类型/QoS编辑、保存与安全导入，共用一个空间上下文。

Task1图谱11组与真实Cytoscape headless探针、Task2共享Scene、Task3配置/控制5组均先RED后GREEN。唯一fresh最终只读review发现2 Important/4 Minor；按完整图/跨桥归属/旧API兼容/资产证据的用户影响将4项升为Important，同一轮六个实测复现均RED→GREEN，无deferred minor。修复子Marker退休队列、枚举转换漏洞、投影截断提示、旧会话标识、start前setLayer订阅兼容和每显示资产统计。assetStats使用结构化可选API，避免两个独立bundle的instanceof身份差异。

原生UI06八组、SDK02十七项、经典页01二十一项通过，37项官方lint通过；预览14/连接12/audit20+路由9及其余CPU回归分别记录。CPU连接fixture初次缺workbenchView变量与stopClassicSubscriptions提取，按实际DOM边界补齐后通过，不是运行网页异常。headless无GL、旧DISPLAY=:1失败保留，当前Xorg/桌面确认为:0。SDK01失败来自runner仅等待HTTP，改为HTTP+WS+demo发现就绪后SDK02通过；产品超时未改变。

Peach01瞬时非零断言、Peach02误设1500ms窗口均失败；官方rosbag2_py离线recv_timestamp实测首云3.319419981s、循环间隙3.338299974s（内部最长2.522s），首Marker约4.298s。Peach03按冻结偏移+1秒窗口4320ms完成同样120秒，4云/8空间/11显示、RGB-D/TF/曲线和2次clock回退。树sumRSS峰2175.23MiB，12/12模型资产，6活动关节，输入不变、无自有残留/监听。额外同客户端回调只计数/stamp，不增网络reader；保留3秒RPC/10秒推进/默认队列，无旧版1800秒或IAB/FPS/Android外推。

本轮plan/ledger/checkpoint及每轮原始结果在runs/ros2_web_workbench_20261102。分发归档和离线核对在artifacts/ros2_web_link，脱离runs索引避免archive包含catalog时的哈希自引用；新索引工具增加graph/workbench源码清单，文件增长单独记录catalog-growth.json，旧历史工具/旧归档不覆盖。未启动真机、相机或Gazebo，IVG SDK FK/IK对比入口保留。

I67离线核对：候选源码归档203文件，恢复锁后npm ci --offline成功，六入口可导入，74个dist文件（JS/声明/map/网页）逐字节一致；observe/graph/workbench/preview/audit/wire与config、普通colcon build/test均通过，37项官方lint零错误/失败/跳过。与0.7.1归档对照，核心SDK四文件及五个桥/HTTP后端源文件SHA完全相同。最终只更新文档/索引后另打包，并以runtime及dist逐文件绑定，原候选包保留；不会用新归档SHA替换原始测试输入。


## I68 — 0.8.1 可视化界面与工具导航（2026-11-02）

用户要求清晰明确，并参考 RViz2 与 rqt 界面。阅读 Jazzy RViz 用户指南、DisplaysPanel 源码、rqt 概念文档及 rqt_graph UI 后，沿用现有 TypeScript/DOM/Cytoscape，未增加依赖。全站主导航改为单个可见功能面板，项目/连接集中在右上角；工作台分为空间、节点关系、接口三个页签，支持键盘切换。显示列表采用紧凑行，选中项展开 Topic 与状态，异常文字直接显示；属性高级项折叠，彩色图和深度图使用不同默认名称，画布扩大且左右面板独立滚动。暂停、断线与旧快照信息可见；切页不发指令、不重建显示或改变订阅寿命。

截图核对发现 Topic 搜索原来会通过匹配 Topic 的发布者扩展无关 Topic，导致图谱过密。真实 Cytoscape CPU 复现先失败，现收窄为该 Topic 与实际发布/订阅节点；节点搜索保留其 Topic 和对端，不展开对端其他 Topic。节点投影也按命中边筛选。右侧实体详情显示 ROS 名称、类型和发布/订阅信息，完整内部 ID 保留在导出；无匹配有明确空态。

最终原生 native-ui-02 工作台8组、native-layout-04 布局/键盘10组、native-classic-02 经典调试21项通过，无pageerror，三轮输入哈希保持不变。官方lint37项通过；TypeScript、graph/workbench/preview及连接探针通过。后代sumRSS峰依次1970.57/1952.43/1865.07MiB，非GPU内存或性能提升证明。宿主按151个自有PID创建身份核对无存活项，22588/22590/8088/9090无监听。Android物理设备未测试。

native-layout-01/02分别因开发探针未等待懒加载和hashchange而失败，改为等待实际可见控件，保留原断言与失败证据。main-build.log记录证据目录before/package.xml被colcon发现为重复包；在本轮证据根目录加入COLCON_IGNORE后重建，不改应用包发现逻辑。0.8.0归档中的SDK、scene、media、audit、launch和五个桥/HTTP源文件合计22个SHA逐字节不变；未启动真机、相机或Gazebo，未迁移Peach/IVG。

原始日志/截图/清理与汇总在[runs/ros2_web_ui_20261102/summary.json](../../../runs/ros2_web_ui_20261102/summary.json)。0.8.0与0.7.1的冻结归档及长期证据不覆盖。本轮归档和安装绑定另存于工作区artifacts/ros2_web_link/ui-v081，不将视觉验收扩展为新的长期通信性能验收。

## I69：真实录制预览、大数据交互和关系图修复（0.8.2，2026-11-02）

用户要求打开回放检查实际可视化、交互和性能。先查宿主机进程/端口/内存，只用已完成的59.471871442秒真实相机+mock记录；ROS域219、22588/22590独立测试。复用replay.launch和既有自有进程守卫，不启硬件、相机或Gazebo。先核W3C/Chrome原生性能API和本地Cytoscape3.34.4源码，见source.md。探针只保存有界数字/短脚本摘要，无附加RAF、业务写操作或payload缓存。

0.8.1 baseline-02、0.8.2中间final-01、最终final-02均完成120秒、2次回退、4同源云/8空间/11总显示；最终样本344828点、RSS2270.32MiB、JSheap115.60MiB，事件报告p95=64ms。并非无卡顿：最终稳定期有77ms长任务，恢复3D有292ms主线程任务/312ms离散事件。没有用本轮短测改写历史IAB/OOM或1800秒结论，详见performance.md。

截图暴露全图文字过小；原生滚轮复现ResizeObserver无条件fit使页签返回丢失视角。改为首次可见时fit，后续只resize；增加既有Cytoscape grid、缩放/1:1/比例提示，长ROS路径anywhere换行和标签高度；网格列数按可见宽度选择，未新增布局库。Peach最终网格约94%比例；过滤仍保留原节点/Topic语义。SDK、Python桥、scene/media/audit和ROS配置未变。

失败与范围：baseline-01的summary严格定位、red2的首轮滚轮限幅前置条件、离线fixture缺新增控件均保留；graph-viewport-red未验证滚轮实际发生，因此不算有效产品证据。red3才复现实际缺陷，green通过，最终回放再验。最终10组布局回归、graph/workbench/check、37项官方lint通过。所有原生轮次均检查PID创建身份、端口和输入哈希；失败轮次也已清理。源码/运行证据、逐阶段原生指标和图像在runs/ros2_web_replay_ui_20261102，使用说明为replay-preview.md。
