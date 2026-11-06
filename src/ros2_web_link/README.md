# ros2_web_link

DDS配置选择补充：检测到当前目录的`DEFAULT_FASTRTPS_PROFILES.xml`时，不注入本包默认XML，由上游处理已有隐式/显式配置；此时需要部署者自行加入所需恢复周期。已有显式环境配置同样优先。

0.8.3 修复无缓存Service的共享准入/创建等待限制和工作台订阅选项丢失，保留缓存默认0，并为本包启动的服务端与录制回放进程分别提供 Fast DDS 100ms 可靠恢复配置。使用标准 XML、保留现有自定义配置优先级，适用于 ARM64 部署准备；Orin NX 16GB 本机浏览器尚未实机验收。修复前后的同门限结果与使用说明见[性能参数文档](doc/jetson-orin-nx-performance.md)。以下0.8.2结果为历史基线。

2026-11-02 完成0.8.2桌面分档压力测试，面向用户指定的 **Orin NX 16GB、本机浏览器** 出具[性能参数与部署文档](doc/jetson-orin-nx-performance.md)。缓存开启的持续/并发/高频查询档通过；cache0两档出现查询超时，两次浏览器长测均在约535秒出现点云循环恢复超期。ARM64默认缓存的原生指纹尚未验证，目标机未连接，**不能标记为NX生产验收通过**。各次结果、CPU/RSS/PSS与待测项分别列明；本轮不改运行源码或发行归档。

0.8.2（2026-11-02）补齐录制回放交互检查，修复关系图切换页签重置缩放/平移，增加缩放按钮、实际比例和紧凑网格。真实数据启动、显示选择与检查范围见[回放预览说明](doc/replay-preview.md)。

0.8.1（2026-11-02）优化网页可视化：参考 RViz2 / rqt 的工具布局，统一全站导航与连接入口，工作台分为空间、关系图和接口三个页签。显示列表、属性与状态更清晰；Topic 搜索只保留实际相关发布/订阅节点。操作见[工作台文档](doc/workbench.md)，本轮核对见[测试指导](doc/testing.md#081-界面核对)。

0.8.0历史核对（2026-11-02）：可组合显示与节点—Topic关系图已实现。新版工作台8组、SDK17项、经典页面21项、Peach录制120秒/8路空间显示/2次时钟回退，以及37项官方lint已核对；六公有入口、离线重建及wire探针已核对；操作与复用见[工作台文档](doc/workbench.md)。下文0.7.1长测保留其原有版本与范围，不能替代新版显示验收。

独立的 ROS 2 Jazzy Web 通信包，包含可复用的浏览器 SDK、可选 3D/图像/曲线组件、上游 rosbridge/rosapi 的组合启动，以及一个可选项目和交互用例的自测网页。自有代码 Apache-2.0；所有运行资源本地提供，无 CDN、云服务、WebSocket 代理或前端框架依赖。

0.7.1历史已完成默认队列64/服务缓存64/共享连接/FastDDS/无XML、无诊断的1800秒Peach完整录制负载：2691次RPC零失败，3个健康客户端与慢读者、两次客户端重建、30次时钟回退、40次暂停/恢复均覆盖。源码离线重建、四公有入口、普通ROS安装与37项官方检查通过；真实ROS＋Chrome表单10组、页面demo按钮12项、SDK17项以及修正后的随包浏览器CLI21项均有独立证据。标准预览已补像素/字节预算及迟到停止边界。桌面真实3D/图像/曲线交互、无丢失CDP轨迹、无CPU profiler对照及LAN地址/触摸模拟另行验证。Android实机按用户本轮要求暂不测试；历史内置浏览器秒级GL阻塞和OOM脚本身份仍未定位，不作所有设备/网络/无限时长性能保证，详见[性能验收](doc/performance.md)。

0.8.3默认cache0，使用有界逐调用Service客户端。所有连接共享线程创建前的Service准入预算（默认128），创建等待默认5秒；无缓存模式也生效。显式非零缓存才复用客户端，其有效槽位为min(缓存配置,在途限额)。缓存只支持已审本机rclpy7.1.12 builtin布局/二进制；未知rclpy布局须选0，rosbridge版本仍固定2.7.1。SDK与cache0不因此支持任意ROS发行版。正式3秒协议、排队/无限等待关闭、参数恢复与4.14MB精确往返已核对。发布者默认10秒计时器保留，Ctrl+C退出可能需约10秒。实现、失败、更正和复跑细节见[开发记录I60–I66](doc/development.md)与[测试指导](doc/testing.md)；旧归档及历史失败不覆盖。

Peach、IVG 是需求样本，本包不依赖它们，也不启动机器人、相机或 dashboard。SDK 不含它们的任务、权限或业务映射；IVG 原有 SDK FK/IK 只读对比路径不受影响。

## 项目选择与交互

自测页可选择Peach在线项目、Peach录制数据、无硬件demo或通用接口。勾选测试项后运行，逐项查看证据、耗时、失败原因和未执行范围。Peach在线配置模板包含17项只读与3项默认关闭的交互，面向真实相机+mock臂，迁移接入时仍需核对当下接口；回放配置10项只读，关闭相机后仍可验证实际RGB-D、TF与点云。均不依赖demo节点，不迁移业务。启动与实际操作见 [项目自测使用说明](doc/project-testing.md)。

完整集成步骤见 [通用集成说明](doc/integration.md)，大型数据与复测命令见 [性能验收](doc/performance.md)。

完整开发与问题记录见[开发、参考、重构和验证记录](doc/development.md)，逐个源码/原始运行文件见[证据索引](doc/evidence-index.md)。失败、撤回方案、修复范围和未执行项均保留；索引中的历史绿灯不等于当前版本全部验收通过。

当前接口覆盖Topic、Service、Action、九类参数、图谱/类型发现与原生Lifecycle；六个独立入口为SDK、scene、media、audit、graph、workbench。请求按单调截止时间结束，共享本地等待上限默认256；断线只恢复SDK订阅，不重发命令；工作台断线会停止并释放自己的显示，需要显式恢复。Topic与图像面板保留项目的预设/显式QoS；Peach录制图像使用本机已验证的Reliable配置，通用sensor预设保持不变。

“可视化工作台”提供可添加、重复、编辑和启停的显示项，以及节点—Topic发布/订阅关系。空间显示共一个画布，图像/曲线/原始消息各自挂载；可下载、导入和本机保存工作区，导入后保持暂停。操作步骤、插件扩展、预算与关系图能力边界见[工作台使用与复用](doc/workbench.md)。

历史0.6.18将显示时钟改为显式选择：SceneOptions.useSimTime默认false，回放项目显式true；独立TfBuffer的clockTopic默认false，使用回放须显式指定/clock。Marker寿命不再混用墙时和ROS时间，暂停不推进、到期及纳秒回退有实际浏览器验证。该轮Peach完整录制页面10项只读通过、12/12模型资源、21 TF与RGB-D/曲线显示及停止重启通过；仍记录到约2秒绘制阻塞，不能宣称冷启动或全部性能问题已解决。

本机回放与渲染的结果、原始失败、RMW差异和未完成项见[性能验收](doc/performance.md)。原共享连接FastDDS多轮长测失败；0.6.15显式双连接在队列16、100ms实验XML和诊断包装下完成30分钟/2688RPC零失败，关闭诊断的原生命周期复测随后在777.54秒因彩图停顿超过10秒失败，1158RPC零失败不能替代全流验收。0.6.17将可选双连接接入项目引擎和网页，默认仍共用连接，demo功能与取消/清理已验证。0.6.16去掉一次整帧RGBA克隆，真实录制两图及完整云的DDS全字节/独立显示参照、实际浏览器画布均通过；不据此宣称FPS/RSS改善。这组历史结果不能代表当前验收状态；现有桌面与默认持续流证据见页首，Android本轮暂不测；历史整窗退出已有同应用组OOM证据，Python脚本身份未知。

## 构建与启动

环境：Ubuntu 24.04、ROS 2 Jazzy、Python 3.12；Node.js 22+ 用于构建和显式CLI验收。

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
# 若系统尚未安装这些包：
# sudo apt install ros-jazzy-rosbridge-suite ros-jazzy-example-interfaces
cd src/ros2_web_link
npm ci
npm run check
# 后续 colcon build 将自动重新构建网页
cd ../..
colcon build --packages-select ros2_web_link
source install/setup.bash
ros2 launch ros2_web_link self_test.launch.py
```

打开 **http://127.0.0.1:8088**，默认项目为“ROS 2 · 无硬件通信示例”。点击“连接”后，“运行所选用例”执行默认6项只读检查；2项交互需明确勾选。若要检查完整通信基础，展开“独立 demo 通信回归”，点击“运行功能自测”。在“3D 观测”点击“开始 3D 观测”查看两个活动关节的模型、TF、Marker、彩色点云与 Path；在“图像与曲线”选择彩色/深度配置后显示图像，点击“开始曲线”查看数值流。页面默认连接同一主机的 9090 端口。命令必须在页面或 CLI 中显式触发，launch 不发目标。

平板与电脑连接同一局域网：

```bash
ros2 launch ros2_web_link self_test.launch.py address:=0.0.0.0
```

平板浏览器打开 `http://电脑局域网IP:8088`；WebSocket 主机跟随网页主机，无需在平板安装 ROS。HTTP/WS 面向受信局域网；不配置公网入口、账号或证书。退出使用 Ctrl+C，按 [测试指导](doc/testing.md) 核对自建进程全部退出。

## 可复用 SDK

构建生成 `dist/sdk/index.js`（ES module）、`.d.ts` 类型声明，以及 `dist/web/ros2-web-link.js`（独立浏览器脚本，含 roslib）。业务页可通过 script 标签使用全局 `Ros2WebLink`，或在打包工程中导入 ESM；ESM 依赖 `roslib@2.1.0`。

```html
<script src="/vendor/ros2-web-link.js"></script>
<script>
const client = new Ros2WebLink.RosClient({url: 'ws://127.0.0.1:9090'});
client.onState(state => console.log(state));
async function start() {
  let stop;
  try {
    await client.connect();
    stop = client.subscribe('/web_link_demo/sensor', 'std_msgs/msg/Float64',
      (message, info) => console.log(message.data, info.receivedAt), {qos: 'sensor'});
    const response = await client.callService('/web_link_demo/add_two_ints',
      'example_interfaces/srv/AddTwoInts', {a: 3, b: 4}, {timeoutMs: 5000});
    console.log(response.sum);
    const handle = client.sendGoal('/web_link_demo/fibonacci',
      'example_interfaces/action/Fibonacci', {order: 8},
      {onFeedback: console.log, timeoutMs: 60000});
    console.log(await handle.result); // 数值 status: 4 成功、5 取消、6 中止
  } finally {
    stop?.();
    client.dispose(); // 本地释放不代表取消远端任务。
  }
}
start().catch(console.error);
</script>
```

公有 API 与生命周期约束详见 [接口](doc/io.md)。业务页绑定 topic/service/action 名称；SDK 不生成第二套业务接口。标准参数通过 `client.parameters('/节点')` 使用 `list/describe/get/set/setAtomically`，不使用 ROS 1 参数服务器假设。v0.5提供`transportLimits`和`transportStats()`，限制分片与socket发送积压并隔离用户回调异常；回调仍应轻量处理。网页可查看传输预算/缓存/错误；`npm run wire`独立运行传输故障核对，不连接机器人。

## 用于其他功能包

1. 引入构建产物或源码模块；页面框架不限，SDK 无 DOM 依赖。
2. 应用 launch 组合 `web.launch.py`（默认 demo=false/profile=robot），按应用接口明确配置 `topics_pub_glob/services_glob/actions_glob`；已有静态服务器可设置 `web:=false`。
3. 用应用的标准消息、服务和 Action 替换示例名称，执行权限继续由 ROS 节点自身的命令门负责。
4. 页面卸载时取消订阅、调用 `dispose()`；跨页后重新读取任务状态，不假设旧请求已取消。

默认桥仅允许向 `/web_link_demo/*` 发布及发送 Action，业务服务仅允许 `/web_link_demo/*`。上游会额外开放 `/rosapi/*` 以支持图谱查询，本包通过 rosapi 节点的 params_glob=[] 禁止其旧参数入口访问参数；标准参数读写走节点自身的 rcl_interfaces 服务。订阅读取默认不限。上游 glob 是访问范围限制，不是机器人运动授权；已有 ROS 执行使能门仍需存在。

功能与依赖版本核对表见 [capabilities.md](doc/capabilities.md)。

## 检验、文档与许可

- 构建与官方 lint：`colcon test --packages-select ros2_web_link`，然后 `colcon test-result --verbose`。
- 显式行为探针：启动自测环境后，在包目录 `npm run probe`；浏览器探针与故障步骤见 [testing.md](doc/testing.md)。`test/` 不增加自定义测试框架。
- 架构与限制：[architecture.md](doc/architecture.md)；公有接口：[io.md](doc/io.md)。文档是包内 Markdown，直接阅读，无额外文档生成工具。
- 六个入口源码位于 `sdk/`、`scene/`、`media/`、`audit/`、`graph/`、`workbench/`；独立页面在 `web/`，ROS/构建/显式探针在 `scripts/`、`launch/`，演示模型在 `description/`。构建包含类型声明与 source map；公共注释说明断线、QoS、超时和取消语义。
- 依赖固定在 `package-lock.json`，ROS 依赖走 Jazzy apt。上游完整源码位于本地 `.upstream/`，获取与替换见 [source.md](doc/source.md)；许可见 [LICENSE](LICENSE) 与构建生成的 `dist/web/THIRD_PARTY_NOTICES.txt`。自有代码 Apache-2.0；roslibjs BSD-2-Clause，Three/gl-matrix/uPlot MIT，URDFLoader Apache-2.0，TF 复用的 Lichtblick 文件 MPL-2.0（逐文件保留）；rosbridge_suite 保留上游 BSD 许可。

当前提供通信 SDK、独立工作台、按需观测组件和标准自测示例。Peach 已复用核心 SDK/桥，接线见 [集成说明](doc/integration.md#peach-当前接线)；采摘/IVG 业务迁移、完整 Lichtblick/Foxglove 界面、浏览器 MCAP 管理和 MoveIt 关节轨迹回放不在本包职责内。Android 实机触控与高带宽相机仍需现场验收。

## 复用观测组件

ESM 分入口为 `dist/sdk/index.js`、`dist/scene/index.js`、`dist/media/index.js`，或源码工程使用包 exports 的根、`/scene`、`/media`。普通 script 页面在核心 bundle 后加载 `ros2-web-scene.js` / `ros2-web-media.js`，分别获得 `Ros2WebScene` / `Ros2WebMedia`。

```js
// 假设 client 已连接；配置字段见 /config.json 和 doc/io.md。
const scene = new Ros2WebScene.SceneView(document.querySelector('#viewport'), client, {
  fixedFrame: 'demo_base', robotDescription: '/web_link_demo/robot_description',
  jointStates: '/web_link_demo/joint_states', tf: '/web_link_demo/tf',
  tfStatic: '/web_link_demo/tf_static', markers: '/web_link_demo/markers',
  pointCloud: '/web_link_demo/points', path: '/web_link_demo/path',
  onStatus: console.log,
});
scene.start();
// 页面卸载时 scene.dispose()；同理 ImageView / SeriesPlot。
```

只启动通信与网页：`ros2 launch ros2_web_link web.launch.py`。一键示例仍使用 `self_test.launch.py`。自定义 profile、端口、资产白名单及单一 launch 接入见 [launch.md](doc/launch.md)。

最终本地源码归档及重建记录位于`runs/ros2_web_transport_20261030/source-delivery-v071/`，完整1800秒验收使用`source-rebuild-v071/`不可变归档。两者生产代码与54个dist一致，后者之后仅修开发浏览器探针和文档，差异在交付绑定记录明确列出；不把旧CLI失败改写成当时已通过。归档文档/索引是打包时快照，工作区doc/evidence-index.md在输出关闭后刷新；完整runs不装进npm运行包，交付测试证据应保留其相对目录结构。
