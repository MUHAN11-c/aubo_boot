# 可组合显示工作台与 ROS 关系图

0.8.3模型材质遵循RViz2：URDF visual显式声明材质时覆盖DAE/GLTF/OBJ自带材质；未声明时保留嵌入材质。显式alpha（含半透明）与具名材质仍生效，只对RobotModel应用，不改变Marker。替换时释放不再使用的旧材质/纹理，保留共享资源与几何；可用`npm run observe`中的材质资源检查及真实URDF浏览器用例核对。

工作台 0.8.2 使用工具面板布局：左侧导航一次显示一个功能页；右上角“连接设置”集中选择项目、设置 WebSocket 和连接/断开。工作台内的“空间与数据”“节点关系”“接口信息”三个页签分别用于观测、关系排查和 IDL 查询。Peach 真实录制数据的启动与交互检查见[回放预览](replay-preview.md)。

本包面向独立功能包使用者提供 `graph` 和 `workbench` 公有入口。它们消费同一个 `RosClient`，不依赖 Peach / IVG 业务节点。通信使用 ROS 2 Topic、Service、Action、参数和 Lifecycle 的原生契约，标准类型由内置显示适配器处理，自定义消息可以先用原始消息显示，再编写本地适配器。

“通用”表示不把业务名称或任务流程写进通信层。浏览器不能装载 RViz 的 C++ / Qt / Ogre 插件，也没有 RViz 全部显示类型；本包没有重新实现 DDS、TF 积分器、参数服务器或 ROS 生命周期管理器。当前发行版验收目标仍是 ROS 2 Jazzy，其他发行版须单独核对桥和消息接口。

## 启动与第一次使用

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
colcon build --packages-select ros2_web_link
source install/setup.bash
ros2 launch ros2_web_link self_test.launch.py
```

构建前须已在包内执行 `npm ci`，依赖安装不是 launch 的副作用。浏览器打开 `http://127.0.0.1:8088/#workbench`；示例没有硬件依赖。局域网访问仍使用既有 `web.launch.py` 的地址与端口参数，不需要平板安装 ROS。用完在启动终端 Ctrl+C，按 [testing.md](testing.md) 核对自有进程和端口退出。

1. 点击“打开工作台”。此时只加载组件，不连接、不订阅、不创建空间 WebGL 画布。
2. 点击右上角“连接设置”，选择项目后点击“连接”，再关闭连接设置面板。模板中的显示默认关闭，勾选所需内容，再点击“启动所选显示”。
3. 点击显示名称，编辑输入 Topic 和 `package/msg/Type`。QoS、CBOR / JSON 与订阅间隔位于“传输设置”折叠项；其余选项位于“显示选项 · JSON”，填写后点击“应用显示属性”。机器人模型有 description 与 joints 两个独立输入。
4. 展开“＋ 添加显示”，选择显示类型，可手填 Topic。刷新关系图后，Topic 输入框提供实际发现的名称；单一已知类型可以自动填入。同类显示可重复添加。
5. 取消勾选释放该显示订阅与数据；删除同时移除配置。暂停工作台释放所有显示资源，但保留用户的启用选择。再次启动是新观测会话，不沿用上次模型数据或 TF 缓存。

经典“3D 观测 / 图像与曲线 / 标准数据 / Topic 订阅”与工作台观测会话互相暂停；项目验收运行前也会停止工作台显示。这样避免同一客户端同一 Topic 使用不同订阅选项。关系图只读查询可独立使用；页面退出会销毁图形与监听器。

## 界面怎样读

- **显示列表**：勾选控制该显示的启用配置；点击名称选中并编辑。行内的“已关闭 / 待启动 / 运行中”表示本地显示状态，不等于数据到达或 TF 正确。选中行显示 Topic 与最新状态；包含缺少 TF、失败等文字的状态直接展开。彩色图和深度图使用不同默认名称。
- **属性面板**：模型描述、关节状态和消息输入分组；高级传输与 JSON 默认折叠。长列表和属性在左侧面板内滚动，右侧主视图保留自己的滚动位置。
- **空间与数据**：共享 3D 画布位于上方，图像、曲线和消息摘要在其下方。鼠标左键拖动旋转、右键平移、滚轮缩放；“重置视角”恢复初始相机。下方状态行显示固定坐标系相关的数据状态；精确资源计数可展开查看。
- **节点关系**：蓝色矩形表示节点，绿色椭圆表示 Topic，箭头表示声明方向；右侧列出所选实体的 ROS 名称、类型、发布/订阅节点或 Topic。图谱全部内容保留在显式 JSON 导出中。
- **接口信息**：服务与提供节点、Action 服务端分列；读取类型与定义是 rosapi 只读查询。旧会话提示同时出现在关系图与接口页，旧列表不会被用来发起定义查询。

主导航与工作台页签只改变可见面板，不会取消任务、重复指令或重建显示；离开视图时仍需通过“暂停显示”释放观测资源。工作台页签支持左右方向键与 Home/End；连接设置内按 Escape 可关闭并返回按钮。窄屏自动改为上下排列，顶部导航横向滚动，未做 Android 物理设备验收。

布局参考 Jazzy [RViz 用户指南](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.rst)中的显示列表、属性与状态，以及 [rqt 概念文档](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-RQt.rst)的工具分区；按钮与图谱筛选对照 [RViz DisplaysPanel](https://github.com/ros2/rviz/blob/jazzy/rviz_common/src/rviz_common/displays_panel.cpp) 和 [rqt_graph UI](https://github.com/ros-visualization/rqt_graph/blob/ros2/resource/RosGraph.ui)。这是浏览器布局适配，不提供 Qt 任意拖拽停靠或加载 RViz 原生插件。

## 显示类型与输入

- `robot`：`std_msgs/msg/String` URDF 与 `sensor_msgs/msg/JointState`。默认描述使用 latched，关节使用 sensor。模型资源继续经过 HTTP `/assets/` 包白名单；网页配置不能放开服务器资源权限。
- `tf`：使用工作区全局 TF、静态 TF、固定坐标系和时钟。动态 TF 为 reliable / volatile，静态 TF 为 reliable / transient_local / depth 100；不按每路显示新建变换树。
- `markers`：`visualization_msgs/msg/MarkerArray`，复用既有类型 0..11、ID / namespace、寿命、删除与 frame_locked 逻辑。单个 `Marker`、RViz 全部材质选项等可先用 raw 检查，未宣称完整支持。
- `cloud`：`sensor_msgs/msg/PointCloud2`，默认每路最多显示 100000 点；解析、字段、颜色与输入字节预算复用既有解码器。采样显示不是原始点云精确重建。
- `path`：`nav_msgs/msg/Path` 的位置折线，使用共同 header；不执行 MoveIt 规划或轨迹命令。
- `image`：`sensor_msgs/msg/Image` 或 `CompressedImage`；默认最多 4000000 像素、16000000 输入字节。PNG / JPEG、RGB / BGR / mono8 / 16UC1 / 32FC1 的边界与旧 ImageView 一致。16UC1 必须填写 `depthScale`（米 / 原始单位），32FC1 按米；compressedDepth 特殊封装不在支持范围内。
- `plot`：任意消息中的 1..8 个数值字段，例如 `position.0`。最多保留 600 点，100ms 合并绘制；不执行表达式。header 时间倒退清样本，没有 header 时标记浏览器接收时间。
- `raw`：任意已安装 ROS 消息类型的有界摘要。二进制数组显示类型与字节数，长数组、长文本、深层结构会截断；不是保存全量图像 / 点云的工具。

所有空间显示共用一个 renderer、相机、轨道控制器、TF 与时钟，显示自身拥有模型、Marker、点云和 Path 资源。总计最多 16 路显示、同时启用最多 8 路空间显示，所有已启用点云的 `maxPoints` 之和不得超过 400000。禁用的显示仍计入 16 路配置上限，但不占空间 / 点云运行预算。

同一客户端同一 Topic 必须使用相同 type / QoS / compression / throttle。添加同 Topic 显示会优先复制已有兼容输入的精确选项；属性编辑产生冲突时会拒绝配置，不创建第二个假想 DDS reader。QoS 是订阅请求，不能仅凭请求值宣称与发布者已匹配。

全局设置中的“应用全局设置并暂停”会更换固定坐标系、TF Topic、时钟或布局，并停止当前会话。勾选 `/clock` 后等待 ROS 时间初始化；回放暂停时不外推墙时，时间回退清动态数据。TF 缺失会隐藏对应对象并报告原因，不能把“收到了点数”当作“已经正确定位”。

## 保存、导入与错误处理

下载和本机保存使用 `schemaVersion: 1` JSON，保存显示名称、输入、选项、布局、固定坐标系与时钟。它不保存连接 URL、命令、Goal、授权状态或订阅缓存。本机保存使用当前浏览器 localStorage；其他应用可通过 `storageKey` 自行区分项目。

导入限制 1 MiB；会检查版本、插件 ID、显示 ID、端口、消息类型、QoS、结构深度和总预算。未知插件不会从网络下载 JavaScript，非法配置保留原有工作区。有效导入 / 恢复后也保持暂停，需要用户重新启动显示；不会自动连接、调用业务服务或发送 Action。

运行中修改显示时，未变更的显示保持运行。变更实例启动失败时，控制器尝试恢复上一有效实例；恢复也失败时暂停整个观测会话。错误通过状态与资源计数报告，不能把配置校验通过当作数据已到达。

## 关系图怎样读

进入“节点关系”页签，点击“刷新关系图”调用标准 `/rosapi/nodes`、`topics`、`services`、`action_servers` 和逐节点 `node_details`，按实际 `publishing` / `subscribing` 数组建立：

```text
发布节点 → Topic → 订阅节点
```

节点和 Topic 使用不同种类 ID，同名不会合并。搜索支持名称和消息类型：匹配 Topic 时只保留该 Topic 的发布者/订阅者；匹配节点时保留其相关 Topic 与对端节点，不展开对端的其他 Topic。无匹配时显示空态提示。可隐藏内部 Topic，切换节点投影模式，选择实体并聚焦直接邻居。方向来自 ROS 声明，不通过字符串名称猜测。循环图使用 Cytoscape 的既有布局，不假设 ROS 图是 DAG。

默认手动刷新。每轮整体截止 10 秒，单次 RPC 不超过 3 秒，并发 4；最多 256 节点、512 Topic、2048 边。超限、节点查询失败、取消或断线会保留可用部分并明确标出原因。手动刷新不自动重试；可显式选择每 5 秒刷新，轮次不重叠，页面隐藏时停止本轮采集。快照带采集时间，不表示持续实时链路。

0.8.2 增加紧凑网格和明确的缩放控件。层级图保留原默认；大图文字较小时，可切换紧凑网格、搜索 Topic 或用 1:1 字号查看局部。比例显示实时跟随鼠标滚轮。页签切换保留视角；“适应画布”、筛选、布局变化和新快照会重新定位。网格改变位置，不改变发布/订阅边或快照内容。

在“接口信息”页签，服务与已证实的提供节点、Action 服务端名称分别列出。读取类型与定义只查询 rosapi 中本机已安装的 IDL，不调用业务服务、不发送 Goal。定义显示是有界预览，较大嵌套 IDL 应结合包源码核对。

本轮不能提供：服务 / Action 的完整客户端边、每个端点的 GID / QoS / 类型关联、隐藏实体的完备清单、类型握手和实际消息传输成功证明。节点投影达到边上限会在图旁标明视图不完整。断开或换桥后保留的快照标记为上一会话并显示来源，重新采集前不使用旧端点列表查询定义。重复节点全名会标记歧义；同 Topic 多类型保留类型集合，不能根据名称认定连接有效。“完整快照”只针对 rosapi 可见范围和本次成功查询。

## 在其他功能包中复用

核心 SDK 不导入图形库。按需选择六个入口：包根 SDK、`/scene`、`/media`、`/audit`、`/graph`、`/workbench`。安装本地 npm 归档即可使用，不需要复制整套自测网页；对应源码、类型声明和 sourcemap 一起分发。

```ts
import {WorkbenchView} from '@local/ros2-web-link/workbench';

// client 和 observer 由应用显式连接；也可以使用同一个 RosClient。
const view = new WorkbenchView(container, {
  getControlClient: () => client,
  getObservationClient: () => observer,
  profile: existingViewProfile,
  storageKey: 'my_robot.workspace.v1',
  beforeResume: () => stopOtherObservationConsumers(),
});
// 断线时调用 syncConnection；卸载应用组件时必须调用 dispose。
view.syncConnection();
// view.dispose();
```

组件使用包内 `web/style.css` 的 `.wb-*` 布局样式，应用也可以按这些 class 自行适配；3D 容器需有非零宽高。`WorkbenchController` 可单独消费 `DisplayRegistry`，由应用提供各 2D 根元素与共享 3D 根元素，从而复用自己的 UI。

扩展适配器通过本地 `registry.register(definition)`：声明 `id/title/spatial/ports`、选项 `validate` 与实例 `create`。实例实现 `start/dispose`，可提供 `stats`；通过 `context.client.subscribe` 订阅，保留 unsubscribe 并在 `dispose` 清掉。空间适配器使用 `context.scene({layers: [...]})` 获取逻辑子显示；不能另建 WebGLRenderer 或拥有公共 TF 寿命。JSON 只引用已注册 ID，插件源码由应用安装与审查。

当前 WorkbenchView 使用默认注册表；需要额外适配器时，优先使用公开 controller / registry 构建应用视图，或在本地修改 View 注册位置。C++ pluginlib / RViz 插件不能直接作为浏览器工厂注册。

## 本轮核对与来源

当前实现与核对步骤在 [实施计划](universal-workbench-plan.md)，逐轮原始证据在工作区 `runs/ros2_web_workbench_20261102/`。CPU 探针使用 `npm run graph`、`npm run workbench`、`npm run observe`；真实浏览器入口为 `scripts/workbench_browser_probe.mjs`，必须使用有界原生 runner，结束查 `resources.json`。本轮功能核对已完成，完整步骤与失败更正见 [testing.md](testing.md) 的 0.8.0 记录；六入口、离线重建、wire与官方lint已验证；分发验证独立记录在工作区 artifacts/ros2_web_link/，不能把旧 0.7.1 的长测当作新工作台通过。

设计核对来自 [RViz Jazzy 插件文档](https://raw.githubusercontent.com/ros2/rviz/jazzy/docs/plugin_development.md)、本机 `rviz_common/display.hpp` / `display_group.hpp` / `ros_topic_display.hpp`、[rqt_graph ROS 2 源码](https://github.com/ros-visualization/rqt_graph/blob/ros2/src/rqt_graph/rosgraph2_impl.py)、本机 rosapi 2.7.1 的 `proxy.py` / `NodeDetails.srv`、[Cytoscape 官方 API](https://js.cytoscape.org/)。依赖版本、API 限制与源码哈希见本轮 `source-review.json`。本包继续保留所有第三方许可证；未把网页操作当作硬件急停或运动授权。

## 项目模板中的订阅选项

0.8.3从项目视图生成模板时，保留机器人描述、关节、Marker、点云、Path、图像和曲线的订阅选项。`scene`中对应字段为`robotDescriptionOptions`、`jointStatesOptions`、`markersOptions`、`pointCloudOptions`、`pathOptions`；图像和曲线使用`subscription`。支持相同的`qos`、`compression`、`throttleMs`。转换后所有显示仍为关闭，启动前可在属性面板核对。

例如BEST_EFFORT Marker发布者应设置`markersOptions: {qos: sensor, compression: cbor, throttleMs: 250}`；模板转换后仍保留该配置，不会恢复成RELIABLE。`npm run workbench`包括URDF/Marker/Path/曲线的转换检查，无需ROS或浏览器。
