# 通用 ROS 2 工作台设计（2026-11-02 历史设计）

本文件保留 0.8.0 实现前确认的设计及约束；其中“下一版”“本轮”指当时上下文。
graph/workbench 已有当前源码，现行能力、接口和验证范围以
[architecture.md](architecture.md)、[io.md](io.md)、[workbench.md](workbench.md)
为准，本文不作为新的待执行计划。原设计希望独立功能包适配不同 ROS 2 框架，
按 RViz2 的显示实例思路选择内容、按 rqt_graph 的声明关系查看计算图；
当时 Peach/IVG 仅作需求样本，不恢复 Gazebo、不连接硬件、不做平板实机测试。

## 目标与兼容含义

按ROS接口和消息类型接入，而不是为MoveIt、Nav2、ros2_control、Peach或IVG分别写通信客户端。Topic、Service、Action、参数、Lifecycle、名称空间、自定义接口、TF与时间语义仍使用ROS 2机制，Web侧复用现有RosClient。业务命令门继续由应用负责。

“可适配各类ROS 2框架”不等于“任意消息自动变成三维图形”或“直接加载RViz原生插件”。桥端须安装实际IDL及其依赖；权限、QoS、资源路径、版本兼容与JS整数表示边界必须可见。陌生消息先提供类型/字段和有界原始值查看，再由Web显示适配器或ROS侧标准消息转换补充语义。实际支持发行版、桥能力和显示类型分别声明，不将Jazzy本机结果写成所有发行版通过。

RViz的Display是C++/Qt/Ogre/pluginlib插件；浏览器复用其职责、属性和生命周期设计，提供本地可修改的TS/JS显示适配器，不执行从ROS消息下载的代码，不使用eval。

## 当前源码的具体缺口

- `sdk/index.ts`的`RosClient.graph()`查询节点、Topic、Service、Action名称列表，未返回节点之间的发布/订阅边。
- `scene/index.ts`的`SceneOptions`固定一组模型、关节、MarkerArray、云、Path与TF来源；`start()`会订阅整组，`setLayer()`只改变显示开关。不能将当前隐藏图层称为已取消订阅。
- 当前网页可手填话题或SceneOptions JSON，但不能按实时类型搜索并添加多份独立显示，亦无通用工作区保存/恢复。
- `/opt/ros/jazzy/share/rosapi_msgs/srv/NodeDetails.srv`仅有node请求以及subscribing/publishing/services名称数组；不能从该契约捏造Service客户端、Action客户端或完整端点QoS。

## 建议的职责划分

保留现有四个公有入口及调用语义。新增按需加载的图谱和工作台入口，核心SDK不导入DOM/Three或图布局库。其余Web工程可分别复用通信、图谱数据、图谱视图或显示管理，不必使用自测整页。

1. **通信与能力发现**：RosClient及现有rosbridge/rosapi。图谱快照使用现有只读服务；返回实际能力、错误和采样时刻，不以缺失响应生成空的“完整系统”。
2. **显示管理**：管理带稳定id的显示配置、启停、属性、状态与清理。仅保存数据配置，不再造ROS参数、TF树或任务框架。
3. **显示适配器**：复用当前图像/点云解码、URDFLoader、Lichtblick TF、Marker和曲线实现；按输入消息类型匹配。应用可在本地构建时注册自己的适配器。
4. **工作台网页**：同一UI内提供显示列表、属性区、共享3D画布、2D图像/曲线/原始值和关系图。已有项目验收与调试表单保留为工具区，项目预设只负责预填，不限制通用接入。

## 像RViz一样选择显示内容

“添加显示”提供按显示类型、按当前Topic两种入口；按Topic入口显示消息类型，推荐匹配适配器，允许手动选择以应对暂未发现的发布者。每项可重命名、修改来源、选择QoS/编码/限频、启用、禁用和删除。允许同类多实例，例如两路点云、多个Path、多个机器人模型或图像。

首批适配器覆盖现有真实能力：RobotModel+JointState、TF、MarkerArray、PointCloud2、Path、Image/CompressedImage、数值曲线，以及任意已安装IDL的有界消息字段查看。LaserScan、OccupancyGrid、Pose/Odometry等注册扩展入口预留，不在这份设计中冒称已经实现。模型和TF是多输入适配器，不能把所有显示强行简化成单一Topic。

所有空间显示共用一个Three渲染器、相机、固定坐标系和TF/时钟上下文。不得每加一份点云或模型就新建WebGL上下文或独立TF树。不同数据源拥有独立资源与状态；一份坏消息或模型加载失败不使其他显示消失。

禁用/删除必须释放该实例持有的订阅、缓冲和图形资源；共用订阅按实际消费者寿命保留，最后一项释放后停止。TF/clock公共资源同样按使用者持有。异步模型加载、旧回调和旧连接不得在禁用后复活。重新启用等待新数据并明确等待/缺TF/类型不匹配/预算错误，不显示为当前成功。

同一SDK会话的同Topic不同类型或不兼容订阅选项不能偷偷复用。现有桥共享reader的限制要显式检查并报冲突，不宣称给每个显示独立QoS选择就获得独立DDS reader。无论多少显示，都保留既有字节、像素、点数、TF历史、曲线样本等边界，并增加整体显示数量/内存预算；具体容量以新版本实测为准。

## 工作区配置

引入独立、带schemaVersion的工作区JSON，包含连接配置、全局fixedFrame/TF/clock配置、显示项和布局。显示项描述id、名称、适配器id、enabled、多输入Topic/type/QoS/编码/限频和类型特有属性。旧profile/project视图通过兼容转换生成初始工作区；不修改ROS部署参数来保存浏览器布局。

支持下载/导入和浏览器本地保存。导入先检查版本、字段、数量、重复id、适配器可用性与预算，失败不破坏当前工作区。导入不执行Service、Action、参数写入或IO，不重放历史命令；连接与观测恢复由用户显式操作。不同ROS域以各自桥连接为边界，不假装浏览器能直接跨所有DDS域发现。

## 像rqt_graph一样显示关系

建议首轮完整实现`发布节点 → Topic → 订阅节点`，支持节点—Topic视图与按Topic汇总的节点关系视图，提供名称/名称空间过滤、隐藏内部话题、聚焦邻居、缩放/平移/适配视口，以及点选后查看名称/已知类型/来源与采样时间。节点和Topic使用不同种类的id，不能因同名而合并；重复节点名、类型信息不足要标记歧义。

关系来自实际ROS发现，禁止根据包名、Topic前缀或项目模板猜连线。这是发现关系图，不是实际流量、时延或QoS握手成功证明。数据未提供端点类型/QoS时显示“未知/未核对”，不能画成已验证连接。

图谱默认手动刷新，可选择低频自动刷新；同一时刻只有一轮快照，查询有整体截止时间和有限并发。暂停/隐藏后停止后续刷新，断开时清会话并标记旧快照。节点在采样中退出或部分查询被拒绝时保留成功部分并明确partial/errors，不能把旧连线当新快照。图规模超过预算须显示不完整范围，不静默截断后声称全图。

**用户2026-11-02已确认范围**：首轮完整实现Pub/Sub关系，Service/Action提供现有名称、类型、接口详情及能证实的服务端信息，单独列出。Service/Action客户端—服务端关系不在首轮；该能力超出现有NodeDetails契约，后续需要额外只读图谱提供能力及兼容协商，不能仅改前端就声称完成。图布局使用成熟开源库，具体库及锁定版本由实现计划结合体积、维护和本地源码复核决定；不手写布局算法。

## 集成与演进约束

- 应用通过适配器注册和标准ROS名称/接口接入。Web不import Peach/IVG业务模块，不加入新的执行权威。
- 原SceneView作为兼容入口保留现有用法；新显示管理逐步复用其渲染能力，旧图层切换语义不能静默改成新的订阅协议。
- 0.7.1归档及其1800秒证据保持原样；0.8候选改动须独立构建/验收，不能继承“旧长测即新版通过”。
- 大数据按实例与总体有界，诊断默认关闭，测试仍使用无硬件demo或已录制数据；测完清所有自有进程与端口。

## 验收清单

1. 在非Peach命名空间的示例、自定义嵌套IDL中发现、查看字段、调用原有接口；自定义显示适配器不改核心SDK即可注册。
2. 同画布添加两份同类空间数据，分别配置/启停/删除；仅一个WebGL上下文。其他显示继续更新，订阅和资源最终归零，禁用期间不继续消耗该输入大流。
3. 图像/云/模型异步加载、缺TF、时间回退、QoS冲突、未知类型、断线及恢复均有实际状态；非法配置导入回滚有效工作区。
4. 原生ROS验证demo input→echo等发布/订阅关系、名称空间和节点退出；只读图谱刷新不发布控制消息。服务端缺能力时明确降级，不造客户端关系。
5. 浏览器真实操作添加/搜索/属性/保存/导入/删除、关系聚焦/过滤/刷新和资源清理；桌面布局与现有单UI入口保持一致。Android实机本轮不测。
6. 按实际触及范围执行TS/构建、现有核心/观测/协议回归及官方lint；新冻结版本执行规定的大数据/交互负载，分别记录浏览器与无浏览器的资源口径和失败，不增加超时或自动重试补绿。

## 参考及核对来源

- [ROS 2 Jazzy接口设计](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Basic/About-Interfaces.rst)：接口由IDL定义，Topic/Service/Action为通用通信契约。
- [Jazzy自定义RViz Display教程](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Tutorials/Intermediate/RViz/RViz-Custom-Display/RViz-Custom-Display.rst)：新消息的显示依赖消息转换或显示插件，不能自动猜测语义。
- [RViz Jazzy插件说明](https://raw.githubusercontent.com/ros2/rviz/jazzy/docs/plugin_development.md)：Display/Panel/Tool/FrameTransformer等扩展点；本机display.hpp、ros_topic_display.hpp核对enabled/topic/QoS/load/save及取消订阅生命周期。
- [rqt_graph ROS 2源码](https://raw.githubusercontent.com/ros-visualization/rqt_graph/ros2/src/rqt_graph/rosgraph2_impl.py)：图谱由节点发布/订阅发现与端点信息组成，区别节点—Topic边与节点关系；本机Jazzy安装源码同时核对。
- 本机rosapi/proxy.py与rosapi_msgs/srv/NodeDetails.srv：核对当前可取得的信息和缺失字段。本次未修改系统库、安装新依赖、启动ROS或浏览器测试；这里只做设计与源码审查。

设计可执行性已由独立只读builder检查，结果nothing important left；图谱范围随后获用户明确选择。实现由当前agent在本会话执行，最终再做独立只读代码审查，禁止委托并行实现。
