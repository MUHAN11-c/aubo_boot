# 功能核对与范围

本包是独立通信与观测组件；Peach、IVG与Rebot-Arm仅作为需求样本，本轮不迁移业务。Gazebo仿真已于2026-10-28由用户裁定为失败项目并暂时屏蔽，不再作为本包联调或验收来源；可用无硬件demo、Peach mock或已有数据回放。

下面区分已经实现的通用能力与仍由业务包/原生工具负责的部分。

| 需求来源 / 常见功能 | 当前实现与入口 | 核对方式 / 边界 |
|---|---|---|
| ROS 图谱、原生消息类型 | SDK graph/interfaceType/describeInterface；graph入口的discoverGraph/GraphView | 实际node_details发布/订阅边，类型/命名空间过滤、邻居聚焦、部分/截断/旧会话提示；服务提供者与Action服务端另列，无客户端/QoS握手伪边 |
| RViz式可组合显示 | workbench入口的registry/controller/view，默认8类适配器 | 多路同类显示、单共享3D上下文、独立启停、配置保存/导入；未知消息raw摘要，不能装载原生RViz插件 |
| Topic、传感QoS、持久配置 | subscribe/publisher、sensor/reliable/latched/staticTf与显式策略、JSON/CBOR | 回显、BE流、晚订阅、清理与断线探针 |
| 大帧、分片与慢客户端 | transportLimits/transportStats、有界JSON重组、CBOR TypedArray、发送预算 | 丢片/乱序/重复/超限/截止时间/回调/重入故障探针；不将SDK预算称为整个浏览器内存限制 |
| 长服务与任务 | Service/Action共用有界本地等待、绝对截止时间；原生Action反馈、取消、全部终态 | 成功、拒绝、取消、中止、超时与断线；无自动补发 |
| rqt参数与生命周期 | 九类参数、描述、逐项/原子写、事件观察、标准Lifecycle服务 | 节点拒绝写入/原子回滚、原生转换；不新增管理器 |
| Rebot-Arm / IVG 模型 | URDFLoader + Three、package资产、JointState同步 | 自带4关节/2活动模型与STL实渲染；不浏览器求解机器人FK/IK |
| RViz TF 与空间数据 | Lichtblick TF复用、Marker0..11、PointCloud2 RGB、Path | 有界解析、精确stamp、零戳显示、frame_locked、lifetime与资源释放 |
| Peach/IVG彩色与深度 | ImageView压缩PNG/JPEG和常见raw编码 | 明确16UC1单位，32FC1米；不支持compressedDepth特殊封装 |
| Peach五点力/关节/状态曲线 | SeriesPlot 1..8数值字段、600点有界缓存 | 业务可指定data、position.0等；不执行表达式 |
| 诊断与日志 | 标准Topic通用订阅、demo diagnostics和网页操作记录 | 保留ROS原消息；未做完整rqt_console检索/过滤界面 |
| 机械臂运动与工具 | 通过标准Service/Action复用应用接口 | 本包不实现新硬件命令门、不直写驱动/SetIO；业务授权保留 |
| 仿真重建开关 | 通用参数/Service/Action已可承载 | 重建算法及开关仍属Peach业务，本轮不修改 |
| IVG硬件SDK FK/IK对比 | 保留现有应用功能 | 不调用、不迁移、不删除 |
| MoveIt关节轨迹回放、规划、Servo | 当前未做专用面板 | Path是位置折线；生产继续使用MoveIt、RViz、控制器原生接口 |
| MCAP录制/回放 | 原生rosbag2录制；replay.launch.py组合回放与自测页 | 已有真实相机录制、回放循环/时间语义与退出检查；浏览器不重写存储播放器 |
| 地图/LaserScan、完整Lichtblick | 当前未集成专用面板 | 可由业务按需增加组件或独立工具；不算本轮验收通过项 |
| Android平板 | 同一HTTP/WS页面、响应布局与按需3D | 用户2026-10-30要求本轮暂不测试实机；未验收，桌面触摸模拟不冒充实机 |

## 固定依赖

| 组件 | 本包固定版本 | 职责与许可 |
|---|---|---|
| ROS 2 / Python | Jazzy / 3.12 | ROS接口与launch；ROS包使用apt |
| rosbridge_suite / roslibjs | 2.7.1 / 2.1.0 | 原生WebSocket桥与浏览器传输；BSD |
| cbor2 | 2.4.1 | 与roslib相同的成熟CBOR/TypedArray解码器；MIT |
| Three / URDFLoader | 0.186.1 / 0.13.1 | WebGL与模型/关节；MIT / Apache-2.0 |
| Lichtblick TF源码 / gl-matrix | v1.30.0 / 3.4.4 | 上游TF树、插值和几何；MPL-2.0 / MIT |
| uPlot | 1.6.32 | 有界时序曲线；MIT |
| image-dimensions | 2.6.0 | PNG/JPEG头尺寸预算预检，无运行依赖；MIT |
| Node / TypeScript / esbuild | 22+ / 7.0.2 / 0.28.2 | 仅本地构建，运行无需Node |

版本是当前锁文件快照，不表示永远最新；升级需按 [source.md](source.md) 复核源码兼容点，并按 [testing.md](testing.md) 重新验收。功能限制与内存上限见 [architecture.md](architecture.md)，公有API见 [io.md](io.md)。

## 当前验收映射（0.7.1）

网页表单的Topic回显、Service成功/非法JSON/超时/本地停止等待、Action反馈与4/5/6/拒绝终态、九类参数写读恢复、生命周期四步转换、图谱/接口和多面板独立启停共10组已在真实ROS＋Chrome执行；页面demo按钮另是12项，SDK探针是17项，三者不互换计数。标准预览与ImageView共用PNG/JPEG头、16MB压缩数据与默认4M像素门，并尊重项目maxPixels；拒绝后可恢复合法图。证据和故障夹具见development.md I65。

Peach录制完整流已覆盖源约17万点/4.14MB、640×480 RGB-D、21 TF和模型12项资源，桌面图层/鼠标/缩放/深度/曲线及LAN主机访问通过。0.7.0默认1800秒2688RPC0失败与0.7.1正式1800秒2691RPC0失败分别记录，不能据此推导任意终端/网络/模型格式/无限时长性能。用户暂不做Android实机，迁移Peach/IVG仍不在本轮。

随包browser CLI当前21组通过，包含项目选择/清空、完整报告下载、停止当前Action并确认5/未执行项、日志清空与布局；这是桌面真实浏览器操作，不是Android实机或Peach业务迁移。旧CLI缺陷与两轮夹具错误见development.md I66，最终包与已验生产代码绑定另有清单。
