# 通用 ROS 2 工作台历史实施计划（0.8.0）

本文件保留 2026-11-02 的已执行计划、当时约束与完成清单。现行接口见
[io.md](io.md)，使用见 [workbench.md](workbench.md)，验证范围见
[testing.md](testing.md)。下文工作流指令仅属于历史计划，不要求在维护时重新执行。

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans；本会话由当前agent执行，只有最终只读代码审查使用fresh reviewer，不委托实现。Steps use checkbox syntax.

**Goal:** 在独立包中实现可组合显示、共享3D画布、配置保存与完整节点—Topic发布/订阅关系图；服务与Action信息单列。

**Architecture:** 保留RosClient和既有四入口。graph模块组合现有rosapi只读服务并用Cytoscape展示；SceneView增加共享渲染上下文/实例生命周期，workbench将标准显示适配器、配置与UI分开。经典工具保持兼容，观测会话互斥避免同话题选项冲突，不引入新ROS协议或业务节点。

**Tech Stack:** Jazzy/rosbridge2.7.1、既有TS/Three/URDFLoader/Lichtblick/uPlot/image-dimensions；唯一新增运行依赖cytoscape3.34.4（MIT、自带index.d.ts、npm元数据已核）。

**Spec:** [universal-workbench-design.md](universal-workbench-design.md)，用户2026-11-02已选择首轮Pub/Sub关系，服务与Action信息另列；builder check为nothing important left。

## Global Constraints

- 只改ros2_web_link及其运行证据；不迁移Peach/IVG，不碰驱动/真机/SetIO/相机/Gazebo。Android本轮不测。
- 现有整个包untracked，.git只读，不创建Git worktree/不提交；保留0.7.1归档，以源码SHA和独立普通安装隔离运行。
- 原四公有入口兼容；新增graph/workbench两个可单独导入入口，核心SDK不引DOM/图形库。
- 所有空间显示共一个WebGLRenderer/相机/TF上下文；不得按显示实例新建WebGL上下文。
- 新显示默认按显式操作启用；禁用/删除释放订阅与实例资源，最后消费者释放才停止共用输入；异步迟到不能复活。
- 同客户端同Topic不同type/options显式拒绝，沿用现有SDK约束，不虚构独立DDS reader。
- 初始工作区schemaVersion=1，maxDisplays=16，maxSpatialDisplays=8，maxTotalCloudPoints=400000；每云<=100000、图像默认4M像素/16MB字节，已有解析/TF/Marker/曲线界限不放宽。
- 图谱默认手动刷新；快照10s整体截止、单次RPC<=3s、并发4，最多256节点/512话题/2048边；缺证/截断明确partial，手动刷新不自动重试。
- 配置JSON<=1MiB；导入先校验再替换，导入不自动连接或执行命令；未知插件/版本/非法配置回滚。
- 原生测试使用空DDS域、自有端口、有时长与内存的独占单元；测完按PID创建身份/端口清理。test/仅官方lint；行为核对为显式CLI探针。

## Review Focus

1. 同名节点/Topic或同Topic多类型信息不足：保持种类id与歧义标记，不伪造已验证连接。
2. 采集中退出/断线/超时、取消与迟到响应：部分结果与时间明确，停止后不继续轮询、不复活旧图。
3. 两份同类显示的disable/dispose及共用输入：一份退出不损坏另一份，最终订阅/图形归零，只有一个GL上下文。
4. 模型异步加载、clock回退、缺TF与新会话：旧数据/资源不得进入新显示；共享上下文所有权清晰。
5. 配置恶意键/未知适配器/超预算/同Topic选项冲突：导入原子拒绝，既有有效状态保留，不执行代码或命令。

---

### Task 1：只读图谱与关系视图

**Files:** Create `graph/model.ts`, `graph/view.ts`, `graph/index.ts`, `scripts/graph_probe.mjs`；Modify package清单/锁、tsconfig、build脚本、CMake新源码安装目录。

**Interfaces:**
- `discoverGraph(client:RosClient,options?:GraphOptions):Promise<GraphSnapshot>`；options为signal/timeoutMs/maxNodes/maxTopics/maxEdges/concurrency。snapshot包含status(complete/partial/failed)、receivedAt、nodes/topics/edges、services/actions、errors、capabilities；edge方向publish/subscribe，仅基于NodeDetails实际数组。
- `GraphView(root, {onSelect?})`提供`setSnapshot(snapshot)`, `filter({text,hideInternal,mode})`, `fit()`, `dispose()`；mode为topics/nodes。Cytoscape内置breadthfirst布局，不自写布局算法。
- 服务记录可含已证实providers；Action只列实际action_servers名称；完整客户端关系/QoS握手能力标false。

- [x] 写graph_probe并执行RED：缺模块；独立fixture覆盖发布→Topic→订阅、namespaced/type信息、同名种类不串、服务provider/Action列表、重复节点歧义、节点失败partial、图上限、截止/Abort/连接代际与有限并发、无发布调用。
- [x] 实现有界采集及纯snapshot结构；使用client.graph与/rosapi/node_details，不改原graph方法；单轮Abort释放本地等待。
- [x] 阅读Cytoscape官方API与本版本源码后锁定安装并实现GraphView；无DOM时可执行model，单独导出核心数据。
- [x] `npm run check/build`、`node scripts/graph_probe.mjs`通过；record源/依赖/日志，任务证据落runs新目录。

### Task 2：单画布多空间显示

**Files:** Modify `scene/index.ts`；Create `scripts/scene_displays_probe.mjs`；原tf/decoder/shader模块保持复用。

**Interfaces:**
- `SceneOptions.layers?:SceneLayer[]`选择首次启用内容，缺省维持现有五类；`SceneView.createDisplay(options:SceneOptions):SceneView`创建共享renderer/camera/TF的逻辑子实例。
- 子实例仍使用start/setLayer/setFixedFrame/stats/dispose；不创建renderer/controls/resize/RAF或拥有公共TF寿命。根实例拥有这些资源并在同一帧更新子实例，`stats()`汇总显示规模。
- 关闭子实例不清根TF/GL；根dispose关闭所有子实例；frame/time改变同步到子实例，clock回退清对应时间数据。

- [x] 写失败探针：layers只订指定输入、两个云/路径/机器人实例数据独立、disable/dispose不破坏peer、共享TF不被子dispose清、根最终释放一次；保留旧Scene探针作为回归。
- [x] 在现有逻辑上增加共享host分支与owned-resource检查；模型/Marker/几何和异步generation仍由各实例拥有，不复制TF或解码算法。
- [x] 严格检查、构建、既有observe与新探针通过；实际浏览器Task4验证GL上下文数、迟到与资源清理。

### Task 3：工作区配置、显示注册与生命周期

**Files:** Create `workbench/config.ts`, `workbench/registry.ts`, `workbench/controller.ts`, `workbench/index.ts`, `scripts/workbench_probe.mjs`。

**Interfaces:**
- `WorkspaceSpec`={schemaVersion:1, fixedFrame, tf, tfStatic, useSimTime, displays:DisplaySpec[], layout:'split'|'stacked'}；DisplaySpec={id,name,plugin,enabled,inputs:Record<string,TopicBinding>,options:Record<string,unknown>}。
- `DisplayRegistry.register(definition)`注册本地插件；definition声明id/title/input ports/空间或2D类别、validate与create(context,spec)；create返回start/dispose/stats。
- 默认插件id为robot/tf/markers/cloud/path/image/plot/raw；Robot两输入、TF取工作区全局，图像支持raw/CompressedImage，未知消息可raw查看；曲线配置fields数组，不eval。
- `validateWorkspace(value,registry):WorkspaceSpec`做有界、严格数据校验；`workspaceFromProfile(profile):WorkspaceSpec`生成显式启停的初始配置。
- `WorkbenchController`提供`resume()`, `pause()`, `add`, `remove`, `setEnabled`, `update`, `replace`, `snapshot`, `stats`, `dispose`；context提供当前control/observation客户端与beforeResume回调；pause不修改用户enabled配置，replace导入后保持暂停。

- [x] 配置/控制探针先RED，覆盖16/8/400k边界、重复id、非法类型/QoS/选项、未知插件/版本、导入回滚、同Topic冲突、共用输入peer寿命、暂停/新会话/迟到、原始摘要与600点曲线预算。
- [x] 用现有组件创建builtins和共享Scene根；显示数量/总云点数校验先于创建，按启用实例计数，失败清理实例并保留可见状态。
- [x] 严格检查/构建/新探针通过；核心RosClient文件保持不变。

### Task 4：同一网页的显示管理和关系图

**Files:** Create `workbench/view.ts`；Modify `web/index.html`, `web/app.js`, `web/project.js`, `web/style.css`, `scripts/build.mjs`；Create显式browser工作台探针。

**Interfaces:**
- `WorkbenchView(root, {getControlClient,getObservationClient,beforeResume,profile,onStatus})`按需加载；显示管理/属性/共享画布、2D视图、关系图与服务/Action详情同页；可通过`syncConnection`, `pause`, `dispose`接入现页。
- 添加按类型/Topic两入口，图谱类型推荐且允许手填；实例enable/delete/rename、输入/QoS/限频/options编辑、工作区下载/文件导入/本地保存与恢复、split/stacked布局。
- 关系图readonly刷新/暂停、搜索/内部Topic过滤、两视图模式、邻居聚焦和详情；5s自动刷新可显式开启，重叠刷新不排队。
- 当前经典观测开始/项目验收会暂停工作台，工作台resume前停止经典观测；不自动恢复命令或改用户显示勾选，连接丢失立即暂停并清资源。

- [x] 先写真实浏览器流程断言作为缺入口RED；沿用已有CSS工业工具风格，所有ROS名称/消息使用textContent或图组件文本，避免HTML注入。
- [x] 原app只接生命周期/动态模块，工作台逻辑保持可独立导入；UI明确complete/partial/failed与实际能力。
- [x] 无硬件demo与已录Peach数据：添加两份同类显示/不同话题、独立启停、模型/图像/曲线、搜索/选择/导出/导入回滚、readonly关系/节点退出/服务Action另列；断开后订阅/pending/canvas/图监听清理。
- [x] instrumentation记录真实getContext对象集合仅一个空间GL上下文，2D canvas另计；不把GL数量/CPU提交等同FPS保证。

### Task 5：版本、集成、完整验证与文档

**Files:** 0.8.0清单、README和包级doc；runs/ros2_web_workbench_20261102新目录，不覆盖0.7.1。

- [x] 版本/六公有入口/许可证/源码map/本地归档与离线重建、独立普通安装核对；未知适配器或桥能力失败显式，不宣称任意发行版或RViz原生插件兼容。
- [x] 受影响功能严格检查、构建、核心SDK与各CPU探针、真实ROS/浏览器及官方colcon lint；运行中输入不可重建。
- [x] 基于新生产文件执行限定大数据/交互容量验证，单独记核心与浏览器口径；保留失败/已知限制，不用旧0.7.1证据代新功能，不不断放宽时长/资源/超时。
- [x] 最终独立只读代码审查，必要修复RED→GREEN；文档写明来源、API、适配器接入、使用步骤、实际验收与未测项，最终清理并刷新证据索引。

## Self-review

图谱独立数据/视图在Task1；单上下文/多实例在Task2；配置与扩展/共用输入寿命在Task3；单UI与实际用户操作在Task4；范围/来源/交付/性能在Task5。Review Focus均归属Task1–4对应断言。graph与workbench依赖RosClient但不修改协议；Task3消费Task2共享Scene接口，Task4消费前两模块的公开方法。没有新增硬件/参数/TF/lifecycle框架，也没有要求用户重新批准已选择的首轮范围。
