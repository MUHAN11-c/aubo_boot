# Peach Web 可视化与剪切采摘架构设计提案

日期：2026-10-13。用途：现场操作 + 实验验证兼顾。状态：设计提案，未部署，未修改业务代码，未运行真机运动或 SetIO。所有页面里的布局示意都是设计样例，不是实时机器人状态。现行系统事实以本次工作树源码为依据；工作区有其他未提交改动，本轮不提交或覆盖它们。

## 1. 推荐方向与可选方案

推荐沿用 peach_web + peach_observability 同源 HTTP，把已有六步实验台组织成现场工作台和实验视图；统一目标上下文、时间与证据口径。研发深度诊断和 MCAP 回放可复用 Foxglove，作为限定访问的只读工具。这样能保留现有授权链和离线静态资源，又能借用成熟可视化能力。[S27–S33]

可选方案 B：完全用 Foxglove 布局替代。优势是标准机器人图层、曲线与回放；代价是要补业务工作流、完成证据与命令控制，需确认当前版本的授权、部署和许可条件。适合工程诊断，作为主现场产品页需要额外产品化。

可选方案 C：浏览器经 rosbridge/roslibjs 直接操作所有 ROS 话题/服务。开发灵活，但扩大控制面并引入与现有 HTTP 重叠的入口。仅为此次界面设计不采用；后续确需通用 ROS 工具时，用成熟库并限制访问。[S32 S33]

本设计不提出另起第二套生命周期、批次状态机、图像传输或记录格式；也不立即拆包。建议的界面分区是产品视图，不意味着要新增六个后端服务。

## 2. 当前实现与证据

### 已有：视图与数据链

顶层 peach_web 是静态资源包；peach_observability 托管 8090。六步、双图像、真实工具 TF、3D/2D降级已实现。

源码：[src/peach_web/README.md](/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach_web/README.md)。

### 已有：批次与分阶段完成

RunHarvest、ControlTask 与 ExecuteTarget 已定义。PREGRASP_ONLY 到位停住；cut命令受理与确认分开，level=7 才计采摘。

源码：[src/peach/peach_interfaces/action/ExecuteTarget.action](/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach/peach_interfaces/action/ExecuteTarget.action)。

### 差距：Web 使能未接线

SetEnables 服务存在，但 debug_actions 的服务表不含该入口。FireStep 仅 PHOTO 接线，其余占位不能当完整实验后端。

源码：[src/peach/peach_observability/peach_observability/debug_actions.py](/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach/peach_observability/peach_observability/debug_actions.py)。

### 差距：余量门与文档存在漂移

cycle.cpp 当前令牌路要求 allowed、新鲜度、目标、预抓取验证与 tool使能；注释明确负径向/轴向余量不直接拒绝。不能按技能文字宣称分余量强制已完成。

源码：[src/peach/peach_arm/src/cycle.cpp](/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach/peach_arm/src/cycle.cpp)。

### 已有但待现场验：剪切反馈

stageVerifyCut 有界等工具DI新沿；超时保持待人工、不自动撤离。该DI是否能证明物理剪断，以及承接证据，仍须实物测量。

源码：[src/peach/peach_arm/src/stages.cpp](/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach/peach_arm/src/stages.cpp)。

### 部署事实：默认进程死检关闭

harvest_system 的 bond_timeout 默认0，tool_profile默认bite_shear_v1，autostart默认false。相机当前默认stereo；不可照旧快照猜配置。

源码：[src/peach/peach_bringup/launch/harvest_system.launch.py](/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach/peach_bringup/launch/harvest_system.launch.py)。

### 已有：身份与新鲜度

当前Web保存scene_epoch绑定选择，融合来源注记拒绝跨场景关联；图像与工具TF分别标陈旧。最新图像不证明与模型同曝光。

源码：[src/peach_web/web/workbench.js](/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach_web/web/workbench.js)。

### 已有：观测与会话工件

state、trajectory、JPEG只读；debug POST转发既有入口。MCAP按节点会话记录，ledger按批次，结束自动报告。

源码：[src/peach/peach_observability/doc/io.md](/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach/peach_observability/doc/io.md)。

特别注意：部分包级文档与技能仍描述“allowed 汇总展示/余量分档强制”，而 cycle.cpp 当前令牌分支直接检查 allowed，并明确负余量不直拦。该漂移只在设计中揭示，本轮没有修改行为或把文档圆回去。后续实现需同时修对应源码与包级活文档。

## 3. 信息架构与主界面

主导航：现场作业 / 目标与几何 / 实验验证 / 故障诊断 / 记录回放 / 配置维护。六个视图共用 run_id、target_id、scene_epoch 与 mode，切页不改变执行状态。记录回放是建议新增页面；现有 MCAP 和报告已经具备。默认进入现场作业，工程详情逐层展开，现有六步入口放在实验验证。

固定顶部条显示：real/mock/replay、工具档、相机前端、批次与目标、批次状态、技能阶段、数据新鲜度、主要阻挡。模式来自启动事实，不用浏览器连接地址猜测。固定操作区显示取消执行与取消批次；它们是应用停轨，界面不得称急停。[S11 S17]

桌面布局：左 20% 为目标列表/六步导航，中 52% 为图像与几何证据，右 28% 为当前下一步、条件清单和完成证据；下方共享批次/技能时间线。宽度是布局起点，不是固定像素。≥1280px 三栏，768–1279px 两栏，<768px 单栏，图像/3D用内部页签，操作摘要留在可见位置。表格在自己的容器横向滚动，不让整页溢出。

现场视图只给出一项明确的主要下一步：开始扫场、选择目标、重新观察、只规划、执行预抓取、等待恢复或结束批次。完整周期必须明确是否包含套入/用刀，且由后端真实能力决定，不通过UI选项改变 execute_pregrasp_only 等部署事实。使能暂时只读；真正开放 Web SetEnables 时以服务回读、人工授权与单一操作权为前提。

实验视图保留已有六步，每步展示输入条件、输出工件、实际运动范围、命令状态、拒因与验收证据；不把占位 FireStep 当成已接六步后端。运动键不设默认焦点，不提供易误触的全局快捷键。主要操作至少44px高是本设计触屏目标；WCAG AA的尺寸基线为24×24px及其例外，并非强制44px。重要状态用文字、图标/线型与色彩共同表达，页签有键盘与ARIA语义。[S34–S36]

## 4. 剪切采摘的核心表达

### 4.1 实测与推断分开

“果实/外袋已检出”“袋轴已融合”“剪切参考可计算”“真实果梗已定位”“剪断已确认”是五件事。当前外袋与袋口几何可以给出工具对位参考；不代表袋内果实表面、果梗中心线、主枝危险距离均已观测。[S02 S03 S05]

图层固定分为：实测 RGB-D/有效深度；单帧感知初值；融合外袋模型；果实包络先验；估计剪切区/工艺偏置；若以后接入则为真实果梗/主枝；实际 tool_axis/sleeve_mouth/cutting_plane；计划与实走轨迹。实测实线，估计虚线，受限区用结构线型并附文字，来源不明则隐藏对应几何并保留原因。底→口的袋轴与真实刀具轴分别展示，不能用TCP默认Z猜实际工具方向。[S22 S27 S28]

### 4.2 两条可评估路线

路线 A：外袋可见几何 + 工艺偏置。较贴近当前相机和工具，但必须用独立测量验证袋口到果梗/切割平面的偏置分布、红/黄袋、袋折叠、遮挡、枝径与工具装配误差；不能只看拟合RMSE就允许剪切。未验证时只给估计区和预抓取参考。

路线 B：近场果梗/主枝语义定位 + 主动观察。参考 Harvey/HortiBot 的果实、果梗、主茎分离；采集带剪切真值的数据，评价三维定位误差和误切风险，再决定是否进入许可模型。未检出时不照搬固定切点兜底。预测补全枝条只能作为离线研究图层，不能冒充实测通道。[S02 S03 S05 S08]

工具几何约束建议采用可追溯的误差预算：径向净空 = 入口有效半径 − 外袋包络半径 − 定位/姿态横向误差预算；轴向净空 = 可切空间 − 工具包络/装配与轴向定位预算。公式是设计框架，具体各项定义、组合方式和阈值由实物标定决定，不能先写任意毫米常数。界面同时显示数值、来源、标定版本、该检查是否真的在后端强制。当前 negative margin 不能呈为已拒绝。

### 4.3 成功证据与当前口径

当前完成等级：0未到预抓取；1到预抓取；2预抓取验证；3套入完成；4刀指令受理；5切断确认；6撤退确认；7采摘确认。checkpoint 是实时里程碑，completion_level 是最高已验证阶段，两种枚举数值不同，禁止通用数字映射。

现行 harvestConfirmed = cut_confirmed ∧ retreat_confirmed；stageVerifyCut 等工具DI新沿。Web如实显示这一软件口径，并附证据来源。刀具到位DI能否证明果梗已剪断，需要现场验证；不能把有DI接口写成物理剪断已经可靠验证。后续更强验收建议增加独立物理脱离/承接证据（合适的视觉变化、力/负载或承接传感组合，阈值须实测），并明确哪些不是当前已消费字段。收集位释放与果实入箱可以做物流结果扩展，不提前改动当前成功口径。[S01 S02 S05]

## 5. 业务流程、状态与恢复

### 01 拍照与建场景

负责：supervisor → scene_perception。输入：相机/内参/TF/关节流可用；运动需既有授权。输出：scene_epoch + 目标观测。拒因/限制：曝光时未静止、TF失配、帧龄过大则采帧拒绝。依据：S12 S39。

### 02 扫场与选目标

负责：supervisor → peach_arm.SurveyScene。输入：BeginScene + 当前批次。输出：confirmed target_id + 场景障碍快照。拒因/限制：SURVEY_ONLY 也可能动臂；选择不改变机器人状态。依据：S06 S11。

### 03 重建与几何核对

负责：target_reconstruction。输入：同戳RGB-D/掩膜 + 精确TF + 独立机位。输出：外袋模型/袋轴/剪切参考 + GraspDecision。拒因/限制：缺果梗观测时显示估计区；模型质量不能证明剪切真值。依据：S02 S03 S05。

### 04 只规划与预抓取

负责：supervisor → arm → MoveIt/MTC。输入：身份七元组 + plan_id + 当前关节。输出：规划结果；执行后预抓取残差/level=2。拒因/限制：PREVIEW不运动；PREGRASP_ONLY停住待ACK，不计采摘。依据：S18 S19。

### 05 套入与剪切

负责：arm.authorizeStage → tool_actuator。输入：新鲜许可 + 使能 + 工具/标定身份 + 预抓取验证。输出：套入完成 → 指令受理 → 工具DI新沿。拒因/限制：CUT_UNCONFIRMED 保持待人工，不自动正常撤离或重发。依据：S02 S17 S21。

### 06 确认、撤离与入账

负责：arm → supervisor → ledger。输入：切断证据 + 撤退证据。输出：当前口径level=7；扩展物理脱离/承接证据后更强验收。拒因/限制：DI是否证明实际剪断须现场验证；取消保留已验证阶段。依据：S01 S04 S11。

批次状态权威在 harvest_fsm；页面按权限镜像显示可操作项，不自行推演完整控制状态机。模型七元组、plan_id、generation、scene_epoch 和 valid_until 要保持绑定；心跳不续许可；换场景、换工具、换标定、换模型则旧预览/许可撤销。纯显示选中与supervisor执行目标分别标注，不能把UI圈选偷换成执行绑定。[S11]

失败恢复矩阵：

- 观测不足/模型失效：撤销旧许可，重新观察或跳过；不以旧模型降级接触。
- 无IK/直线规划失败：不执行残缺轨迹，显示失败阶段，可换目标或重观察。
- 预抓取到位但残差未过：保留已到位证据，等待处理，不能进入用刀。
- PREGRASP_ONLY终局：到位停住并等待ACK；ACK成功后才派下一颗，不能把停位视为回收完成。
- PAUSE_PENDING：解释在途接触需要收口，不承诺立即停车；需要即时应用取消则使用独立取消入口。
- CUT_UNCONFIRMED：可能仍连枝，保持现状待现场确认；不自动正常撤离、不重发用刀、不自动resume。
- 保护停止/硬件急停：人工处理现场及示教器复位、取消并丢弃旧goal，再重新授权下发。网页不提供远程上电、解锁或自动继续。[S17]
- 浏览器HTTP超时/网络断开：保留unknown；查询真实在途/终态，取消入口独立；禁止自动重试POST。[S11 S37]

## 6. 架构与接口策略

现行路径：RGB-D → scene_perception（事实）→ target_reconstruction（模型/许可）；supervisor（唯一批次权威）→ peach_arm（逐阶段命令门）→ MoveIt/MTC/Pilz → 控制器/柜。Survey完成后场景障碍快照写PlanningScene world，arm按自己的职责写ACM。CheckReachability是调度到臂的只读可达查询，感知不产生运动。

peach_interfaces是契约包，不是运行进程。brain是三节点同进程，但仍通过ROS接口，不据此声称已有rclpy零拷贝。四托管节点由Nav2 lifecycle manager有序启动/反向清理；bond已接但默认0关闭，不能只看Active图标推断有失联降级。[S12 S14 S15]

读侧保持 /api/state、/api/trajectory、两路JPEG；状态与大图分离，图像后台编码有界缓存，不在HTTP请求线程跑TF或ROS等待，不为每个浏览器再建raw订阅。当前单在途轮询已经可用，优先整理数据契约和证据。确有延迟需求时再引入SSE单向状态/事件，保留序号和全量快照恢复，不靠流重连重新提交运动。[S38 S40]

写侧仅转发明确白名单；RunHarvest/Execute/Build/Survey保留ROS action语义，短控制用service。前端不直接SetIO，不写执行/安全/许可bool。后端debug门、supervisor权限/使能、arm ExecutionAuthority分别回读展示。前端disabled只减少误点击，无法代替后端强制。[S11 S21]

建议新增只读操作登记：operation_id、request_id、ROS goal UUID、动作键、当前状态、submitted_at、last_update、error_code、result_evidence。操作状态 proposed→submitting→accepted→running→succeeded/failed/canceled；unknown是浏览器观察不确定态，不能覆盖后台事实。提交记录按事务关联，任何明确结果都回读；存量服务accepted和action accepted不混用。不把重复request_id直接视为可安全重放用刀，实际幂等须由对应动作事务和硬件命令去重实现。

建议视图聚合统一schema_version与快照序号；保留source、frame_id、header_stamp_ns、received/log_time、age、run/target/epoch/revision、available/unknown_reason。纳秒用字符串，未知量用null而非0。时间显示将曝光、消息记录、处理和浏览器收到分别表明；回放log时间可用于排序，重建仍用曝光精确TF。三维预览图压缩、点云抽稀不作为毫米级真值或许可输入。[S22 S27 S29]

## 7. 功能清单（当前/建议/待验）

P0 = 首轮整理或必须先完成的证据表达/验收；P1 = 经后端/传感器条件验证后开放；P2 = 按实测需求研究或扩展。P0不是把所有新增项都宣称已有。

### 现场作业

- **F01 固定运行条：模式/工具/批次/阶段/数据龄** — P0 / 已有需整理。接口/证据：/api/state.startup + task_executor + job。验收：任何页可辨 real/mock/replay；模式未知则显示未知，不猜 real。。依据：S20 S36。
- **F02 当前目标作业票与最高完成等级** — P0 / 已有需整理。接口/证据：job + ledger.harvest/verification。验收：预抓取 level=2 显示定位已验证；只有 level=7 计采摘确认。。依据：S02 S11。
- **F03 下一步条件清单与主要阻挡** — P0 / 已有需整理。接口/证据：permissions / blockers / enables / grasp_decision。验收：逐条说明：事实、检查是否启用、通过/拒绝/未知、恢复入口。。依据：S21 S24。
- **F04 全局取消执行与取消批次** — P0 / 已有。接口/证据：POST debug/cancel + ControlTask.CANCEL_NOW。验收：独立于普通提交锁；请求已发与动作已终止分开显示；软件停不标急停。。依据：S11 S17。
- **F05 三档使能的 Web 管理** — P1 / 建议新增。接口/证据：SetEnables 服务存在，HTTP 转发未接。验收：execution→grasp→tool 依赖；显示实际回读与拒因；人工授权与开关分开。。依据：S06 S21。
- **F06 启动批次：已选/全部/只扫场** — P0 / 已有需整理。接口/证据：RunHarvest.intent + target_ids。验收：SURVEY_ONLY 标明可能动臂；profile_id/selection_mode 未消费不呈有效选项。。依据：S06 S11。
- **F07 暂停、待暂停与恢复** — P0 / 已有需整理。接口/证据：ControlTask + state_seq。验收：PAUSE_PENDING 明示等在途接触收口；不能宣传为即时停轨。。依据：S11 S17。
- **F08 结束摘要与补采清单** — P0 / 已有需整理。接口/证据：ledger / HarvestSummary。验收：发现、尝试、预抓取验证、确认采摘、跳过、失败分别计数。。依据：S01 S04。

### 目标与几何

- **F09 目标列表与图像/模型联动选中** — P0 / 已有需整理。接口/证据：perception.targets.observations。验收：切目标只改变选中；scene_epoch 变更立即撤销旧选择。。依据：S07 S09 S28。
- **F10 稳定识别图/即时检测图与帧龄** — P0 / 已有。接口/证据：/api/images/perception.jpg + raw.jpg。验收：分别标 stamp/frame/age；暂停画面不暂停机器人。。依据：S28 S37。
- **F11 深度有效掩膜与袋口局部证据** — P0 / 建议新增。接口/证据：现有 raw RGB-D/geometry diagnostics；需预览适配。验收：缺深度与远景混入可见；有效像素分母和 ROI 明示，不编造质量分。。依据：S03 S39。
- **F12 三维袋轴、真实刀轴、剪切平面、轨迹** — P0 / 已有。接口/证据：/api/trajectory.tool_frames + refined + xyz。验收：base_link 与单位可见；TF 陈旧不补默认轴；二维降级保留数值。。依据：S22 S27。
- **F13 实测/融合/推断几何图层区别** — P0 / 建议新增。接口/证据：refined_context + geometry.jsonl；来源字段扩展。验收：外袋模型、果实包络先验、真实果梗观察用文字与线型区分。。依据：S03 S08。
- **F14 独立机位、曝光静止与拒帧明细** — P0 / 已有需整理。接口/证据：reconstruction.status/diagnostics + params。验收：积分帧数与机位数分开；拒帧显示静止/TF/内参/邻目标原因。。依据：S05 S39。
- **F15 分能力许可面板** — P0 / 已有需整理。接口/证据：geometry/pregrasp/sleeve/cut capability + valid_until。验收：不只画 allowed；有效期冻结、身份不符/过期不展示为可执行。。依据：S02 S11。
- **F16 径向/轴向余量与实际检查状态** — P0 / 建议新增。接口/证据：radial_margin_m/axial_margin_m + 后端检查说明。验收：单位 mm；当前负余量不直拦必须明示；启用门需机械常数标定后后端实现。。依据：S02 S19。
- **F17 果梗/主枝语义与剪切区独立估计** — P2 / 待研究验证。接口/证据：新算法及必要 IDL 扩展，不能前端自算授权。验收：评估定位误差、遮挡、误切保护区；数据覆盖红/黄袋与不同枝径。。依据：S03 S05 S07。
- **F18 工艺偏置模型的证据与版本** — P1 / 建议新增。接口/证据：工具档案 + config/calibration revision；模型来源扩展。验收：显示外袋参考→偏置→估计剪切区；标定样本和误差可追溯。。依据：S02 S22。

### 实验验证

- **F19 保留六步实验台** — P0 / 已有。接口/证据：workbench.js 六步 + debug endpoints。验收：拍照建场景→扫场选果→重建核对→只规划→预抓取→完整周期。。依据：S06 S18。
- **F20 每步输入、输出、拒因与验收证据** — P0 / 建议新增。接口/证据：既有 job/debug/diagnostics 聚合。验收：切步骤不发命令；每步有“本次是否动臂/接触/用刀”文字。。依据：S11 S18。
- **F21 计划和实走轨迹对照** — P1 / 建议新增。接口/证据：需提供计划轨迹与 plan_id，现有实走 TCP xyz。验收：虚线计划/实线实走；缺计划数据显示未提供，不拿实走冒充预览。。依据：S18 S19 S27。
- **F22 回拍照位的规划/执行反馈** — P0 / 已有需整理。接口/证据：photo_pose_service + execution_enabled。验收：使能关时仅规划须明确；恢复锁、在途周期与陈旧数据阻挡可读。。依据：S19 S37。
- **F23 接触前确认摘要** — P0 / 已有需整理。接口/证据：发送前状态+目标+epoch+revision 复核。验收：确认后发送前再核对；换场景和过期令牌不会消费旧点击。。依据：S11 S17。
- **F24 动作事务与未知结果查询** — P0 / 建议新增。接口/证据：现有 debug 响应；建议请求登记/查询读接口。验收：queued/accepted/running/unknown/终态分开；POST不重试，页面恢复先查在途。。依据：S11 S37。
- **F25 工具指令/反馈/物理脱离三层证据** — P0 / 已有需整理。接口/证据：checkpoint + ToolActuator + harvest/verification。验收：SetIO ACK≠切断；DI新沿≠默认证明实际果梗脱离；证据来源可读。。依据：S02 S05。
- **F26 独立剪断/承接确认** — P1 / 待研究验证。接口/证据：刀具 DI 与视觉/力/承接传感；IDL 验证字段扩展。验收：无反馈维持未确认；无法确认不自动正常撤离，不重发剪刀指令。。依据：S02 S05。
- **F27 样本注记与实验标签** — P1 / 建议新增。接口/证据：现有 session 工件；新增只读关联/注记存储。验收：记录袋色、遮挡、光照、真值测量、试验模式；注记不能修改执行结果。。依据：S01 S09。

### 故障诊断

- **F28 自检逐项 pass/warn/fail/skip** — P0 / 已有。接口/证据：selfcheck + startup。验收：低接收到达率与抓取质量不同；autostart 闩锁不等于手动开批统一硬门。。依据：S13 S24。
- **F29 节点健康、bond 启用态与最后心跳** — P0 / 已有需整理。接口/证据：diagnostics + lifecycle + startup.bond_timeout。验收：bond_timeout=0 明示未启用；未知/STALE 不画绿。。依据：S12 S14 S15。
- **F30 失败码→证据→下一步恢复指引** — P0 / 建议新增。接口/证据：FailureCode + CanonicalEvent + logs。验收：定位失败可重观测；CUT_UNCONFIRMED 保持并待现场处理；禁止一键全恢复。。依据：S04 S17 S21。
- **F31 人工恢复 ACK 与真实回读** — P0 / 已有需整理。接口/证据：ControlTask.ACKNOWLEDGE_RECOVERY + expected_state_seq。验收：ACK 成功才消耗状态序号；ACK 不等于恢复硬件保护停止。。依据：S17 S21。
- **F32 日志、事件、审计与阶段时间线** — P0 / 已有。接口/证据：logs / pipeline / debug.audit / events。验收：按 run/target/failure 查询；滚动离底不抢焦点；审计与普通日志分开。。依据：S04 S36。
- **F33 硬件表与工具反馈状态** — P0 / 已有需整理。接口/证据：robot.joints/status + 工具反馈聚合待补。验收：电流标 SDK 单位；状态只观测；缺工具传感器显示未接，不置零正常。。依据：S17 S24。
- **F34 解释权限与命令门实际拒因** — P1 / 建议新增。接口/证据：HTTP debug门 + supervisor permissions + authorizeStage。验收：前端提示不做授权；后端拒因回读，不复制一套独立控制 FSM。。依据：S11 S21。

### 记录回放

- **F35 会话目录、录制队列/丢帧/预算** — P0 / 已有。接口/证据：record / startup / bag_report。验收：session与批次不是一对一；回收 bag 二进制留报告和审计。。依据：S25 S26。
- **F36 只读离线回放时间轴** — P1 / 建议新增。接口/证据：MCAP + bag_reader + 同源回放适配，或 Foxglove。验收：进入回放撤销所有运动入口；模式明确；不向真机域重放命令。。依据：S25 S26 S29。
- **F37 跨图像/模型/TF/事件时间关联** — P1 / 建议新增。接口/证据：曝光stamp / log_time / publish_time 三类时间。验收：显示时差；仅有 latest 图片时不声称已同步；seek回退清来源缓存。。依据：S27 S29 S39。
- **F38 两次实验 A/B 比较** — P1 / 建议新增。接口/证据：startup 配置指纹 + ledger + bag_report。验收：同模式/目标条件分组；展示参数差异、拒因与阶段耗时，不混算成功率。。依据：S06 S09 S29。
- **F39 报告导出与证据包** — P1 / 已有需整理。接口/证据：bag_report.md/json + ledger + debug audit。验收：导出引用目录、配置revision与证据缺失；不假造未记录图像。。依据：S23 S25 S26。

### 配置维护

- **F40 工具/相机/标定档案只读页** — P0 / 已有需整理。接口/证据：startup + params + tool_frames。验收：显示 bite_shear_v1 等实际profile；切档不伪装运行时可安全即时切换。。依据：S22 S27。
- **F41 参数差异、分级编辑与应用时机** — P1 / 建议新增。接口/证据：既有ROS参数；需明确写白名单与生效范围。验收：空闲期修改可写项，审计/回读；标定、关节序、驱动项不可网页随意改。。依据：S23 S37。
- **F42 部署参数与运行意图分开** — P0 / 已有需整理。接口/证据：autostart / hardware_mode / enables。验收：autostart只作部署事实；网页按钮不产生真机授权或上电。。依据：S12 S17。
- **F43 离线资源、触屏与键盘操作** — P0 / 已有需整理。接口/证据：现有静态资源/CSP/ARIA。验收：主要操作≥44px设计尺寸；键盘焦点可见；颜色+文字，低动态模式。。依据：S34 S35 S36。
- **F44 局域网访问与单一操作者租约** — P1 / 建议新增。接口/证据：默认127.0.0.1；新增认证/租约后再远程部署。验收：只读访客与操作权限分开；断线不自动续执行，不改变臂侧授权。。依据：S21 S30 S32。

### 架构维护

- **F45 保留 HTTP 聚合与明确命令白名单** — P0 / 已有需整理。接口/证据：peach_observability HTTP + ROS action/service。验收：主 Web 不引入任意 publish/SetIO；仍以 supervisor 和臂侧为权威。。依据：S11 S32 S33。
- **F46 状态推送与事件补齐** — P2 / 建议新增。接口/证据：可选 SSE；先保留已有单在途轮询。验收：序号断档补快照；广播不重复动作；图片和轨迹单独降采样。。依据：S38 S40。
- **F47 研发 Foxglove 只读布局** — P2 / 建议新增。接口/证据：独立诊断桥 + allowlist，ROS2用SDK现行bridge。验收：复用3D/曲线/回放；禁任意执行话题写入；不代替现场作业页。。依据：S27 S29 S30 S31。
- **F48 分层验收与现场剪切数据集** — P0 / 建议新增。接口/证据：纯核/假API/隔离mock/现场测量。验收：mock验证身份与命令链；真实剪断、标定与损伤留现场独立测量。。依据：S01 S02 S20 S23。

## 8. 记录回放与统计口径

复用MCAP、startup配置事实、批次ledger、debug审计和bag_report。node session与batch不是一对一：一个会话可有多个request_id。回放严格只读，独立于真机ROS_DOMAIN_ID；切入时取消UI待提交上下文，不能让录制的命令自动产生新动作。时间seek倒退清选择/来源缓存，保留事件时间游标。[S25 S26 S29]

主指标分母明确：检测召回率需人工标注真值；可达率=可达目标/已确认候选；预抓取验证率=level≥2/预抓取尝试；当前确认采摘率=level7/真实FULL尝试；真实脱离率/承接率/果损与误切率另以独立现场判据。mock、冻结回放、真实相机mock臂、real分别统计；PREGRASP_ONLY不混入采摘成功率。时间同时展示成功样本周期、全部尝试耗时及各阶段中位数/P95（样本不足明示），不要只报平均成功周期。[S01 S04 S20]

实验维度：袋色、光照、遮挡级、有效深度比例、独立机位数、真实枝径/切点、工具profile、相机前端、模型/标定/配置版本。研究报告的成功率只作原条件结果，不能用于跨作物排行榜或向peach承诺指标。

## 9. 分阶段落地与验收

阶段A：统一信息与真值表达（主首屏、六步、完成等级、实际命令门与缺失证据）。复用现有前端；补剪切区来源和拒因必要字段。通过后再开放新增控制能力。

阶段B：后端证据链与恢复（操作登记、未知结果查询、使能转发、工具反馈来源、分级配置）。新增入口必须复用supervisor/arm门与审计，不能在observability生成新控制权威。负余量门先做工具标定和预算验证，再决定实现及默认值，同轮改活文档。

阶段C：离线回放、A/B与验收报告。优先使用现有bag_reader和Foxglove能力，避免自造播放器存储。运行层隔离与浏览器只读同时验证。

阶段D：果梗/主枝感知、独立脱离与承接证据、主动补视。是否实施由套袋桃数据和物理验证决定，不把研究预测作为默认剪切许可。

验收门：

1. 假API界面：real/mock/replay、断线、陈旧、确认期间换场景、迟到结果、服务拒绝、HTTP202/504、WebGL丢失、未知TF/模型身份、窄屏与键盘；不得连真机。
2. 纯核/接口：权限序号、许可过期冻结、模型身份失效、完成等级与checkpoint映射、命令去重、剪切反馈超时与恢复锁；核对manifest。
3. 独立mock域：整栈生命周期、动作取消、使能撤销、失联、会话记录/报告/清理；mock通过只证明接线和逻辑。
4. 静态几何实测：曝光同步、相机内外参、工具TF、剪切平面与袋口/果梗偏置、各种袋色/遮挡的误差预算；给出独立真值和测量不确定度。
5. 真机PREGRASP_ONLY：人授权、预抓取定位与姿态残差、多组独立目标与故障恢复。
6. 真机套入与剪切：工具反馈DI含义、断枝/不断枝反例、损伤/承接与人工撤权，确认实际物理脱离；具备后才批准FULL验收。任何测试结束立即停净并核对残留进程。

本轮验证只针对设计工件：数据完整性、链接索引、浏览器布局与交互、总图可读性；不宣称上述机器人验收已经通过。

## 10. 一手资料索引与适用限制

总量52项，基础40项列于本节，新增12项见第11节。ROS基础优先官方；农业项目按作者论文/团队代码采用研究证据，明确历史ROS1、非剪切、室内实验与未验证可复用代码的限制。文档站有迁移/反爬：Nav2使用当前官方路径并以Jazzy源码交叉核对；ROS QoS用官方测试站Jazzy镜像；REP用官方源码。没有把读取失败的旧URL计作有效来源。未在本轮运行任何外部农业仓库，不以星数或“开源”证明成熟度。

### S01 [SWEEPER · 欧盟官方结项报告](https://cordis.europa.eu/project/id/644313/reporting)

类别：剪切采摘。版本/环境：2015–2018 项目；官方历史报告。查阅：2026-10-13。

借鉴：采摘系统包括检测、接近、切割、承接、放置与人机界面；按遮挡和栽培条件报告结果。

适用限制：历史温室甜椒项目；不证明套袋桃精度，也不采用其 ROS Indigo 接线。

### S02 [Harvey · 甜椒抓持与剪切系统](https://arxiv.org/abs/1810.11920)

类别：剪切采摘。版本/环境：2018 作者论文。查阅：2026-10-13。

借鉴：抓持与剪切位置分别估计，工具解耦；分别记录感知和脱离失败。

适用限制：改造场景下的试验结果不能作为 peach 成功率目标；论文不是可直接部署的 Jazzy 包。

### S03 [Harvey · 田间果梗检测比较](https://arxiv.org/abs/1709.10275)

类别：剪切采摘。版本/环境：2017 作者论文。查阅：2026-10-13。

借鉴：果梗语义检测加三维结构过滤，区分果实中心与关键切割位置。

适用限制：甜椒果梗可见条件不同于套袋桃；不能用袋口代替真实果梗。

### S04 [Harvey · 田间试验经验](https://arxiv.org/abs/1706.06203)

类别：剪切采摘。版本/环境：2017 作者论文。查阅：2026-10-13。

借鉴：把完整周期的检测、抓持、脱离失败分段分析，记录现场条件。

适用限制：旧研究系统；只借鉴试验和失败分类，不复用 ROS 1 运行框架。

### S05 [HortiBot · 主动感知与双臂剪切](https://arxiv.org/html/2403.15306v1)

类别：剪切采摘。版本/环境：2024 作者全文。查阅：2026-10-13。

借鉴：分别检测果实、果梗与主茎，主动观察、力反馈与抓持/剪切/放置阶段。

适用限制：三臂室内甜椒实验；其缺果梗时固定切点兜底不迁入 peach 真机默认流程。未确认可复用的完整开源控制栈。

### S06 [OSUrobotics/apple-harvest](https://github.com/OSUrobotics/apple-harvest)

类别：农业 GitHub。版本/环境：ROS 2 研究代码；README 已核对。查阅：2026-10-13。

借鉴：分阶段使能、控制策略选择、分阶段录制、冻结视觉数据与仿真。

适用限制：苹果拉扭/吸附采摘，不是剪切套袋桃；README 不构成 Jazzy 兼容或现场安全保证。

### S07 [rick-dn/robofruit](https://github.com/rick-dn/robofruit)

类别：农业 GitHub。版本/环境：ICRA 2022 论文配套代码。查阅：2026-10-13。

借鉴：将 picking point、关键点、可采性与成熟度作为独立标注。

适用限制：草莓感知研究；不提供完整硬件命令门，也不证明毫米级剪切。

### S08 [Real-Sim 3D Branch Completion](https://github.com/suptimq/Real-Sim_3D_Branch_Completion)

类别：农业 GitHub。版本/环境：IROS 2024 配套代码；MIT。查阅：2026-10-13。

借鉴：树枝遮挡、点云补全与骨架可作为离线评估方向。

适用限制：补全是预测，界面须与实测点云分开；不能把预测枝条当已观测的剪切许可。

### S09 [BUP-ST20 · 时空 RGB-D 与跟踪数据](https://github.com/Agricultural-Robotics-Bonn/agrobot-BUP-ST20)

类别：农业 GitHub。版本/环境：IJRR 2025 配套数据与代码。查阅：2026-10-13。

借鉴：RGB-D、相机标定、跨帧 ID 与独立评估划分一起管理。

适用限制：甜椒监测数据不含 peach 所需的真实刀具剪断证据；需自建套袋桃剪切标注。

### S10 [jsather/harvester-sim](https://github.com/jsather/harvester-sim)

类别：农业 GitHub。版本/环境：历史 ROS Kinetic / Gazebo 仿真。查阅：2026-10-13。

借鉴：按 description、control、moveit、world 分工，生成采摘试验场景。

适用限制：ROS 1 历史研究参考；不迁移 catkin 或旧 Gazebo API。

### S11 [ROS 2 Actions · 官方设计](https://design.ros2.org/articles/actions.html)

类别：ROS 官方。版本/环境：ROS 2 设计规范。查阅：2026-10-13。

借鉴：长任务用 Goal / Feedback / Result / Cancel；取消请求与终态分别确认。

适用限制：设计规范不是当前 Python API 手册；实现时仍核对 Jazzy 源码。

### S12 [ROS 2 Managed Nodes · 官方设计](https://design.ros2.org/articles/node_lifecycle.html)

类别：ROS 官方。版本/环境：ROS 2 生命周期设计。查阅：2026-10-13。

借鉴：configure 建资源，activate 运行；状态转换与资源释放可观测。

适用限制：生命周期 Active 不表示进程一直存活，需心跳或 watchdog。

### S13 [ROS 2 Jazzy QoS · 官方测试站镜像](https://repo.test.ros2.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

类别：ROS 官方。版本/环境：Jazzy 官方文档镜像。查阅：2026-10-13。

借鉴：发布/订阅 QoS 须兼容；页面区分接收到达率与设备采集率。

适用限制：默认传感 profile 的 BEST_EFFORT 不代表本项目每路发布端必须相同；按 manifest 与实测核对。

### S14 [Nav2 Lifecycle Manager](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/)

类别：ROS 官方。版本/环境：Rolling 文档；下一条 Jazzy 源码交叉核对。查阅：2026-10-13。

借鉴：有序启动、反向关闭，bond 断联后降级托管栈。

适用限制：peach 当前 bond_timeout 默认为 0；页面不得宣称已开启进程死检。

### S15 [Nav2 Lifecycle Manager · Jazzy 源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_lifecycle_manager/src/lifecycle_manager.cpp)

类别：ROS 官方。版本/环境：jazzy 分支。查阅：2026-10-13。

借鉴：核对 bond_timeout、transition 与 manage_nodes 行为。

适用限制：借用生命周期模式，不引入导航业务或第二套管理器。

### S16 [Nav2 Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)

类别：ROS 官方。版本/环境：Rolling 官方说明。查阅：2026-10-13。

借鉴：应用过滤应处于命令输出末端；官方明确没有硬实时安全认证。

适用限制：不能当臂与刀的硬件急停，也不建议把底盘速度节点直接套到 AUBO。

### S17 [UR Driver · Robot State Helper](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/robot_state_helper.html)

类别：ROS 官方。版本/环境：Rolling 官方驱动说明。查阅：2026-10-13。

借鉴：保护停/急停释放后继续程序可能恢复旧动作；故障恢复先停止再重新下发。

适用限制：仅借鉴恢复语义；本仓不接 dashboard、不远程上电或解锁。

### S18 [MoveIt 2 · Task Constructor 教程](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)

类别：ROS 官方。版本/环境：MoveIt 2 main / Rolling。查阅：2026-10-13。

借鉴：任务拆成阶段，分别展示规划解、失败解和执行结果。

适用限制：设计借鉴阶段内省；不把 Rolling 示例直接当 Jazzy API。

### S19 [MoveIt 2 · Pilz 工业规划器](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html)

类别：ROS 官方。版本/环境：MoveIt 2 main / Rolling。查阅：2026-10-13。

借鉴：PTP / LIN / CIRC 各有约束；规划与执行应分别展示。

适用限制：接触直线不得用自由空间绕行替代；规划场景不证明现场人员安全。

### S20 [ros2_control · Mock Components](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)

类别：ROS 官方。版本/环境：Jazzy。查阅：2026-10-13。

借鉴：GenericSystem 镜像指令到状态，验证接线与 MoveIt 集成。

适用限制：理想镜像不验证碰撞力、剪断、电流、真实急停或刀具反馈。

### S21 [Autoware · Fail-safe 命令门](https://docs.autoware.org/main/design/autoware-architecture-v1/interfaces/ad-api/features/fail-safe/)

类别：ROS 官方。版本/环境：Autoware main 设计。查阅：2026-10-13。

借鉴：故障状态、处置进行中和处置结果分别表达；最终命令门统一仲裁。

适用限制：只借鉴架构，不能把车辆 MRM 状态机械映射成臂安全等级。

### S22 [REP-103 · 单位与坐标系](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst)

类别：ROS 官方。版本/环境：REP 官方源码。查阅：2026-10-13。

借鉴：内部米与右手坐标系；光学坐标系与 base_link 显式区分。

适用限制：Web 可为人展示毫米，但必须同时标单位与 frame，不能改变 IDL 单位。

### S23 [REP-2004 · 质量声明](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-2004.rst)

类别：ROS 官方。版本/环境：REP 官方源码。查阅：2026-10-13。

借鉴：将测试、依赖、变更与质量声明形成可核验的验收证据。

适用限制：不把应用栈写成已达到 QL1 或功能安全认证。

### S24 [ros/diagnostics · diagnostic_updater](https://github.com/ros/diagnostics/tree/ros2/diagnostic_updater)

类别：ROS 官方。版本/环境：ROS 2 官方包。查阅：2026-10-13。

借鉴：复用 /diagnostics，把健康任务与人读故障解释对应。

适用限制：HTTP 健康颜色不能替代节点实际诊断；新鲜度须另外检查。

### S25 [rosbag2 · Jazzy](https://github.com/ros2/rosbag2/tree/jazzy)

类别：ROS 官方。版本/环境：jazzy 分支。查阅：2026-10-13。

借鉴：复用记录、回放与存储插件，隔离回放域。

适用限制：回放原动作/IO 话题不得注入真机域；时间倒退需重置显示缓存。

### S26 [MCAP · 官方指南](https://mcap.dev/guides)

类别：可视化与 Web。版本/环境：官方格式与工具。查阅：2026-10-13。

借鉴：用现有 MCAP 保存带 schema 的机器人多模态数据。

适用限制：不为 Web 重写 bag 存储；压缩图像只能作为预览，不能充当精确深度。

### S27 [Foxglove · 3D Panel](https://docs.foxglove.dev/docs/visualization/panels/3d)

类别：可视化与 Web。版本/环境：当前官方文档。查阅：2026-10-13。

借鉴：采用图层、固定坐标系、变换历史、选中对象与跨面板观察。

适用限制：只读显示可用 latest TF；曝光积分仍必须精确 stamp，显示偏移不写回标定。

### S28 [Foxglove · Image Panel](https://docs.foxglove.dev/docs/visualization/panels/image)

类别：可视化与 Web。版本/环境：当前官方文档。查阅：2026-10-13。

借鉴：图像与注释叠加，展示对象信息、相机来源与时间。

适用限制：视觉叠加不增加观测证据；袋轴、实际工具轴必须分别标注。

### S29 [Foxglove · Playback](https://docs.foxglove.dev/docs/visualization/playback)

类别：可视化与 Web。版本/环境：当前官方文档。查阅：2026-10-13。

借鉴：统一时间游标、跳转事件、倍速与比较；明示 log / publish 时间。

适用限制：回放顺序时间与重建曝光时间用途不同，不能只选一种替代全部时间语义。

### S30 [Foxglove · ROS Bridge](https://docs.foxglove.dev/docs/fleet/bridge)

类别：可视化与 Web。版本/环境：当前官方文档。查阅：2026-10-13。

借鉴：成熟 C++ 桥支持 ROS 2 schema 和高带宽可视化，可作为研发只读旁路。

适用限制：通用桥含控制能力；生产工作台不暴露任意 publish / service，桥更新后需复核 deny/allowlist。

### S31 [Foxglove SDK · 官方 GitHub](https://github.com/foxglove/foxglove-sdk)

类别：可视化与 Web。版本/环境：当前官方仓库。查阅：2026-10-13。

借鉴：ROS 2 bridge 现行源码归入 SDK 仓；以当前官方链接选库。

适用限制：旧 ros-foxglove-bridge 仓已是 ROS 1 方向，不把它作为新的 ROS 2 集成入口。

### S32 [RobotWebTools · rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

类别：可视化与 Web。版本/环境：当前官方项目。查阅：2026-10-13。

借鉴：通用 ROS Web 桥成熟，可比较协议与诊断连接方式。

适用限制：任意话题发布和服务调用面过宽；本项目现有同源 HTTP 足够支撑主工作流。

### S33 [RobotWebTools · roslibjs](https://github.com/RobotWebTools/roslibjs)

类别：可视化与 Web。版本/环境：当前官方项目。查阅：2026-10-13。

借鉴：浏览器 ROS 通信已有标准库，无需再造通用客户端协议。

适用限制：不建议仅为了 UI 改版引入整套通用控制桥；不沿用未核验的 ROS 1 action API。

### S34 [W3C WAI · Tabs Pattern](https://www.w3.org/WAI/ARIA/apg/patterns/tabs/)

类别：可视化与 Web。版本/环境：ARIA APG。查阅：2026-10-13。

借鉴：页签明确选中态，支持箭头、Home/End 和可见焦点。

适用限制：页面切换只改变视图，不能隐式执行机器人动作。

### S35 [W3C WCAG 2.2 · 目标尺寸](https://www.w3.org/WAI/WCAG22/Understanding/target-size-minimum.html)

类别：可视化与 Web。版本/环境：WCAG 2.2 AA 解释。查阅：2026-10-13。

借鉴：最低目标尺寸规则为 24×24 CSS px（含例外）；现场主要操作建议至少 44px 高。

适用限制：44px 是本设计的触屏选择，不称为 WCAG AA 一律强制 44px。

### S36 [W3C WCAG · Status Messages](https://www.w3.org/WAI/WCAG22/Understanding/status-messages.html)

类别：可视化与 Web。版本/环境：WCAG 2.2。查阅：2026-10-13。

借鉴：异步结果有文字与可访问状态播报，颜色不是唯一语义。

适用限制：频繁遥测不全部播报；只播报重要动作终态与阻挡变化。

### S37 [MDN · Fetch API](https://developer.mozilla.org/en-US/docs/Web/API/Fetch_API/Using_Fetch)

类别：可视化与 Web。版本/环境：当前官方 Web 文档。查阅：2026-10-13。

借鉴：检查 HTTP 状态与业务结果；AbortController 只终止浏览器等待。

适用限制：abort fetch 不会自动取消后端 ROS goal；POST 不自动重试。

### S38 [MDN · Server-sent Events](https://developer.mozilla.org/en-US/docs/Web/API/Server-sent_events/Using_server-sent_events)

类别：可视化与 Web。版本/环境：当前官方 Web 文档。查阅：2026-10-13。

借鉴：需要推送时可使用单向 SSE，断线后补快照与事件序号。

适用限制：当前轮询已可用；SSE 是后续建议，重连不重发运动请求。

### S39 [ROS 2 · message_filters](https://github.com/ros2/message_filters)

类别：ROS 官方。版本/环境：官方仓库；实现时用 Jazzy 安装源码。查阅：2026-10-13。

借鉴：复用时间同步和 Cache；曝光前后关节状态与同戳掩膜分开核对。

适用限制：同机位多帧不等于多视；slop 配对不是物理同步保证。

### S40 [ROS image_common · image_transport](https://github.com/ros-perception/image_common)

类别：ROS 官方。版本/环境：官方 ROS 图像基础库。查阅：2026-10-13。

借鉴：ROS 高带宽链复用标准图像传输；浏览器获取有界预览缓存。

适用限制：不为每个浏览器额外堆 raw DDS 订户，也不升级 numpy 破坏 cv_bridge ABI。


## 11. 第二轮：完整高密度工作台（2026-10-13）

本轮用户要求布局尽可能展示更多内容，参考央视《创新进行时》20260612《果园里的“特种兵”（二）》。原tv.cctv.cn链接无法读取，tv.cctv.com同视频ID可读取节目标题与简介；未获得逐帧视频内容，因此没有推断节目里具体操作界面、设备参数、剪切动作或成功率。该节目只作果园作业背景参考。[S41]

资料扩展到52项、功能扩展到66项。原第7/10节的48/40项是首轮清单，本节列新增内容并成为总量权威；机器可读design-data.json和交互清单均为52/66。

### 11.1 完整工作台结构

默认打开界面布局，完整高密度模式：顶部固定运行事实条（模式、工具、批次、状态、目标、重建、使能、数据源）；左侧候选/六步与场景、时限、图层；中间双相机预览、图像/深度/曝光证据、模型/几何、三维TCP与工具/轨迹来源；右侧命令条件、工具证据阶梯、拒因和恢复；下方固定运行摘要；工程区同时展现流程泳道、算法耗时、六轴硬件、重建机位、许可身份与余量、节点诊断、记录完整性、事务与审计、回放A/B、参数标定、果园工艺扩展和来源验收。

高密度不靠无限缩小字号：大屏三栏加多面板证据区，中屏把检查区下移，手机按主内容重排，表格/空间图在自己的容器滚动。提供标准密度、检查区/工程区收起和证据类别过滤；选择目标跨面板保持身份，展示选择永不成为执行绑定。保存布局只保存视图，不保存使能/武装/待发指令。[S42 S47 S48 S52]

### 11.2 数据真实性

本稿两张图来自 runs/live_camera_mock_web_full_20261013_1442/perception-before.jpg 与 raw-before.jpg；快照取同目录state-before.json。已明确：这些工件不保证同一曝光。图上target_0是当时检测叠加标签，而快照observations尚未形成锁定列表，不能把画面标签伪装成supervisor绑定目标。

显示的TCP、关节与算法耗时取保存字段；角度从rad换算为度。模型IDLE、未开批、使能关闭、bond关闭、工具反馈未发生、柜侧电流/温度未提供都保持原义。空间网格仅示意投影，不绘制不存在的融合模型、刀轴或规划轨迹。没有时序样本就不造零曲线；耗时条列采样字段，不声称串行占比或相同统计窗口。回放slider只演示游标控件，明确不加载时间序列。

### 11.3 参考适配与扩展边界

UR主工作区/侧栏/运行栏、ABB工艺操作屏用于信息组织。普通网页不继承示教器或专用安全硬件能力。工业HMI告警按原因、对象、时间合并；普通告警ACK与恢复ACK区分。Foxglove变量、布局、状态泳道与比较用于对象联动和复盘。ISA只读取公开范围，没有读取完整付费标准或宣称合规。[S42–S51]

果园工作点、树/行/扇区、容器容量、成熟度、承接和入箱状态作为待接需求，当前固定座产品不显示假导航到位、假满箱率或“袋色=成熟度”。真实剪断与承接仍需独立传感和现场验收，未接时不得让所有完成节点自动变绿。

### 11.4 本轮新增功能

- **F49 高密度工作台与标准密度切换** — 现场作业 / 建议新增 / P0。接口：布局状态，本地保存。验收：密度仅改变排版；关键阻挡和取消位置始终可见。。依据：S42 S48 S52。
- **F50 概览/对象/证据三层钻取** — 现场作业 / 建议新增 / P0。接口：共享目标上下文 +现有state。验收：点击目标联动图像/几何/日志；检查区可收起，不改变执行目标。。依据：S42 S47。
- **F51 告警按目标与稳定原因码合并** — 故障诊断 / 建议新增 / P1。接口：diagnostics / events / failure_code。验收：重复事件只累积次数；保留首次/最近时间，ACK不能替代恢复ACK。。依据：S45 S46。
- **F52 点云/模型/刀具图层显隐与聚焦** — 目标与几何 / 建议新增 / P0。接口：现有3D几何及TF，点云需适配。验收：图层显隐只影响展示；隐藏障碍不删除PlanningScene。。依据：S27 S47。
- **F53 计划/执行/残差/反馈四轨泳道** — 实验验证 / 建议新增 / P1。接口：pipeline+checkpoint+pregrasp+tool反馈。验收：各轨有源时间与单位；阶段跳过不显示成已执行。。依据：S18 S50 S51。
- **F54 用户布局预设与可恢复检查区** — 配置维护 / 建议新增 / P1。接口：仅浏览器布局存储。验收：保存侧栏宽度/面板可见性；不保存使能、武装、待发送命令。。依据：S42 S48。
- **F55 双会话时间偏移与工况一致性** — 记录回放 / 建议新增 / P1。接口：两个MCAP/启动配置/目标身份。验收：明确对齐事件与offset；同样的target_id字符串不足以配对两场目标。。依据：S09 S49。
- **F56 作业点/树/扇区上下文** — 现场作业 / 待研究验证 / P2。接口：需产品作业点IDL与部署适配。验收：当前固定座显示本场；未来导航到位须真实反馈，不用模拟地图宣称已接。。依据：S01 S41。
- **F57 承接、收集位与容器容量** — 现场作业 / 待研究验证 / P2。接口：需承接/容器传感与物流状态接口。验收：区分切断、带离、释放和入箱；未接传感器显示未接，不估计满箱率。。依据：S01 S02 S44。
- **F58 单果/扇区时限与观察预算** — 现场作业 / 已有需整理 / P1。接口：RunHarvest timeout / view_policy。验收：显示剩余预算及耗时统计口径；超时跳过不等于采摘失败自动重试。。依据：S06 S11。
- **F59 候选被筛除原因与排序依据** — 目标与几何 / 已有需整理 / P0。接口：targets_filtered事件/优先级/可达性。验收：未锁定、贴边、无深度、不可达可追踪；网页不重新选出绕门目标。。依据：S06 S47。
- **F60 按动作定位的命令可用矩阵** — 故障诊断 / 建议新增 / P0。接口：真实permissions+debug门+arm拒因。验收：规划/观察/预抓取/接触/工具/恢复分别表达；unknown不能画通过。。依据：S11 S43。
- **F61 记录覆盖率与数据完整性说明** — 记录回放 / 建议新增 / P1。接口：record drops / bag schema / topic范围。验收：漏帧/限速/缺图像时明示；记录缺失不当事件从未发生。。依据：S25 S26。
- **F62 标定样本、独立真值与误差预算页** — 实验验证 / 建议新增 / P1。接口：标定revision/测量样本/geometry工件。验收：列出样本工况与独立测量，不把模型RMSE当刀具对位精度。。依据：S02 S03 S23。
- **F63 启动事实与实际运行参数差异** — 配置维护 / 建议新增 / P1。接口：startup 与实时params镜像。验收：部署默认、当前回读和拟变更分别列；无需重启/需重新建模/需重启分类。。依据：S23 S44。
- **F64 资源/吞吐与观察质量分离** — 故障诊断 / 已有需整理 / P0。接口：metrics+timing+接收探针+record。验收：设备采集率、节点到达率、推理率、网页刷新率分开，未测量不猜。。依据：S13 S24 S51。
- **F65 面板字段来源与能力接线索引** — 架构维护 / 建议新增 / P0。接口：schema/api/IDL/manifest索引。验收：每块显示已有/待适配/需硬件/预留；不把设计控件当已接后端功能。。依据：S23 S44。
- **F66 交班与退出前检查摘要** — 配置维护 / 建议新增 / P1。接口：在途动作/恢复锁/录制/运行事实。验收：退出页面不表示任务停止；确认动作终态、停栈和录制收口分别检查。。依据：S17 S24 S41。

### 11.5 本轮新增参考

- **S41 [央视《创新进行时》果园里的“特种兵”（二）](https://tv.cctv.com/2026/06/12/VIDEKwa7XMM4x7PWXtDgUVHb260612.shtml)** — 2026-06-12；已读取节目页与简介。借鉴：把果园作业组织、人工负担与采收效率纳入产品需求背景。 限制：未获得视频逐帧画面；不是设备界面规格、算法性能或安全验收证据。
- **S42 [UR PolyScope X · 界面分区](https://www.universal-robots.com/manuals/EN/HTML/SW10_11/Content/prod-usr-man/complianceUR15/PolyX_SW/psx_polyX_overview.htm)** — SW10.11 官方用户手册。借鉴：固定系统头栏、主工作区、可收起的多任务侧栏与运行控制栏。 限制：只借鉴信息组织；不把UR示教运动/上电按钮复制到peach网页。
- **S43 [UR PolyScope X · Operator Screen](https://docs.universal-robots.com/PolyScopeX_SDK_Documentation/build/SDK-v0.21/HowToGuides/operatorscreen.html)** — SDK v0.21 官方文档；通过官方版本导航核对。借鉴：工艺操作屏展示受控配置与程序状态；自动运行时限制手动操作。 限制：借鉴权限与模式分离；实现必须继续由supervisor和arm强制。
- **S44 [ABB OmniCore UI / AppStudio](https://www.abb.com/global/en/areas/robotics/products/controllers/omnicore/omnicore-ui)** — ABB 当前官方产品说明。借鉴：针对工艺定制操作屏，操作/监控/维护使用一致的界面语言。 限制：ABB安全硬件/E-Device能力不属于普通peach浏览器，也不证明本应用达到对应标准。
- **S45 [ISA101 · HMI 委员会公开范围](https://www.isa.org/standards-and-publications/isa-standards/isa-standards-committees/isa101)** — 公开范围说明；未读取付费标准。借鉴：导航、显示、告警、帮助、历史与配置需要统一设计治理。 限制：仅引用公开范围方向，不宣称已读取完整标准或ISA101合规，不复制标准条款。
- **S46 [Siemens · Alarm Dashboard](https://www.siemens.com/en-gb/products/dwg-alarm-dashboard/)** — 官方产品说明；检索摘要已核对。借鉴：告警按原因、对象和时间分组，过滤重复，提供复盘与报告。 限制：借鉴告警组织，非引入Siemens软件；诊断ACK不能解除机器人恢复锁。
- **S47 [Foxglove · Variables](https://docs.foxglove.dev/docs/visualization/variables)** — 当前官方文档。借鉴：共享选中对象变量，让图像、3D、曲线、原始消息一起切换目标。 限制：视图选中与执行绑定分开；跨epoch不能沿用目标选择。
- **S48 [Foxglove · Layouts](https://docs.foxglove.dev/docs/visualization/layouts)** — 当前官方文档。借鉴：保存多面板布局和用户工作区，按作业/调试/复盘组织信息。 限制：保存布局不保存待发送运动指令，也不恢复执行开关。
- **S49 [Foxglove · Comparison Mode](https://docs.foxglove.dev/docs/visualization/comparison-mode)** — 当前官方文档。借鉴：复盘时对照两个数据源，明确偏移与对齐基准。 限制：时间对齐不证明同一果实或同一工况，需额外身份与条件约束。
- **S50 [Foxglove · State Transitions](https://docs.foxglove.dev/docs/visualization/panels/state-transitions)** — 当前官方文档。借鉴：离散阶段用状态泳道显示，便于定位等待、切换和拒绝。 限制：批次FSM与臂阶段分轨，不用一个进度百分比替代全部状态。
- **S51 [Foxglove · Plot](https://docs.foxglove.dev/docs/visualization/panels/plot)** — 当前官方文档。借鉴：用有单位的时序曲线观察残差、延迟、资源与工具反馈。 限制：没有样本就不画零值趋势；插值或滤波不能伪造物理剪断事件。
- **S52 [W3C WCAG · Reflow](https://www.w3.org/WAI/WCAG22/Understanding/reflow.html)** — WCAG 2.2 官方解释。借鉴：窄屏重排而不是整体缩小，二维场景/表格在各自容器滚动。 限制：高密度设计仍需可读与键盘操作，不能靠小字号强塞所有面板。
