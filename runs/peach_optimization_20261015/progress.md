# Peach 优化实施记录 — 用户批准方案 2026-10-15

- 基线：保留当前未提交修改，before.patch 与 git_status.txt 已保存。
- 执行方式：当前工作区内分阶段实施，不提交无关改动，不创建平行实现。
- 阶段：1 基线进行中；2 身份/许可待修；3 职责拆分待做；4 Web/RViz待做；5 文档待做；6 全量验收待做。
- 安全：硬件模式 mock；真实相机只读；tool=false；仅清理本轮所属进程。
- 基线已知：上轮 FULL 2/3；第二轮模型/许可乱序；RViz Ogre GL buffer 异常。

## 阶段进展
- 臂侧：7个新增回归RED→GREEN，TargetCache 19/19；完整原生行为CTests 15/15通过。
- 拆分：target_observers.cpp负责消息解码，status.cpp负责诊断/完成证据；接口与运动逻辑不变。
- 模型时间：数据年龄改用冻结生成时间；决策原子提交，FULL有界等同目标许可；同版本不续签。
- Web：app.js 1505→418行；5文件93项断言通过；HTTP静态白名单已联动更新。
- Python：425行为测试通过；同版本模型/许可统一冻结时刻，单独机位最多MEDIUM。
- 文档：9包架构+README+测试指导重写，最终联合入口待补。
- Ruling: 使用既有工作区并保存before.patch，不创建不含当前修复的独立checkout；当前为用户测试分支，保留全部既有修改。
- Ruling: Python第一轮baseline因pytest同名测试与全仓lint采集问题未完成；python_baseline.log包含修改后的测试，不冒充性能前基线。

- 实际短测：相机连接成功，startup状态广播缺陷阻断启动；已精确SIGINT收尾，residual_pids=[]。发现rclpy默认SIGINT提前shutdown导致disable无法发，runner已改自管信号，纯回归19通过。
- 一轮正式colcon9包：8包绿，harvester仅import顺序失败，修正后剩docstring D213已改；arm重跑仅sandbox XML schema访问失败，escalated重跑已通过。无质量门放宽。
- 关停：E1复现std::thread原生join后仍joinable导致terminate；改future ready + std::thread::join，新增2回归通过，E1与3工具档位系统复核通过。
- 默认流程：既有并行Build路径成为默认，避免真实冻结生成时间在串行observe内老化；新鲜度/碰撞/能力门未降。
- 用户补充2026-10-15 09:27：工具自身Z轴尽量少拧腕；有效IK与走廊筛后先abs roll再加权jointcost，原0/±30/±60扫描不扩大；RED→GREEN。
- 真实短测live_short_parallel：1组模型/1次level6/5s RViz与流检查通过，清理residual=[]。该短测诊断epoch错误已修，模型统计为空不可用，正式验收将用更严格完整身份重跑。
- 性能：冻结8帧×6计时，几何纯核P95 67.449→54.179ms；源码SHA相同，不能归因为算法提速。范围不含YOLO/SAM/ROS/规划。
- 用户最终滚转要求2026-10-15 09:33：工具自身Z额外滚转最多45°；覆盖之前60候选，统一表0/±15/±30/±45，整体倾斜测地线110°门不混同。
- 最终审查修复：Web验收匹配完整7元身份+冻结生成/有效时间；统计真实decision entry/cut/axis，拒零轴/零行程/非正径；frozen清理失败不可exit0。
- Python世代：canonical observation/Build goal提供epoch，旧调度镜像不能倒写；跨run/epoch清产物即时撤销；TF/prepare/commit复核generation，取消/停用/超时失效。336行为测试通过。
- 回放首次正式失败保留：frozen_final.log exit3，discovered1 attempted0。旧夹具硬设maxtravel20mm挡30mm候选，已删除夹具覆盖并保留产品范围，待重跑。
- 系统最终首轮E1 C03回归：滚转0有效IK但完整PTP被原TCP recede门拒；需候选按absroll排序后逐一完整规划，只有无运动的规划拒绝可重试，取消/授权拒绝不得重试。修复进行中。
- 第一次长测形成优化前基线：30模型完整身份匹配，无Build失败，轴相对首帧最大1.502°/P951.265°，入口最大单轴range3.88mm，剪切2.12mm，袋径3.34mm；model P9517.09s/max46.72s。已1次level6，第二次运行中由本轮runner SIGINT受控结束，目的为装入FrameStore/直线优先新变更；不将其计作最终十轮或长稳验收。
- 真实相机Web图片640×480加载正常，放大/关闭可操作，控制台error=[]；8视图响应式操作控件无水平溢出；截图web_camera_enlarged.png。
- 用户09:50轨迹要求：尽可能直线或小圆弧；修复LIN几何合格仍先走PTP的错误顺序，改LIN完整规划先行。套入/撤退沿轴直线；PTP备用+2cm圆滑连接原有，不虚称精确CIRC。
