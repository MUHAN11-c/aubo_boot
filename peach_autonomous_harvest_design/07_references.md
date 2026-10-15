# 07 整体设计参考

24项主参考用于整体职责、成熟组件、闭环和界面组织。作者农业论文借鉴工艺与系统经验，不将作物条件和研究成功率迁成peach指标。央视节目仅核对节目页简介，未逐帧查看；不据此推断具体界面。前两轮52项细节参考另存于背景材料，本阶段不继续深挖算法/API。

## R01 [Nav2 · 导航与应用调度层级](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/)

采用方向：导航任务位于系统自主任务之下；应用决定导航、作业和资源处置。

适用边界：用于模块与流程规划；不在本阶段选择算法、参数或具体调用方式。

## R02 [Nav2 · Waypoint Follower](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/waypoint_follower/)

采用方向：参考按点推进与到位作业；复杂果园任务仍需应用级编排。

适用边界：用于模块与流程规划；不在本阶段选择算法、参数或具体调用方式。

## R03 [Nav2 · Docking](https://docs.nav2.org/jazzy/tutorials/general_tutorials/using_docking/)

采用方向：成熟站点对接能力；实际补能/卸载反馈需要设备与产品适配。

适用边界：用于模块与流程规划；不在本阶段选择算法、参数或具体调用方式。

## R04 [REP-105 · 移动平台坐标系](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst)

采用方向：统一地图、里程计与平台坐标系语义，明确移动后场景和数据作用域。

适用边界：用于模块与流程规划；不在本阶段选择算法、参数或具体调用方式。

## R05 [Nav2 · Jazzy 源码](https://github.com/ros-navigation/navigation2/tree/jazzy)

采用方向：定位、导航、避障、任务和生命周期已有成熟组件；实施时核对本发行版。

适用边界：用于模块与流程规划；不在本阶段选择算法、参数或具体调用方式。

## R06 [SWEEPER · 欧盟官方结项报告](https://cordis.europa.eu/project/id/644313/reporting)

采用方向：采摘系统包括检测、接近、切割、承接、放置与人机界面；按遮挡和栽培条件报告结果。

适用边界：历史温室甜椒项目；不证明套袋桃精度，也不采用其 ROS Indigo 接线。

## R07 [Harvey · 甜椒抓持与剪切系统](https://arxiv.org/abs/1810.11920)

采用方向：抓持与剪切位置分别估计，工具解耦；分别记录感知和脱离失败。

适用边界：改造场景下的试验结果不能作为 peach 成功率目标；论文不是可直接部署的 Jazzy 包。

## R08 [HortiBot · 主动感知与双臂剪切](https://arxiv.org/html/2403.15306v1)

采用方向：分别检测果实、果梗与主茎，主动观察、力反馈与抓持/剪切/放置阶段。

适用边界：三臂室内甜椒实验；其缺果梗时固定切点兜底不迁入 peach 真机默认流程。未确认可复用的完整开源控制栈。

## R09 [OSUrobotics/apple-harvest](https://github.com/OSUrobotics/apple-harvest)

采用方向：分阶段使能、控制策略选择、分阶段录制、冻结视觉数据与仿真。

适用边界：苹果拉扭/吸附采摘，不是剪切套袋桃；README 不构成 Jazzy 兼容或现场安全保证。

## R10 [ROS 2 Actions · 官方设计](https://design.ros2.org/articles/actions.html)

采用方向：长任务用 Goal / Feedback / Result / Cancel；取消请求与终态分别确认。

适用边界：设计规范不是当前 Python API 手册；实现时仍核对 Jazzy 源码。

## R11 [ROS 2 Managed Nodes · 官方设计](https://design.ros2.org/articles/node_lifecycle.html)

采用方向：configure 建资源，activate 运行；状态转换与资源释放可观测。

适用边界：生命周期 Active 不表示进程一直存活，需心跳或 watchdog。

## R12 [Nav2 Lifecycle Manager](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/configuring_lifecycle_manager/)

采用方向：有序启动、反向关闭，bond 断联后降级托管栈。

适用边界：peach 当前 bond_timeout 默认为 0；页面不得宣称已开启进程死检。

## R13 [Nav2 Collision Monitor](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/)

采用方向：应用过滤应处于命令输出末端；官方明确没有硬实时安全认证。

适用边界：不能当臂与刀的硬件急停，也不建议把底盘速度节点直接套到 AUBO。

## R14 [UR Driver · Robot State Helper](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/robot_state_helper.html)

采用方向：保护停/急停释放后继续程序可能恢复旧动作；故障恢复先停止再重新下发。

适用边界：仅借鉴恢复语义；本仓不接 dashboard、不远程上电或解锁。

## R15 [MoveIt 2 · Task Constructor 教程](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html)

采用方向：任务拆成阶段，分别展示规划解、失败解和执行结果。

适用边界：设计借鉴阶段内省；不把 Rolling 示例直接当 Jazzy API。

## R16 [ros2_control · Mock Components](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html)

采用方向：GenericSystem 镜像指令到状态，验证接线与 MoveIt 集成。

适用边界：理想镜像不验证碰撞力、剪断、电流、真实急停或刀具反馈。

## R17 [Autoware · Fail-safe 命令门](https://docs.autoware.org/main/design/autoware-architecture-v1/interfaces/ad-api/features/fail-safe/)

采用方向：故障状态、处置进行中和处置结果分别表达；最终命令门统一仲裁。

适用边界：只借鉴架构，不能把车辆 MRM 状态机械映射成臂安全等级。

## R18 [rosbag2 · Jazzy](https://github.com/ros2/rosbag2/tree/jazzy)

采用方向：复用记录、回放与存储插件，隔离回放域。

适用边界：回放原动作/IO 话题不得注入真机域；时间倒退需重置显示缓存。

## R19 [MCAP · 官方指南](https://mcap.dev/guides)

采用方向：用现有 MCAP 保存带 schema 的机器人多模态数据。

适用边界：不为 Web 重写 bag 存储；压缩图像只能作为预览，不能充当精确深度。

## R20 [UR PolyScope X · 界面分区](https://www.universal-robots.com/manuals/EN/HTML/SW10_11/Content/prod-usr-man/complianceUR15/PolyX_SW/psx_polyX_overview.htm)

采用方向：固定系统头栏、主工作区、可收起的多任务侧栏与运行控制栏。

适用边界：只借鉴信息组织；不把UR示教运动/上电按钮复制到peach网页。

## R21 [UR PolyScope X · Operator Screen](https://docs.universal-robots.com/PolyScopeX_SDK_Documentation/build/SDK-v0.21/HowToGuides/operatorscreen.html)

采用方向：工艺操作屏展示受控配置与程序状态；自动运行时限制手动操作。

适用边界：借鉴权限与模式分离；实现必须继续由supervisor和arm强制。

## R22 [ABB OmniCore UI / AppStudio](https://www.abb.com/global/en/areas/robotics/products/controllers/omnicore/omnicore-ui)

采用方向：针对工艺定制操作屏，操作/监控/维护使用一致的界面语言。

适用边界：ABB安全硬件/E-Device能力不属于普通peach浏览器，也不证明本应用达到对应标准。

## R23 [Foxglove · Layouts](https://docs.foxglove.dev/docs/visualization/layouts)

采用方向：保存多面板布局和用户工作区，按作业/调试/复盘组织信息。

适用边界：保存布局不保存待发送运动指令，也不恢复执行开关。

## R24 [央视《创新进行时》果园里的“特种兵”（二）](https://tv.cctv.com/2026/06/12/VIDEKwa7XMM4x7PWXtDgUVHb260612.shtml)

采用方向：把果园作业组织、人工负担与采收效率纳入产品需求背景。

适用边界：未获得视频逐帧画面；不是设备界面规格、算法性能或安全验收证据。


## Web与调试专项参考

- [Foxglove Layouts](https://docs.foxglove.dev/docs/visualization/layouts)：多角色/模块工作区与复用布局；许可/部署条件另核对。
- [Foxglove Playback](https://docs.foxglove.dev/docs/visualization/playback)：统一时间游标、事件跳转与多模态回放。
- [PlotJuggler](https://github.com/PlotJuggler/PlotJuggler)：工程时序诊断可复用成熟工具，不在主Web重造全部分析能力。
- [Groot2](https://www.behaviortree.dev/groot/)：采用BehaviorTree.CPP的模块可做树内省；不强制将全项目改为BT，版本/许可核对后再接。
- [RViz Jazzy](https://github.com/ros2/rviz/tree/jazzy)：ROS空间、规划与导航研发验证；与业务工作台互补。

## docs本地方案资料归并

- [内容合并与采摘核基线](10_docs_merge_and_baseline.md)：v1各专题的模块、页面与验收去向；历史问题作为待核线索。
- [原docs资料归档](archive/docs_2026-10-13/README.md)：两份原方案及全部22个图件/图源，原字节保留。v1作历史基线，v2仅归档、暂不采用。
- [原始文件校验与章节映射](docs-merge-record.json)：保留原路径、新路径和SHA-256，便于追溯。

原方案中的外部参考及当时核验日期仅保留历史来源，本轮未重新浏览或验证这些外链。
