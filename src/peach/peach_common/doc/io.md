# peach_common IO 与公共边界

本包是 Python 库，不提供独立 ROS 节点或业务 topic/service/action。顶层
`peach_common` 重导出 yaml_params 与 lifecycle API，其他功能从子模块导入。

## 参数与几何输入

`yaml_params.attach(node, yaml_path, ...)` 读取 ROS YAML 叶子，已有 launch
覆盖优先；`validate` 校验单值，`preview` 校验试算副本。on-set 不提交副作用，
ROS 事务成功后 post-set 才原地更新命名空间并执行一次 `on_commit`。
`on_commit` 不在启动时调用。GPL 消费节点使用各自 ParamListener，不经本函数。

`grasp_standoffs.load_grasp_standoffs()` 读取 share 中的
`config/grasp_standoffs.yaml`，返回入口相对袋底与视觉预抓取相对入口后撤。
scene/reconstruction/manipulation overlay 将米数注入调用包的已声明参数。
文件是普通键值 YAML，不能直接当 ROS ParameterFile；arm 独有轨迹净空
`moveit.approach_clearance_m` 不在本文件中。

`tool_profiles` 从 `aubo_description/config/<profile_id>.yaml` 校验/读取档案；
`parse_tool_profile` 不依赖 ROS，加载与 launch 替换在调用时导入 ament/launch。
仅注入源码明确消费的 profile_id、内径、行程/刀口轴向与工具包络字段，
不自动注入档案 IO 登记项。档案加载按 profile_id 进程内缓存。

## 传输与生命周期

`qos.latched()` 为 RELIABLE/TRANSIENT_LOCAL depth1；`stream`/`reliable`
为 RELIABLE/VOLATILE KEEP_LAST；`sensor` 为 BEST_EFFORT/VOLATILE。
具体话题仍须遵守接口清单，不能用工厂默认值替代各节点接线约定。

`ensure_lifecycle_active` 只尝试 unconfigured→inactive→active，不创建 executor。
`create_bond` 为调用节点建立默认 `/bond` 心跳；收包仍需节点 spin，调用方负责
适时 `break_bond`。缺 bondpy 返回 None 并告警。

## 文件、会话与日志

`paths.runs_root` 优先取显式绝对路径，再取 AUBO_RUNS_DIR /
AUBO_HARVEST_DATA_DIR，最后探测工作区或回退 cwd/runs。
`safe_component` 只净化目录段；`ensure_within` 对实际路径解析后检查包含性，
由 writer 自行调用，不能宣称抵抗检查之后的并发路径替换。

`recording_session.write_session_state` 使用临时文件、fsync 与 replace 写
`session.json`，带 session_id、state、owner_pid、Linux 进程起始 tick 和更新
墙钟。`active_session` 只返回唯一 active 且 PID/start tick 仍匹配的会话；
不存在或有多个活写者时返回 None，不依据 bag mtime 选择。

`RosLoggerHandler`/`bridge_module_loggers` 将指定 stdlib logger 转发进调用节点
日志；同一 logger 不重复挂 handler。`EventMeter` 第一次及每 every 次发摘要，
默认 WARN，级别可配置。公开函数清单见 [README](../README.md)。
