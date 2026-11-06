# peach_common 架构

## 职责

共享基础库；不运行节点，不定义 ROS 业务 IDL。参数、工具档案注入及记录会话
文件约定由本库供调用包复用。

## 源码地图

`peach_common/yaml_params.py`：读取 YAML、声明叶子、预览/提交参数；`param_rules.py`：范围及使能依赖校验；`qos.py`：传输档；`paths.py`：runs 根及路径防御；`lifecycle.py`：自激活与 bond；`ros_log_bridge.py`、`event_meter.py`：日志与计数。`grasp_standoffs.py` 读取共享视觉后撤并生成参数 overlay；`tool_profiles.py` 从 aubo_description 校验/加载工具档案并注入；`recording_session.py` 原子写写者状态并验证唯一 active 会话。

## 入口、输入输出与参数

能力包调用 Python 函数；没有独立节点的 topic/service/action。`lifecycle` helper
可为调用节点创建 `/bond` 心跳。共享视觉几何在 `config/grasp_standoffs.yaml`，
工具档案在 aubo_description；节点运行参数仍由调用包声明。公共 API、导入与
文件边界见 [io.md](io.md)。

## 生命周期、并发与失败

不拥有 executor；生命周期与 bond 句柄由调用节点创建和销毁。QoS 模块延迟导入 rclpy；工厂调用时才需要 rclpy。工具档案纯解析可离线执行，standoff 模块导入依赖 ament_index_python。`ensure_within` 由 writer 显式调用，不是所有路径 API 自动保证。bond 缺依赖会告警降级，不等于心跳保护有效。

`yaml_params.attach` 的 on-set 回调只校验单项值和试算副本，不修改实时命名空间或执行 `on_commit`。只有 ROS 参数事务全部通过后，post-set 回调才原地更新缓存并调用一次 `on_commit`；其他验证器拒绝时 ROS 值、缓存与提交副作用都保持不变。

## 验证与限制

`colcon test --packages-select peach_common --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。
