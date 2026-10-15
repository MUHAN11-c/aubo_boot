# peach_common 架构

## 职责

共享基础库；不运行节点、不承载业务契约。

## 源码地图

`peach_common/yaml_params.py`：读取 YAML、声明叶子、预览/提交参数；`param_rules.py`：范围及使能依赖校验；`qos.py`：传输档；`paths.py`：runs 根及路径防御；`lifecycle.py`：自激活与 bond；`ros_log_bridge.py`、`event_meter.py`：日志与计数。

## 入口、输入输出与参数

能力包调用 Python 函数；没有 ROS topic/service/action 或独立运行参数。配置由调用包提供，不在本库复制业务默认值。

## 生命周期、并发与失败

不拥有 executor；生命周期与 bond 句柄由调用节点创建和销毁。QoS 模块延迟导入 rclpy；纯核不要求启动 ROS。参数校验失败返回拒因，路径写入前校验包含关系；bond 缺依赖会告警降级，不等于心跳保护有效。

## 验证与限制

`colcon test --packages-select peach_common --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。
