# peach_interfaces 架构

## 职责

能力间唯一自定义 IDL 契约；标准 ROS 消息优先。无节点、无 launch、无算法实现。

## 源码地图

`msg/`、`srv/`、`action/`：字段、常量、单位和语义；`config/interface_manifest.yaml`：图名、QoS及生产消费方；`scripts/check_interface_manifest.py`：清单双向核对；`CMakeLists.txt`：rosidl 生成。

## 入口、输入输出与参数

主要长任务 RunHarvest/SurveyScene/BuildTargetModel/ExecuteTarget/MoveTo；短查询和开关 BeginScene/CheckReachability/ControlTask/SetEnables；观测、模型、许可和作业消息见 [README](../README.md) 与 [io](io.md)。无运行参数。

## 生命周期、并发与失败

类型包无生命周期或线程。身份 target_id、批次 request/run/cycle、模型修订与 clearance 需匹配，冻结 valid_until 不续签。reserved_interfaces 仅保留名字，不代表存在服务端。失败码与业务成功由生产端填，action SUCCEEDED 不替代结果字段。

## 验证与限制

`colcon test --packages-select peach_interfaces --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。
