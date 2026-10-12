# peach2_interfaces

v2 孵化采摘栈的 ROS 2 契约包，包含 11 个消息、5 个服务和 4 个可取消动作。只生成接口并安装图契约，不运行节点。v1 的 `peach_interfaces` 与本包是两个版本的独立契约；不要跨版本混用。

## 公有 API

- `msg/`：目标观测、模型、抓取决策、使能、工具状态与批次状态。
- `srv/`：开场景、生成场景快照、可达性查询、读取决策、设置使能。
- `action/`：`RunBatch`、`ObserveTarget`、`HarvestTarget`、`MoveTo`。
- [config/interfaces.yaml](config/interfaces.yaml)：图名、类型、生产者/消费者和 QoS 的单一清单；也记录使用标准 `std_msgs` / `std_srvs` 的接口。

字段含义、单位和帧以各 IDL 定义为准。改接口须同时更新清单与所有使用端；观察结果和抓取许可不能当作绕过臂侧命令门的运动授权。

## 构建、使用与验证

在工作区根目录 source Jazzy 后执行：

```bash
colcon build --packages-select peach2_interfaces
source install/setup.bash
ros2 interface show peach2_interfaces/action/RunBatch
colcon test --packages-select peach2_interfaces
colcon test-result --verbose
```

生成接口可由 Python `from peach2_interfaces.msg import TargetModel` 或 C++ `#include "peach2_interfaces/msg/target_model.hpp"` 使用。跨包验证按 [v2 测试指导](../peach2_system_tests/doc/testing.md) 执行；v2 仍须通过后续验收门才能替换生产入口。

## 文档与许可

本页、IDL 与清单随源码维护，无独立文档生成步骤。包元数据声明 BSD-3-Clause，许可文本见 [工作区 LICENSE](../../../LICENSE)。
