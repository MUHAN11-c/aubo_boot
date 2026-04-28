# tool_changer_interface — 快换与手爪管理接口定义

ROS 2 接口定义包，为 `tool_changer` 包提供消息与服务类型。

## 消息

| 消息 | 说明 |
|------|------|
| **ToolChangerStatus** | 快换盘状态：工具 ID / 名称 / 类型 / 连接状态 / 参数 JSON |
| **ToolIOStatus** | 工具端 IO 状态：数字量 + 模拟量输入/输出数组 |

## 服务

| 服务 | 说明 |
|------|------|
| **RunGripperSwap** | 执行快换操作，direction: `gripper0_to_gripper2` / `gripper2_to_gripper0` / `gripper2` |
| **ChangeTool** | 按 tool_id 切换工具 |
| **GetCurrentTool** | 查询当前工具 ID / 名称 / 类型 / 参数 |

## 依赖

- `std_msgs` — 标准消息头
- `rosidl_default_generators` — 代码生成

## 编译

```bash
colcon build --packages-select tool_changer_interface
```
