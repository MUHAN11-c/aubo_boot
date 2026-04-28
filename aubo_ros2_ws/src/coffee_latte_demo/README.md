# coffee_latte_demo — 咖啡拉花演示

自包含的咖啡拉花演示包：ROS2 IO 控制节点（对接 Aubo 驱动硬件 IO）、Web 前端面板、工具网格文件。

## 节点

### latte_node

| 属性 | 值 |
|------|-----|
| 功能 | 咖啡拉花 IO 控制：通过 `/aubo_driver/set_io` 驱动 DO2/DO4，订阅 `/aubo_driver/io_states` 反馈 DI2/DI3/DI4 |
| 依赖 | rclpy, std_msgs, std_srvs, demo_interface |

**IO 映射**（Aubo 驱动自动将 `io_index` + 32 映射到硬件引脚）：

| 逻辑 IO | 服务/话题 | 硬件引脚 |
|---------|----------|---------|
| DO2（打花开关） | `/set_latte_do2` → `/aubo_driver/set_io(io_index=2)` | 34 |
| DO4（咖啡开关） | `/set_latte_do4` → `/aubo_driver/set_io(io_index=4)` | 36 |
| DI2（咖啡反馈） | `/aubo_driver/io_states.digital_inputs[2]` | 34 |
| DI3（打花反馈） | `/aubo_driver/io_states.digital_inputs[3]` | 35 |
| DI4（警告反馈） | `/aubo_driver/io_states.digital_inputs[4]` | 36 |

> 若 `demo_interface` 不可用，节点将降级为纯内存状态机（不影响启动）。

### 话题

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/latte_di_status` | std_msgs/String | 每 5s 发布 DO2/DO4 + DI2/DI3/DI4 综合状态 |

### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/set_latte_do2` | std_srvs/SetBool | 打花开关（true=开 / false=关） |
| `/set_latte_do4` | std_srvs/SetBool | 咖啡开关（true=开 / false=关） |

## 运行

```bash
ros2 run coffee_latte_demo latte_node
```

## 网格文件

包含 gripper1 咖啡拉花工具附件的 STL 网格（从 aubo_description 复制）：

| 网格文件 | 说明 |
|----------|------|
| `gripper1coffeecup_link.stl` | 咖啡杯工具（视觉 + 碰撞） |
| `gripper1milkcup_link.stl` | 奶杯工具（视觉 + 碰撞） |
| `coffee_link.stl` | 咖啡主体链接 |
| `cup0_link.stl` | 杯子链接 |

原网格文件仍保留在 `aubo_description/meshes/` 供 URDF 引用。

## 前端面板

`web/public/coffee_latte_panel.html` — 咖啡拉花 Web 控制面板：
- 机械臂 URDF 3D 模型
- 咖啡流程示意（出杯/定位 → 萃取与奶泡 → 拉花执行）
- DO2/DO4 开关按钮（通过 `latte_io.js` 绑定 rosbridge → `/set_latte_do2` / `/set_latte_do4`）
- DI 反馈信号灯（订阅 `/latte_di_status`，实时显示 DI2/DI3/DI4 状态）
- 关节曲线与末端位姿监控

前端面板由 `aubo_ros2_web_dashboard` 的 FastAPI 网关统一 serve，本包保留 canonical 副本。`latte_io.js` 模块在页面加载时自动初始化 IO 绑定。
