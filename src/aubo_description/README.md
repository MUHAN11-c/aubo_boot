# aubo_description

AUBO E5 工作单元 URDF / xacro / mesh。采摘栈通过 `robot_state_publisher` 消费展开后的 `robot_description`。包职责见 [docs/architecture.md](../../docs/architecture.md) §3 驱动层。

## 关键文件

- `urdf/aubo_e5.urdf.xacro`：整机描述
- `urdf/components/tcp.xacro` + 三个 wrapper（`tcp_{shear_v1,bite_shear_v1,adaptive_shear_v1}.xacro`）+ 对应 `config/<profile>.yaml` 档案：三把剪切手（2026-09-28 起替换两把套袋圆柱）；帧名冻结 `tool_axis/cutting_plane/tcp/sleeve_mouth/tool_body_link`，TCP 姿态 Z=开口、XY=刀口；网格在法兰系（=wrist3_Link 系）下建模、tool_body_link 挂 parent 落位
- `urdf/aubo_e5.ros2_control.xacro`：**只读**。按 `hardware_mode` 选 mock / real 插件并填 `robot_ip`
- `scripts/step_inspect.py` / `step_extract_tool.py`（FreeCAD snap 无头）/ `decimate_export.py` / `step_render_check.py`（Blender 4.5 无头）：STEP 总装→法兰系工具网格的可复现管线（对齐验收：CAD 原点=wrist3_Link 原点已数值+渲染双重验证）

## 关节顺序（权威）

`shoulder_joint, upperArm_joint, foreArm_joint, wrist1_joint, wrist2_joint, wrist3_joint`

本包不含控制器与运动逻辑。改几何会同时影响 MoveIt、手眼 TF 和碰撞。
