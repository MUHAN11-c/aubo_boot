# 统一机器人模型与资产口径

当前模型是 `harvester_unified.urdf.xacro`：`platform_base_link` 为根，车体、两履带、激光支架、AUBO E5、相机、快换、三选一工具与收集筐构成一棵树。`harvester_robot.urdf.xacro` 是历史两模式源码，当前入口不再调用。外部 `aubo_description`、工具档案、相机标定与驱动栈只读。

## 装配与物理代理

- AUBO 六关节 origin、axis、limit、mass 和惯量与只读来源逐项对照；关节序冻结。臂链只移除来源的 `world/world_joint`，以固定安装座接入平台；三处极小惯性字面量漂移已对齐。
- BUNKER GLB 已是 Z-up，默认包络为 1.05×0.71×0.54 m，底面 z=0。它按项目名义包络缩放，不是厂商实机尺寸复刻。完整网格已含履带，不再重复叠加履带 box 外观；box 碰撞与惯性仍保留。
- 激光位于 `(0.58,0,0.45) m`，使用接入车体前壁的水平支架；删除原先从传感器延伸到地面的视觉长杆。收集筐固定在 `(-0.34,0,0.55) m`，底板下表面与 0.54 m 车体顶面接齐；惯性按五面均匀板代理计算，重心在筐原点上方约 45.1 mm。
- 平台质量、摩擦与 VelocityControl 是工程代理：直接控制机体速度、履带零摩擦，不验证真实履带牵引、侧滑、悬架、倾覆或承载。工具 CAD 和质量沿用只读描述，不宣称已完成实物称重。
- 三档工具均应包含 `tcp`，其变换必须等于对应工具档案；未知 `tool_profile` 在 xacro 阶段拒绝，不能生成缺末端帧的半成品。

## 相机与可见性

固定标定 TF 由 RSP 单一发布；`wrist3_Link → camera_link → camera_color/depth_frame → *_optical_frame` 与 active 标定一致。Gazebo RGB-D 挂在保留下来的 `wrist3_Link`，传感器局部 +X 对应 ROS 光学 +Z；固定关节合并不能吞掉传感器。激光合并到平台根后保留原位置和 `lidar_mount` 帧标识。

相机 CAD 以实体网格表达，未建真实透明光学孔；离线射线证明光心位于壳体/安装板网格内部，但这不证明标定错误，也不等于 Gazebo 成像验收。恢复腕部与相机外观可见，取消全局透明/隐藏。Gazebo sensors 8 的 RGB-D 源码未读取 `Camera::VisibilityMask()`，不能把只改 mask 当作有效去遮挡方案。

RGB-D 使用 [PS800-E1 厂商数据表](https://en.percipio.xyz/wp-content/uploads/sites/2/2023/12/3D-Industrial-Camera-PS800-E1-EN.pdf) 的工作范围 0.2–1.0 m、水平视场 63°、最大 2.5 Hz，输出降采样至 640×480。它是理想针孔 RGB-D 代理，不复刻标定内参、RGB/IR基线、畸变、结构光噪声、反光或真实遮挡材质；视觉闭环需重新独立验收。

## 碰撞与导航包络

MoveIt 与 RSP 使用同一 URDF；Gazebo 从该 URDF 离线转换 SDF。MoveIt 负责规划碰撞，Gazebo 默认未启用机器人自碰撞物理，不将两者混同。

`config/assembly.srdf` 仅补充平台与五个固定安装邻接件的允许接触对，和只读臂 SRDF 合并；其余碰撞对保持检查。2026-10-29 使用 MoveIt 2.12.4 原生 PlanningScene/FCL 离线核对三工具完整机体：`harvest_stow` 均为 `self_collision=0, bounds=1`。这证明所检姿态，不涵盖所有运动轨迹。

固定剪切手没有开合自由度，持果用原生 DetachableJoint 表达。仅工具碰撞 mask=1、动态袋 mask=2、其余机器人（包括筐）mask=3：工具与袋在释放时不接触；不额外过滤工具与筐。这是刚性抓取/释放近似，不验证剪切力学。

三档名义 `harvest_stow` 的合并碰撞包络为 X `[-0.525,0.605]`、Y `[-0.479694,0.355]` m。每档在六关节 ±0.08 rad 收臂容差内检查 64 个角点和固定种子 4096 个内部样本，负 Y 最远达到 `-0.513662 m`。Nav2 local/global footprint 均为 X ±0.64、Y ±0.56 m；只有收臂且状态新鲜才允许底盘运动。抽样加余量是工程证据，不能当作连续区间、制动距离或形变的数学保证。[Nav2 Jazzy footprint 指南](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/) 要求包络反映实际机体形状。

## 场景资产与离线核对

袋 GLB 是旧 Y-up 资产；由 `sim_worlds/mesh_geometry.py` 保存 SHA256、实测包络、袋底和扎丝平面，并统一变换。默认袋宽 75 mm、厚 60 mm、底到扎口 90 mm；不规则上沿使完整视觉高约 131.78 mm，不能再用旧 80 mm 高盒代替。碰撞盒、COM、均匀盒惯量、manifest 与 MoveIt 代理同源。内果经非均匀缩放后是椭球，不能继续宣称直径 55 mm 的标准球。

项目根离线执行（不启动 ROS 节点或 Gazebo）：

```bash
scripts/sim check python3 scripts/check_models.py
```

检查源模型三工具的连通/唯一父链、关节序/轴/限位/收臂初值、质量与主惯量、资源存在性、来源一致性、收臂碰撞包络、URDF→SDF 传感器保留，以及实际 GLB/袋语义/动态代理一致性。证据写 `.runtime/model-audit/current/`。这不是行为单测，不替代任意姿态碰撞、运行 TF、Gazebo 成像、接触动力学或真机尺寸验收。`colcon test` 仍只运行官方 ament lint。

旧树 GLB、field 整体网格与 reconstruction v10/v11 独立袋资产分别保留版本边界；它们未纳入当前可采摘 demo 的通过范围。新模型检查修改后，先前成功采摘日志不能作为当前版本的运行验收。
