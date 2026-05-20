# Changelog

## 2026-05-20 (架构审计与修复)

### 安全修复
- **joint_trajectory_controller**: `precomputed_idx_` 改为 `std::atomic<size_t>`，修复 update()/sendLoop() 线程间数据竞争
- **latte_debug_panel.py**: 修正导入 `apply_start_pose` → `retarget_trajectory`，修复 ImportError
- **trajectory_pipeline.py**: 新增 `auto_execute` 参数（默认 false），`_delayed_start` 受此参数保护

### 前端质量
- **Robot3dViewer**: `clearTrajectoryOverlay()` 遍历并 dispose 所有 geometry/material，修复 GPU 显存泄漏
- **Robot3dViewer**: `trajectoryOverlay` 监听移除 `deep: true`（数据整体替换），修复 sceneMgr 未就绪时数据静默丢失
- **CoffeeLatteView**: DO 开关失败时回滚 UI 状态 + 显示错误消息
- **CoffeeLatteView**: 新增 `frozenStartPose` 快照机制，确保预览和执行使用同一位姿
- **CoffeeLatteView**: 新增位姿移动警告（执行时检测机器人是否从预览位姿移动 >10mm）
- **LatteControls**: 消息颜色区分成功（绿）/ 失败（黄）/ 进行中（蓝）
- **LatteTransformControls**: RPY 保存后显示 "已保存 ✓" 反馈

### 后端修复
- **trajectory_pipeline.py**: `_extract_cup_params/_extract_pour_params` 移除 `or` 操作符（会静默覆盖显式零值）
- **latte_trajectory_preview.py**: `_check_latte_available()` 修正为检查 `latte_imitation` 包（非 `ament_index_python`）
- **latte_trajectory_preview.py**: 缺失/零点 `start_pose` 返回 HTTP 400（非静默回退到原点）
- **latte_trajectory_preview.py**: `episode_idx` 验证仅在录制回放模式下执行 for generated patterns

### 架构清理
- **PATTERN_TYPES**: 精简为仅心形（heart），`useLatteSession` 默认 `patternType='heart'`
- **LattePatternSelector**: 移除永久隐藏的 Episode slider 和 Tulip layers slider
- **useLatteSession**: 移除死代码 `execMode`
- **useLattePreview**: 移除死代码 `lastData`
- **grasp.ts**: 补充 `latte-di-topic`、`svc-latte-do2`、`svc-latte-do4`、`svc-latte-replay` 设置项
- **useRos.ts**: `setupSubs()` 前清理旧话题订阅，`onUnmounted` 全量清理
- **CoffeeLatteView**: `connOk` 直接使用模块级 `connected` ref，修复组件挂载时连接已建立导致按钮永远灰色

### 第二轮审查修复 (2026-05-20)
- **ROS**: 未知 mode 返回 `success=False`；壶嘴 RViz2 marker 应用 `R @ offset`；主管线异常启用 `logger.exception()`
- **前端**: `sameTopic` 提取到 `lib/utils.ts` 为 `sameRosTopic`；移除 Robot3dViewer 死代码 `sameTopic`
- **前端**: 执行按钮加 `title` tooltip；移除 `mode=undefined` 多余覆盖
- **BFF**: "零 ROS 依赖"→"不查 TF/不初始化 rclpy"；`success` 语义添加注释

### 第三轮审查修复 (2026-05-20)
- **无障碍**: 所有 Latte 面板 label-input 添加 `for`/`id` 关联；LatteControls 消息 `role="alert"`
- **LogView**: `onUnmounted` 恢复原始 console 函数 + 清理事件监听
- **配置**: `TOOL_LIST` 补充 `gripper1`；`DEFAULT_CUP` 位置从 (0,0)→(0.5,-0.1)；`latte_imitation` 版本统一 0.2.0
- **安全**: `DashboardView` http:// → `//` 协议相对；VisionGraspView `getElementById`→template ref
- **清理**: 删除死组件 `PoseCard.vue`、`GraspControls.vue`
- **BFF**: `speed_scale` 验证添加注释（preview 只算几何）

### 第四轮审查修复 (2026-05-20)
- **Docs**: USAGE.md URL更新为Vue3 SPA路由(/vision, /latte, /monitor, /log, /settings)
- **Docs**: DEPLOYMENT.md RobotWebTools标记废弃；frontend-migration-plan.md标记已完成
- **性能**: `useJointChart` Array.shift() O(n)→RingBuf O(1) 环形缓冲区
- **清理**: 死组件 PoseCard.vue, GraspControls.vue 删除
- **安全**: DashboardView http://→`//` 协议相对；VisionGraspView getElementById→template ref

### 第五轮审查修复 (2026-05-20)
- **架构**: 软件架构规则写入 CLAUDE.md (三层分离/rosbridge协议/参数回调/职责边界)
- **参考**: BotBrain, rosbridge v2.1.0, ROSA (NASA JPL), Foxglove SDK

### 累计修复统计 (5轮)
- 🔴 严重: 5 (C++数据竞争, Python导入错误, spin UB, DO假开关, 零点回退)
- 🟠 高: 12 (GPU泄露, 结果覆盖, 订阅泄露, start_pose快照, 壶嘴marker, 未知mode等)
- 🟡 中: 20+ (无障碍, 死代码, 文档同步, 版本统一, 环缓冲, 配置补全等)
- **总修改文件**: 35+ | **删除死代码**: 4文件 | **新增**: CHANGELOG.md

### 文档同步
- 全局替换 `/home/mu/IVG2.0/` → `/home/mu/aubo_boot/` (CLAUDE.md, README.md, DEPLOYMENT.md)
- `architecture.md`: 接口计数修正 51 → 52 (17 msg + 35 srv)
- 创建本 CHANGELOG.md

### 参考
- [rosbridge Protocol v2.1.0](https://github.com/RobotWebTools/rosbridge_suite)
- [ROS 2 Parameters Design](https://design.ros2.org/articles/ros_parameters.html)
- [C++ Core Guidelines CP.2: Avoid data races](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#cp2-avoid-data-races)
- [BotBrain: Next.js + ROS2 Architecture](https://github.com/botbotrobotics/BotBrain)
- [Three.js How to dispose of objects](https://threejs.org/docs/#manual/en/introduction/How-to-dispose-of-objects)
