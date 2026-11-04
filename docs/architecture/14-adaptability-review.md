# 全栈适配性评估

## 方法

- **官方基线**：ROS 2 Jazzy 生命周期/参数/Executor/QoS/launch、Nav2 lifecycle、MoveIt/RViz、rosbag2、FastAPI。
- **社区主流**：Nav2 的 lifecycle + GPL + diagnostics；独立进程 + launch 组合；rosbag2 MCAP；FastAPI/uvicorn 本地监控台。
- **Peach 约束**：Orin 目标、真实相机、mock 臂、无真机 SetIO 授权、单 Web、单 bag、Python GIL、保留 FSM。
- **判定**：不把“能跑”当“适配”；明确“官方标准 / 生态主流 / 项目自研 / 未验证”。

参考来源见 [13-references.md](13-references.md)。

## 总表

| 组件 | 当前选择 | 官方/社区适配 | 评分 | 主要差距 |
|---|---|---|---|---|
| 包架构 | 能力包 + metapackage | 接近 Nav2/Autoware | 8.5 | 仍有 `peach_vegetation` 遗留 attach；部分旧文档 |
| Lifecycle | nav2_lifecycle_manager + bond | 官方/主流 | 8.0 | `bond_timeout=0` 默认关闭死检 |
| Executor/并发 | scene 单线程 + worker；recon/sup 多线程 | 符合 | 8.0 | Python GIL/相机 CPU 未 profiling |
| 参数 | GPL 六包 | 生态主流，非核心标准 | 7.0 | 0.7.6 旧；Python 生成器缺陷；仍有自定义包装 |
| 错误码 | FailureCode + 枚举名 | 项目自研，结构合理 | 7.0 | 纯核仍有数字；部分 reason 自由文本 |
| Web 后端 | FastAPI + uvicorn | 主流 | 8.0 | 无 OpenAPI；CSP `unsafe-inline`；无鉴权 |
| Web 前端 | 原生 HTML/CSS/JS + Canvas | 本地监控台合理 | 7.0 | 无组件化/E2E；力页几何硬编码 |
| RViz | `harvest_app.rviz`，去 MotionPlanning | 应用视图合理，非完整 MoveIt | 5.5 | MoveIt 插件 glibc 崩溃；规划面板不可用 |
| 录制 | 节点内 rosbag2 MCAP | 官方 API，非 CLI | 7.5 | 自定义 Recorder 封装；长时预算未复验 |
| 诊断 | diagnostic_updater + 聚合 | 官方/主流 | 7.0 | 仅部分节点接入；告警分级有限 |
| 力 | serial_force + sim | 项目自研 | 7.0 | 未接真机；五点在 Web 硬编码 |
| 相机 | peach_stereo | 驱动适配，非标准 | 5.5 | CPU 240%；Orin 未验证 |
| 性能 | x86 13.74 FPS | 达标但非目标机 | 5.5 | 无 TensorRT/Orin 数据；CPU 瓶颈 |
| 测试 | 官方 lint + runner + 历史记录 | 部分适配 | 5.0 | 自动 runner target 不一致；10×30×1800 未过 |
| 文档 | 架构/ADR/矩阵/参考 | 较好 | 7.5 | 仍有旧路径/旧口径残留 |

## 逐项分析

### 1. 包架构

**适配点**

- 能力包独立、共享 `peach_interfaces`/`peach_common`/`peach_vision_core`，与 Nav2/Autoware 的“能力包 + bringup”一致。
- `peach_harvester` 变 metapackage，避免旧聚合包继续承载代码。

**差距**

- `peach_vegetation` 仍使用 `attach`，不在主链但仍是遗留路径。
- 旧 `peach_harvester` 文档已删，但部分包文档仍有历史描述。

**建议**：保持；主链不要再新增跨包 import；`peach_vegetation` 单独立项，不混入采摘验收。

### 2. Lifecycle 与托管

**适配点**

- scene/recon/arm/supervisor 由 Nav2 lifecycle manager 管理；arm 有 bond。
- Web/observability 自激活，不强行纳入四节点名单，符合“监控旁路”定位。

**差距**

- `bond_timeout=0.0` 默认关闭，进程死检不是默认保护；文档已说明，但生产部署必须显式评估。
- 未做真实 bond 故障注入。

**建议**：在部署 profile 中明确 `bond_timeout`；如果开启，先在 mock 注入节点崩溃验证。

### 3. Executor / 并发

**适配点**

- scene：SingleThreadedExecutor + bounded latest-frame worker，避免多线程 executor 与推理 GIL 争用。
- recon/supervisor：各自 MultiThreadedExecutor，进程隔离。

**差距**

- 相机 240% CPU、scene 99% CPU，GPU 仅 41%；当前瓶颈在 CPU 前后处理，不在 executor 模型。
- 未做 per-stage profiling 固化。

**建议**：保持进程模型；先 profile 相机解码/整流，再考虑 GPU 预处理/TensorRT；不要为了“多线程”改 scene。

### 4. 参数/GPL

**适配点**

- 六个生产/观测包已迁移 GPL；描述/范围/动态参数统一。

**差距**

- 本机 0.7.6 非上游 1.3.1；Python 生成器整数 `IntegerRange` 缺陷；跨字段/非空/派生仍需自定义代码。
- `get_params()` deepcopy 可能在高频路径产生开销。

**建议**：锁定版本；构建期检查坏 IntegerRange；跨字段留在项目层；升级前做独立节点试验。

### 5. 错误码与日志

**适配点**

- `FailureCode` 作为 IDL 单一来源；supervisor 枚举名 + `failure_code_n`；`CUT_FRUIT_CLEARANCE=35` 拆分。

**差距**

- 纯核 `tool_budget` 仍写数字 `35`；部分 reason/free text 仍存在。
- 日志字段没有机器校验，依赖约定。

**建议**：加 `check_failure_codes.py` 静态扫描；纯核数字改为生成常量或集中常量表；日志字段在关键路径统一。

### 6. Web 后端

**适配点**

- FastAPI + uvicorn 本地同源；CSP、body limit、allowlist；长动作 `run_in_threadpool`，动作期间状态可刷新。
- 与旧 `http.server` 相比，路由/类型/并发更清晰。

**差距**

- 关闭了 OpenAPI docs；CSP 为力页开了 `style-src 'unsafe-inline'`；无鉴权，默认只应绑定 loopback。
- 无自动化 HTTP 合约测试。

**建议**：局域网部署前加反向代理/鉴权；把 `/api/state`、`/api/force`、debug allowlist 做成 contract test。

### 7. Web 前端

**适配点**

- 总览/监控/调试/分析/力页统一 00–04；原生 JS 无框架，适合本地低依赖。
- 力页完整 CAD 工具视图 + 五点/tcp/CoP，有盘面/整机切换。

**差距**

- 力页 PADS/角度/通道标签硬编码在 JS；换工具/改线要改前端。
- 无前端组件测试/E2E；主要靠 headless 截图。
- 分析页仍以路径/成功率为主，未完全按新能力包组织。

**建议**：把 PADS/通道标签从 `/api/force` 或参数 API 返回；加 Playwright 冒烟；分析页增加“按能力包/会话”过滤。

### 8. RViz

**适配点**

- `harvest_app.rviz` 保留 RobotModel/TF/相机/感知 Marker/TCP，去掉了会触发 glibc mutex 的 MotionPlanning 面板；默认 launch 使用 app 视图。

**差距**

- 完整 MoveIt 规划面板不可用；`harvest.rviz` 仍可能崩溃。
- 不是官方 MoveIt 默认视图，而是项目定制；RViz 不是验收唯一证据。

**建议**：把 MoveIt 插件崩溃记录为外部缺陷；应用监控用 app 视图，规划调试另开专用环境；不要宣称完整 MoveIt RViz 已验证。

### 9. 录制/分析

**适配点**

- 单一 rosbag2 MCAP writer；会话 bag/报告/回收统一。

**差距**

- 自定义 Recorder 封装，非 `ros2 bag record` CLI；长时 100GB 预算/30min soak 未复验。
- 分析页与 bag_reader 仍有旧 topic 别名。

**建议**：保持单一 writer；加一次 `ros2 bag info` + 全 bag 读取回归；不要恢复第二条录制路径。

### 10. 诊断/自检

**适配点**

- `diagnostic_updater` + `/diagnostics` 聚合；selfcheck 输出 pass/warn/skip；force 关闭时正确 skip。

**差距**

- 不是所有节点都有诊断任务；告警等级/恢复动作有限。
- 自检结果是软门，不等于安全授权。

**建议**：逐步给 scene/recon/supervisor 增加诊断任务；保持 selfcheck 只读。

### 11. 力/工具

**适配点**

- `serial_force` 真实源 + `force_sim_node` 同契约；Web 实时页；五点 CAD 图。
- `source:=serial|sim` 与物理串口隔离。

**差距**

- 未接真实力传感器/工具；L7 未验证；Web PADS 与工具档案绑定。

**建议**：用工具档案生成 Web 几何；真实力测试另授权；不要把模拟力当抓取许可。

### 12. 相机/性能

**适配点**

- 真实相机链路可用；15s soak camera/depth/joints fresh；x86 13.74 FPS。

**差距**

- 相机节点 240% CPU；GPU 41%；Orin 未实测；TensorRT 未实现。
- RViz/观测/录制会争 CPU。

**建议**：先做相机/profile；目标机跑 benchmark；Orin 前不要宣称性能达标。

### 13. 测试

**适配点**

- 官方 lint + 历史短测 + 本轮矩阵；文档引用 10-16/10-21/10-22 记录。

**差距**

- 自动 runner `full_regression2` target 记录不一致；10×30×1800 未通过；无 Playwright E2E。
- 测试仍依赖人工截图/日志。

**建议**：先修 runner target 选择/记录；再跑 1/1、短矩阵、10×30×1800；最后加 Web E2E。

### 14. 文档

**适配点**

- 架构/ADR/回归矩阵/参考来源齐全；历史记录已引用。

**差距**

- 仍有旧 `record_bag`/force_web 残留描述；部分包文档仍是历史口径。

**建议**：以 `rg` 做旧名扫描；文档与 manifest/源码同轮更新。

## 优先级

**P0（阻塞验收）**

1. runner target 记录/选择不一致；
2. 10×30×1800 未通过；
3. L7/SetIO 未验证。

**P1（生产风险）**

4. RViz MoveIt 插件 glibc 崩溃；
5. 相机 CPU 240% / GPU 利用率低；
6. GPL 0.7.6 旧版本与 Python 生成器缺陷；
7. Web CSP/no-auth 只适合 loopback。

**P2（适配性优化）**

8. 力页几何从配置/API 派生；
9. Web E2E/HTTP contract；
10. 文档旧名/旧口径清理。

## 总体判断

- **架构与单轮功能：适配良好。**
- **Web/录制/诊断：适配可接受，需补自动化与安全边界。**
- **RViz/相机/GPL 版本：存在明确 workaround/版本风险。**
- **测试闭环：当前最大短板，自动全量未通过。**
- **Orin/L7：未验证，不能宣称完成。**

一句话：**Peach 的“结构”已经接近主流 ROS 2 工程；“验收证据”和“目标机性能”还没有达到同一水平。**

## 本轮整体扫描后的新增修正

- `harvest_system.launch.py` 的 `rviz_config` 默认值实际仍是 `harvest.rviz`，与描述不一致；已改为 `harvest_app.rviz` 并重新构建。这是本轮发现的实际默认值缺陷。
- 清理 `peach_bringup`/`peach_system_tests`/`peach_interfaces`/`peach_observability` 中残留的 `record_bag`、`yaml_params.attach`、`harvest.rviz` 旧口径；`peach_vegetation` 的 legacy attach 保留并标注。
- `frozen_one_view.yaml` 的 `brain.launch` 注释改为能力包 launch。

## 仍需优化（按优先级）

1. **P0 测试**：runner target 不一致；10×30×1800 未过；L7/SetIO 未验证。
2. **P0 参数**：所有 GPL 参数目前“可 set”但部分不生效；需 `read_only`/动态回调审计。重建开关已拆为 `reconstruction_enabled`/`skip_reconstruction`/`capture.auto_mode` 三层，task YAML 冲突已 fail closed；其余包的动态/静态审计仍未完成。
3. **P1 性能**：相机 240% CPU、GPU 41%；Orin/TensorRT 未测。
4. **P1 RViz**：完整 MotionPlanning 面板仍崩溃；应用视图是 workaround。
5. **P1 GPL**：0.7.6 旧版本；Python 生成器缺陷；升级策略未定。
6. **P2 Web**：无鉴权、CSP 允许 inline；力页几何硬编码；缺 E2E。
7. **P2 录制/诊断**：30 分钟 soak、全 bag 读取、诊断覆盖未复验。
