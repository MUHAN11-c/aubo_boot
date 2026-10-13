# `peach_observability` 架构（v1 活文档）

> 2026-10-10 用户裁定自仓库级 `docs/architecture.md` 拆写入包。系统级架构在 [peach_bringup/doc/architecture.md](../../peach_bringup/doc/architecture.md)。**源码与本文互相更新，改一边须同一轮改另一边**。

- **禁止：** 发 `RunHarvest`。PAUSE 是节点 Inactive，不是批次 `ControlTask` 暂停。

#### `peach_observability`（监）

- **作用：** HTTP（默认 `127.0.0.1:8090`）过程页：只读监控（订阅各包状态与原始话题）+ 会话 bag 过程记录（决策 0019：随节点启停开合 `runs/session_*/bag`，栈停自动出 `bag_report.md/json` 并按 `record.max_total_bag_gb` 预算回收旧 bag）+ 柜侧硬件表（TCP xyz/quat、六轴角/速度/电流/温度/跟随误差）+ 末端主平面轨迹（按点列跨度最大的两轴投影，不固定 X–Y 俯视）+ 单步调试 POST（决策 0018，无令牌）。整栈 include 时 **不进 lifecycle 名单**，节点 `main()` 在 spin 前自行 `configure/activate`。作业票下方对照实测 TCP、起止弦与预抓取/入口。
- **类 / 配置：** `ObservabilityNode`、`ObservabilityState`；参数 `config/observability.yaml`。静态页资源在顶层资源包 `peach_web`（2026-10-10 自本包 `web/` 拆出；`start_http` 经 `get_package_share_directory('peach_web')/web` 解析，本包 `exec_depend` 它），Tab 分「过程 / 调试」。首屏是当前果实作业票（发现→拍照→锁定→观察→许可→靠近→工具→撤离→完成）。抓取档关闭时靠近/工具标 **gated**，不得显示成已勾上。健康走 `/diagnostics` 周期任务（5s；W15 双轨 + 2026-09-29 扩充：`session_recorder` 报队列水位/丢帧/目录、`ingest_liveness` 报镜像键最热年龄 ≤10s OK / ≤60s WARN / 更久 STALE、`selfcheck` 投影最近一次自检裁决；对齐 peach_arm W5 与 vegetation 的做法）。
- **`/api/state` 区段：** `perception` / `reconstruction` / `refined` / `manipulation`（含 `status` 与 `hypothesis`）/ `task_executor`（含 `state_seq`/`scene_epoch` 镜像）/ **`pipeline`**（调度 FSM 与技能周期阶段时序）/ **`ledger`**（批次账本直播）/ `robot`（柜侧 `status` + latest TF `tcp` 摘要：xyz/quat/路径长/弦长/绕行比/Δz + `joints` 六轴角/速度/电流/温度/跟随误差）/ `metrics` / `record` / `params` / `startup`（启动事实镜像）/ `selfcheck`（最近一次自检报告）/ `diagnostics`（`/diagnostics` 聚合）/ **`job`**（派生作业票：过程线、档位、`why`、base_link 坐标含预抓取）/ **`logs`**（`/rosout` 有界环，默认 200 条：`stamp` / `level` / `name` / `msg`，消息截断 400 字）/ **`debug`**（`enabled` / `motion_enabled` + 最近操作环形缓冲）。不再用 `approach` / `orchestration`。
- **相机帧率自检（2026-10-12）：** color/depth 探针用 RELIABLE、volatile、depth=5，与当前 stereo/percipio/replay 默认发布端一致。测量值是本节点接收到达率，不是设备采集帧率；低于 `selfcheck.camera_min_hz`（包括零帧）仅 WARN，不令 `selfcheck_passed` 为 false。手动 RunHarvest 原本就不消费此闩锁；采集数据是否可用于抓取仍由感知/重建/臂侧现有有效性与许可门判定。相机单独实测：相同流 RELIABLE 接收约 14 Hz，BEST_EFFORT 仅 0.6–0.8 Hz，旧自检把探针丢帧误报为相机低率。若另行将发布端切成 BEST_EFFORT，须同步调整探针 QoS。
- **完成证据：** `job` 按同批同目标账本的 `completion_level` 保留实际完成阶段；取消终局不抹掉已经验证的预抓取（level=2）。感知 `HARVESTED` 表示目标已结算，采摘确认仍以账本确认字段 / level=7 为准。参数镜像包含 supervisor 的 `execute_pregrasp_only` 与 `reconstruct_in_trajectory`，网页可以明确说明执行档和并行重建配置。
- **`/api/trajectory`：** 末端点列（平坦 `xyz` + 相位）+ 作业票路标 + 与 RViz 同源的 Marker 字典。只读，不进 MCAP。
- **调试 POST：** `POST /api/debug/<action>`。`debug.enabled` 默认 true（false→503）；无令牌。运动类另需 `debug.motion_enabled`（默认 false→423）。审计落 `runs/debug_audit/<日期>.jsonl`。后端仍转发全部既有端点（`debug.endpoints.*` 键冻结；`manage_nodes_service` 直接使用 Nav2 服务类型，服务缺席时 503，见 io.md）；**页面只暴露本管线**：BeginScene / SurveyScene / Build / finalize / ExecuteTarget（含 `set_execution_armed` 武装/解除）/ 去拍照位 / RunHarvest / ControlTask（PAUSE / RESUME / SKIP_TARGET / ACK 恢复 / CANCEL_NOW）。`PREVIEW` 与 BeginScene 不受运动门拦。技能 `ExecutionAuthority` 与调度/重建门**原样生效**。

### 只读预览与工具 TF 缓存（2026-10-13）

两条现有调试图像订阅由预览与会话记录共享，录制关时仍可查看。
`ImagePreview` 每流仅保留一个最新待编码槽与一个 JPEG；所有流共用单个工作线程，
两流合计全局至多 5 Hz（公平轮转最新槽），CvBridge 转 BGR8 后由 OpenCV 缩图及 JPEG80 编码。
HTTP 请求不转换图像、不等待 ROS 服务或 TF。坏帧使该流缓存失效；图像与工具帧
元数据采用 ROS header / ROS 时钟计算年龄，1 s 陈旧仅影响展示。

现有 TCP timer 同时缓存 base_link 下的 `tool_axis` / `sleeve_mouth` / `cutting_plane`
真实 TF（不推断 TCP Z）；关闭 TCP 轨迹记录仍采工具帧供页面只读展示。
cleanup / destroy 关闭工作线程、清图像槽、销毁采样 timer 并 unregister TF listener；
全量录制发现显式跳过共享图像话题，不叠订户。apt 依赖 `cv_bridge` / `python3-opencv`，
不新增 pip 副本，不改变 numpy 1.26.4。
