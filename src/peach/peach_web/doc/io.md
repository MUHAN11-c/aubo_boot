# peach_web 输入输出

接口来源为 `web/api.js`、`web/rosbridge.js` 与观测包
[`doc/io.md`](../../peach_observability/doc/io.md)。本包不定义新的 ROS 接口。

## 页面入口

监控的重建阶段耗时来自既有 HTTP/ROSbridge 状态中的 `reconstruction.diagnostics.timing.phase_ms_last`：`tsdf_extract`、`cylinder_or_sphere_refit`、`captured_landmarks`、`fuse_bag_views`、`geometry_record`、`finalize_publish`，单位毫秒。缺失、非有限、负数或错误类型不生成采样；显式零值有效。`finalize_ms_last>0` 可另显示总定稿时间；各项不构成同步可加总的耗时分解。没有新增 ROS 接口或 Web 请求。

- `/`、`/overview.html`：总览；`/monitor.html` 与兼容 `/index.html`：监控。
- `/debug.html`：调试；`/analysis.html`：历史分析；`/force.html`：五点力。

## 实时与历史输入

`GET /api/transport` 提供 `default/url/groups/endpoints/topics`。默认 WS 模式下，
`/api/state`、`/api/force`、`/api/trajectory` 的页面请求分别读
`/peach_observability/web_state`、`web_force`、`web_trajectory` 的 JSON 投影；
HTTP 模式访问同名 GET。状态约 1 Hz，力与轨迹约 4 Hz，后端仅有订阅者时发布。
总览读状态与力，监控读状态与轨迹，调试读状态，五点力页只读力。

两幅 JPEG 使用 `/api/images/{perception,raw}.jpg`，元数据来自状态投影；
`/api/analysis/records`、`/api/analysis`、`/api/analysis/image` 和
`/api/analysis/success` 均为 HTTP 只读历史入口。力页没有接触段或 20 秒记录接口；
当前 `/force/sample` 原始帧由观测节点录入 bag，旧 `force/episode_*` 仅供历史分析兼容。

## 操作输出与结果

HTTP 写参数用 `POST /api/params`，调试用 `POST /api/debug/<action>`；
WS 模式将这些请求转为白名单节点的标准参数服务、既有业务服务或四个 Action。
具体端点从 `/api/transport` 读取，映射在 `rosbridge.js`；不发布运动话题或新增 SetIO。

参数写入逐项返回，200/207/400 区分全成/部分成功/全部拒绝，未知写入可为 504。
部分成功不回滚，须读回。HTTP 动作 202 表示在途；服务外层受理不代替内层
`accepted/success`，Action 终态不代替账本业务结果。L2、L6、L7 分别核对预抓取、
撤退与采摘确认，工具关闭的 L6 只证明干跑。

状态超过 5 秒或断线时标记陈旧并暂停普通操作；取消保留不表示请求已送达。
力帧及力投影以 1.5 秒门限清除旧读数/CoP。图像、模型和 TF 各有自己的身份与时间，
页面不能将它们推断为同步或有效运动许可。详细错误、恢复与收尾见
[操作手册](debug-testing.md)。
