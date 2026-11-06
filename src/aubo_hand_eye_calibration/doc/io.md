# aubo_hand_eye_calibration 输入输出

以下为默认节点名；自定义重命名会改变 `~` 端点，Web 客户端中的绝对名称不自动改写。
来源为 `server.py`、`extrinsics_publisher.py`、`web_gateway.py` 与 `aubo_msgs` IDL。

## ROS 入口与状态

- `/hand_eye_calibration_server/run`：`aubo_msgs/action/RunHandEyeCalibration`。
  `plan_only=true` 只预检；否则执行采集。`method`、`pose_source`、`solve_target`
  空字符串采用节点默认（auto、poses、hand_eye），非法枚举报错。
  `return_to_start` 是 goal 布尔字段，CLI 省略时为 false，Web 默认发送 true。
  只有 joint 求解才填充结果内参；其它路径保留生成消息默认值（九个零、空畸变数组、
  零 joint RMS），不能当作单位阵或已求得的内参。
- `/hand_eye_calibration_server/activate`：`aubo_msgs/srv/ActivateHandEyeCalibration`，
  只接受已有且通过质量门的 candidate ID，标定忙时拒绝。
- `/hand_eye_extrinsics_publisher/reload`：`std_srvs/srv/Trigger`，重新读取 active。
- `/hand_eye_calibration_server/status`：`std_msgs/msg/String` JSON，
  RELIABLE / TRANSIENT_LOCAL / depth1，包含阶段、位姿进度、结果与失败明细。
- `/hand_eye_calibration_server/preview/compressed`：JPEG `CompressedImage`，
  sensor QoS（BEST_EFFORT / VOLATILE / depth5）；预览 header 沿用输入图像。
- `/tf_static`：`TFMessage`，RELIABLE / TRANSIENT_LOCAL / depth1，
  默认 `wrist3_Link → camera_link`；`/diagnostics` 的 `extrinsics_source` 报告来源。

服务器订阅 `image_topic`（默认 `/camera/color/image_raw`）、`camera_info_topic`
（默认 `/camera/color/camera_info`）和 `/joint_states`，均用 sensor QoS。
它直接调用 `/move_action` 的 `moveit_msgs/action/MoveGroup`。
latest TF 用于腕部姿态及视点定位；捕获观测按处理时刻筛选，不按曝光 stamp 配对。

外参求解输出 `wrist_from_camera_optical`，发 TF 时使用转换后的
`wrist_from_camera_root`。默认 optical 为 `camera_color_optical_frame`；
相机根到 optical 的内部变换由相机前端发布，不由本发布器重复建立。

## 文件与失败语义

`hand_eye/candidates/<id>.yaml` 保存求解结果；`hand_eye/active.yaml` 是激活副本，
路径可覆盖。启动缺文件时发 nominal 并 WARN；已有文件无法读取/帧名不符时
启动 nominal 回退并 ERROR。reload 失败仅返回 false，保留先前 TF。
激活回执需同时看磁盘结果与 reload 消息，文件写入成功不等于 TF 已更新。

`apply_intrinsics` 支持 cameracalibrator 的 `ost.yaml` tar 成员、OST YAML、
candidate/active 的 `intrinsics` 节。默认预期 640×480，支持 plumb_bob 五系数
或 rational_polynomial 八系数；`--dry-run` 只校验显示，`--output` 显式改目标。
默认写 `src/percipio_camera/config/color_camera_info.yaml` 后需重建安装资源并重启前端。

## 独立 Web

`web_gateway` 默认 `127.0.0.1:8088`，使用 Python `ThreadingHTTPServer`，仅接受
回环监听地址；与 Peach 8090 FastAPI 和 ros2_web_link 的独立 8088 工作台不同，
不能同时占用同一地址端口。

只读 GET：`/api/status`、`/api/robot`、`/api/events`（SSE）、`/api/candidates`、
`/api/candidates/<id>`、`/api/active`、`/api/fps`、`/preview.jpg`、`/preview.mjpeg`。
POST：`/api/preflight`、`/api/plan`、`/api/run`、`/api/cancel`、`/api/activate`。
plan/run 返回已请求发送，不代表采集或求解完成；取消请求不等于已停止。
本客户端不实现 Peach 的 debug 审计或 ExecutionAuthority，动作状态须从本包接口核对。
