# peach2_observability

Peach v2 **只读**观测层：订阅采摘图话题与 `/diagnostics`，聚合成内存快照，经 HTTP **GET** 暴露（默认 `127.0.0.1:8091`）。
不进 lifecycle 托管名单；**不创建**任何 service/action 客户端，不能触发运动、SetIO 或 lifecycle 变更（对照旧栈 8090 调试 POST，见 inventory P0）。

## 节点

| 可执行文件 | 图名 | 说明 |
|-----------|------|------|
| `peach2_observability` | `peach2_observability` | 普通 `rclpy` 节点 |

### 订阅（只读）

| 话题 | 类型 |
|------|------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| `/peach/task/state` | `peach2_interfaces/BatchState` |
| `/peach/end_effector/tool_state` | `peach2_interfaces/ToolState` |
| `/peach/manipulation/recovery_required` | `std_msgs/Bool` |
| `/peach/target_model/models` | `peach2_interfaces/TargetModelArray` |
| `/peach/perception/observations` | `peach2_interfaces/TargetObservationArray`（镜像为摘要） |
| `/peach/enables` | `peach2_interfaces/Enables` |

### HTTP（GET only）

| 路径 | 说明 |
|------|------|
| `GET /api/state` | 内存快照 JSON |
| `GET /api/diagnostics` | 诊断条目 |
| `GET /api/ledger/<request_id>` | 读 `runs/<request_id>/ledger.json`（路径净化） |
| `GET /` | 静态页（轮询 `/api/state`） |

`POST` / `PUT` / `DELETE` → **405**。绑定地址与端口见 `config/observability.yaml`（默认 loopback 8091）。

### 会话 bag（可选，默认关）

`session_bag.enabled: true` 时用 **subprocess** 启动 `ros2 bag record -s mcap`，输出到
`runs/session_<时间>/bag/`，话题集固定（见 `session_bag.BAG_RECORD_TOPICS`）。节点退出时
SIGINT → 超时 SIGTERM → SIGKILL，避免挂死 bag 占用 RELIABLE 订户。

## 参数

通过 ROS 参数 `config_file` 指向 YAML（launch 默认 `share/.../config/observability.yaml`）：

| 键 | 含义 |
|----|------|
| `host` / `port` | HTTP 绑定 |
| `runs_root` | 账本与会话根目录；空则用 `PEACH_RUNS_ROOT` 或 `<cwd>/runs` |
| `session_bag.enabled` | 是否录 bag |
| `session_bag.sigint_timeout_s` / `term_timeout_s` | bag 进程收尾超时 |

话题名**不是**参数（固定契约，见 `peach2_interfaces/config/interfaces.yaml`）。

## 公有 Python API（零 ROS，可单测）

| 模块 | 符号 |
|------|------|
| `peach2_observability.params` | `ObservabilityParams`, `load_params`, `resolve_runs_root` |
| `peach2_observability.snapshot` | `SnapshotStore`, `summarize_observations`, `batch_state_dict`, … |
| `peach2_observability.ledger` | `sanitize_request_id`, `read_ledger`, `ledger_path` |
| `peach2_observability.session_bag` | `BAG_RECORD_TOPICS`, `build_record_command`, `stop_bag_process`, `SessionBagRecorder` |
| `peach2_observability.http_server` | `HttpServer`, `ReadOnlyBackend` |

## 构建与测试

```bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --base-paths src/peach2 --packages-select peach2_observability \
  --packages-skip peach2_interfaces peach2_core \
  --build-base build/v2/peach2_observability \
  --install-base build/v2/peach2_observability_install
colcon test --base-paths src/peach2 --packages-select peach2_observability \
  --packages-skip peach2_interfaces peach2_core \
  --build-base build/v2/peach2_observability \
  --install-base build/v2/peach2_observability_install
colcon test-result --test-result-base build/v2/peach2_observability --verbose
```

## 怎么跑（需授权与整栈时由 bringup Include；此处仅文档）

```bash
ros2 launch peach2_observability observability.launch.py
# 浏览器：http://127.0.0.1:8091/
```

## 接口需求

无（契约已在 `peach2_interfaces`）。

## 已知限制 / TODO(M0)

- 无 bag 离线报告 CLI（旧栈 `peach_bag_report` 未迁）。
- 无 TCP 轨迹 / TF 采样（v2 首版只镜像契约话题）。
- HTTP 无 TLS/SROS2；仅 loopback 默认可接受，局域网暴露需另行加固。
