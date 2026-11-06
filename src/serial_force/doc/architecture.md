# serial_force 架构

## 职责与入口

本包包含五点串口力驱动、同接口虚拟平面力源，以及独立 CoP 对中实验/离线分析工具。`setup.py` 的现行 console scripts 为 `serial_force_node`、`force_sim_node`、`force_cop_control`、`force_cop_trials`、`force_cop_analyze`。

`serial_force.launch.py source:=serial|sim` 互斥选择一个力源，默认 serial；两种源的节点名都是 `serial_force`，均非 lifecycle。采摘整栈在 `force_enabled=true` 时只 include 默认串口源，不调用 CoP 控制工具。主链不根据力值授予运动许可。

## 源码地图

| 文件 | 当前职责 |
|---|---|
| `serial_force/force_node.py` | 独占串口、非阻塞轮询/重连、解码、原子帧与四个 array 发布、诊断 |
| `serial_force/force_protocol.py` | 零 ROS 的 0xA5 32 字节切帧/校验/解码、克转牛顿 |
| `serial_force/force_status.py` | AD 与力等效行程联合判 OK/SATURATED/OPEN/INVALID |
| `serial_force/force_sim_node.py` | 按 base_link→tcp TF、冻结虚拟平面与线性弹簧生成同契约力帧 |
| `serial_force/force_sim_common.py` | CoP、五点几何、关节顺序、实验 HOME 与旋转函数 |
| `serial_force/cop_control.py`、`cop_trials.py` | 独立 CoP→IK→JTC 实验；非 harvest_system 授权链的一部分 |
| `serial_force/cop_analysis.py` | 实验 JSONL 转 CSV、Markdown 与图；不运行 ROS |
| `tools/`、`web/face_*.png` | 开发期 CAD 底图生成器及原图；运行页面在 peach_web |

旧 `force_episode.py` 没有现行入口或调用方，已与旧文档一起移入工作区 `_archive/source_docs_2026-11-06/`。历史 `force/episode_*.csv/json` 的读取兼容由 observability 保留；当前源码没有自动生成这些分段文件的写者。

## 数据流

```text
/dev/force → 200 Hz 非阻塞串口轮询 → 约 100 Hz 固件帧
          → force_protocol → force_status
          → /force/sample + /force/{points,travel,ad,status}
                                      ↓
            peach_observability → 同一会话 MCAP / web_force / HTTP /api/force
                                      ↓
                         peach_web /force.html（只读）

base_link→tcp → force_sim_node → 同名 /force/*（独立实验时替代串口源）
```

串口轮询周期 `read_period_s=0.005 s` 即 200 Hz，不等于采样率；采集板约 100 Hz 主动上报，一个轮询可解析多帧。`ForceSample.header.stamp` 是主机 ROS 接收/解码时刻，固件没有可恢复的采样时间。原子消息绑定源 UUID、序号、通道号、N/mm/AD/status；不能按不同 array 的到达次序拼帧。

## 显示与记录

[peach_web](../../peach/peach_web/README.md) 提供 `/force.html`，由 [peach_observability](../../peach/peach_observability/doc/io.md) 托管 HTTP/API 与 ROSbridge 聚合状态。力页展示五点、通道状态和 CoP；本包不提供第二套 HTTP/SSE、`/record/*`、`/episodes*` 或手动 20 秒记录端点。

observability 订阅 `/force/sample`，core/std/all 录制档均将原子帧写入会话 bag。分析页可读取旧接触段 sidecar；有 bag 不代表已经生成接触段。CoP 实验工具另将结果默认写到 `$ROS_HOME/serial_force/`，它不是采摘会话录制器。

## 并发与失败

串口节点使用单 executor；`serial.Serial(..., timeout=0, exclusive=True)` 保证非阻塞与单拥有者。打开失败按 2 秒间隔重试候选端口，读失败关闭后重连。解析失败跳字节重扫；诊断按配置帧率检查，OPEN 警告节流，SATURATED 是可表示的饱和状态。

仿真源每 0.01 秒查询最新 tcp TF；缺 TF 时不发布，首次有效 TF 冻结平面。它不打开串口，也不具备真实力学、固件状态或主链执行授权的等价性。CoP 工具会直接调用 IK/JTC，只供明确授权的独立 mock 实验；单独运行不会自动继承臂侧 ExecutionAuthority。

6.0 mm 同时可能表示满量程和断线，必须结合 AD 分类。绝对力未做砝码标定，`travel_mm` 是力等效量。点位、协议及限制见 [geometry.md](geometry.md)、[io.md](io.md)、[validation.md](validation.md)；历史仿真实验见 [doc/sim](sim/README.md)。
