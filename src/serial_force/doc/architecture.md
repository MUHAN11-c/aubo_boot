# serial_force 架构

## 职责

USB 串口五点力传感器驱动：把咬合末端 IMS-C04A 五片薄膜力敏电阻的 0xA5 帧
解析为 SI 牛顿、力等效毫米、原始 AD 与通道状态，发布 `/force/*`；附带一个
只读调试页 `8090 force page`，用于实时 5 点/CoP 观察和接触段记录。

- **不是** lifecycle 节点，不进 `peach_lifecycle_manager` 名单；
- **不是** 控制/许可节点：不参与抓取授权、不发运动、不写 PlanningScene；
- 力值只用于显示、记录和离线分析；闭环力对准在独立包 `serial_force`。

## 源码地图

| 文件 | 作用 |
|---|---|
| `serial_force/force_node.py` | ROS 节点：串口重连、100 Hz 定时读、发布 `/force/sample` 与四个兼容 array、`/diagnostics` |
| `serial_force/force_protocol.py` | 纯 Python：0xA5 32 字节切帧/校验/解码，克→牛，行程 0.1 mm/LSB |
| `serial_force/force_status.py` | 纯 Python：按 AD + 行程分 OK / SATURATED / OPEN / INVALID |
| `serial_force/force_episode.py` | 纯 Python：接触段状态机；起段/封口与 `peach_common.recording_session` 会话绑定 |
| `serial_force/8090 force page.py` | 调试 HTTP/SSE 服务；只订 `/force/sample`，不碰串口 |
| `web/8090 force page.html` | 页面：5 点、CoP、盘面/整机、回放、20 s 记录 |
| `web/face_*.png` | 与 URDF 同一 visual STL 的 220/72 mm 底图 |
| `tools/` | 一次性资产生成器（渲染底图、五点核对图），不随节点运行 |

## 数据流

```text
/dev/force --115200--> force_node
  A5 帧 -> force_protocol.decode_force_frame
        -> ForceFrame(grams, travel_mm, ad)
        -> force_status.classify -> status[5]
        -> /force/sample (原子帧) + /force/points|travel|ad|status
                                   |
                              8090 force page
                                   |
                  SSE /stream + /record/* + /episodes*
                                   |
                  runs/session_*/force/episode_NNN.{csv,json}
```

`/force/sample` 是唯一完整帧来源；`8090 force page` 不按不同 array 话题的到达顺序拼帧。
`/force/points` 等四个 array 保留给既有消费者（调度、仿真、分析页）。

## 进程与启动

- `ros2 launch serial_force serial_force.launch.py`：只起驱动；
- `ros2 launch serial_force 8090 force page.launch.py`：只起调试页（可与驱动并行）；
- `peach_bringup real_web.launch.py` include 本包的 `8090 force page.launch.py`，
  由 `8090 force page_port` 控制端口；`harvest_system.launch.py` 只 include 驱动。

一个串口只能有一个拥有者：`serial_force_node` 用 `serial.Serial(..., exclusive=True)`
打开 `/dev/force`；不要在整栈之外再起第二个节点。`8090 force page` 只订话题，不受此限。

## 会话与记录

`8090 force page` 只在**当前仍在写的采摘会话** `runs/session_*/` 下写 `force/`：

- 自动接触段：合力 ≥ `ONSET_N`（0.15 N）开始，回到 < `RELEASE_N`（0.08 N）
  并稳住 `RELEASE_HOLD_S`（0.35 s）封口；180 s 未归零按 `max_duration` 封口；
- 断流、源 UUID 变化、会话关闭、退出分别以明确 reason 封口，不伪称 `returned`；
- 每帧先 append `.partial.jsonl`，内存行集完成后原子写 `episode_NNN.csv` + `.json`，
  再 append `index.jsonl`；异常退出后下次 `8090 force page` 只恢复到原目录；
- 手动 20 s 记录写 `force_20s_<stamp>_<id>.csv`，同一目录；
- 起段时绑定唯一 `state=active` 的 `session.json` 写者；没有活会话就不自动记，
  手动请求明确拒绝；已有段不挪入后来会话。

## 并发与失败

- `serial_force_node`：单 executor；串口打开失败每 2 s 重试；读失败关口后重连；
  解析失败丢字节重扫，不阻塞定时器；
- `8090 force page`：HTTP 线程 + ROS spin 线程 + `_lock`/`_flush_lock`；SSE 每 0.1 s
  推快照；每个 runs 根一个 `.8090 force page.lock`，防两个 web 进程抢同一段号；
- 串口/帧率异常走 `/diagnostics`（`serial` + `/force/points` 频率窗 0.6–1.4×）；
- 6.0 mm 是合法满量程也是断线哨兵：由 AD 区分，不能只看 mm。

## 边界

不负责采摘调度、MoveIt、底盘、IMU、力闭环。`8090 force page` 是调试客户端，
不是第二控制面；它不能 SetIO、不能使能、不能绕过臂侧 `ExecutionAuthority`。
绝对力未做砝码标定，`travel_mm` 是力等效而非独立位移；详见
[validation.md](validation.md) 与 [hardware.md](hardware.md)。
