# 数据记录

## 唯一路径

会话 bag 使用 `peach_observability` 节点内的 rosbag2 MCAP 单写线程。configure 创建 Recorder 并异步开会话，activate 启动 std/all 的补充图发现订阅；deactivate 停止 HTTP、Web 流、性能采样、自检与通配发现订阅，但固定订阅和 Recorder 保留，不能把它当作停录或封口；cleanup/shutdown/destroy 才关闭记录器并进入报告收尾。收尾状态、写入错误和报告分别检查，不能把节点退出当作 bag 已成功封口。独立的 `record_bag.launch.py` 旁路已删除，避免两套录制路径。

- 输出：`runs/session_*/bag/`；
- 记录：状态/事件/观测/重建/动作/诊断/`/rosout`（按配置）；
- `save_images` / `save_clouds` 不是全局禁录门：all 的两幅预览图绕过 save_images，std/all 图发现仍可能补录点云；固定订阅、发现白名单/排除集、关键段与限速共同决定记录集合；
- 体积预算、报告与回收由 observability 统一负责。

## 不记录

- `src/peach_sim` 不在范围；
- Web 静态资源不进 bag；
- `/force/sample` 原子帧进入同一会话；CoP 由五点派生。当前没有旧 episode/20秒CSV 写者，分析模块只读兼容历史 sidecar。

精确话题、QoS、队列/封口、限速与回收边界见 [observability IO](../../src/peach/peach_observability/doc/io.md)。core/std/all 是记录档位，不能把 all 解释为无限速、无排除或无丢帧。
