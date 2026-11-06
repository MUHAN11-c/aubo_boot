# peach_system_tests 输入输出

本包安装人工检查工具，不注册产品节点或自定义 IDL；`colcon test` 仅运行
`CMakeLists.txt` 的官方 ament lint。详细步骤与结果口径见 [testing.md](testing.md)。

## 已运行栈的工具

- `drive_harvest.py`：向 `/peach_supervisor/set_enables` 和 `run_harvest`
  发送请求；默认设置 execution=true、tool=false，`--grasp` 决定抓取使能。
  `--skip-enables` 仅跳过使能写入，仍发送 goal。退出 0 要求 Action SUCCEEDED
  且 result.success；该通用驱动没有 mock 模式核验，也不负责停外部栈或复原使能。
- `probe_topics.py`：有限时订阅指定 Image/PointCloud2，默认 RELIABLE，
  `--no-reliable` 可作 BEST_EFFORT 对照。打印计数/平均到达率/末次规格；
  即使全零也返回 0，须检查 `ALL_TOPICS_ZERO` 和实际计数。
- `check_planning_scene.py`：只读 `/get_planning_scene`，打印世界对象和 ACM；
  服务缺失或超时返回非零，查询成功的 0 不代表所有条目符合预期。

## 托管 mock 夹具

`run_frozen_grasp.py` 组合冻结 launch 与 `drive_frozen_grasp.py`；bag 只读，
重发节点提供相机/点云与 `/frozen_camera_replay/pause`。驱动核验 mock 插件和
暂停服务，再写夹具参数/使能并发 PICK_ALL；第一颗终局后取消自己的批次。
退出 0 按单果 summary、暂停与墙钟条件判定，不要求自然完成全部目标。
`--skip-launch` 不停止外部栈，wrapper 只收尾自己的 request 与进程组。

`run_live_camera_mock.py` 启动真实 stereo 相机与 mock 臂，核验启动事实，
通过原生 ROS 服务/Action 和 8090 快照采样。`--reconstruction` 与
`--no-reconstruction` 选择本轮策略，`--models>0` 必须启用重建；作业参数来自
产品默认或 `--task-params-file`。本轮默认不重建、models=0、PREGRASP_ONLY，
工具始终关闭，不自动 ACK。报告、快照、日志、进程退出/清理证据及
`records/{session_*,<request_id>,…}` 写入独立 `--output`；默认域 86。

## 离线工具

`benchmark_perception.py` 读取 RGB-D `.npz` 并实际运行模型，输出官方推理周期
与整段墙钟统计。`gen_baseline_inventory.py` 导出当前 YAML 叶值、接口名和函数索引，
不比较历史快照。`analyze_harvest_record.py <run> [--out …]` 读取记录并生成 PNG、
CSV、`summary.json`；默认输出 `<run>/analysis`，完整运动周期只计 SUCCEEDED 且 L6 以上。

工具退出码、Action 终态、账本完成等级、相机/RViz 活度和清理结果分别核对；
任一脚本均不替代操作员的真机授权。

冻结回放默认读取 `share/peach_system_tests/data/rgbd_mock_v1/bag`、索引0；
环境 `PEACH_FROZEN_BAG` 与显式CLI可覆盖。随仓包包含原曝光stamp、CameraInfo与注册点云。
