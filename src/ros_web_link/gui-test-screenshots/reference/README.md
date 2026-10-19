# 参考窗口截图（2026-10-19）

用官方 TB3 仿真 + `sim_demo.launch.py gui:=true rviz:=true` 打开真实窗口后抓取，
用于校准 Web 监控台的信息层级，而不是复制它们的深色皮肤。

| 文件 | 来源 | 用于校准 |
|---|---|---|
| `rviz2_tb3.png` | RViz2 + `tb3_navigation2.rviz` | Displays 树、Global Options、Views、状态栏、工具按钮 |
| `gazebo_gui.png` | Gazebo Sim (Harmonic) | 3D 视口/网格、Entity Tree、底部播放条 |
| `rqt_graph.png` | `rqt_graph`（Nodes only，关闭 Dead sinks/Leaf topics 后点 Refresh） | 节点椭圆、话题边、过滤/分组工具栏 |

## 运行备注

- `rqt_graph` 启动时若画布空白，点工具栏最左侧 **Refresh ROS graph** 即可；
  ROS 2 Jazzy 的 rqt_graph 不做自动刷新。
- `rqt_plot` 启动会打印一条上游 Qt 警告
  `QLayout::removeWidget: Cannot remove a null widget.`，不影响绘图/订阅。
- 用 `ros2 launch ... gui:=true rviz:=true` 时不要直接 source 整个工作区
  （历史 peach 包的 `AMENT_PREFIX_PATH` 会让 launch 找不到 `peach2_target_model`）；
  用 `test/e2e_harness.py` 的 curated env（LaunchStack）或干净 shell。
