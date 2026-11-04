# Peach 录制回放与大数据预览

此入口把已经录制的真实 RGB-D 相机数据、mock 机械臂状态和 Peach 观测结果送入独立工作台，用于检查显示、通信和交互。无需打开相机，不启动 Gazebo、真机驱动或 Peach 业务算法。回放图谱中的发布者是 `rosbag2_player`，不能据此宣称重新验证了 Peach 采摘执行流程。

## 启动

先确认没有旧测试实例占用 8088/9090 或同一 ROS 域。本工作区已有录制文件时：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/mu/Desktop/aubo_e5_jazzy_ws
source install/setup.bash
pgrep -af 'ros2 launch|ros2 bag|component_container|extrinsics_publisher|ros2 run'

ROS_DOMAIN_ID=219 ros2 launch ros2_web_link replay.launch.py \
  bag:=/home/mu/Desktop/aubo_e5_jazzy_ws/runs/ros2_web_live_camera_mock_20261028/capture_before_camera_off/peach_rgbd_mock_60s \
  project:=peach_replay_large \
  asset_packages:=ros2_web_link,aubo_description \
  web_port:=8088 ws_port:=9090
```

打开 [工作台](http://127.0.0.1:8088/#workbench)。独立安装本包的其他机器需自行复制录制目录，并安装模型对应的 `aubo_description`；大型数据不打包进 npm 归档。`bag` 必须包含已结束录制生成的 `metadata.yaml` 和 MCAP 文件。默认 1 倍速、循环播放，保留录制时间和 `/clock`。

## 查看和操作

1. 点击“打开工作台”，在“连接设置”中确认项目为“Peach · 完整相机点云回放”，点击连接，再关闭面板。
2. 勾选模型、TF、Marker、点云、轨迹、彩色图像、深度图像和数值曲线，点击“启动所选显示”。正常应加载 6 个活动关节、12 个模型资源和 21 个 TF 帧。模型坐标系为 `base_link`。
3. 3D 区域拖动旋转、右键平移、滚轮缩放；“重置视角”恢复初始相机。右侧区域向下滚动可查看彩色图、深度图和曲线；左侧显示列表与属性单独滚动。
4. 点击显示名称编辑属性。“传输设置”包括 QoS、CBOR/JSON 和订阅间隔；“显示选项”包括点数或曲线预算。更改后应用属性。相同 Topic 的多个显示须使用兼容的传输设置。
5. 进入“节点关系”刷新图谱。层级图适合追踪方向，紧凑网格便于查看大量实体；搜索 Topic 或消息类型后查看发布者和订阅者。通过 `−`、`＋`、`1:1 字号` 和“适应画布”控制缩放，拖动空白处平移。切换页签会保留缩放与平移；重新筛选、布局或采集快照会重新布局。
6. “接口信息”选择 Topic，再读取类型与定义。它只查询 IDL，不调用业务服务。
7. 暂停显示应释放订阅、图像和 WebGL，保留所选配置。再次启动应重新获取数据；断开连接会停止显示，需要手动连接和启动。

录制长约 59.47 秒，两个 MCAP 合计约 1.21 GB。每次循环开始，点云第一帧在约 3.32 秒到达，彩色结果图约 4.31 秒；工作台收到时钟回退会清除上一轮动态数据，这是预期行为。不能把这个边界空档当作持续断流。离开循环边界后超过 10 秒没有新样本，则需检查订阅、QoS 和桥日志。

## 大数据检查的范围

默认完整点云每帧最高约 4.15 MB，经 CBOR 传输，订阅间隔 100ms；彩色/深度原始图分别以 500ms 间隔送到网页。这不是相机全帧率性能承诺。单个点云显示 `maxPoints` 上限 100000，本数据抽样后约 8.6 万点。

容量检查可再添加 3 个相同点云 Topic 的显示，总计 4 路云、8 路空间显示、11 路显示，实际合计约 34.5 万点，总配置预算 400000。它们复用同一 Topic 的网络订阅，用于检验渲染、解码回调和独立资源；不能当作 4 台相机或 4 条独立网络流的吞吐结果。超过空间或点数预算的配置应被拒绝，其他显示继续工作。

实际检查应同时记录：模型资源是否齐全；每路数据计数是否推进；时钟回退后的恢复；图谱和接口查询是否返回；显示开关是否影响其他显示；页面错误；暂停/断开后的订阅和 GL 数量；整套进程 RSS；首次加载与稳定阶段的长任务。当前原始记录在 [本轮证据目录](../../../runs/ros2_web_replay_ui_20261102/)，结论见 [性能记录](performance.md)。

`scripts/workbench_replay_probe.mjs` 是 120 秒桌面回放检查，须显式提供 HTTP 地址、`CHROME_PATH`、新输出目录 `WEB_LINK_EVIDENCE` 和冻结录制时序文件 `WEB_LINK_REPLAY_TIMING`。使用本轮 `run_browser.py` 可在隔离域启动、限时限内存并清理进程。脚本通过真实控件操作；记录原生长任务、长动画帧和离散事件，不额外运行 RAF 或保留图像 payload。事件阈值 16ms、duration 为 8ms 量化，连续拖动和滚轮不在 Event Timing 内；自动操作总耗时包含等待与截图，不能当作用户输入延迟、INP 或 GPU 耗时。

## 结束

先在网页暂停并断开，再在启动终端 Ctrl+C。等待 launch 正常退出，核对本次 `rosbag2_player`、`rosapi`、桥和 HTTP 进程已退出，端口已释放。只清理本次拥有的进程；不要用宽泛 `pkill` 停止他人的任务。自动测试也必须检查 `resources.json` 中的残留、端口和输入哈希，不能只看网页 PASS。
