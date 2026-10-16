# peach_stereo 架构

## 职责

PS800-E1 主机单图案立体 RGB-D 前端，与 percipio 设备端前端互斥。

## 源码地图

`src/stereo_camera_node.cpp`：SDK 打开/采集、校正、SGBM、配准、滤波、发布及诊断；`config/stereo_camera.yaml`：部署参数；`launch/stereo_camera.launch.py`：namespace、内参与重启策略。

## 入口、输入输出与参数

`ros2 launch peach_stereo stereo_camera.launch.py`；整栈由 camera_enabled/camera_frontend 选择。发布 `/camera/color/image_raw`、`depth/image_raw`、两路 camera_info、`depth_registered/points` 及 `/diagnostics`。深度 uint16×0.25mm；点云按字段名读取 xyz/rgb/confidence。

## 生命周期、并发与失败

普通 C++ 节点，不进托管名单。采集/匹配是重路径；launch 在进程失败后延迟3秒 respawn。激光/曝光/标定配置失败拒启，TIME_SYNC 失败告警；关闭应释放设备并复位激光。两前端不可同时连接。参数由部署 YAML 与 C++ 声明共同核对，裸 ros2 run 不自动加载部署 YAML。部署 hh4+半分辨率隔离台架约 13.7 gps；`harvest_system` 真实相机+mock 在不钉 OpenMP 时 `stack.log` 记 12.0–13.1 gps、drop=0（2026-10-16 `live_mock_20261016_183842`）。钉 `OMP_NUM_THREADS=1` 或他程序抢核可落到约 6 gps，仍 drop=0。近距质量、温升与长期稳定性需独立验收，历史帧率不是产品保证。

## 验证与限制

`colcon test --packages-select peach_stereo --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。
