# peach_stereo

PS800-E1主机侧单图案立体RGB-D相机前端。彩色/双目IR同帧组采集，经校正、SGBM、深度配准及有效值滤波，输出与percipio前端同构的话题。它是相机前端，不做目标感知或运动决策；同一相机连接独占，两前端不得同时启动。

## 运行与配置

```bash
ros2 launch peach_stereo stereo_camera.launch.py device_ip:=169.254.10.110
# 整栈选择；只连接相机，臂仍mock
ros2 launch peach_bringup harvest_system.launch.py hardware_mode:=mock \
  camera_enabled:=true camera_frontend:=stereo autostart:=false
```

先按 [测试指导](../peach_system_tests/doc/testing.md) 装载Jazzy、工作区和aubo_py3.12；numpy固定1.26.4。整栈默认相机关闭，开启后前端默认stereo；percipio需显式选择。源码入口 `src/stereo_camera_node.cpp`，部署参数 `config/stereo_camera.yaml`，launch传设备IP与共享彩色标定文件。裸ros2 run不自动加载部署YAML。

主要配置：SGBM模式/视差范围/唯一性、median_ksize、avg_k、temporal_k、曝光/激光及彩色内参。部署使用hh4与temporal_k=3；temporal_k仅允许1/3/5，1关闭时域中值。avg_k按多帧有效值平均并降低输出节拍，temporal_k为滑窗有效中值、每帧发布，两者不可混为同一效果。以当前YAML和C++校验为完整参数来源。

## 输出与单位

- `/camera/color/image_raw`：彩色raw图像。
- `/camera/depth/image_raw`：配准到彩色的uint16深度，原值×0.25mm；0为无效。
- `/camera/color/camera_info`、`/camera/depth/camera_info`：对应内参。
- `/camera/depth_registered/points`：彩色点云，额外confidence FLOAT32；消费端按字段名读取，勿假定rgb/confidence槽位置。
- `/diagnostics`：设备、激光、速率和丢弃等健康。

相机光学帧保持与percipio同构。感知边界必须按depth_scale_unit换算，不把uint16本身视为毫米。frame stamp及同步状态须与精确stamp TF一同核对。

## 生命周期、失败与限制

普通C++节点，不进入四节点生命周期名单。launch失败退出后延迟3秒respawn。激光/曝光/标定写入失败拒启，TIME_SYNC失败告警降级；停栈释放设备并复位激光。长期满功率激光温升、近于额定0.4–0.8m范围的光学质量仍需现场验证。

依赖percipio源码中vendored camport4 SDK头文件与安装库，不改驱动栈。只发布raw Image，不自行扩展压缩传输；大消息QoS、订阅残留与FastDDS共享内存问题须通过有限时探针定位。部署hh4+半分辨率隔离台架约13.7 gps；真实相机+mock整栈在不钉OpenMP时约12–13 gps（2026-10-16短测，见测试指导）。钉OpenMP或他程序抢核可落到约6 gps。历史台架帧率/精度不是当前环境的验收保证，切前端后重验配准、深度、手眼及目标几何，不能以新手眼数值掩盖未定位的配准误差。

## 构建与验证

```bash
colcon build --packages-select peach_stereo
colcon test --packages-select peach_stereo --return-code-on-test-failure
colcon test-result --test-result-base build/peach_stereo --verbose
```

源码测试说明见 [test/README.md](test/README.md)，职责/并发见 [架构](doc/architecture.md)。真实相机+mock+RViz、30组模型/10轮/30分钟验收及bag封口和精确清理见测试指导。测试结束必须关闭相机与本轮进程；mock和相机测试不证明真实接触/剪切或硬件急停安全。
