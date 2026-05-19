# REP 2003 -- Sensor Data and Map QoS Settings

- **Status**: Draft
- **Type**: Standards Track
- **Source**: https://www.ros.org/reps/rep-2003.html

## 核心内容 — 传感器与地图 QoS 标准

### 地图 QoS
- **发布者**: `reliable` + `transient-local`（确保新订阅者能收到历史地图数据）
- 深度由设计者决定，单帧深度对静态地图是合理选择
- 适用于：占用栅格、特征地图、稠密表征等

### 传感器驱动 QoS
- **发布者**: `SystemDefaultsQoS`（允许不可靠传输）
- **订阅者**: `SensorDataQoS`
- 适用于：相机、IMU、激光扫描、GPS、深度、测距仪等
- **不适用于**：传感器数据处理的后阶段

### 设计理由
ROS 2 的 QoS 不兼容组合可能导致数据完全无法传输，标准化常见接口的 QoS 可以避免此类问题。

### 兼容性注意
- `RELIABLE` 发布者不能兼容 `BEST_EFFORT` 订阅者
- `TRANSIENT_LOCAL` 发布者不能兼容 `VOLATILE` 订阅者

完整内容见：https://www.ros.org/reps/rep-2003.html
