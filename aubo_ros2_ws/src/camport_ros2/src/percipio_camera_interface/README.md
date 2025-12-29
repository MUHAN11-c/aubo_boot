# Percipio Camera Interface

这个功能包提供了Percipio相机的状态监控和控制接口，所有接口均基于 `percipio_camera` 节点原有的API和话题。

## 功能

### 1. **相机状态发布** (`/camera_status` Topic)

发布相机的实时状态信息，包括：
- 相机ID
- 连接状态
- 分辨率（宽度和高度）
- 帧率
- 曝光时间
- 增益
- 触发模式

**依据：**
- 分辨率：订阅 `/{camera_name}/color/camera_info` 话题获取（参考 `percipio_camera_node.cpp:643`）
- 帧率：通过统计 `/{camera_name}/color/image_raw` 话题的接收频率计算（参考 `percipio_camera_node.cpp:650`）
- 连接状态：订阅 `/{camera_name}/device_event` 话题获取（参考 `percipio_camera_node.cpp:318, 607, 614, 621`）
- 触发模式：通过ROS2参数服务获取 `device_workmode` 参数（参考 `percipio_camera_node_driver.cpp:54-55`）

### 2. **设置相机参数** (`/set_camera_parameters` Service)

支持设置：
- **曝光时间**（单位：微秒）
  - 依据：通过ROS2参数服务设置 `exposure_time` 参数（参考 `percipio_camera_node.cpp:552-567`）
  - 底层实现：`percipio_device.cpp:set_exposure_time()` 使用 `TYSetFloat`/`TYSetInt` API（参考 `TYDefs.h:TY_FLOAT_EXPOSURE_TIME_US, TY_INT_EXPOSURE_TIME`）
  
- **增益**（单位：dB）
  - 依据：通过ROS2参数服务设置 `gain` 参数（参考 `percipio_camera_node.cpp:569-585`）
  - 底层实现：`percipio_device.cpp:set_gain()` 使用 `TYSetInt` API，对于彩色图像使用 `TY_INT_R_GAIN, TY_INT_G_GAIN, TY_INT_B_GAIN`（参考 `TYDefs.h:TY_INT_GAIN, TY_INT_R_GAIN, TY_INT_G_GAIN, TY_INT_B_GAIN`）
  
- **分辨率**（需要重启流）
  - 注意：分辨率更改需要重启相机流，当前版本暂不支持动态更改
  - 如需更改分辨率，请停止节点，修改launch文件中的 `color_resolution` 或 `depth_resolution` 参数后重新启动
  
- **触发模式**（0:连续, 1:软触发, 2:硬触发）
  - 依据：通过ROS2参数服务设置 `device_workmode` 参数（参考 `percipio_camera_node_driver.cpp:104-109`）
  - 注意：`device_workmode` 被标记为不可变参数，需要重启设备才能生效（参考 `percipio_camera_node_driver.cpp:123`）

### 3. **软件触发** (`/software_trigger` Service)

发送软件触发命令到相机。

**依据：**
- 发布 `/{camera_name}/trigger_event` 话题，消息内容为 "SoftTrigger"（参考 `percipio_camera_node.cpp:278-280`）
- `percipio_camera` 节点订阅该话题，收到 "SoftTrigger" 后调用 `device_ptr->send_softtrigger()`（参考 `percipio_camera_node.cpp:272-274`）
- 底层实现：`percipio_device.cpp:send_softtrigger()` 调用 `TYSendSoftTrigger` API（参考 `percipio_device.cpp:1692`）

## 话题和服务接口

### 订阅的话题（从 `percipio_camera` 节点）

| 话题名称 | 消息类型 | 说明 | 依据 |
|---------|---------|------|------|
| `/{camera_name}/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 获取相机分辨率和标定信息 | `percipio_camera_node.cpp:643` |
| `/{camera_name}/color/image_raw` | `sensor_msgs/msg/Image` | 计算帧率 | `percipio_camera_node.cpp:650` |
| `/{camera_name}/device_event` | `std_msgs/msg/String` | 获取设备连接状态 | `percipio_camera_node.cpp:318, 607, 614, 621` |

### 发布的话题

| 话题名称 | 消息类型 | 说明 | 依据 |
|---------|---------|------|------|
| `/{camera_name}/camera_status` | `percipio_camera_interface/msg/CameraStatus` | 相机状态信息 | 新增接口 |
| `/{camera_name}/trigger_event` | `std_msgs/msg/String` | 软件触发命令 | `percipio_camera_node.cpp:278-280` |

### 提供的服务

| 服务名称 | 服务类型 | 说明 | 依据 |
|---------|---------|------|------|
| `/{camera_name}/set_camera_parameters` | `percipio_camera_interface/srv/SetCameraParameters` | 设置相机参数 | 通过ROS2参数服务调用（参考 `percipio_camera_node_driver.cpp:54-55`） |
| `/{camera_name}/software_trigger` | `percipio_camera_interface/srv/SoftwareTrigger` | 软件触发 | 通过 `trigger_event` 话题实现（参考 `percipio_camera_node.cpp:272-274`） |

### 使用的ROS2参数服务

| 参数名称 | 类型 | 说明 | 依据 |
|---------|------|------|------|
| `exposure_time` | `float` | 曝光时间（微秒） | `percipio_camera_node.cpp:552-567` |
| `gain` | `float` | 增益（dB） | `percipio_camera_node.cpp:569-585` |
| `device_workmode` | `string` | 触发模式（trigger_off/trigger_soft/trigger_hard） | `percipio_camera_node_driver.cpp:104-109` |

## 使用方法

### 1. 编译功能包

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select percipio_camera_interface
source install/setup.bash
```

### 2. 启动节点

**前提条件：** 确保 `percipio_camera` 节点已经启动。

```bash
# 启动相机节点（在另一个终端）
ros2 launch percipio_camera percipio_camera.launch.py camera_name:=camera

# 启动控制节点
ros2 launch percipio_camera_interface camera_control.launch.py camera_name:=camera
```

### 3. 查看相机状态

```bash
ros2 topic echo /camera/camera_status
```

**输出示例：**
```
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: "camera_link"
camera_id: "camera"
is_connected: true
resolution_width: 1280
resolution_height: 960
frame_rate: 30.0
exposure_time: 10000.0
gain: 10.0
trigger_mode: 0
```

### 4. 设置相机参数

#### 设置曝光时间

```bash
ros2 service call /camera/set_camera_parameters \
  percipio_camera_interface/srv/SetCameraParameters \
  "{camera_id: 'camera', exposure_time: 10000.0, gain: 0.0, resolution_width: 0, resolution_height: 0, trigger_mode: -1}"
```

**说明：**
- `exposure_time > 0` 时设置曝光时间（单位：微秒）
- `gain >= 0` 时设置增益（单位：dB）
- `trigger_mode >= 0` 时设置触发模式（0:连续, 1:软触发, 2:硬触发）
- `resolution_width > 0 && resolution_height > 0` 时尝试设置分辨率（需要重启流，可能不会立即生效）

#### 设置增益

```bash
ros2 service call /camera/set_camera_parameters \
  percipio_camera_interface/srv/SetCameraParameters \
  "{camera_id: 'camera', exposure_time: 0.0, gain: 12.0, resolution_width: 0, resolution_height: 0, trigger_mode: -1}"
```

#### 同时设置曝光时间和增益

```bash
ros2 service call /camera/set_camera_parameters \
  percipio_camera_interface/srv/SetCameraParameters \
  "{camera_id: 'camera', exposure_time: 15000.0, gain: 8.0, resolution_width: 0, resolution_height: 0, trigger_mode: -1}"
```

### 5. 软件触发

**前提条件：** 相机必须处于软触发模式（`device_workmode=trigger_soft`）。

```bash
ros2 service call /camera/software_trigger \
  percipio_camera_interface/srv/SoftwareTrigger \
  "{camera_id: 'camera'}"
```

## 消息和服务定义

### CameraStatus.msg

```msg
std_msgs/Header header
string camera_id
bool is_connected
uint32 resolution_width
uint32 resolution_height
float32 frame_rate
float32 exposure_time
float32 gain
int8 trigger_mode
```

**字段说明：**
- `header`: 标准消息头，包含时间戳和frame_id
- `camera_id`: 相机唯一标识符
- `is_connected`: 相机连接状态（true:已连接, false:已断开）
- `resolution_width`: 分辨率宽度（像素）
- `resolution_height`: 分辨率高度（像素）
- `frame_rate`: 帧率（Hz），通过统计图像接收频率计算
- `exposure_time`: 当前曝光时间（微秒），从参数服务获取或记录上次设置的值
- `gain`: 当前增益值（dB），从参数服务获取或记录上次设置的值
- `trigger_mode`: 触发模式（0:连续, 1:软触发, 2:硬触发）

### SetCameraParameters.srv

**Request:**
```msg
string camera_id          # 要设置的相机ID，需与camera_name匹配
float32 exposure_time     # 目标曝光时间（微秒），>0时设置
float32 gain              # 目标增益值（dB），>=0时设置
uint32 resolution_width   # 分辨率宽度（像素），>0时尝试设置（需重启流）
uint32 resolution_height  # 分辨率高度（像素），>0时尝试设置（需重启流）
int8 trigger_mode         # 目标触发模式（0:连续, 1:软触发, 2:硬触发），>=0时设置
```

**Response:**
```msg
bool success              # 参数设置成功为true
string message            # 参数设置结果信息或错误原因
```

### SoftwareTrigger.srv

**Request:**
```msg
string camera_id          # 要触发的相机ID，需与camera_name匹配
```

**Response:**
```msg
bool success              # 触发命令执行成功为true
string message            # 响应信息
```

## 技术实现细节

### 参数设置流程

1. **服务调用** → `camera_control_node` 接收 `SetCameraParameters` 请求
2. **参数服务** → 通过 `rclcpp::SyncParametersClient` 调用 `percipio_camera` 节点的参数服务
3. **参数回调** → `percipio_camera_node_driver::onSetParameters()` 接收参数变更（参考 `percipio_camera_node_driver.cpp:117-147`）
4. **参数处理** → `percipio_camera_node::updateParameter()` 处理参数（参考 `percipio_camera_node.cpp:328-597`）
5. **设备层** → `percipio_device::set_exposure_time()` 或 `set_gain()` 调用SDK API（参考 `percipio_device.cpp:1734-1860`）

### 软件触发流程

1. **服务调用** → `camera_control_node` 接收 `SoftwareTrigger` 请求
2. **话题发布** → 发布 `/{camera_name}/trigger_event` 话题，消息内容为 "SoftTrigger"
3. **话题订阅** → `percipio_camera_node` 的 `trigger_event_subscriber_` 接收消息（参考 `percipio_camera_node.cpp:278-280`）
4. **触发处理** → `topic_callback()` 检测到 "SoftTrigger" 字符串，调用 `device_ptr->send_softtrigger()`（参考 `percipio_camera_node.cpp:272-274`）
5. **SDK调用** → `percipio_device::send_softtrigger()` 调用 `TYSendSoftTrigger` API（参考 `percipio_device.cpp:1682-1693`）

### 状态获取流程

1. **分辨率** → 订阅 `/{camera_name}/color/camera_info`，从消息的 `width` 和 `height` 字段获取
2. **帧率** → 订阅 `/{camera_name}/color/image_raw`，统计最近2秒内的图像接收频率
3. **连接状态** → 订阅 `/{camera_name}/device_event`，检测 "DeviceOffline" 或 "DeviceConnect" 字符串
4. **触发模式** → 通过参数服务获取 `device_workmode` 参数值

## 注意事项

1. **依赖关系**：确保 `percipio_camera` 节点已经启动，且 `camera_name` 参数匹配
2. **命名空间**：所有话题和服务都在 `/{camera_name}/` 命名空间下（参考 `percipio_camera.launch.py:172`）
3. **分辨率更改**：分辨率更改需要重启相机流，当前版本暂不支持动态更改。如需更改分辨率，请停止节点，修改launch文件中的 `color_resolution` 或 `depth_resolution` 参数后重新启动
4. **触发模式更改**：`device_workmode` 被标记为不可变参数，需要重启设备才能生效（参考 `percipio_camera_node_driver.cpp:123`）
5. **曝光时间和增益**：设置前会自动关闭自动曝光和自动白平衡功能，并查询有效范围进行验证
6. **软件触发**：只有在软触发模式（`device_workmode=trigger_soft`）下才能使用软件触发功能

## 示例脚本

### Python示例：设置曝光时间和增益

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from percipio_camera_interface.srv import SetCameraParameters

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        self.client = self.create_client(SetCameraParameters, '/camera/set_camera_parameters')
        
    def set_exposure_and_gain(self, exposure_time, gain):
        request = SetCameraParameters.Request()
        request.camera_id = 'camera'
        request.exposure_time = exposure_time
        request.gain = gain
        request.resolution_width = 0
        request.resolution_height = 0
        request.trigger_mode = -1
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f'Success: {future.result().message}')
        else:
            self.get_logger().error(f'Failed: {future.result().message}')

def main():
    rclpy.init()
    controller = CameraController()
    controller.set_exposure_and_gain(10000.0, 10.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python示例：软件触发

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from percipio_camera_interface.srv import SoftwareTrigger

class TriggerController(Node):
    def __init__(self):
        super().__init__('trigger_controller')
        self.client = self.create_client(SoftwareTrigger, '/camera/software_trigger')
        
    def trigger(self):
        request = SoftwareTrigger.Request()
        request.camera_id = 'camera'
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f'Trigger success: {future.result().message}')
        else:
            self.get_logger().error(f'Trigger failed: {future.result().message}')

def main():
    rclpy.init()
    controller = TriggerController()
    controller.trigger()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 参考文档

- `percipio_camera` 节点实现：`src/percipio_camera/src/percipio_camera_node.cpp`
- `percipio_camera` 设备层实现：`src/percipio_camera/src/percipio_device.cpp`
- `percipio_camera` 启动文件：`src/percipio_camera/launch/percipio_camera.launch.py`
- SDK API文档：`src/percipio_camera/camport4/include/TYDefs.h`, `TYApi.h`
