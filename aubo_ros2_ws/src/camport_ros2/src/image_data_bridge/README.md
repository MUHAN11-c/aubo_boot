# Image Data Bridge

图像数据桥接节点，将相机发布的彩色图像转换为 `ImageData` 消息格式，供手眼标定工具使用。

## 功能

- 订阅相机彩色图像话题（`color/image_raw`）
- 订阅相机状态话题（`/camera_status`）以获取相机ID
- 发布 `ImageData` 消息到 `/image_data` 话题
- 支持可选的JPEG压缩编码以减小数据传输量

## 依赖

- `rclcpp`
- `sensor_msgs`
- `percipio_camera_interface`
- `cv_bridge`
- `opencv2`

## 编译

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select image_data_bridge
source install/setup.bash
```

## 使用方法

### 基本使用

```bash
ros2 launch image_data_bridge image_data_bridge.launch.py
```

### 自定义参数

```bash
ros2 launch image_data_bridge image_data_bridge.launch.py \
    input_image_topic:=color/image_raw \
    camera_status_topic:=/camera_status \
    output_topic:=/image_data \
    camera_id:=DA3234363 \
    use_jpeg_encoding:=true \
    jpeg_quality:=90
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `input_image_topic` | string | `color/image_raw` | 输入图像话题（相机发布的彩色图像） |
| `camera_status_topic` | string | `/camera_status` | 相机状态话题 |
| `output_topic` | string | `/image_data` | 输出ImageData话题 |
| `camera_id` | string | `DA3234363` | 相机ID（相机状态不可用时的默认值） |
| `use_jpeg_encoding` | bool | `false` | 是否使用JPEG编码压缩图像 |
| `jpeg_quality` | int | `90` | JPEG压缩质量（1-100，仅在use_jpeg_encoding=true时有效） |

## 输入话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `color/image_raw` | `sensor_msgs/Image` | 相机发布的彩色图像（RGB8或BGR8格式） |
| `/camera_status` | `percipio_camera_interface/CameraStatus` | 相机状态信息（用于获取相机ID） |

## 输出话题

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/image_data` | `percipio_camera_interface/ImageData` | 包含图像和相机ID的ImageData消息 |

## 工作流程

1. 节点订阅相机发布的彩色图像话题（`color/image_raw`）
2. 节点订阅相机状态话题（`/camera_status`）以获取相机ID
3. 当收到图像消息时：
   - 如果启用了JPEG编码，将图像压缩为JPEG格式
   - 创建 `ImageData` 消息，包含：
     - Header（时间戳和坐标系）
     - Camera ID（从相机状态或参数获取）
     - Image（原始或JPEG压缩的图像数据）
4. 发布 `ImageData` 消息到 `/image_data` 话题

## 注意事项

- 输入图像必须是彩色图像（RGB8或BGR8格式），不支持深度图或灰度图
- 如果相机状态话题不可用，将使用参数中指定的默认相机ID
- JPEG编码可以显著减小数据传输量，但会损失一定的图像质量
- 建议JPEG质量设置为85-95之间，以获得较好的压缩比和图像质量平衡

## 与手眼标定工具的集成

此节点是手眼标定工具（`hand_eye_calibration`）的输入数据提供者。启动此节点后，手眼标定工具就可以从 `/image_data` 话题接收图像数据。

完整的启动流程：

1. 启动相机节点
2. 启动此桥接节点
3. 启动手眼标定节点

```bash
# 终端1: 启动相机
ros2 launch percipio_camera percipio_camera.launch.py

# 终端2: 启动桥接节点
ros2 launch image_data_bridge image_data_bridge.launch.py

# 终端3: 启动手眼标定工具
ros2 launch hand_eye_calibration hand_eye_calibration_launch.py
```

