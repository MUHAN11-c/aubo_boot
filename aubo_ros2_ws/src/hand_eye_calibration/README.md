# 单目相机手眼标定工具

基于Web UI的交互式手眼标定系统，用于单目相机与机器人的手眼标定。

## 功能特点

- 🎨 **现代化Web界面**：自适应全屏设计，美观易用
- 📷 **相机标定验证**：验证相机内参标定精度
- 🤖 **手眼标定**：支持Eye-in-Hand和Eye-to-Hand标定
- ✅ **精度验证**：标定结果实时验证
- 📊 **数据可视化**：实时显示标定过程和结果

## 功能模块

### 1. 相机标定精度验证
- 加载相机内参数
- 采集验证图像
- 检测标定板角点
- 计算重投影误差
- 保存验证结果

### 2. 手眼标定
- 实时显示相机图像和机器人位姿
- 多位姿数据采集（建议至少5组）
- 自动进行手眼标定计算
- 显示标定误差
- 保存标定结果

### 3. 手眼标定精度验证
- 加载手眼标定结果
- 采集测试数据
- 计算坐标转换精度
- 统计误差数据
- 保存验证日志

## 安装依赖

```bash
# 安装Python依赖
pip3 install flask flask-cors opencv-python numpy

# 或使用系统包管理器
sudo apt-get install python3-flask python3-opencv python3-numpy
```

## 编译

```bash
cd ~/RVG_ws
colcon build --packages-select hand_eye_calibration
source install/setup.bash
```

## 启动

### 方式1：使用launch文件启动（推荐）

```bash
source ~/RVG_ws/install/setup.bash
ros2 launch hand_eye_calibration hand_eye_calibration_launch.py
```

### 方式2：自定义参数启动

```bash
ros2 launch hand_eye_calibration hand_eye_calibration_launch.py \
    web_host:=0.0.0.0 \
    web_port:=8080 \
    camera_topic:=/camera/image_raw \
    robot_status_topic:=/robot_status
```

### 方式3：直接运行节点

```bash
ros2 run hand_eye_calibration hand_eye_calibration_node
```

## 使用方法

1. **启动节点**
   ```bash
   ros2 launch hand_eye_calibration hand_eye_calibration_launch.py
   ```

2. **打开Web界面**
   
   在浏览器中访问：`http://localhost:8080`
   
   如果在远程机器上运行，请使用机器的IP地址。

3. **相机标定精度验证**
   - 点击"相机标定精度验证"选项卡
   - 加载相机参数文件
   - 采集标定板图像
   - 检测角点并计算误差

4. **手眼标定**
   - 点击"手眼标定"选项卡
   - 移动机器人到不同位置和姿态
   - 每个位置点击"捕获当前位姿"
   - 采集至少5组数据后，点击"开始标定"
   - 查看标定结果和误差
   - 点击"保存标定结果"

5. **手眼标定精度验证**
   - 点击"手眼标定精度验证"选项卡
   - 加载手眼标定结果
   - 采集测试数据
   - 查看验证结果和统计信息

## 配置文件

相机标定参数存储在：`config/cameraParams.xml`

手眼标定结果存储在：`config/hand_eye_calibration.yaml`

## 话题订阅

- `/camera/image_raw` (sensor_msgs/Image): 相机图像
- `/robot_status` (interface/RobotStatus): 机器人状态（包含位姿信息）

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| web_host | string | 0.0.0.0 | Web服务器监听地址 |
| web_port | int | 8080 | Web服务器端口 |
| camera_topic | string | /camera/image_raw | 相机图像话题 |
| robot_status_topic | string | /robot_status | 机器人状态话题 |

## API接口

Web界面通过以下REST API与ROS2节点通信：

- `GET /api/current_image`: 获取当前相机图像
- `GET /api/robot_pose`: 获取当前机器人位姿
- `POST /api/camera_calibration/verify`: 验证相机标定
- `POST /api/hand_eye/capture_pose`: 捕获位姿
- `POST /api/hand_eye/calibrate`: 执行手眼标定
- `POST /api/hand_eye/verify`: 验证手眼标定精度

## 注意事项

1. 确保相机和机器人驱动节点已经启动
2. 标定前请确保相机已完成内参标定
3. 手眼标定时建议采集8-15组不同位姿的数据
4. 位姿分布要尽量均匀，覆盖工作空间
5. 标定过程中保持标定板清晰可见

## 故障排查

### 1. Web界面无法访问
- 检查节点是否正常启动
- 检查防火墙设置
- 确认端口未被占用

### 2. 无相机图像显示
- 检查相机话题是否正确：`ros2 topic list`
- 确认相机驱动是否运行
- 查看话题数据：`ros2 topic echo /camera/image_raw`

### 3. 无机器人位姿数据
- 检查机器人状态话题是否正确
- 确认机器人驱动是否运行
- 查看话题数据：`ros2 topic echo /robot_status`

## 技术栈

- **后端**: ROS2 (Python), Flask
- **前端**: HTML5, CSS3, JavaScript
- **图像处理**: OpenCV
- **数学计算**: NumPy

## 版本信息

- 版本: 1.0.0
- 作者: Developer
- 许可: MIT

## 更新日志

### v1.0.0 (2024-10-18)
- ✨ 初始版本发布
- ✨ 实现Web UI框架
- ✨ 支持三大功能模块
- ✨ 实时图像和位姿显示

