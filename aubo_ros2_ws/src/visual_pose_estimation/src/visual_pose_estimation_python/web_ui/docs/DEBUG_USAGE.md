# Debug 选项卡使用说明

## 功能概述

Debug 选项卡参考了 `trigger_depth.py` 的实现，提供了实时图像显示和参数调节功能，用于调试和优化深度图像处理参数。

## 主要功能

### 1. 实时图像显示

Debug 选项卡提供四个图像显示区域：

- **深度图像 (Depth Image)**: 显示原始深度图，使用颜色映射（蓝色=近，红色=远，黑色=无效）
- **彩色图像 (Color Image)**: 显示原始RGB彩色图
- **二值化深度图 (Binary Depth Image)**: 显示深度阈值过滤后的二值化图像，并标注检测到的轮廓
- **预处理图像 (Preprocessed Image)**: 显示最终的预处理结果，白色背景上叠加目标区域

### 2. 参数调节

#### 深度阈值 (Depth Threshold)
- **最小深度 (Min Depth)**: 0-65535，设置深度值的最小阈值
- **最大深度 (Max Depth)**: 0-65535，设置深度值的最大阈值

#### 轮廓面积 (Contour Area)
- **最小面积 (Min Area)**: 0-100000，过滤掉面积小于此值的轮廓
- **最大面积 (Max Area)**: 0-1000000，过滤掉面积大于此值的轮廓

#### 连通域过滤 (Component Filter)
- **最小宽高比 (Min Aspect)**: 0-10.0，过滤宽高比小于此值的连通域
- **最大宽高比 (Max Aspect)**: 0-10.0，过滤宽高比大于此值的连通域
- **最小宽度 (Min Width)**: 0-500像素，过滤宽度小于此值的连通域
- **最小高度 (Min Height)**: 0-500像素，过滤高度小于此值的连通域
- **最大数量 (Max Count)**: 1-10，保留的连通域最大数量（按面积排序）

### 3. 控制按钮

- **📷 采集图像**: 触发相机拍摄新图像，并开始实时刷新显示
- **🔄 刷新显示**: 手动刷新当前图像显示
- **💾 保存阈值**: 将当前参数保存到配置文件 `configs/debug_thresholds.json`

## 使用流程

### 启动服务

```bash
# 启动Web服务器
cd /home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui
python3 scripts/http_bridge_server.py
```

### 打开网页

在浏览器中访问 `http://localhost:8000`，点击 **Debug** 选项卡。

### 调试步骤

1. **采集图像**
   - 点击"📷 采集图像"按钮
   - 系统会触发相机拍摄，并开始自动刷新（每500ms）
   - 四个图像区域会实时显示处理结果

2. **调节参数**
   - 拖动滑动条调节参数
   - 参数更新后会自动刷新图像显示
   - 观察二值化图像和预处理图像的变化

3. **保存参数**
   - 当找到合适的参数后，点击"💾 保存阈值"
   - 参数会保存到 `configs/debug_thresholds.json`
   - 下次打开Debug选项卡时会自动加载这些参数

4. **停止刷新**
   - 切换到其他选项卡时，自动停止刷新
   - 再次切换回Debug选项卡时，会继续刷新

## API 接口

Debug 功能使用以下API接口：

- `POST /api/debug/capture`: 触发相机采集图像
- `POST /api/debug/get_images`: 获取当前的四个处理后的图像
- `POST /api/debug/update_params`: 更新单个参数
- `POST /api/debug/get_params`: 获取所有当前参数
- `POST /api/debug/save_thresholds`: 保存参数到配置文件

## 配置文件

参数保存在 `configs/debug_thresholds.json`，格式如下：

```json
{
  "min_depth": 0,
  "max_depth": 65535,
  "contour_min_area": 10,
  "contour_max_area": 100000,
  "min_aspect": 0.3,
  "max_aspect": 4.0,
  "min_width": 60,
  "min_height": 60,
  "max_count": 3,
  "depth_scale": 0.25
}
```

## 技术细节

### 图像处理流程

1. **深度图处理**
   - 归一化深度值到 0-255
   - 应用JET颜色映射
   - 标记无效值为黑色

2. **二值化处理**
   - 根据深度阈值创建二值掩码
   - 查找轮廓
   - 根据面积、宽高比、尺寸过滤轮廓
   - 绘制过滤后的轮廓和边界框

3. **预处理**
   - 创建白色背景
   - 将二值化掩码区域的彩色图叠加到背景上

### 实时刷新机制

- 切换到Debug选项卡时启动定时器（500ms间隔）
- 定时器调用 `debugRefreshImages()` 获取最新图像
- 参数更新时立即刷新图像
- 切换离开Debug选项卡时停止定时器

## 参考

本功能参考了 `trigger_depth.py` 的实现：
- `/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation/scripts/trigger_depth.py`

主要参考的功能包括：
- 深度图可视化
- 二值化阈值调节
- 轮廓检测和过滤
- 参数保存和加载

## 注意事项

1. 确保ROS2相机节点正在运行，发布以下话题：
   - `/camera/depth/image_raw`
   - `/camera/color/image_raw`

2. 图像刷新频率为500ms，如果网络延迟较大可能会有卡顿

3. 参数调节时会实时更新图像，调节过快可能导致请求堆积

4. 深度图的 `depth_scale` 参数默认为 0.25（mm/unit），根据相机型号可能需要调整

## 故障排除

### 图像不显示
- 检查ROS2相机节点是否运行
- 检查话题是否正确发布：`ros2 topic list`
- 查看浏览器控制台的错误信息

### 参数不生效
- 确认参数已保存到配置文件
- 检查 `configs/debug_thresholds.json` 是否存在
- 重新加载网页

### 刷新太慢
- 检查网络连接
- 降低图像分辨率
- 调整刷新间隔（修改 JavaScript 中的 500ms）
