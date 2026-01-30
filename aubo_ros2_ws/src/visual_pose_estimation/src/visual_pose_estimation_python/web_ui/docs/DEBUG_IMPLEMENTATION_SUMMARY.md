# Debug 选项卡实现总结

## 实现概述

参考 `/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation/scripts/trigger_depth.py` 的实现，为 visual_pose_estimation_python 的网页 UI 的 Debug 选项卡添加了实时图像显示和参数调节功能。

## 修改的文件

### 1. http_bridge_server.py

**文件路径**: `/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/scripts/http_bridge_server.py`

**主要修改**:

#### 1.1 添加了新的API路由（在 do_POST 方法中）

```python
elif self.path == '/api/debug/get_images':
    self.handle_debug_get_images()
elif self.path == '/api/debug/update_params':
    self.handle_debug_update_params()
elif self.path == '/api/debug/get_params':
    self.handle_debug_get_params()
elif self.path == '/api/debug/save_thresholds':
    self.handle_debug_save_thresholds()
elif self.path == '/api/debug/capture':
    self.handle_debug_capture()
```

#### 1.2 添加了 Debug API 处理函数

- **handle_debug_capture()**: 处理图像采集请求
- **handle_debug_get_images()**: 获取实时的四个处理后的图像（深度、彩色、二值化、预处理）
- **handle_debug_update_params()**: 更新单个调试参数
- **handle_debug_get_params()**: 获取所有当前参数
- **handle_debug_save_thresholds()**: 保存参数到配置文件
- **load_debug_params()**: 加载调试参数
- **save_debug_params()**: 保存调试参数
- **process_depth_for_display()**: 处理深度图用于显示
- **normalize_depth_to_uint8()**: 归一化深度图到uint8
- **process_binary_image()**: 处理二值化图像
- **process_preprocessed_image()**: 处理预处理图像

#### 1.3 添加了必要的导入

```python
import math  # 添加到文件开头
```

### 2. index.html

**文件路径**: `/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui/index.html`

**主要修改**:

#### 2.1 更新了 debugState 对象

```javascript
const debugState = {
    depthImage: null,
    colorImage: null,
    binaryImage: null,
    preprocessedImage: null,
    refreshInterval: null,
    isRefreshing: false,
    params: {
        min_depth: 0,
        max_depth: 65535,
        contour_min_area: 10,
        contour_max_area: 100000,
        min_aspect: 0.3,
        max_aspect: 4.0,
        min_width: 60,
        min_height: 60,
        max_count: 3
    }
};
```

#### 2.2 重写了 Debug 功能函数

- **debugCaptureImage()**: 采集图像并启动自动刷新
- **debugRefresh()**: 手动刷新显示
- **debugRefreshImages()**: 从服务器获取最新图像并更新显示
- **updateDebugImage()**: 更新单个图像显示区域
- **startDebugAutoRefresh()**: 启动自动刷新定时器（500ms间隔）
- **stopDebugAutoRefresh()**: 停止自动刷新
- **updateDebugParam()**: 更新参数并刷新显示
- **debugSaveThresholds()**: 保存当前参数到配置文件
- **loadDebugParams()**: 加载保存的参数

#### 2.3 更新了 switchTab 函数

```javascript
// 如果切换到debug选项卡，加载参数并启动自动刷新
if (tabName === 'debug') {
    loadDebugParams();
    startDebugAutoRefresh();
} else {
    // 离开debug选项卡时停止自动刷新
    stopDebugAutoRefresh();
}
```

## 功能特性

### 1. 实时图像显示

- 四个图像区域同时显示：深度图、彩色图、二值化图、预处理图
- 自动刷新频率：500ms
- 切换到Debug选项卡时自动开始刷新
- 切换离开时自动停止刷新

### 2. 参数实时调节

- 9个可调节参数
- 滑动条调节，实时更新显示
- 参数值显示在滑动条旁边
- 参数更新立即反映在图像上

### 3. 参数保存和加载

- 参数保存到 `configs/debug_thresholds.json`
- 打开Debug选项卡时自动加载保存的参数
- 支持手动保存当前参数

### 4. 图像处理流程

#### 深度图处理
- 归一化到0-255
- JET颜色映射（蓝色=近，红色=远）
- 无效值标记为黑色

#### 二值化处理
- 深度阈值过滤
- 轮廓检测
- 多条件过滤（面积、宽高比、尺寸）
- 轮廓可视化

#### 预处理
- 白色背景
- 目标区域彩色叠加

## 与 trigger_depth.py 的对应关系

| trigger_depth.py 功能 | Web UI Debug 实现 |
|---------------------|------------------|
| ImageProcessor 类 | http_bridge_server.py 中的 process_* 函数 |
| OpenCV 窗口显示 | HTML 的四个图像显示区域 |
| cv2.createTrackbar | HTML range input 滑动条 |
| 键盘快捷键 | 网页按钮 |
| JSON 配置文件 | configs/debug_thresholds.json |
| 实时图像回调 | JavaScript 定时器轮询 |
| 参数调节即时生效 | oninput 事件触发 API 调用 |

## 配置文件格式

**文件路径**: `configs/debug_thresholds.json`

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

## API 接口说明

### 1. 采集图像

```
POST /api/debug/capture
Content-Type: application/json

Request:
{
  "camera_id": "207000152740"
}

Response:
{
  "success": true,
  "message": "图像采集成功"
}
```

### 2. 获取图像

```
POST /api/debug/get_images
Content-Type: application/json

Response:
{
  "success": true,
  "has_images": true,
  "depth_image": "data:image/jpeg;base64,...",
  "color_image": "data:image/jpeg;base64,...",
  "binary_image": "data:image/jpeg;base64,...",
  "preprocessed_image": "data:image/jpeg;base64,...",
  "stats": {
    "total_contours": 10,
    "filtered_contours": 3,
    "in_range_pixels": 15000
  },
  "features": {}
}
```

### 3. 更新参数

```
POST /api/debug/update_params
Content-Type: application/json

Request:
{
  "param_name": "min_depth",
  "param_value": 100
}

Response:
{
  "success": true,
  "message": "参数 min_depth 已更新为 100"
}
```

### 4. 获取参数

```
POST /api/debug/get_params
Content-Type: application/json

Response:
{
  "success": true,
  "params": {
    "min_depth": 0,
    "max_depth": 65535,
    ...
  }
}
```

### 5. 保存参数

```
POST /api/debug/save_thresholds
Content-Type: application/json

Response:
{
  "success": true,
  "message": "阈值已保存到 configs/debug_thresholds.json"
}
```

## 使用流程

1. 启动服务：`python3 scripts/http_bridge_server.py`
2. 打开浏览器访问 `http://localhost:8000`
3. 点击 **Debug** 选项卡
4. 点击 **📷 采集图像** 按钮
5. 拖动滑动条调节参数，观察图像变化
6. 找到合适的参数后，点击 **💾 保存阈值**

## 依赖要求

- Python 3.8+
- ROS2 Humble
- OpenCV (cv2)
- NumPy
- 相机话题：
  - `/camera/depth/image_raw`
  - `/camera/color/image_raw`

## 测试验证

### 测试步骤

1. **启动服务**
   ```bash
   cd /home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui
   python3 scripts/http_bridge_server.py
   ```

2. **检查ROS2话题**
   ```bash
   ros2 topic list | grep camera
   ```
   应该看到 `/camera/depth/image_raw` 和 `/camera/color/image_raw`

3. **访问网页**
   - 打开浏览器访问 `http://localhost:8000`
   - 切换到 Debug 选项卡

4. **测试功能**
   - 点击采集图像，检查四个图像区域是否显示
   - 调节滑动条，检查图像是否实时更新
   - 保存参数，检查配置文件是否生成
   - 刷新页面，检查参数是否正确加载

### 预期结果

- 图像每500ms自动刷新
- 参数调节立即生效
- 配置文件正确保存和加载
- 切换选项卡时自动停止/启动刷新

## 注意事项

1. **性能考虑**
   - 自动刷新间隔为500ms，可根据需要调整
   - 如果网络延迟大，可能需要增加间隔
   - 图像处理在服务器端进行，客户端只负责显示

2. **参数范围**
   - 深度阈值：0-65535（uint16范围）
   - 面积阈值：根据实际图像尺寸调整
   - 宽高比：0-10.0（0.1精度）

3. **故障排除**
   - 如果图像不显示，检查ROS2话题是否正常发布
   - 如果参数不生效，检查浏览器控制台的错误信息
   - 如果刷新太慢，检查网络连接和服务器负载

## 未来改进方向

1. **性能优化**
   - 添加WebSocket支持，实现真正的实时推送
   - 图像压缩优化，减少网络传输
   - 添加图像缓存机制

2. **功能扩展**
   - 添加更多图像处理参数
   - 支持多相机切换
   - 添加图像录制和回放功能
   - 支持参数预设保存和加载

3. **用户体验**
   - 添加参数重置按钮
   - 添加图像缩放和平移功能
   - 添加参数曲线图显示
   - 添加快捷键支持

## 文件清单

1. **修改的文件**
   - `http_bridge_server.py`: 添加了 Debug API 处理函数
   - `index.html`: 更新了 JavaScript 实现 Debug 功能

2. **新增的文件**
   - `DEBUG_USAGE.md`: 使用说明文档
   - `DEBUG_IMPLEMENTATION_SUMMARY.md`: 实现总结文档（本文件）

3. **生成的配置文件**
   - `configs/debug_thresholds.json`: 参数配置文件（运行时生成）

## 总结

本次实现成功地将 `trigger_depth.py` 的核心功能迁移到了网页 UI 的 Debug 选项卡中，实现了：

✅ 实时图像显示（深度、彩色、二值化、预处理）
✅ 参数实时调节（9个参数）
✅ 自动刷新机制（500ms间隔）
✅ 参数保存和加载
✅ 完整的API接口
✅ 详细的使用文档

用户现在可以通过网页界面方便地调试和优化深度图像处理参数，无需使用 OpenCV 窗口。
