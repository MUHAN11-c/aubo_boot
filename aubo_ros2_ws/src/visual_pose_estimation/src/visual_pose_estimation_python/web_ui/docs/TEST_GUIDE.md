# Web UI 测试指南

## ✅ 服务已启动成功

从日志可以看到：

```
✓ Web UI 启动成功！
访问地址:
  http://localhost:8089
  http://localhost:8089/index.html
```

**端口状态**: ✅ 8089端口正在监听  
**ROS2节点**: ✅ http_bridge_node_python 已初始化  
**工作空间**: ✅ /home/mu/IVG/aubo_ros2_ws 已加载

---

## 🌐 访问Web界面

### 方法1: 使用浏览器（推荐）

1. 打开浏览器（Chrome、Firefox或Edge）
2. 访问以下任一地址：
   ```
   http://localhost:8089
   http://localhost:8089/index.html
   ```

3. 您应该看到：
   - 紫色渐变背景的页面
   - "Visual Pose Estimation - Python Edition" 标题
   - 图像上传区域
   - 工件ID输入框
   - 模板管理区域

### 方法2: 使用命令行测试

```bash
# 测试主页
curl http://localhost:8089/ | head -20

# 测试状态接口
curl http://localhost:8089/status

# 测试模板列表（需要POST请求）
curl -X POST http://localhost:8089/api/list_templates \
  -H "Content-Type: application/json" \
  -d '{}'
```

---

## 🧪 功能测试清单

### 1. 基础功能测试

- [ ] 打开Web页面
- [ ] 看到完整的用户界面
- [ ] 状态指示灯显示为绿色
- [ ] 顶部显示"系统就绪"

### 2. 图像上传测试

- [ ] 点击上传区域可以选择文件
- [ ] 拖拽图像文件到上传区域
- [ ] 图像预览正常显示
- [ ] 可以清除已上传的图像

### 3. 姿态估计测试

- [ ] 上传一张测试图像
- [ ] 输入工件ID（例如: test_01）
- [ ] 点击"估计姿态"按钮
- [ ] 等待处理（显示加载动画）
- [ ] 查看结果（检测数量、置信度、可视化图像）

### 4. 模板管理测试

- [ ] 点击"刷新模板列表"按钮
- [ ] 模板列表正常显示
- [ ] 可以选择模板
- [ ] 选中的模板高亮显示
- [ ] 工件ID自动填充

### 5. 模板标准化测试

- [ ] 选择一个工件
- [ ] 点击"标准化模板"按钮
- [ ] 确认操作提示
- [ ] 等待处理完成
- [ ] 查看处理结果

---

## 🔍 故障排查

### 问题1: 浏览器无法访问页面

**症状**: 访问 http://localhost:8089 显示"无法访问"

**检查**:
```bash
# 确认服务在运行
lsof -i:8089

# 查看服务日志
cat /home/mu/.cursor/projects/home-mu-IVG/terminals/19981.txt
```

**解决**:
- 如果端口未监听，重新启动: `./start_web_ui.sh`
- 如果显示错误，查看日志中的错误信息

### 问题2: 页面显示但功能不工作

**症状**: 点击按钮无响应，或显示错误

**检查**:
```bash
# 确认ROS2服务在运行
ros2 service list | grep estimate_pose
ros2 service list | grep list_templates
```

**解决**:
- 确保 visual_pose_estimation_python 节点在运行
- 重启主节点: 
  ```bash
  ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
  ```

### 问题3: 姿态估计失败

**症状**: 点击"估计姿态"后显示错误

**可能原因**:
1. 工件ID不存在
2. 模板未配置
3. 图像格式不支持

**解决**:
1. 检查工件ID是否正确
2. 使用"刷新模板列表"查看可用的工件
3. 确保图像是JPG或PNG格式

---

## 📊 预期行为

### 正常的姿态估计流程

1. **上传图像** → 图像显示在预览区
2. **输入工件ID** → 输入框显示文本
3. **点击估计姿态** → 显示"正在处理..."加载动画
4. **等待3-5秒** → 处理完成
5. **显示结果**:
   - 右上角显示"成功检测到 N 个对象"
   - 检测数量更新
   - 平均置信度更新
   - 可视化图像显示检测框
   - 底部显示各个检测对象的详细信息

### 正常的模板列表流程

1. **点击刷新模板列表** → 显示加载动画
2. **等待1-2秒** → 处理完成
3. **显示结果**:
   - 顶部显示"加载了 N 个模板"
   - 模板列表显示缩略图和信息
   - 可以点击选择模板

---

## 💻 开发者工具

### 浏览器控制台

按 `F12` 或 `Ctrl+Shift+I` 打开开发者工具，查看：

1. **Console** 标签:
   - JavaScript错误
   - API请求响应
   - 调试信息

2. **Network** 标签:
   - HTTP请求
   - 响应时间
   - 响应内容

### 有用的控制台命令

在浏览器控制台执行：

```javascript
// 检查服务状态
fetch('/status').then(r => r.json()).then(console.log)

// 测试模板列表API
fetch('/api/list_templates', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: '{}'
}).then(r => r.json()).then(console.log)
```

---

## 🎯 下一步

如果所有测试通过：

1. ✅ 开始使用Web UI进行实际工作
2. ✅ 参考 [使用示例.md](docs/使用示例.md) 了解更多用法
3. ✅ 查看 [README.md](README.md) 获取完整文档

如果遇到问题：

1. 📋 查看 [快速开始.md](docs/快速开始.md) 的常见问题
2. 📋 检查ROS2节点和服务状态
3. 📋 查看浏览器控制台错误信息
4. 📋 查看服务器日志输出

---

## 📞 获取帮助

如果问题仍未解决：

1. 收集以下信息：
   - 错误信息（浏览器控制台 + 终端输出）
   - ROS2服务状态 (`ros2 service list`)
   - 浏览器和操作系统版本

2. 查看相关文档：
   - README.md
   - 快速开始.md
   - 使用示例.md

---

**测试完成后记得查看**: [QUICK_REFERENCE.md](QUICK_REFERENCE.md) 快速参考卡

**祝测试顺利！** 🚀
