# 修复说明 - ListTemplates 服务接口问题

## ✅ 已修复的问题

**错误信息**: `'ListTemplates_Request' object has no attribute 'templates_dir'`

**原因**: 服务定义使用 `workpiece_id` 字段，但代码中使用了错误的字段名 `templates_dir`

## 🔧 修复内容

### 1. HTTP桥接服务器 (`scripts/http_bridge_server.py`)

**修改内容**:
- 将 `list_templates(templates_dir="")` 改为 `list_templates(workpiece_id="")`
- 将 `request.templates_dir` 改为 `request.workpiece_id`
- 移除了响应中不存在的字段 (`template_paths`, `pose_ids`)
- 简化了模板列表处理逻辑

### 2. 主节点通信模块 (`visual_pose_estimation_python/ros2_communication.py`)

**修改内容**:
- 将 `request.templates_dir` 改为 `request.workpiece_id`
- 添加了对 `workpiece_id` 参数的正确处理
- 正确设置响应的 `success` 和 `error_message` 字段
- 添加了 `workpiece_ids` 字段到响应中

## 📋 服务接口定义

根据 `/home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/interface/srv/ListTemplates.srv`:

### 请求 (Request)
```
string workpiece_id  # 工件ID（可选，如果为空则列出所有工件）
```

### 响应 (Response)
```
bool success                # 是否成功
string error_message        # 错误消息（如果失败）
string[] template_ids       # 模板ID列表
string[] workpiece_ids      # 工件ID列表
```

## 🚀 如何应用修复

修复已经应用到代码中。现在需要：

### 步骤 1: 重新编译（已完成）

```bash
cd /home/mu/IVG/aubo_ros2_ws
colcon build --packages-select visual_pose_estimation_python --symlink-install
```

✅ 编译成功

### 步骤 2: 重启主节点

如果主节点正在运行，需要重启它：

```bash
# 终止旧进程
pkill -f visual_pose_estimation_python

# 重新启动
source /home/mu/IVG/aubo_ros2_ws/install/setup.bash
ros2 launch visual_pose_estimation_python visual_pose_estimation_python.launch.py
```

### 步骤 3: 启动 Web UI

在新终端中：

```bash
cd /home/mu/IVG/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python/web_ui
./start_web_ui.sh
```

**预期输出**:
```
========================================
✓ Web UI 启动成功！
========================================

访问地址:
  http://localhost:8089
  http://localhost:8089/index.html
```

## 🧪 测试修复

### 测试 1: 使用浏览器

1. 打开浏览器访问 http://localhost:8089
2. 点击"刷新模板列表"按钮
3. 应该能够正常列出模板，不再显示错误

### 测试 2: 使用命令行

```bash
# 测试列出所有模板
ros2 service call /list_templates interface/srv/ListTemplates "{workpiece_id: ''}"

# 测试列出特定工件
ros2 service call /list_templates interface/srv/ListTemplates "{workpiece_id: 'workpiece_01'}"
```

### 测试 3: 使用 curl

```bash
# 测试 HTTP API
curl -X POST http://localhost:8089/api/list_templates \
  -H "Content-Type: application/json" \
  -d '{"workpiece_id": ""}'
```

**预期响应**:
```json
{
  "success": true,
  "templates": [...],
  "count": N,
  "workpiece_ids": [...]
}
```

## 📊 修改对比

### 之前（错误）
```python
# HTTP桥接服务器
def list_templates(self, templates_dir=""):
    request = ListTemplates.Request()
    request.templates_dir = templates_dir  # ❌ 字段不存在

# 主节点
template_root = request.templates_dir if request.templates_dir else self.template_root  # ❌ 字段不存在
```

### 之后（正确）
```python
# HTTP桥接服务器
def list_templates(self, workpiece_id=""):
    request = ListTemplates.Request()
    request.workpiece_id = workpiece_id  # ✅ 正确字段

# 主节点
if request.workpiece_id:  # ✅ 使用正确字段
    workpiece_path = template_root_path / request.workpiece_id
    ...
```

## ⚠️ 注意事项

1. **主节点必须重启** - 修改后的代码只有在重启主节点后才会生效
2. **Web UI 需要重启** - 修改后的 HTTP 服务器也需要重启
3. **环境变量** - 确保正确 source 了工作空间环境

## 🔍 故障排除

### 问题: 仍然报相同错误

**解决方法**:
1. 确认已重新编译: `colcon build --packages-select visual_pose_estimation_python`
2. 确认已重启主节点
3. 确认已重启 Web UI
4. 检查是否 source 了正确的工作空间

### 问题: 模板列表为空

**解决方法**:
1. 检查模板目录是否存在
2. 确认模板目录路径配置正确
3. 查看主节点日志: `ros2 node info /visual_pose_estimation_python_node`

### 问题: 服务调用超时

**解决方法**:
1. 检查主节点是否运行: `ros2 node list | grep visual`
2. 检查服务是否可用: `ros2 service list | grep list_templates`
3. 查看主节点日志输出

## 📞 获取帮助

如果问题仍然存在：

1. 查看主节点日志
2. 查看 Web UI 终端输出
3. 使用浏览器开发者工具查看网络请求
4. 参考 TEST_GUIDE.md 进行完整测试

---

**修复日期**: 2026-01-20  
**修复状态**: ✅ 已完成  
**需要操作**: 重启主节点和 Web UI
