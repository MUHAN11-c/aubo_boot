# 位姿文件管理功能更新文档

**更新日期**：2026-01-12  
**版本**：v1.0  
**功能**：位姿文件服务器端存储和管理

---

## 📋 功能概述

本次更新实现了位姿文件在服务器端的自动存储和管理功能，用户在前端记录的位姿数据会自动保存到服务器的 `config/poses/` 目录，加载时可以从服务器文件列表中选择。

---

## 🎯 主要改动

### 1. 目录结构变化

#### 新增目录
- **路径**：`config/poses/`
- **用途**：存储前端自动保存的位姿配置文件
- **文件命名**：`auto_hand_eye_poses_YYYYMMDD_HHMMSS.json`

#### 目录结构
```
config/
├── poses/                          # 新增：位姿文件存储目录
│   ├── auto_hand_eye_poses_20260112_141508.json
│   ├── auto_hand_eye_poses_20260112_142030.json
│   └── ...
├── auto_hand_eye_poses_optimized_10_v2.json
├── auto_hand_eye_poses_optimized.json
└── ...
```

---

### 2. 后端API接口

#### 2.1 保存位姿文件 API

**接口**：`POST /api/poses/save`

**功能**：将位姿数据保存到服务器 `config/poses/` 目录

**请求格式**：
```json
{
  "version": "4.0",
  "calibrationType": "eye-in-hand",
  "calibrationMethod": "pose-based-auto",
  "savedAt": "2026-01-12T14:15:08.201Z",
  "recordedPoses": [
    {
      "position": { "x": 0.468, "y": -0.074, "z": 0.518 },
      "orientation": { "x": 0.707, "y": 0.707, "z": 0, "w": 0 }
    },
    ...
  ]
}
```

**响应格式**：
```json
{
  "success": true,
  "filename": "auto_hand_eye_poses_20260112_141508.json",
  "filepath": "/path/to/config/poses/auto_hand_eye_poses_20260112_141508.json",
  "message": "位姿文件已保存: auto_hand_eye_poses_20260112_141508.json"
}
```

**错误响应**：
```json
{
  "success": false,
  "error": "错误信息"
}
```

#### 2.2 列出位姿文件 API

**接口**：`GET /api/poses/list`

**功能**：获取服务器上所有位姿文件列表

**响应格式**：
```json
{
  "success": true,
  "files": [
    {
      "filename": "auto_hand_eye_poses_20260112_141508.json",
      "size": 6325,
      "modified": "2026-01-12 14:15:08"
    },
    ...
  ],
  "count": 2
}
```

**错误响应**：
```json
{
  "success": false,
  "error": "错误信息"
}
```

#### 2.3 加载位姿文件 API

**接口**：`POST /api/poses/load`

**功能**：从服务器加载指定的位姿文件

**请求格式**：
```json
{
  "filename": "auto_hand_eye_poses_20260112_141508.json"
}
```

**响应格式**：
```json
{
  "success": true,
  "filename": "auto_hand_eye_poses_20260112_141508.json",
  "data": {
    "version": "4.0",
    "calibrationType": "eye-in-hand",
    "calibrationMethod": "pose-based-auto",
    "savedAt": "2026-01-12T14:15:08.201Z",
    "recordedPoses": [...]
  }
}
```

**错误响应**：
```json
{
  "success": false,
  "error": "错误信息"
}
```

---

### 3. 前端功能改动

#### 3.1 保存功能 (`handleSaveAllPosesToFile`)

**改动前**：
- 使用浏览器下载功能，保存到用户本地
- 文件名：`auto_hand_eye_poses_${Date.now()}.json`

**改动后**：
- 优先保存到服务器 `config/poses/` 目录
- 如果服务器保存失败，自动回退到本地下载
- 文件名：`auto_hand_eye_poses_YYYYMMDD_HHMMSS.json`（带时间戳）

**代码变更**：
```javascript
// 改动前：同步函数，直接下载
function handleSaveAllPosesToFile() {
    // ... 创建Blob并下载
    a.download = `auto_hand_eye_poses_${Date.now()}.json`;
    a.click();
}

// 改动后：异步函数，优先保存到服务器
async function handleSaveAllPosesToFile() {
    try {
        const response = await fetch('/api/poses/save', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(config)
        });
        // ... 处理响应，失败时回退到本地下载
    } catch (error) {
        // ... 回退到本地下载
    }
}
```

#### 3.2 加载功能 (`handleLoadAllPosesFromFile`)

**改动前**：
- 直接打开本地文件选择器
- 只能从本地文件系统选择文件

**改动后**：
- 优先显示服务器文件列表（模态对话框）
- 支持从服务器列表选择或本地文件选择
- 保留原有的本地文件选择功能

**新增函数**：
- `showPoseFileSelectionDialog(files)` - 显示文件选择对话框
- `loadPoseFileFromServer(filename)` - 从服务器加载文件
- `loadPoseFileFromLocal()` - 从本地文件加载
- `processLoadedPoseData(config)` - 处理加载的位姿数据（通用函数）

**代码变更**：
```javascript
// 改动前：直接打开文件选择器
function handleLoadAllPosesFromFile() {
    const input = document.createElement('input');
    input.type = 'file';
    input.onchange = function(e) { /* ... */ };
    input.click();
}

// 改动后：优先显示服务器文件列表
async function handleLoadAllPosesFromFile() {
    try {
        const listResponse = await fetch('/api/poses/list');
        const listResult = await listResponse.json();
        if (listResult.success && listResult.files.length > 0) {
            showPoseFileSelectionDialog(listResult.files);
            return;
        }
    } catch (error) {
        // 失败时使用本地文件选择
    }
    loadPoseFileFromLocal();
}
```

---

## 📁 修改的文件

### 后端文件

#### `hand_eye_calibration/hand_eye_calibration_node.py`

**位置**：`_register_routes()` 方法内，`/api/hand_eye/verify` 路由之后

**新增代码**：
- `@self.app.route('/api/poses/save', methods=['POST'])` - 保存位姿文件（约40行）
- `@self.app.route('/api/poses/list', methods=['GET'])` - 列出位姿文件（约45行）
- `@self.app.route('/api/poses/load', methods=['POST'])` - 加载位姿文件（约35行）

**代码行数**：约120行

### 前端文件

#### `web/static/script_v2_auto_calib_addon.js`

**修改的函数**：
- `handleSaveAllPosesToFile()` - 改为异步函数，添加服务器保存逻辑（约65行）
- `handleLoadAllPosesFromFile()` - 改为异步函数，添加服务器文件列表显示（约30行）

**新增的函数**：
- `showPoseFileSelectionDialog(files)` - 显示文件选择对话框（约85行）
- `loadPoseFileFromServer(filename)` - 从服务器加载文件（约30行）
- `loadPoseFileFromLocal()` - 从本地文件加载（约25行）
- `processLoadedPoseData(config)` - 处理加载的位姿数据（约35行）

**代码行数**：新增约270行

---

## 🔄 使用流程

### 保存位姿文件

1. 用户在Web界面记录位姿数据
2. 点击"💾 保存"按钮
3. 系统调用 `/api/poses/save` API
4. 文件保存到 `config/poses/auto_hand_eye_poses_YYYYMMDD_HHMMSS.json`
5. 如果保存失败，自动回退到本地下载

### 加载位姿文件

1. 用户在Web界面点击"📂 加载"按钮
2. 系统调用 `/api/poses/list` API 获取文件列表
3. 如果有服务器文件，显示文件选择对话框
4. 用户可以选择：
   - 从服务器列表选择文件（显示文件信息：文件名、大小、修改时间）
   - 点击"📁 从本地文件加载"按钮，使用本地文件选择器
5. 如果服务器没有文件或获取失败，直接使用本地文件选择器

---

## ✅ 兼容性说明

### 向后兼容

- ✅ **保留本地文件选择功能**：如果服务器保存/加载失败，自动回退到本地操作
- ✅ **支持v4.0格式**：文件格式保持不变
- ✅ **API错误处理**：所有API调用都有错误处理，不会影响原有功能

### 数据格式

- ✅ **文件格式不变**：仍使用v4.0格式（JSON）
- ✅ **数据结构不变**：`recordedPoses` 字段保持不变
- ✅ **字段兼容性**：完全兼容现有的位姿数据格式

---

## 🧪 测试建议

### 功能测试

1. **保存功能测试**：
   - 记录3个以上位姿
   - 点击"💾 保存"按钮
   - 验证文件是否保存到 `config/poses/` 目录
   - 验证文件内容是否正确

2. **加载功能测试**：
   - 点击"📂 加载"按钮
   - 验证文件列表是否正确显示
   - 选择服务器文件，验证加载是否成功
   - 选择本地文件，验证加载是否成功

3. **错误处理测试**：
   - 模拟服务器错误（如目录不存在）
   - 验证是否正常回退到本地操作
   - 验证错误提示是否正确

### 边界情况测试

1. **空文件列表**：服务器没有文件时，应显示本地文件选择器
2. **网络错误**：网络断开时，应回退到本地操作
3. **文件损坏**：损坏的文件应显示错误提示
4. **大量文件**：测试文件列表较多时的显示性能

---

## 📊 技术细节

### 文件命名规则

- **格式**：`auto_hand_eye_poses_YYYYMMDD_HHMMSS.json`
- **时间戳格式**：`YYYYMMDD_HHMMSS`（年月日_时分秒）
- **示例**：`auto_hand_eye_poses_20260112_141508.json`

### 文件路径获取

后端使用以下逻辑获取config目录：

```python
try:
    from ament_index_python.packages import get_package_share_directory
    config_dir = os.path.join(
        get_package_share_directory('hand_eye_calibration'),
        'config'
    )
except:
    config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')
```

### 安全性

- ✅ **路径安全检查**：加载文件时验证文件路径，防止目录遍历攻击
- ✅ **文件类型验证**：只处理 `.json` 文件
- ✅ **文件名验证**：只处理 `auto_hand_eye_poses_` 开头的文件

---

## 🐛 已知问题

无

---

## 📝 后续改进建议

1. **文件管理界面**：
   - 添加文件删除功能
   - 添加文件重命名功能
   - 添加文件预览功能

2. **文件搜索**：
   - 支持按日期范围筛选
   - 支持按位姿数量筛选
   - 支持文件名搜索

3. **批量操作**：
   - 支持批量删除文件
   - 支持批量导出文件

4. **文件版本管理**：
   - 添加文件版本控制
   - 支持文件回滚

---

## 📚 相关文档

- [Config目录说明](./README.md)
- [位姿配置版本对比](./COMPARISON.md)
- [API接口文档](../README.md)

---

**文档版本**：v1.0  
**最后更新**：2026-01-12  
**维护者**：开发团队
