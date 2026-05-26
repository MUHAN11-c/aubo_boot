# 监控记录重命名 + 调试页面加载记录

## 需求

1. 监控页面 (`tf_monitor_panel`) 记录的快照可以重命名，不再只有序号和时间戳
2. 调试页面 (`debug_panel`) 能查看已保存的记录，选择后加载关节角/位姿到运动控制输入框

## 方案选型依据

### localStorage vs IndexedDB vs BroadcastChannel

| 维度 | localStorage | IndexedDB | BroadcastChannel |
|------|-------------|-----------|-----------------|
| 容量 | 5MB（够 1000+ 条快照）| 无限制 | 无持久化 |
| API | 同步，极简 | 异步，复杂 | 仅消息传递 |
| 跨标签页通知 | `storage` 事件（内建）| 需额外方案 | 原生支持 |
| 持久化 | 是（刷新不丢）| 是 | 否（标签页关闭即丢）|

**结论**: 记录量小 + 低频变更 + 需要持久化 → **localStorage + storage 事件** 是最优解。

- Hello Robot 官方 `stretch_web_teleop` 的 waypoint 管理同样使用 localStorage JSON 序列化模式（[源码](https://github.com/hello-robot/stretch_web_teleop)）
- 项目已有 `log-bus.js` 使用 IndexedDB + BroadcastChannel 是因为日志量巨大（5000条环形缓冲+高频写入），本场景不需要
- 2025 年社区共识：轻量状态同步优先用 localStorage + storage 事件；高频实时同步才加 BroadcastChannel

### 单数组 vs 分项 key

Stretch Web Teleop 使用分项 key（`map_[poseName]`、`recording_[recordingName]`）因其录制的轨迹序列可达数百 KB，需按需加载单个记录避免全量解析。本场景每条快照仅约 1KB，全量加载开销可忽略（100条 = 100KB），使用单数组 key 更简洁，且重命名操作只需更新数组元素的一个字段（分项 key 方案需复制/删除 key）。

### 内联编辑: `<span>` → `<input>` 替换 vs contenteditable

`contenteditable` 存在跨浏览器行为不一致、无法限制单行、Enter 键行为不可控等问题。`<span>` 点击替换为 `<input>` 是内联编辑的成熟模式，行为完全可控，且天然支持 `maxlength`、`Enter`/`Escape` 键盘事件。

## 数据模型

```javascript
// localStorage key: "ivg_monitor_records" → JSON Array
[{
  name: "咖啡拉花位置",       // 默认 "快照 #N"，用户可编辑，最长 60 字符
  timestamp: "2026-05-26T...",
  mode: "real" | "simulation" | "unknown",
  end_effector: {
    position_m: { x, y, z },
    orientation_quaternion: { x, y, z, w },
    euler_rpy_deg: { roll, pitch, yaw }
  },
  joints: [
    { name: "shoulder_joint", position_rad: ..., position_deg: ... },
    // ... 共 6 个关节
  ]
}]
```

## 文件变更

### 1. 新建 `js/core/record_store.js`

```javascript
const KEY = 'ivg_monitor_records';
const MAX_NAME_LEN = 60;

export function loadRecords() {
  try { return JSON.parse(localStorage.getItem(KEY)) || []; }
  catch { return []; }
}

export function saveRecords(arr) {
  try { localStorage.setItem(KEY, JSON.stringify(arr)); return true; }
  catch (e) { console.warn('[record_store] localStorage 写入失败', e); return false; }
}

export function clearRecords() { localStorage.removeItem(KEY); }
```

### 2. 修改 `tf_monitor_panel.js`

| 函数 | 变更 |
|------|------|
| `records` 初始化 | `loadRecords()` 替代 `[]` |
| `onRecordSnapshot()` | 新增 `name: "快照 #" + (records.length + 1)` 字段；`saveRecords()` 持久化 |
| **新增** `onRenameRecord(index)` | `<span>` → `<input>` 替换；Enter/失焦保存；Escape 取消；空白回退默认名 |
| **新增** `onDeleteRecord(index)` | `confirm("删除记录「xxx」？")` → `.splice(i,1)` → `saveRecords()` → `updateHistoryUI()` |
| `onClearHistory()` | `confirm("确定清空全部 N 条记录？")` → `clearRecords()` → `records = []` |
| `updateHistoryUI()` | 每条渲染：可点击名称 + 元信息行（时间/模式/关节摘要）+ 删除按钮 |

### 3. 修改 `tf_monitor_panel.html`

- `export-history` 条目结构改为两行：名称行 + 信息行
- CSS 新增: `.record-name-edit`（内联编辑输入框）、`.record-delete-btn`（删除按钮）

### 4. 修改 `debug_panel.js`

- `import { loadRecords } from './core/record_store.js'`
- **新增** `setupRecordLoader()`:
  - 渲染 `<select id="dbg-record-select">` + 详情预览 `<div>` + 两个 action 按钮
  - `refreshRecordSelect()`: 从 `loadRecords()` 重建下拉选项，保留当前选中项
  - `onSelectRecord()`: 选中后预览区显示关节角(rad) + 位姿(XYZ + RPY)
  - `onLoadJoints()`: `records[idx].joints[i].position_rad` → `$('dbg-j'+(i+1)).value`
  - `onLoadPose()`: `position_m` / `orientation_quaternion` → MoveL 输入框
  - `window.addEventListener('storage', e => { if (e.key === KEY) refreshRecordSelect(); })`
- 在 `init()` 中调用 `setupRecordLoader()`

### 5. 修改 `debug_panel.html`

在关节空间运动卡片上方新增"记录快照"卡片：
- `<select>` 下拉（未选择时显示 "-- 请选择 --"）
- 选中后显示详情：模式标签、关节角(rad)列表、位姿 XYZ + RPY
- "加载关节角到 MoveJ" 按钮（未选择时 disabled）
- "加载位姿到 MoveL" 按钮（未选择时 disabled）
- 操作结果提示区（绿色成功 / 黄色警告）

## 交互规格

### 重命名
1. 点击名称 `<span>` → 替换为 `<input>`，值 = 原名称，自动 focus + 全选
2. Enter → 保存 `input.value.trim()` 或回退默认名 → `saveRecords()` → 恢复 `<span>`
3. 失焦 (blur) → 同 Enter
4. Escape → 恢复原名称，不保存 → 恢复 `<span>`
5. 空白名称 → 自动回退为 "快照 #N"
6. 超长名称 → `maxlength="60"` 硬限制

### 删除
1. 点击 ✕ → `confirm("删除记录「xxx」？")`
2. 确认 → `splice(i,1)` → `saveRecords()` → `updateHistoryUI()`

### 清空
1. 点击"清空记录" → `confirm("确定清空全部 N 条记录？此操作不可恢复。")`
2. 确认 → `clearRecords()` → `records = []` → `updateHistoryUI()`

### 调试页面加载
1. 下拉选择记录 → 预览区立即显示关节角(rad 保留 4 位) + 位姿摘要(XYZ 保留 3 位 + RPY 角度)
2. 点击"加载关节角" → J1-J6 填入 `position_rad`，绿色提示 "✅ 已加载关节角"
3. 点击"加载位姿" → XYZ + QX/QY/QZ/QW 填入，绿色提示 "✅ 已加载位姿"
4. 未选择记录时两个按钮均为 `disabled` 状态

### 跨页面同步
- 调试页面监听 `window` 的 `storage` 事件，过滤 `key === 'ivg_monitor_records'`
- 监控页面新增/重命名/删除/清空 → localStorage 变更 → 调试页面自动刷新下拉列表
- 调试页面重新打开时（DOMContentLoaded）从 localStorage 读取最新数据

### 边界情况
- localStorage 满：`saveRecords()` 返回 false，`onRecordSnapshot()` 弹 `alert("存储空间不足，请清理旧记录")`
- 空记录列表：下拉显示 "-- 暂无记录 --" 并禁用；调试页面卡片显示空状态提示
- 同名记录：不检测（允许同名，通过序号区分）

## 不涉及

- 后端改动：无
- 记录内容变更：保持现有 `buildSnapshot()` 数据格式不变，仅增加 `name` 字段
- 调试页面现有功能不变：新增卡片为增量添加
