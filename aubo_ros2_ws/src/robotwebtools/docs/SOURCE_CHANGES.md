# 源码与脚本修改记录（robotwebtools）

本文件记录 **相对上游 RobotWebTools** 在本工作区内的 **全部 intentional 变更**，便于审阅、合并上游与回归测试。路径默认相对于仓库根目录：

`IVG2.0/aubo_ros2_ws/src/robotwebtools/`

下文记为 **`${ROBOTWEBTOOLS}`**。

---

## 1. 快速索引

| 日期 | 范围 | 摘要 |
|------|------|------|
| 2026-04-23 | `ros3djs` | `Urdf.js`：`ROSLIB.URDF_*` → `ROSLIB.UrdfType.*`，与 roslibjs v2 对齐。 |
| 2026-04-23 | `copy_runtime_js_assets.sh` | 移除对 ros3d 构建产物的 `perl` 字符串替换；复制脚本仅负责汇总。 |
| 2026-04-23 | `docs/`、`README.md` | 新增/更新集成说明与本文档。 |

---

## 2. 背景：roslibjs v2 与 ros3djs 的 URDF 类型 API

### 2.1 roslibjs v2（未改上游，仅作对照）

- **类型定义文件**：`${ROBOTWEBTOOLS}/roslibjs/packages/roslib/src/urdf/UrdfTypes.ts`
- **枚举 `UrdfType` 数值**（与旧版 `ROSLIB.URDF_*` 约定一致）：

| 成员 | 数值 |
|------|------|
| `SPHERE` | 0 |
| `BOX` | 1 |
| `CYLINDER` | 2 |
| `MESH` | 3 |

- **公开导出**：`${ROBOTWEBTOOLS}/roslibjs/packages/roslib/src/RosLib.ts` 中 `export { UrdfType, ... } from "./urdf/UrdfTypes.ts"`。
- **浏览器 ESM 用法**：`import * as ROSLIB from 'roslib'` 时，可通过 **`ROSLIB.UrdfType.MESH`** 等访问；**不存在** `ROSLIB.URDF_MESH` 等旧属性名（未导出则命名空间上为 `undefined`）。

### 2.2 ros3djs 上游原始行为

- **文件**：`${ROBOTWEBTOOLS}/ros3djs/src/urdf/Urdf.js`
- **原始逻辑**：使用 `visual.geometry.type === ROSLIB.URDF_MESH` 及 `switch` 中的 `ROSLIB.URDF_BOX` 等，假定与 **旧版 roslib** 全局常量一致。

### 2.3 运行时问题（现象 → 根因）

1. `UrdfModel` 解析后，`geometry.type` 为 **数字**（例如 mesh 为 `3`）。
2. 在 roslibjs v2 下 `ROSLIB.URDF_MESH === undefined`。
3. 严格相等 `3 === undefined` 为假，**网格分支永远不执行** → 无 `MeshResource`、无对 `/api/ivg/robot-mesh/` 等路径的请求、3D 中看不到连杆网格。

### 2.4 不可行的业务层「补丁」（记录原因，避免重复尝试）

- 在应用代码中对 `import * as ROSLIB` 的命名空间执行 `ROSLIB.URDF_MESH = 3`：**ESM 模块命名空间对象对导出绑定为不可写**，严格模式下会 **TypeError**，且 **ros3d 包内** 自有 `import * as ROSLIB` 闭包，外层赋值本身也不可靠。
- 因此兼容必须在 **ros3d 源码** 中改为使用 **`ROSLIB.UrdfType.*`**，或通过 **重新构建** 使产物一致；不应依赖复制脚本的二次替换。

---

## 3. 修改条目 A：`ros3djs/src/urdf/Urdf.js`

### 3.1 元数据

| 项 | 值 |
|----|-----|
| **绝对路径（示例）** | `/home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/ros3djs/src/urdf/Urdf.js` |
| **相对路径** | `ros3djs/src/urdf/Urdf.js` |
| **修改日期** | 2026-04-23 |
| **性质** | 相对 RobotWebTools/ros3djs 上游的 **fork 补丁** |

### 3.2 行级对照（逻辑位置）

> 行号随文件演进可能偏移，以 **代码内容** 为准。

| 位置（约） | 修改前（上游语义） | 修改后（当前仓库） |
|------------|-------------------|-------------------|
| mesh 分支条件 | `if (visual.geometry.type === ROSLIB.URDF_MESH)` | `if (visual.geometry.type === ROSLIB.UrdfType.MESH)` |
| `createShapeMesh` / `switch` | `case ROSLIB.URDF_BOX:` | `case ROSLIB.UrdfType.BOX:` |
| 同上 | `case ROSLIB.URDF_CYLINDER:` | `case ROSLIB.UrdfType.CYLINDER:` |
| 同上 | `case ROSLIB.URDF_SPHERE:` | `case ROSLIB.UrdfType.SPHERE:` |

### 3.3 源码中增加的注释

在 mesh 分支上方增加一行说明（便于后续合并时识别 IVG 意图）：

```text
// roslibjs v2：几何类型使用 UrdfType 枚举（与旧版 ROSLIB.URDF_* 数值一致）
```

### 3.4 构建链路与受影响产物

`ros3djs` 使用 **Grunt**（`npm run build` → `grunt build`）：

1. **Pipe / transpile**：`src/**/*.js` → **`src-esm/**/*.js`**（生成物，勿手改 `src-esm`，应以 `src` 为准再 build）。
2. **Rollup**：`src-esm/index.js` → **`build/ros3d.esm.js`**、**`build/ros3d.cjs.js`**、**`build/ros3d.js`**、**`build/ros3d.min.js`**。

**修改 `src/urdf/Urdf.js` 后必须执行**：

```bash
cd "${ROBOTWEBTOOLS}/ros3djs"
npm run build
```

否则 `build/` 仍为旧逻辑。

### 3.5 验证构建是否已包含新 API（建议命令）

```bash
# 应无匹配（或仅出现在注释/示例 vendor 中）
rg "ROSLIB\.URDF_(MESH|BOX|CYLINDER|SPHERE)" "${ROBOTWEBTOOLS}/ros3djs/build/ros3d.esm.js"

# 应能匹配到 UrdfType
rg "ROSLIB\.UrdfType\.(MESH|BOX|CYLINDER|SPHERE)" "${ROBOTWEBTOOLS}/ros3djs/build/ros3d.esm.js"
```

---

## 4. 修改条目 B：`copy_runtime_js_assets.sh`

### 4.1 元数据

| 项 | 值 |
|----|-----|
| **路径** | `copy_runtime_js_assets.sh`（`${ROBOTWEBTOOLS}` 根下） |
| **修改日期** | 2026-04-23 |

### 4.2 已删除逻辑（历史存档，便于 diff 理解）

此前在复制 ros3d 到 `runtime_js_assets` 之后曾调用函数 **`patch_ros3d_roslib_v2_urdf`**，对以下文件就地 `perl -pi` 替换：

- `runtime_js_assets/build/ros3d/ros3d.esm.js`
- `runtime_js_assets/build/ros3d/ros3d.js`
- `runtime_js_assets/build/ros3d/ros3d.cjs.js`
- `runtime_js_assets/build/ros3d/ros3d.min.js`

**替换规则（已废弃，勿再使用）**：

```text
ROSLIB.URDF_MESH      → ROSLIB.UrdfType.MESH
ROSLIB.URDF_BOX       → ROSLIB.UrdfType.BOX
ROSLIB.URDF_CYLINDER  → ROSLIB.UrdfType.CYLINDER
ROSLIB.URDF_SPHERE    → ROSLIB.UrdfType.SPHERE
```

**删除原因**：

- 与用户约定一致：**复制脚本只负责「编译产物的收集与目录布局」**，不在此阶段做语义修补。
- 兼容性已前移至 **`ros3djs/src/urdf/Urdf.js`** + 正式 **`npm run build`**。

### 4.3 当前脚本仍保留的职责（非 ros3d 补丁）

- 从 `roslibjs/packages/roslib/dist/`、`ros3djs/build/`、`ros2djs/build/` **复制** JS。
- 复制示例 vendor（three、roslib.global.js 等）到 `runtime_js_assets/libs/`。
- 复制 / 生成 `roslib-esm/importmap.js` 与 **vendor**（含 `populate_roslib_vendor_from_node_modules`，在 `packages/roslib/vendor` 缺失时从 `roslibjs/node_modules` 拷贝依赖）。
- 生成根级 **`runtime_js_assets/importmap.js`**（`roslib` → `./build/roslib/RosLib.js`，`ros3d` → `./build/ros3d/ros3d.esm.js` 等）。

---

## 5. 文档类变更

| 路径 | 说明 |
|------|------|
| `docs/INTEGRATION_OVERVIEW.md` | 集成总览、脚本原则、问题现象与解决思路。 |
| `docs/SOURCE_CHANGES.md` | 本文档：全细节修改记录。 |
| `README.md` | 增加「相关文档」指向上述两篇。 |

---

## 6. 关联包：`aubo_ros2_web_dashboard`（不在 robotwebtools 目录内）

以下仅作 **交叉引用**，避免排查 URDF 时遗漏：

| 说明 | 路径（相对 `aubo_ros2_ws/src`） |
|------|--------------------------------|
| 3D 会话、URDF 参数、`ROS3D.UrdfClient` | `aubo_ros2_web_dashboard/web/public/js/view3d/session.js` |
| 左栏 URDF 独立 rosbridge 与 `IvgRos3dView3dSession` | `aubo_ros2_web_dashboard/web/public/js/vision_grasp/urdf_panel.js` |
| TF：`ROSLIB.ROS2TFClient` | `aubo_ros2_web_dashboard/web/public/js/view3d/tf_clients.js` |
| Launch 中 `robotwebtools` / `runtime_js_assets` | `aubo_ros2_web_dashboard/launch/web_dashboard.launch.py` |

曾用于调试的 **ingest / fetch 埋点** 已在问题修复后 **全部移除**；正式兼容不依赖 dashboard 内对 `ROSLIB` 的赋值。

**2026-04-23 起 — `aubo_ros2_web_dashboard` 前端精简（摘要）**

| 变更 | 说明 |
|------|------|
| 删除 `web/public/js/view3d/urdf_loader.js` | 自定义 Param/rosapi + `ROS3D.Urdf` 已无引用；3D 仅 `ROS3D.UrdfClient`（`session.js`）。 |
| 删除 `web/public/js/ivg_rosbridge_bytes.js` | 无脚本加载、无引用。 |
| 删除 `ivg_transport` 二进制相关 API | `onBinary` / `connectBinary` / `parseBrowserBinary` 等与当前 roslib JSON 栈无关。 |
| 移除「`view3d-use-topic-tf-only`」 | `IvgRos3dTfClient` 从未实现该分支；已从页面与 `session.js` 去掉。 |
| `ivg_runtime.js`：`rosbridgeWebSocketUrlFromRuntime` | 与 `ivg_transport` 共用，避免 WebSocket URL 规则分叉。 |
| `session.stop` | 释放 `UrdfClient` 前调用 `urdf.unsubscribeTf()`。 |
| `tf_clients.js` | `IvgRos3dTfClient` 缩进与模块头注释修正。 |

---

## 7. 合并上游 ros3djs 时的建议流程

1. 拉取/合并上游 `ros3djs`。
2. 检查 **`src/urdf/Urdf.js`** 是否仍使用 `ROSLIB.URDF_*`。
3. 若上游已改为 `UrdfType`，可 **删除** 本 fork 中与上游重复的差异，保留注释与否按团队规范。
4. 若上游未改，**重新应用** 本节「修改条目 A」的对照表。
5. 执行 `npm run build`，并对 `build/ros3d.esm.js` 做 **第 3.5 节** 的 `rg` 验证。
6. 执行 `copy_runtime_js_assets.sh` 或完整 `build_robotwebtools.sh`，刷新 `runtime_js_assets`。

---

## 8. 记录格式（后续追加条目时请沿用）

每条建议包含：

- **日期**
- **子项目 / 文件路径（相对 `${ROBOTWEBTOOLS}`）**
- **摘要**
- **原因 / 现象 / 根因**
- **具体 diff 或对照表**
- **构建或验证命令**
- **与上游合并时的注意事项**

---

## 9. 未修改的上游组件（对照用）

| 组件 | 说明 |
|------|------|
| **roslibjs** | 未为 URDF 兼容性改 TS 源码；运行时依赖其已存在的 `UrdfType`。本地化见 `README.md`（importmap、vendor）。 |
| **ros2djs** | 当前无与本 URDF 问题相关的补丁记录。 |
| **`ros3djs/examples/vendor/roslib.js`** | 示例内置旧版全局 roslib（含 `URDF_MESH` 等），与 **dashboard ESM 栈** 分离；排查 dashboard 时不要与 `runtime_js_assets/build/roslib/RosLib.js` 混淆。 |

---

## 8. roslib v2 已知移除的 API：`ServiceRequest`

### 8.1 背景

roslib v2 移除了 `ServiceRequest` 包装类。调用 Service 时直接传普通对象即可，不再需要 `new ROSLIB.ServiceRequest({})`。

### 8.2 影响范围

| 文件 | 行 | 当前代码 | 风险 |
|------|-----|---------|------|
| `ros3djs/src/interactivemarkers/InteractiveMarkerClient.js` | 72 | `new ROSLIB.ServiceRequest({})` | 若使用 InteractiveMarker 会抛 TypeError |

### 8.3 修复方案（如需启用 InteractiveMarker）

```diff
- var request = new ROSLIB.ServiceRequest({});
+ var request = {};
```

### 8.4 当前状态

IVG 项目不使用 InteractiveMarker 功能，此问题暂不影响运行。若后续需要，按 8.3 方案修复源码并重新 `npm run build` 喵~

---

## 9. roslib v2 API 兼容性速查表（ros3d 相关）

| roslib API | v1 | v2 | 对 ros3d 的影响 |
|-----------|-----|-----|---------------|
| `ROSLIB.Ros` | ✅ | ✅ `export { Ros }` | 无影响 |
| `ROSLIB.Topic` | ✅ | ✅ `export { Topic }` | 无影响 |
| `ROSLIB.Service` | ✅ | ✅ `export { Service }` | 无影响 |
| `ROSLIB.Param` | ✅ | ✅ `export { Param }` | 无影响 |
| `ROSLIB.UrdfModel` | ✅ | ✅ `export { UrdfModel }` | 无影响 |
| `ROSLIB.URDF_MESH` (3) | ✅ | ❌ 移除 | 已修：补丁① → `ROSLIB.UrdfType.MESH` |
| `ROSLIB.ServiceRequest` | ✅ | ❌ 移除 | 见第 8 节 |
| `ROSLIB.Pose` | ✅ | ✅ `export { Pose }` | 无影响 |
| `ROSLIB.Transform` | ✅ | ✅ `export { Transform }` | 无影响 |
| `ROSLIB.TFClient` | ✅ | ✅ `export { TFClient }` | 无影响 |
| `ROSLIB.ROS2TFClient` | — | ✅ v2 新增 | 无影响 |

> **设计原则**：RobotWebTools 上游多年不更新，roslib v2 的 API 变动需在本仓库自行适配。所有修改记录在此文档，补丁在源码层修复 + 重新编译，不对构建产物做二次加工喵~

*最后更新: 2026-05-16*
