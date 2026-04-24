# RobotWebTools 集成说明（优化与调整总览）

本文档概括 **`${ROBOTWEBTOOLS}`**（即 `IVG2.0/aubo_ros2_ws/src/robotwebtools`）内 **roslibjs / ros3djs / ros2djs** 的集成方式、已做优化、问题根因与运维流程。  
**逐文件、逐行的修改清单** 见同目录 **`SOURCE_CHANGES.md`**（主记录）。

---

## 1. 目录与交付物

| 路径 | 作用 |
|------|------|
| `roslibjs/` | roslibjs 源码与 `packages/roslib/dist/` 构建产物。 |
| `ros3djs/` | ros3djs 源码与 `build/ros3d*.js`。 |
| `ros2djs/` | ros2djs 源码与 `build/ros2d*.js`。 |
| `runtime_js_assets/` | **运行时汇总目录**（由 `copy_runtime_js_assets.sh` 生成，非手改）。 |
| `build_robotwebtools.sh` | 离线全量构建三库并调用复制脚本。 |
| `copy_runtime_js_assets.sh` | 仅复制 / 汇总 / 生成 importmap，**不对 JS 做语义补丁**。 |

---

## 2. 脚本职责（当前原则）

| 脚本 | 应做 | 不应做 |
|------|------|--------|
| `build_robotwebtools.sh` | 各子项目官方 `npm run build` / `grunt build`；最后调用 `copy_runtime_js_assets.sh`。 | 对 `build/` 内 JS 做 `sed`/`perl` 替换以「修兼容性」。 |
| `copy_runtime_js_assets.sh` | 拷贝 dist/build、示例 libs、roslib importmap 与 vendor；写统一 `importmap.js`。 | 对已拷贝的 ros3d 做 `ROSLIB.URDF_*` → `UrdfType` 等二次加工（已废弃，见 `SOURCE_CHANGES.md` 第 4 节）。 |

**原则**：库与库之间的 API 不一致，在 **源码** 修正并 **重新编译**；脚本保持可审计、可重复。

---

## 3. roslibjs v2 ↔ ros3djs：URDF 几何类型

### 3.1 现象（用户可见）

- URDF 参数（如 `/robot_state_publisher:robot_description`）能拉取，TF 正常，但 **3D 中无机械臂网格** 或 **无对 mesh 代理的请求**。

### 3.2 技术根因（摘要）

| 项目 | 行为 |
|------|------|
| **roslibjs v2** | `UrdfType` 枚举：`SPHERE=0, BOX=1, CYLINDER=2, MESH=3`；**不**导出 `ROSLIB.URDF_MESH` 等旧名。 |
| **ros3djs（上游）** | `Urdf.js` 使用 `ROSLIB.URDF_MESH` 等与旧 roslib 一致的比较。 |
| **结果** | `geometry.type === 3` 与 `ROSLIB.URDF_MESH === undefined` 比较失败 → mesh 分支不执行。 |

### 3.3 解决方案（本仓库）

- **源码**：`ros3djs/src/urdf/Urdf.js` 改为 `ROSLIB.UrdfType.MESH` / `BOX` / `CYLINDER` / `SPHERE`。
- **构建**：`cd ros3djs && npm run build`。
- **分发**：`copy_runtime_js_assets.sh` 原样复制 `build/ros3d.esm.js` 等。

详细行级对照、验证命令、合并上游步骤见 **`SOURCE_CHANGES.md`** 第 2–3、7 节。

### 3.4 为何不在 dashboard 里「补常量」

- 对 ESM `import * as ROSLIB` **赋值**易触发 **TypeError**，且与 ros3d 内部引用的 roslib **不是同一套可写对象**。
- 正式方案以 **ros3d 源码 + 构建** 为准。

---

## 4. 与 `aubo_ros2_web_dashboard` 的衔接

| 环节 | 说明 |
|------|------|
| **静态资源** | Launch / 网关将 `runtime_js_assets` 映射为浏览器可访问路径（如 `/js/robotwebtools/...`，以实际 launch 为准）。 |
| **importmap** | 页面加载 `runtime_js_assets/importmap.js`，将 `roslib`、`ros3d` 指到本地文件。 |
| **URDF** | `view3d/session.js` 使用 `ROS3D.UrdfClient`、`ROSLIB.Param`；mesh 基路径同源 API（如 `/api/ivg/robot-mesh/`）。 |
| **TF** | `view3d/tf_clients.js` 使用 `ROSLIB.ROS2TFClient`（tf2_web_republisher 链路）。 |

Dashboard 内 **调试埋点** 已在问题关闭后移除；与 robotwebtools 相关的 **源码级** 变更仍只记在 **`SOURCE_CHANGES.md` 第 6 节** 作交叉引用。

---

## 5. 推荐工作流（修改 ros3d 后）

1. 编辑 `ros3djs/src/urdf/Urdf.js`（或其它 `src/` 文件）。
2. `cd ros3djs && npm run build`。
3. `bash copy_runtime_js_assets.sh` 或 `bash build_robotwebtools.sh`。
4. 重启 Web / 网关，浏览器 **硬刷新**。
5. 使用 `SOURCE_CHANGES.md` **§3.5** 的 `rg` 命令确认产物。

---

## 6. 相关文档

| 文档 | 内容 |
|------|------|
| **`SOURCE_CHANGES.md`** | **完整修改记录**：路径、行对照、废弃的 perl 逻辑、验证与合并清单。 |
| **`../README.md`** | 离线构建、npm 离线变量、vendor 本地化、importmap 细节。 |
