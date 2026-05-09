# robotwebtools 本地化与离线构建说明

本文档记录本目录下 `roslibjs`、`ros3djs`、`ros2djs` 的拉取版本、构建流程、离线本地化改造细节，以及在其他功能包中的调用方式。

**相关文档**

- `docs/INTEGRATION_OVERVIEW.md`：集成总览、脚本职责、URDF 问题根因与推荐工作流（精简版）。
- `docs/SOURCE_CHANGES.md`：**完整修改记录**（路径、行级对照、`UrdfType` 数值表、构建链路与产物、已删除的 `perl` 补丁存档、验证命令、合并上游 checklist、与 `aubo_ros2_web_dashboard` 交叉引用）。

---

## 1. 目录与目标

- 工作目录：`/home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools`
- 目标：
  - 拉取 RobotWebTools 官方稳定版本
  - 按官方构建流程产出构建结果
  - 提供仅构建脚本（不参与拉取）
  - 将运行依赖与示例全部本地化，尽量不依赖外网

---

## 2. 官方仓库与版本（稳定 release）

通过 GitHub release 查询并拉取的版本如下：

- `RobotWebTools/roslibjs`: `2.1.0`
- `RobotWebTools/ros3djs`: `1.1.0`
- `RobotWebTools/ros2djs`: `0.10.0`

拉取后目录：

- `robotwebtools/roslibjs`
- `robotwebtools/ros3djs`
- `robotwebtools/ros2djs`

---

## 3. 构建脚本

脚本文件：

- `robotwebtools/build_robotwebtools.sh`

设计原则：

- 仅构建，不拉取
- 默认全量构建（`roslibjs` + `ros3djs` + `ros2djs`）
- 离线优先（`NPM_CONFIG_OFFLINE=true`）
- 任一步失败立即退出（`set -euo pipefail`）

使用方式：

```bash
bash /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/build_robotwebtools.sh
```

构建产物位置：

- roslibjs：`roslibjs/packages/roslib/dist/`
- ros3djs：`ros3djs/build/`
- ros2djs：`ros2djs/build/`

---

## 4. `node_modules` 的作用说明

- `node_modules` 主要用于**构建阶段**（打包、转译、压缩、lint、test 依赖）
- 业务使用通常读取 `build/` 或 `dist/` 产物，不直接依赖整个 `node_modules`
- 但若需再次构建，仍需要本地依赖（`node_modules` 或可用离线缓存）

---

## 5. API 调用方式分析（基于源码/产物）

### 5.1 ros3djs

- `ros3d.js`（IIFE）为浏览器全局模式，依赖全局 `ROSLIB`
- `ros3d.esm.js` 为 ESM 模块模式，内部 `import * as ROSLIB from 'roslib'`
- 说明：
  - 传统 `<script>` 方式：需先提供全局 `ROSLIB`
  - 工程化方式（Vite/Webpack/Rollup）：优先使用 ESM

### 5.2 ros2djs

- `ros2d.js` 为全局模式（`ROS2D`）
- 依赖全局 `ROSLIB` 与 `createjs`（EaselJS）

### 5.3 roslibjs

- 当前版本 `dist/RosLib.js` 为 ESM 产物
- 若直接浏览器运行，需要 import map 提供依赖映射

---

## 6. 全本地化改造内容

为满足“不需要联网”的目标，已执行如下改造。

本节给出“实现级细节”：具体改了哪些文件、依赖从哪里拷贝到哪里、HTML 替换规则是什么、如何验证。

### 6.1 roslibjs import map 本地化

修改文件：

- `roslibjs/packages/roslib/importmap.js`

实现细节：

- 原 import map 的 `scopes` 中存在 `https://unpkg.com/...` 远程映射
- 现改为只保留 `imports`，并全部指向 `./vendor/...` 本地路径
- `roslib` 入口保留为本地 `./dist/RosLib.js`

当前本地 import map 关键映射：

- `bson -> ./vendor/bson/lib/bson.mjs`
- `cbor2 -> ./vendor/cbor2/lib/index.js`
- `eventemitter3 -> ./vendor/eventemitter3/dist/eventemitter3.esm.js`
- `fast-png -> ./vendor/fast-png/lib/index.js`
- `uuid -> ./vendor/uuid/dist/index.js`
- `ws -> ./vendor/ws/browser.js`
- `@cto.af/wtf8 -> ./vendor/@cto.af/wtf8/lib/index.js`
- `fflate -> ./vendor/fflate/esm/browser.js`
- `iobuffer -> ./vendor/iobuffer/lib/iobuffer.js`

依赖拷贝来源与目标（从 `roslibjs/node_modules` 到 `packages/roslib/vendor`）：

- `bson/lib -> vendor/bson/lib`
- `cbor2/lib -> vendor/cbor2/lib`
- `eventemitter3/dist/eventemitter3.esm.js -> vendor/eventemitter3/dist/eventemitter3.esm.js`
- `fast-png/lib -> vendor/fast-png/lib`
- `uuid/dist -> vendor/uuid/dist`
- `ws/browser.js -> vendor/ws/browser.js`
- `@cto.af/wtf8/lib -> vendor/@cto.af/wtf8/lib`
- `fflate/esm -> vendor/fflate/esm`
- `iobuffer/lib -> vendor/iobuffer/lib`

### 6.2 ros3djs 示例本地化

目标目录：

- `ros3djs/examples/vendor/`

本地 vendor 文件来源：

- `ros3djs/node_modules/three/build/three.js -> examples/vendor/three.js`
- `ros3djs/node_modules/eventemitter2/lib/eventemitter2.js -> examples/vendor/eventemitter2.js`
- `ros3djs/node_modules/roslib/build/roslib.js -> examples/vendor/roslib.js`
- `ros3djs/node_modules/three/examples/js/loaders/ColladaLoader.js -> examples/vendor/ColladaLoader.js`
- `ros3djs/node_modules/three/examples/js/loaders/STLLoader.js -> examples/vendor/STLLoader.js`

HTML 替换规则（`ros3djs/examples/**/*.html`）：

- `https://static.robotwebtools.org/threejs/current/three.js`
  -> `./vendor/three.js`（`html-import` 子目录为 `../vendor/three.js`）
- `https://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js`
  -> `./vendor/eventemitter2.js`（`html-import` 子目录为 `../vendor/eventemitter2.js`）
- `https://static.robotwebtools.org/roslibjs/current/roslib.js`
  -> `./vendor/roslib.js`（`html-import` 子目录为 `../vendor/roslib.js`）

URDF 示例额外处理：

- `ros3djs/examples/urdf.html` 中
  - `https://static.robotwebtools.org/threejs/current/ColladaLoader.js`
    -> `./vendor/ColladaLoader.js`
  - `https://static.robotwebtools.org/threejs/current/STLLoader.js`
    -> `./vendor/STLLoader.js`
  - `path : 'http://resources.robotwebtools.org/'`
    -> `path : '/'`

备注：`path : '/'` 表示 URDF 资源改由本地静态服务根路径提供。

### 6.3 ros2djs 示例本地化

目标目录：

- `ros2djs/examples/vendor/`

本地 vendor 文件来源：

- `ros2djs/node_modules/easeljs/lib/easeljs.js -> examples/vendor/easeljs.js`
- `ros2djs/node_modules/eventemitter2/lib/eventemitter2.js -> examples/vendor/eventemitter2.js`
- `ros3djs/node_modules/roslib/build/roslib.js -> examples/vendor/roslib.js`

HTML 替换规则（`ros2djs/examples/**/*.html`）：

- `https://static.robotwebtools.org/EaselJS/current/easeljs.js`
  -> `./vendor/easeljs.js`
- `https://static.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js`
  -> `./vendor/eventemitter2.js`
- `https://static.robotwebtools.org/roslibjs/current/roslib.js`
  -> `./vendor/roslib.js`

说明：`easeljs` 在本地缺失时已补装并复制到 `examples/vendor`。

### 6.4 已改动文件范围（便于审计）

- `roslibjs/packages/roslib/importmap.js`
- `roslibjs/packages/roslib/vendor/**`
- `ros3djs/examples/**/*.html`
- `ros3djs/examples/vendor/**`
- `ros2djs/examples/**/*.html`
- `ros2djs/examples/vendor/**`

### 6.5 外链校验结果

在 `robotwebtools` 下（排除 `README.md` 文档文本），已清理运行相关外链：

- `https://static.robotwebtools.org`（示例运行路径中已移除）
- `https://unpkg.com`（import map 运行路径中已移除）
- `http://resources.robotwebtools.org`（URDF 示例运行路径中已移除）

> 注：项目 README 说明文档中可能仍保留外链文本，这不影响离线运行。

### 6.6 本地化验证命令（可直接复核）

```bash
# 1) 校验运行路径是否还含外链（忽略文档）
rg "https://static\\.robotwebtools\\.org|https://unpkg\\.com|http://resources\\.robotwebtools\\.org" \
  /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools \
  --glob "!*.md"

# 2) 校验 roslib importmap 是否为本地映射
rg "\"imports\"|\"https://unpkg\\.com\"" \
  /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/roslibjs/packages/roslib/importmap.js

# 3) 抽样检查示例引用是否指向 ./vendor
rg "vendor/(three|eventemitter2|roslib|easeljs|ColladaLoader|STLLoader)\\.js" \
  /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/ros3djs/examples \
  /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/ros2djs/examples
```

### 6.7 本地化时 JS 库如何下载/获取

本项目推荐两种方式：**在线一次性下载后本地固化**，或**纯离线从已有 node_modules 拷贝**。

#### A. 在线一次性下载（推荐初始化时使用）

说明：仅用于首次准备依赖；完成后把 `vendor` 和 `node_modules` 固化在本地即可离线运行。

```bash
# 1) 进入各目录安装依赖（首次）
cd /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/roslibjs && npm install
cd /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/ros3djs  && npm install
cd /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/ros2djs  && npm install

# 2) ros2d 示例需要 easeljs（若未随原依赖带入）
npm install easeljs --prefix /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/ros2djs
```

#### B. 纯离线获取（已有 node_modules 时）

说明：不联网，直接从本地 `node_modules` 复制到运行目录。

- `ros3djs/examples/vendor/` 所需文件来源：
  - `ros3djs/node_modules/three/build/three.js`
  - `ros3djs/node_modules/eventemitter2/lib/eventemitter2.js`
  - `ros3djs/node_modules/roslib/build/roslib.js`
  - `ros3djs/node_modules/three/examples/js/loaders/ColladaLoader.js`
  - `ros3djs/node_modules/three/examples/js/loaders/STLLoader.js`
- `ros2djs/examples/vendor/` 所需文件来源：
  - `ros2djs/node_modules/easeljs/lib/easeljs.js`
  - `ros2djs/node_modules/eventemitter2/lib/eventemitter2.js`
  - `ros3djs/node_modules/roslib/build/roslib.js`
- `roslibjs/packages/roslib/vendor/` 所需文件来源：
  - `roslibjs/node_modules/{bson,cbor2,eventemitter3,fast-png,uuid,ws,@cto.af/wtf8,fflate,iobuffer}`

可用下面命令快速检查“下载/获取是否齐全”：

```bash
ls /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/ros3djs/examples/vendor
ls /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/ros2djs/examples/vendor
ls /home/mu/IVG2.0/aubo_ros2_ws/src/robotwebtools/roslibjs/packages/roslib/vendor
```

#### C. 固化建议（避免后续误联网）

- 固定使用本地路径（`./vendor/...`），不要在 HTML 里写 CDN
- 保留 `vendor` 目录与已验证可用的 `importmap.js`
- 离线构建统一使用：`build_robotwebtools.sh`

---

## 7. 其他功能包如何调用（API 角度）

### 7.1 script 标签方式（传统页面）

调用顺序建议：

1. 先加载依赖（本地文件）
2. 再加载 `ros3d.js` / `ros2d.js`
3. 使用 `ROSLIB` + `ROS3D` / `ROS2D` API

例如（ros3d）：

```html
<script src="/vendor/three.js"></script>
<script src="/vendor/eventemitter2.js"></script>
<script src="/vendor/roslib.js"></script>
<script src="/vendor/ros3d.js"></script>
<script>
  const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
  const viewer = new ROS3D.Viewer({ divID: 'viewer', width: 800, height: 600 });
</script>
```

### 7.2 ESM 方式（推荐）

- 在现代前端工程中使用 `ros3d.esm.js` + `roslib` 进行模块化导入
- 优点：依赖关系清晰，易与现有构建链集成

---

## 8. 常见问题

### Q1：有 `roslibjs/node_modules` 是否一定不联网？

不一定。

- 构建阶段：通常可离线（依赖已在本地）
- 浏览器运行阶段：若 import map 指向外网，仍会联网

本项目已将 import map 改为本地映射，避免该问题。

### Q2：构建失败提示缺少目录

请确认以下目录存在：

- `roslibjs`
- `ros3djs`
- `ros2djs`

---

## 9. 当前状态总结

- 三个官方库已拉取到稳定版本
- 已有统一离线构建脚本，默认全量构建
- API 调用方式已梳理（全局模式 + ESM 模式）
- 示例与 roslib import map 已完成本地化改造
- 运行路径外链已清理（文档说明外链除外）


完整加载链路（源码级追踪）

  UrdfClient 构造函数                                              [UrdfClient.js:32-64]
    │
    ├─ new ROSLIB.Param({ros, name})                              ← 准备参数查询
    │   └─ getParam.get(callback)                                 ← WebSocket 异步获取 URDF XML
    │       │
    │       └─ 回调: XML 到达                                     ← ── 异步分界线 ──
    │           │
    │           ├─ new ROSLIB.UrdfModel({string})                  ← 同步解析 XML
    │           │
    │           ├─ new Urdf({urdfModel, path, tfClient, ...})     [Urdf.js:28-103]
    │           │   │                                              ← 同步遍历所有 link
    │           │   ├─ for each link.visuals:
    │           │   │   ├─ new MeshResource({path, resource, ...}) [MeshResource.js:24-49]
    │           │   │   │   └─ STLLoader.load(uri, callback)      [MeshLoader.js:114-133]
    │           │   │   │       └─ 回调: meshRes.add(mesh)        ← ── 异步: 每个 mesh 到达 ──
    │           │   │   │
    │           │   │   ├─ new SceneNode({frameID, pose, tfClient, object: meshRes})
    │           │   │   │   └─ tfClient.subscribe(frameID, tfUpdate) ← 订阅 TF
    │           │   │   │
    │           │   │   └─ this.add(sceneNode)                    ← 同步加入 Urdf 树
    │           │   │
    │           │   └─ Urdf 构造完成 (所有 SceneNode 就位, mesh 还在 HTTP 下载中)
    │           │
    │           └─ that.rootObject.add(that.urdf)                  ← rootObject 只有 1 个 child!

  场景图结构：
  rootObject (newRoot)              ← children.length === 1
    └── Urdf (THREE.Object3D)       ← 唯一子节点
        ├── SceneNode (base_link)   ← visible=false, 等 TF 更新
        │   └── MeshResource
        │       └── THREE.Mesh      ← STL HTTP 异步加载完成后才加入
        ├── SceneNode (shoulder_link)
        │   └── ...
        └── ...
