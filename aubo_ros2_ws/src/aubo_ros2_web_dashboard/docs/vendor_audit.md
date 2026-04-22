# Vendor 审计与补丁说明

本文记录 `web/public/js/vendor/` 下第三方文件的来源、当前仓库是否改动、为什么要改，以及后续升级时应如何复现。

## 目标

- 明确哪些文件是官方原始文件。
- 明确哪些文件只是下载/打包后的直接拷贝。
- 明确哪些文件做过本地补丁，以及补丁的必要性。
- 避免下次升级 `roslib/ros2d/ros3d/three` 时丢失 ROS 2 / Humble 兼容修复。

## 来源总览

| 当前文件 | 上游来源 | 当前生成方式 | 是否有本地修改 |
|---|---|---|---|
| `web/public/js/vendor/eventemitter2.js` | `eventemitter2@6.4.9`（上游源码等价于 [EventEmitter2 v6.4.9](https://raw.githubusercontent.com/EventEmitter2/EventEmitter2/v6.4.9/lib/eventemitter2.js)） | `scripts/bundle_roslib2_browser.sh` 直接复制 | 否 |
| `web/public/js/vendor/roslib-2.iife.js` | npm `roslib@2.x` | `scripts/bundle_roslib2_browser.sh` 用 esbuild 打 IIFE | 否（但由本仓库构建，不是上游直发单文件） |
| `web/public/js/vendor/easeljs.min.js` | `easeljs@1.0.2`（等价于官方 build） | `scripts/bundle_roslib2_browser.sh` 直接复制 | 否 |
| `web/public/js/vendor/three.min.js` | `three@0.89.0`（three.js r89） | `scripts/bundle_roslib2_browser.sh` 直接复制 | 否 |
| `web/public/js/vendor/ros2d.min.js` | `ros2d@0.10.0` 官方 build | `scripts/bundle_roslib2_browser.sh` 复制后补丁 | 是 |
| `web/public/js/vendor/ros3d.min.js` | `ros3d@1.1.0` 官方 build | `scripts/bundle_roslib2_browser.sh` 复制后补丁 | 是 |

## 单一事实来源

现在仓库只保留一条 vendor 生成链：

1. `scripts/bundle_roslib2_browser.sh`
   - 从 npm 安装 `eventemitter2 / roslib / ros2d / ros3d / three / easeljs`。
   - 生成 `roslib-2.iife.js`，并同步其余浏览器产物到 `web/public/js/vendor/`。
   - 对 `ros2d.min.js`、`ros3d.min.js` 执行 ROS 2 / Humble 必需补丁。

官方原始文件 URL 仍保留在本文和 `web/public/js/vendor/README.txt`，仅作为审计参考，不再作为仓库内的第二条生成链。

## 当前本地补丁

### 1. `ros2d.min.js`

补丁位置：`scripts/bundle_roslib2_browser.sh`

```bash
perl -i -pe 's#nav_msgs/OccupancyGrid#nav_msgs/msg/OccupancyGrid#g' "$VENDOR/ros2d.min.js"
```

修改前：

- `nav_msgs/OccupancyGrid`

修改后：

- `nav_msgs/msg/OccupancyGrid`

原因：

- ROS 2 下经 rosbridge 暴露的话题类型通常是 `package/msg/Type`。
- 上游 `ros2djs` 仍保留 ROS 1 风格短类型名，浏览器直接订阅会和 Humble 现场的类型串不一致。

不改会怎样：

- `topics_lab` 里的 2D 地图无法按预期订阅 `OccupancyGrid`。

### 2. `ros3d.min.js`

补丁位置：`scripts/bundle_roslib2_browser.sh`

#### 2.1 `throttle_rate` 空值逻辑

```bash
perl -i -pe 's/this\.throttle_rate=e\.throttle_rate\|\|null/this.throttle_rate=null==e.throttle_rate?null:e.throttle_rate/g' "$VENDOR/ros3d.min.js"
```

修改前：

- `this.throttle_rate = e.throttle_rate || null`

修改后：

- `this.throttle_rate = null == e.throttle_rate ? null : e.throttle_rate`

原因：

- 上游把 `0` 当成 falsy，误改成 `null`。
- Humble + rosbridge 对 `subscribe` 参数校验更严格，`null` 最终会触发 `Invalid value: None`。

不改会怎样：

- 点云等 3D 订阅在浏览器侧可能直接失败，或者出现难以定位的桥侧报错。

#### 2.2 ROS 1 短类型名替换为 ROS 2 `package/msg/Type`

脚本会把如下类型批量替换：

- `visualization_msgs/MarkerArray` -> `visualization_msgs/msg/MarkerArray`
- `visualization_msgs/Marker` -> `visualization_msgs/msg/Marker`
- `sensor_msgs/PointCloud2` -> `sensor_msgs/msg/PointCloud2`
- `sensor_msgs/LaserScan` -> `sensor_msgs/msg/LaserScan`
- `nav_msgs/OccupancyGrid` -> `nav_msgs/msg/OccupancyGrid`
- 以及其它 `geometry_msgs` / `nav_msgs` / `visualization_msgs` 相关类型

原因：

- `ros3djs` 仍有 ROS 1 风格消息类型字串。
- 本项目运行在 ROS 2 + Humble 上，桥接时需要显式 `package/msg/Type`。

不改会怎样：

- `Marker`、`LaserScan`、`PointCloud2`、路径/姿态等 3D 场景对象会出现订阅失败或解析失败。

## 不直接修改 vendor，但对其行为做兼容的项目层补丁

### `web/public/js/ivg_roslib_ros2_humble_compat.js`

这不是 vendor 文件本体，但必须和 vendor 一起理解：

1. **Topic 参数归一化**
   - 把 `throttle_rate`、`queue_length`、`queue_size` 的 `null/''` 归一成整数。
   - 目的同样是适配 Humble rosbridge 更严格的参数校验。

2. **URDF 枚举预注入**
   - 在 `ros3d.min.js` 加载前补齐 `ROSLIB.URDF_MESH` 等枚举。
   - 因为 `ros3d.min.js` 打包时会对 `ROSLIB` 做一次快照，如果当时枚举不存在，后续机械臂 mesh 分支不会执行。

## 项目层对官方行为的额外修正

除了 vendor 产物本身，项目还做了两类“官方库上层兼容”：

1. `web/public/js/view3d/*.js`
   - 处理 ros3d 内嵌 `THREE` 与页面 `THREE` 不是同一实例的问题。
   - 处理大写 `.DAE/.STL` 网格扩展名与透明材质异常。
   - 处理点云抽样、浏览器 `/tf` 回退模式、URDF 参数读取等现场逻辑。

2. `aubo_ros2_web_dashboard/gateway/routes/robot_mesh.py`
   - 浏览器常会把 URDF 中的大写网格扩展名规范成小写。
   - 服务端因此增加了“share 目录内不区分大小写解析文件名”的能力。

## 升级标准流程

1. 先确认目标版本组合，优先与当前 `three r89 + ros3d` 兼容。
2. 用 `scripts/bundle_roslib2_browser.sh` 生成新 vendor。
3. 对照本文检查：
   - `ros2d.min.js` 是否仍需要 `package/msg/Type` 替换。
   - `ros3d.min.js` 是否仍存在 `throttle_rate || null` 问题。
   - `ros3d.min.js` 是否仍保留 ROS 1 短类型名。
4. 回归：
   - `topics_lab` 的 2D 地图。
   - `topics_lab` 的 3D 点云 / LaserScan / Marker。
   - `vision_grasp_panel` 左栏 URDF。
5. 如上游已原生修复，再删除对应本地补丁，并更新本文。

## 维护建议

- **不要**手工直接编辑 `vendor/*.min.js` 后不留来源说明。
- 如必须改第三方产物，优先把修改写进 `scripts/bundle_roslib2_browser.sh`。
- 官方原始文件 URL 仅保留作审计参考；仓库内实际生成以 `scripts/bundle_roslib2_browser.sh` 为准。
