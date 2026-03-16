# IDE 导入解析修复说明

当 Cursor/VSCode 中 Pyright/basedpyright 报「无法解析导入」时，按以下方法配置。

## 1. 无法解析 `models.graspnet`、`utils.xxx`、`graspnetAPI`

**原因**：这些模块位于 `graspnet-baseline` 子目录，运行时通过 `sys.path.insert()` 注入，但静态分析器不会执行该逻辑。

**修复**：将 `graspnet-baseline` 加入分析路径。

### 方法 A：pyrightconfig.json

在项目根或 `aubo_ros2_ws/` 下创建/编辑 `pyrightconfig.json`：

```json
{
  "extraPaths": [
    "aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/foxy/lib/python3.8/site-packages"
  ]
}
```

若 pyrightconfig 在 `aubo_ros2_ws/` 下，则写：

```json
{
  "extraPaths": [
    "src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/foxy/lib/python3.8/site-packages"
  ]
}
```

### 方法 B：.vscode/settings.json

```json
{
  "python.analysis.extraPaths": [
    "/home/mu/IVG/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/foxy/lib/python3.8/site-packages"
  ],
  "python.autoComplete.extraPaths": [
    "/home/mu/IVG/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/foxy/lib/python3.8/site-packages"
  ]
}
```

路径请按实际工作目录替换。

---

## 2. 无法解析 `rclpy`、`visualization_msgs`、`geometry_msgs` 等 ROS 包

**原因**：ROS 2 包安装在 `/opt/ros/<distro>/lib/pythonX.X/site-packages`，不在默认 Python 路径中。

**修复**：将 ROS 的 site-packages 加入分析路径。

### 配置示例

在 `pyrightconfig.json` 的 `extraPaths` 中添加：

```
/opt/ros/foxy/lib/python3.8/site-packages
```

若使用 ROS 2 Humble/其他版本，请替换为对应 distro 和 Python 版本路径。

---

## 3. 完整配置参考

### IVG 项目根 pyrightconfig.json

```json
{
  "extraPaths": [
    "aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/foxy/lib/python3.8/site-packages"
  ]
}
```

### .vscode/settings.json（根或 aubo_ros2_ws）

```json
{
  "python.analysis.extraPaths": [
    "/opt/ros/foxy/lib/python3.8/site-packages",
    "/home/mu/IVG/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline"
  ],
  "python.autoComplete.extraPaths": [
    "/opt/ros/foxy/lib/python3.8/site-packages",
    "/home/mu/IVG/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline"
  ]
}
```

---

## 4. 应用配置后仍报错时

1. **重载窗口**：`Ctrl+Shift+P` → 「Developer: Reload Window」
2. **检查工作区根目录**：确保 pyrightconfig.json 与打开的文件在同一工作区内
3. **确认路径正确**：`/opt/ros/foxy/...` 需对应本机 ROS 版本与 Python 版本
