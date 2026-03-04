# IDE 导入解析修复说明

当 Cursor/VSCode 中 Pyright/basedpyright 报「无法解析导入」时，按以下方法配置。

**工作区说明**：若工作区根目录为 `IVG2.0`，则配置放在 `IVG2.0/` 下；若只打开 `aubo_ros2_ws`，则放在 `aubo_ros2_ws/` 下。路径请按实际工作区根目录替换。

---

## 1. 无法解析 `models.graspnet`、`utils.xxx`、`graspnetAPI`

**原因**：这些模块位于 `graspnet-baseline` 子目录，运行时通过 `sys.path.insert()` 注入，静态分析器不会执行该逻辑。

**修复**：将 `graspnet-baseline` 的**绝对路径**或**相对工作区根的路径**加入分析路径。

### 方法 A：pyrightconfig.json

在工作区根目录（如 `IVG2.0/` 或 `aubo_ros2_ws/`）创建/编辑 `pyrightconfig.json`。

若工作区根为 **IVG2.0**（相对路径以 IVG2.0 为基准）：

```json
{
  "extraPaths": [
    "aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ]
}
```

若工作区根为 **aubo_ros2_ws**：

```json
{
  "extraPaths": [
    "src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ]
}
```

### 方法 B：.vscode/settings.json

在工作区根的 `.vscode/settings.json` 中配置（路径必须为**绝对路径**）：

```json
{
  "python.analysis.extraPaths": [
    "/home/mu/IVG2.0/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ],
  "python.autoComplete.extraPaths": [
    "/home/mu/IVG2.0/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ]
}
```

将 `/home/mu/IVG2.0` 替换为你本机的工作区根路径。

---

## 2. 无法解析 `rclpy`、`visualization_msgs`、`geometry_msgs` 等 ROS 包

**原因**：ROS 2 的 Python 包不在默认搜索路径中。Humble 下常见位置为：

- `/opt/ros/humble/lib/python3.10/site-packages`
- **`/opt/ros/humble/local/lib/python3.10/dist-packages`**（`rclpy` 等常在此）

**修复**：将上述两个路径都加入 `extraPaths`（尤其是 **local/.../dist-packages**，否则可能仍无法解析 `rclpy`）。

### 配置示例

在 `pyrightconfig.json` 的 `extraPaths` 中同时包含：

```
/opt/ros/humble/lib/python3.10/site-packages
/opt/ros/humble/local/lib/python3.10/dist-packages
```

若使用其他 ROS 版本（如 Foxy），请替换为对应 distro 和 Python 版本路径（如 `foxy`、`python3.8` 等）。

---

## 3. 完整配置参考（IVG2.0 + ROS 2 Humble）

### 工作区根目录：IVG2.0

**IVG2.0/pyrightconfig.json**

```json
{
  "extraPaths": [
    "aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ]
}
```

**IVG2.0/.vscode/settings.json**

```json
{
  "ROS2.distro": "humble",
  "python.analysis.extraPaths": [
    "/home/mu/IVG2.0/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ],
  "python.autoComplete.extraPaths": [
    "/home/mu/IVG2.0/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline",
    "/opt/ros/humble/lib/python3.10/site-packages",
    "/opt/ros/humble/local/lib/python3.10/dist-packages"
  ]
}
```

---

## 4. 应用配置后仍报错时

1. **重载窗口**：`Ctrl+Shift+P` → 「Developer: Reload Window」
2. **检查工作区根**：确保 `pyrightconfig.json` 与当前打开的文件在同一工作区内（若开的是 IVG2.0，配置应在 IVG2.0 根下）
3. **确认 rclpy 路径**：本机执行  
   `find /opt/ros/humble -name rclpy -type d`  
   若出现在 `local/.../dist-packages` 下，则必须把该路径加入 `extraPaths`
4. **确认 Python 版本**：ROS Humble 使用 Python 3.10，路径中的 `python3.10` 需与系统一致
