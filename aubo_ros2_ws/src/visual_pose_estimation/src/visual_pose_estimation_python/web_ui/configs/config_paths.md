# 配置文件路径说明

本目录 `web_ui/configs` 为**默认配置目录**，所有阈值、参数等初始化文件默认在此读取/写入。路径可在各配置模块代码中配置覆盖。

## 默认路径（web_ui/configs）

| 文件 | 用途 | 读取/写入 | 可配置方式 |
|------|------|-----------|------------|
| **debug_thresholds.json** | Debug 阈值（二值化、连通域等） | 读+写 | `ConfigReader(debug_thresholds_path=...)`、`load_debug_thresholds(debug_thresholds_file=...)`；Web 端 `ParamsManager(config_path=...)` |
| **app_config.json** | 模板根路径、拍照姿态固定旋转 | 只读 | 本文件内 `template_root`、`camera_pose_fixed_orientation` |
| **camera_intrinsics.yaml** | 相机内参（标定未指定时从此目录优先加载） | 只读 | 置于本目录，标准文件名 |
| **hand_eye_calibration.yaml** | 手眼标定结果（未指定 calib_file 时从此目录加载） | 只读 | 置于本目录，标准文件名 |
| **camera_pose.json** | （示例/备份用，非运行时主配置） | - | - |

## 各模块默认路径与可配置

### 1. ConfigReader（visual_pose_estimation_python/config_reader.py）

- **默认路径**：`web_ui/configs/debug_thresholds.json`（包内相对路径 `_PKG_ROOT/web_ui/configs/debug_thresholds.json`）
- **可配置**：`ConfigReader(debug_thresholds_path="...")` 或 `load_debug_thresholds(debug_thresholds_file="...")`
- **备用**：当默认路径不存在时（如从 install 运行）使用 `FALLBACK_DEBUG_THRESHOLDS_PATH`

### 2. ParamsManager（web_ui/scripts/params_manager.py）

- **默认路径**：`ROOT_DIR/configs/debug_thresholds.json`，其中 `ROOT_DIR = web_ui`，即 `web_ui/configs/debug_thresholds.json`
- **可配置**：`ParamsManager(config_path="...")`

### 3. HTTP 桥接（web_ui/scripts/http_bridge_server.py）

- **app_config**：默认 `web_ui/configs/app_config.json`（`Path(__file__).resolve().parent.parent / "configs"`）
- **模板根目录**：优先从 `app_config.json` 的 `template_root` 读取；否则按包结构推导或备用绝对路径

### 4. 模板根目录（template_root）

- **节点/launch**：由 launch 参数 `template_root` 传入（默认可为 `/home/.../visual_pose_estimation/templates`）
- **Web 保存/列表**：由 `get_templates_dir()` 提供，优先 `app_config.json` 的 `template_root`，与节点保持一致时请在该文件中配置相同路径

## 小结

- 阈值、参数等**默认路径**均为 **web_ui/configs**（如 `debug_thresholds.json`、`app_config.json`）。
- 标定标准文件名：**camera_intrinsics.yaml**、**hand_eye_calibration.yaml**。
- 各模块均支持在代码中传入路径覆盖默认值；`app_config.json` 内可配置模板根路径与拍照姿态固定旋转。
