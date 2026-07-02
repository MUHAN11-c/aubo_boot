# IVG 2.0 代码库版本文档

> 生成日期: 2026-05-15 | 验证状态: ✅ 全量编译通过 (16/16 包)

## 系统环境

| 组件 | 版本 |
|------|------|
| OS | Ubuntu 22.04.5 LTS |
| Kernel | 6.8.0-111-generic |
| GCC | 11.4.0 |
| Node.js | 20.20.2 |
| npm | 10.8.2 |
| Python | 3.10.12 |

---

## ROS 2 Humble (deb 包)

| 包 | 版本 |
|---|------|
| `ros-humble-ros-core` | Humble (2024 LTS) |
| `ros-humble-moveit` | 2.5.9-1jammy.20260505 |
| `ros-humble-moveit-common` | 2.5.9-1jammy.20260304 |
| `ros-humble-rosbridge-suite` | 2.0.6-1jammy.20260425 |
| `ros-humble-rosbridge-server` | 2.0.6-1jammy.20260425 |
| `ros-humble-rosbridge-library` | 2.0.6-1jammy.20260421 |
| `ros-humble-rosbridge-msgs` | 2.0.6-1jammy.20260414 |

### Boost

| 包 | 版本 |
|---|------|
| libboost-all-dev | 1.74.0.3ubuntu7 |

---

## 前端 (Vue 3 + Vite)

### 运行时依赖 (dependencies)

| npm 包 | 声明版本 | 实际安装 |
|--------|---------|----------|
| vue | ^3.5.13 | 3.5.34 |
| vue-router | ^4.5.0 | 4.6.4 |
| pinia | ^2.3.0 | 2.3.1 |
| element-plus | ^2.9.8 | 2.14.0 |
| @element-plus/icons-vue | ^2.3.1 | 2.3.2 |
| @vueuse/core | ^12.8.0 | 12.8.2 |
| roslib | ^2.1.0 | 2.1.0 |
| three | ^0.184.0 | 0.184.0 |
| @types/three | ^0.184.1 | 0.184.1 |

### 构建工具 (devDependencies)

| npm 包 | 声明版本 | 实际安装 |
|--------|---------|----------|
| vite | ^6.3.0 | 6.4.2 |
| @vitejs/plugin-vue | ^5.2.3 | 5.2.3 |
| typescript | ~5.7.3 | 5.7.3 |
| vue-tsc | ^2.2.0 | 2.2.12 |
| tailwindcss | ^4.1.13 | 4.3.0 |
| @tailwindcss/vite | ^4.1.13 | 4.1.13 |
| unplugin-auto-import | ^19.1.0 | 19.3.0 |
| unplugin-vue-components | ^28.5.0 | 28.8.0 |
| @types/node | ^25.8.0 | 25.8.0 |

### 关键约束
- `roslib` npm 包的 `Ros` 类 `isConnected` 是 getter 读 JS 私有字段 `#e` — **不能放入 Vue 响应式系统**（`shallowRef`/`reactive` 都不行）。连接状态通过独立 `ref(false)` 暴露。
- Vite 增量构建可能缓存旧模块 — 修改不生效时 `rm -rf dist/ .vite/` 强制重建。

---

## Python (pip3 --user)

| 包 | 版本 | 备注 |
|---|------|------|
| numpy | **1.23.5** | ⚠️ 必须 1.x。2.x 与 cv2/cv_bridge/transforms3d 不兼容 |
| opencv-python | 4.9.0.80 | 与 numpy 1.23.5 兼容 |
| torch | 2.12.0+cu130 | |
| torchvision | 0.27.0+cu130 | |
| torchaudio | 2.11.0+cu130 | |
| scipy | 1.13.1 | |
| pillow | 12.2.0 | |
| transforms3d | 0.3.1 | 依赖 `np.float`（仅 numpy 1.x 有此别名） |
| fastapi | 0.136.1 | Web Dashboard 网关 |
| uvicorn | 0.46.0 | ASGI 服务器 |
| httpx | 0.28.1 | 视频代理 HTTP 客户端 |
| websockets | 16.0 | WebSocket 代理 |
| graspnetAPI | 1.2.11 | 本地源: `src/graspnet_ros2/graspnet-baseline/graspnetAPI` |
| pointnet2 | 0.0.0 | 本地源: `src/graspnet_ros2/graspnet-baseline/pointnet2` |
| knn-pytorch | 0.1 | 本地源: `src/graspnet_ros2/graspnet-baseline/knn` |
| matplotlib | **系统版** | ⚠️ pip 版 (3.10.9) 与系统 `mpl_toolkits` 冲突，须卸载 pip 版 |

### Python 依赖兼容性矩阵

| 组合 | 状态 |
|------|------|
| numpy 2.2.6 + opencv 4.9.0 | ❌ `AttributeError: _ARRAY_API not found` |
| numpy 2.2.6 + transforms3d 0.3.1 | ❌ `AttributeError: np.float not found` |
| numpy 2.2.6 + cv_bridge (ROS Humble) | ❌ `AttributeError: _ARRAY_API not found` |
| numpy 1.23.5 + opencv 4.9.0 | ✅ |
| numpy 1.23.5 + transforms3d 0.3.1 | ✅ (有 DeprecationWarning，但可用) |
| numpy 1.23.5 + cv_bridge (ROS Humble) | ✅ |
| matplotlib 3.10.9 (pip) + mpl_toolkits (system) | ❌ `ImportError: cannot import name 'docstring'` |
| matplotlib (system) + mpl_toolkits (system) | ✅ |

---

## 工作空间包 (ROS 2 colcon)

| 路径 | 包名 | 类型 |
|------|------|------|
| `src/ivg_interfaces` | ivg_interfaces | ament_cmake (52 接口: 17 msg + 35 srv) |
| `src/ivg_utils` | ivg_utils | ament_python |
| `src/aubo_ros2_driver` | aubo_driver_ros2 + demo_driver | ament_cmake |
| `src/aubo_ros2_web_dashboard` | aubo_ros2_web_dashboard | ament_python + Vue 3 |
| `src/tool_changer` | tool_changer | ament_cmake |
| `src/coffee_latte_demo` | latte_imitation (已合并) | ament_python (废弃) |
| `src/graspnet_ros2` | graspnet_ros2 | ament_python |
| `src/hand_eye_calibration` | hand_eye_calibration | ament_python |
| `src/latte_imitation` | latte_imitation | ament_python |
| `src/camport_ros2` | — | COLCON_IGNORE (已废弃) |
| `src/moveit_ros_planning` | — | COLCON_IGNORE |
| `src/moveit_ros_planning_interface` | — | COLCON_IGNORE |
| `src/moveit_ros_visualization` | moveit_ros_visualization | 本地复刻（含 URDF 切换补丁） |
| `src/percipio_camera` | percipio_camera | ament_python |
| `src/vision_perception` | vision_perception | ament_python |
| `src/visual_pose_estimation_python` | visual_pose_estimation_python | ament_python |

---

## AUBO SDK

### 当前版本 (lib64 — 启动脚本使用)

| 文件 | 版本 |
|------|------|
| `libauborobotcontroller.so.1.3.1` | `src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/aubocontroller/` |
| `libauborobotcontroller.so.1.3` | → 软链接 |
| `libauborobotcontroller.so.1` | → 软链接 |
| `libauborobotcontroller.so` | → 软链接 |
| `liblog4cplus.so` | `src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/log4cplus/` |
| `libprotobuf.so` | `src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/protobuf/` |
| `libconfig++.so` | `src/aubo_ros2_driver/aubo_driver_ros2/lib/lib64/config/` |

### 旧版本 (lib32 — 保留但未使用)

| 文件 | 版本 |
|------|------|
| `libauborobotcontroller.so.1.0.6` | `src/aubo_ros2_driver/aubo_driver_ros2/lib/lib32/` |

### LD_LIBRARY_PATH (来自 start_aubo_new_driver.sh)

```
.../lib64/aubocontroller:.../lib64/log4cplus:.../lib64/config:.../lib64/protobuf
```

---

## 已删除

| 目录/包 | 原因 | 替代 |
|---------|------|------|
| `src/robotwebtools/` | ros3djs/ros2djs 过时，roslibjs 改用 npm | npm `roslib@2.1.0`，npm `three@0.184.0` |
| `src/robotwebtools/ros3djs/` | 陈旧，Three.js r89 | npm `three@0.184.0` + 自定义 `lib/three_urdf/` |
| `src/robotwebtools/ros2djs/` | 未使用 | — |
| `web/src/lib/ros3d_patches.ts` | ros3djs 已删除 | — |
| pip `matplotlib` 3.10.9 | 与系统 mpl_toolkits 冲突 | 系统 matplotlib |
| pip `numpy` 2.2.6 | 与 cv2/transforms3d 不兼容 | numpy 1.23.5 |
