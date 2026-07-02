# 项目依赖清单

## 当前前端依赖 (package.json)

| 包名 | 版本 | 用途 |
|------|------|------|
| vue | ^3.3.8 | 前端框架 |
| vue-router | ^4.2.5 | 路由 |
| pinia | ^2.1.7 | 状态管理 |
| three | ^0.158.0 | 3D渲染引擎 |
| axios | ^1.6.2 | HTTP客户端 |
| element-plus | ^2.4.4 | UI组件库 |
| @element-plus/icons-vue | ^2.1.0 | 图标 |
| @types/three | ^0.158.3 | Three.js类型 |
| @vitejs/plugin-vue | ^4.5.0 | Vite插件 |
| vite | ^5.0.0 | 构建工具 |
| eslint | ^8.54.0 | 代码检查 |

## 当前后端依赖 (requirements.txt)

| 包名 | 版本 | 用途 |
|------|------|------|
| fastapi | 0.104.1 | Web框架 |
| uvicorn[standard] | 0.24.0 | ASGI服务器 |
| websockets | 12.0 | WebSocket |
| pydantic-settings | 2.5.0 | 配置管理 |
| numpy | >=1.21.0 | 数值计算 |
| psutil | 5.9.0 | 系统监控 |

## 计划新增前端依赖

| 包名 | 版本 | 用途 |
|------|------|------|
| @foxglove/ws-protocol | latest | Foxglove WS客户端 |
| @foxglove/rosmsg2-serialization | latest | ROS2 CDR解析 |
| @foxglove/cdr | latest | CDR编解码 |
| urdf-loader | latest | URDF模型加载 |

## 计划新增后端依赖

| 包名 | 用途 |
|------|------|
| ros-humble-foxglove-bridge | C++ ROS2 bridge (apt) |

## 参考项目

| 项目 | GitHub | Stars | 用途 |
|------|--------|-------|------|
| foxglove-sdk | foxglove/foxglove-sdk | 240 | Foxglove C++ SDK |
| rosbridge_suite | RobotWebTools/rosbridge_suite | ~1200 | 标准rosbridge |
| roslibjs | RobotWebTools/roslibjs | 818 | ROS JS客户端 |
| robot_viewer | fan-ziqi/robot_viewer | 441 | URDF查看器 |
| urdf-loaders | gkjohnson/urdf-loaders | - | Three.js URDF加载器 |
