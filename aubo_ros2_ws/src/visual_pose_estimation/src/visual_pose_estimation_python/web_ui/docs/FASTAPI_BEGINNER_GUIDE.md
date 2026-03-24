# 零基础学习 FastAPI

这份文档面向第一次接触 FastAPI、同时又要看懂本项目 Web 后端的读者。目标不是只讲概念，而是帮助你把概念和当前工程代码一一对应起来。

---

## 1. 先理解：FastAPI 是什么

FastAPI 是一个 Python Web 框架。你可以把它理解成：

- 它负责接收浏览器或前端发来的 HTTP 请求
- 它负责把请求分发到对应的 Python 函数
- 它负责把 Python 函数的返回值变成 JSON 响应
- 它还可以提供 WebSocket、静态文件、接口参数校验、依赖注入等能力

如果你以前接触过：

- `Flask`：FastAPI 和 Flask 一样能写接口，但参数声明更清晰，类型提示更友好
- `Django`：FastAPI 更轻，适合做 API 服务
- `http.server`：FastAPI 比原生 HTTP 方式更现代，结构化更强，可维护性更高

---

## 2. 为什么这个项目要用 FastAPI

这个项目不是普通网页项目，而是一个要长期承载以下能力的系统：

- ROS2 通信
- 相机采图
- 视觉姿态估计
- 模板管理
- 机器人控制
- 抓取流程控制
- Debug 图像预览
- 后续可能继续接入深度学习、强化学习、YOLO、大模型、流式任务输出

这类系统对 Web 后端的要求不是“能跑就行”，而是：

- 能分层
- 能扩展
- 能持续加接口
- 能支持长任务
- 能支持 WebSocket
- 能和 ROS2 的生命周期配合

FastAPI 正适合承担这样的角色。

---

## 3. FastAPI 的最小工作方式

一个最简单的 FastAPI 服务，本质就是：

```python
from fastapi import FastAPI

app = FastAPI()

@app.get("/hello")
def hello():
    return {"message": "hello"}
```

含义是：

- `app = FastAPI()`：创建一个 Web 应用
- `@app.get("/hello")`：声明一个 GET 路由
- 访问 `/hello` 时，执行 `hello()`
- 返回的字典会自动变成 JSON

在本项目里，这个思路没有变，只是把它工程化、模块化了。

---

## 4. 本项目里 FastAPI 从哪里启动

当前入口在：

- `visual_pose_estimation_python/web/server.py`
- `visual_pose_estimation_python/web/app.py`

启动顺序可以这样理解：

1. `server.py`
   - 解析命令行参数：`--host`、`--port`、`--reload`
   - 调用 `uvicorn.run(create_app(), ...)`

2. `app.py`
   - 创建 FastAPI 实例
   - 注册 CORS
   - 挂载静态资源
   - 注册所有路由
   - 在应用生命周期中启动/停止 ROS2 bridge

3. `uvicorn`
   - 负责真正把这个 Python Web 服务跑起来

简单说：

- `server.py` 是启动器
- `app.py` 是应用装配器
- `uvicorn` 是运行容器

---

## 5. 本项目里的 FastAPI 目录怎么理解

当前 Web 后端主目录：

- `visual_pose_estimation_python/web/`

你可以把它理解成下面几层：

### 5.1 入口层

- `app.py`
  - 创建 FastAPI 应用
- `server.py`
  - 启动 uvicorn

### 5.2 路由层

- `routers/system.py`
- `routers/camera.py`
- `routers/pose.py`
- `routers/templates.py`
- `routers/robot.py`
- `routers/grasp.py`
- `routers/debug.py`

路由层只做一件事：

- 把 URL 映射到某个服务函数

例如：

- `/api/capture_image` -> `camera.py`
- `/api/list_templates` -> `templates.py`
- `/api/get_robot_status` -> `robot.py`

路由层尽量不写复杂业务逻辑。

### 5.3 服务层

- `services/native_api.py`

这是 Web 业务逻辑层，负责：

- 检查参数
- 调用 ROS2 bridge
- 处理模板文件
- 处理图像编码/解码
- 统一把错误转换为 HTTP 错误

简单说：

- 路由层负责“接”
- 服务层负责“做”

### 5.4 依赖注入层

- `dependencies.py`

作用是给路由层提供“共享对象”，比如：

- `paths`
- `ros_bridge`
- `native_service`
- `ws_manager`

FastAPI 的 `Depends(...)` 机制可以理解为：

- 不用每个路由自己 new 一遍对象
- 统一从应用状态里取共享服务

### 5.5 ROS2 bridge 层

- `ros_bridge/manager.py`
- `ros_bridge/node_runtime.py`

这是本项目最关键的一层。

为什么需要它？

因为 FastAPI 是 Web 框架，ROS2 是机器人通信框架，它们不是天然同一种运行模式。

所以这里做了一个“桥”：

- `RosBridgeManager`
  - 管理 ROS2 bridge 的启动、停止、状态
  - 在后台线程里持续 `spin`

- `ROS2Node`
  - 真正封装相机触发、姿态估计、模板服务、机器人控制、夹爪切换等 ROS2 调用

换句话说：

- FastAPI 不直接到处写 `rclpy`
- 所有 ROS2 交互尽量收口到 bridge 层

### 5.6 WebSocket 层

- `ws/manager.py`

作用是：

- 管理多个 WebSocket 连接
- 支持状态广播
- 支持 `ping/pong`

这为后面做长任务、流式日志、实时状态更新打下基础。

### 5.7 运行时支持层

- `runtime_support.py`
- `params_manager.py`
- `resources.py`

这些文件负责：

- 找配置路径
- 找模板目录
- 管理 debug 参数
- 统一处理姿态归一化
- rembg 运行时支持

你可以把它们看成“Web 公共工具层”。

---

## 6. 路由、服务、ROS2 bridge 三层是怎么串起来的

以“采图”为例：

1. 浏览器请求：
   - `POST /api/capture_image`

2. 路由层：
   - `routers/camera.py` 接到请求
   - 调用 `NativeWebService.capture_image(...)`

3. 服务层：
   - `services/native_api.py`
   - 调用 `ros_bridge.node.capture_image(...)`
   - 把图像编码成 base64
   - 返回 JSON

4. ROS2 bridge 层：
   - `ros_bridge/node_runtime.py`
   - 调用 ROS2 相机服务
   - 临时订阅图像话题
   - 等待深度图和彩色图返回

这条链就是：

前端 -> 路由 -> 服务 -> ROS2 bridge -> ROS2 服务/话题

---

## 7. 什么是生命周期 lifespan

在 `app.py` 里有一个很重要的概念：

- `lifespan`

它的作用是：

- Web 服务启动时做什么
- Web 服务关闭时做什么

本项目里它负责：

- 启动 `ros_bridge`
- 广播 WebSocket 启动消息
- 关闭时停止 `ros_bridge`
- 广播关闭消息

为什么这很重要？

因为 ROS2 节点不是普通变量，不能随便扔在那里不管。

如果没有生命周期管理，常见问题会是：

- 重复初始化 `rclpy`
- 服务退出时节点没关干净
- 后台线程残留
- 启动异常后状态不明确

---

## 8. 什么是依赖注入 Depends

例子：

```python
@router.post("/capture_image")
def capture_image(
    payload: dict | None = Body(default=None),
    service: NativeWebService = Depends(get_native_service),
):
    return service.capture_image(payload)
```

这里的含义是：

- 这个路由需要一个 `service`
- `service` 不在函数里手动创建
- FastAPI 会通过 `get_native_service()` 自动提供

这样做的好处：

- 少写重复代码
- 更容易统一管理共享对象
- 测试时更容易替换假对象

---

## 9. WebSocket 在这里为什么重要

很多初学者会问：

- 现在只是普通接口，为什么一开始就加 WebSocket？

原因是这个项目后续很可能出现这些场景：

- 长时间视觉任务
- 抓取流程实时状态推送
- 后台算法执行进度条
- 日志流输出
- 模型推理中间状态

如果只靠普通 HTTP：

- 前端要不停轮询
- 延迟高
- 结构不优雅

WebSocket 的好处是：

- 一次连接
- 后端可主动推送
- 适合实时状态同步

所以这次迁移不是“等以后再说”，而是先把能力骨架搭好。

---

## 10. 本项目里 FastAPI 相比旧方式的直接收益

### 10.1 代码更容易维护

旧方式常见问题：

- 一个大文件里混了 HTTP、业务、ROS2、文件读写、Debug 逻辑

现在拆成：

- 路由
- 服务
- ROS2 bridge
- 工具层

修改时更容易定位。

### 10.2 更适合持续扩展

以后新增：

- YOLO 接口
- 大模型接口
- 强化学习训练状态接口
- 批处理任务接口

都可以按模块继续加，不需要往一个超级大文件里硬塞。

### 10.3 测试更容易写

现在已经有：

- `test/test_web_app.py`

说明这套结构已经适合：

- 单独测健康检查
- 单独测路由
- 单独用假 bridge 做隔离测试

### 10.4 更适合 ROS2 与 Web 共存

FastAPI 负责 Web 入口，ROS2 bridge 负责 ROS2 运行时，两边职责更清楚。

---

## 11. 初学者读代码的推荐顺序

如果你想真正看懂这个项目，建议按这个顺序读：

1. `visual_pose_estimation_python/web/server.py`
   - 明白服务怎么启动

2. `visual_pose_estimation_python/web/app.py`
   - 明白应用怎么组装

3. `visual_pose_estimation_python/web/routers/system.py`
   - 明白最基础的路由、状态、WebSocket

4. `visual_pose_estimation_python/web/routers/camera.py`
   - 看一个简单业务接口

5. `visual_pose_estimation_python/web/services/native_api.py`
   - 明白业务逻辑放在哪里

6. `visual_pose_estimation_python/web/ros_bridge/manager.py`
   - 明白 Web 如何管理 ROS2 生命周期

7. `visual_pose_estimation_python/web/ros_bridge/node_runtime.py`
   - 明白真正的 ROS2 调用怎么做

8. `test/test_web_app.py`
   - 明白怎么验证这些接口

---

## 12. 初学者最容易混淆的几个点

### 12.1 `app.py` 和 `server.py` 有什么区别

- `app.py`：定义应用
- `server.py`：启动应用

### 12.2 路由层和服务层有什么区别

- 路由层：URL 映射
- 服务层：业务逻辑

### 12.3 `RosBridgeManager` 和 `ROS2Node` 有什么区别

- `RosBridgeManager`：管理 ROS2 bridge 的生命周期
- `ROS2Node`：实际执行 ROS2 通信

### 12.4 为什么不在每个路由里直接写 ROS2 调用

因为那样会让：

- 路由越来越重
- 测试越来越难
- 生命周期越来越难管理

---

## 13. 这套结构未来还能怎么扩展

后续如果加新能力，推荐按下面思路扩展：

### 13.1 新增 AI 推理接口

- 新增路由文件，例如 `routers/model.py`
- 在 `services/` 增加对应 service 方法
- 如需调用 ROS2 或推理节点，再扩 bridge 或单独建 service

### 13.2 新增长任务接口

- HTTP 发起任务
- WebSocket 回传进度
- 后台线程或任务队列执行

### 13.3 新增管理后台

- 保持现在静态前端方式继续迭代
- 以后需要时再引入 Vue/React

当前先不引入前端框架，是为了先把后端骨架和接口边界稳定下来。

---

## 14. 一句话总结

FastAPI 在这个项目里，不只是“换一个 Web 框架”，而是把原本耦合在一起的：

- HTTP 请求处理
- 业务逻辑
- ROS2 通信
- 静态资源服务
- WebSocket 实时能力

拆成了清晰、可维护、可扩展的工程结构。

如果你把这一点看懂，就已经不只是“学会 FastAPI”，而是开始理解“如何用 FastAPI 搭一个机器人项目的 Web 后端”。

## 延伸阅读

- `FASTAPI_MIGRATION_GUIDE.md`
  - 看完整迁移替换过程
- `FASTAPI_TESTING_GUIDE.md`
  - 看如何编写测试验证代码
