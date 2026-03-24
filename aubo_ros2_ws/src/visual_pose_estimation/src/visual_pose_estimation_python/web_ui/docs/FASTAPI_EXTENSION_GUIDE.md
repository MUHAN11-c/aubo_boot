# FastAPI 后续开发实战指南

这份文档不是讲“FastAPI 是什么”，而是讲：

- 后续如果你继续开发这个项目
- 想新增一个 Web 接口
- 想新增一个 ROS2 能力
- 想补一套测试
- 想确认改动不会破坏现有结构

应该怎么做。

它适合已经看过下面几份文档之后继续往下走的人：

- `FASTAPI_WEB.md`
- `FASTAPI_BEGINNER_GUIDE.md`
- `FASTAPI_MIGRATION_GUIDE.md`
- `FASTAPI_TESTING_GUIDE.md`

---

## 1. 先记住一个总原则

后续开发时，最重要的不是“功能能不能先跑起来”，而是：

- 新功能要接到对的层
- 不要把逻辑重新写回“单体大文件模式”

这个项目现在已经分成了：

- 路由层
- 服务层
- ROS2 bridge 层
- 公共支持层
- 测试层

所以新增功能时，优先考虑：

- 这个逻辑应该放在哪一层

而不是：

- 哪个文件最方便随手加进去

---

## 2. 当前架构的开发路线图

你可以把当前开发路径记成下面这条链：

前端请求  
-> 路由层 `routers/*.py`  
-> 服务层 `services/native_api.py`  
-> ROS2 bridge `ros_bridge/node_runtime.py`  
-> ROS2 服务 / 话题 / 节点  
-> 返回结果  
-> 测试验证 `test/test_web_app.py`

这条链就是你以后新增功能时的默认开发路线。

---

## 3. 新增一个普通 Web 接口，应该怎么做

假设你要新增一个接口：

- `POST /api/foo_action`

推荐按下面 6 步做。

### 第 1 步：先判断它属于哪个业务域

先不要急着写代码，先问自己：

- 它是相机相关吗？
- 它是模板相关吗？
- 它是机器人相关吗？
- 它是抓取相关吗？
- 它是 debug 相关吗？
- 还是应该新增一个新模块？

如果它已经明显属于某个已有域，就放进对应路由文件：

- 相机 -> `routers/camera.py`
- 模板 -> `routers/templates.py`
- 机器人 -> `routers/robot.py`
- 抓取 -> `routers/grasp.py`
- debug -> `routers/debug.py`

如果它不属于已有域，再考虑新建：

- `routers/model.py`
- `routers/task.py`
- `routers/yolo.py`

### 第 2 步：在路由层只做“接线”

路由层只负责：

- 定义 URL
- 接收参数
- 调用 service

路由不要写复杂业务。

推荐写法示例：

```python
@router.post("/foo_action")
def foo_action(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.foo_action(payload)
```

这里的关键思想是：

- 路由只是入口
- 具体怎么做，交给 service

### 第 3 步：把业务逻辑放进 `NativeWebService`

例如在：

- `visual_pose_estimation_python/web/services/native_api.py`

增加：

```python
def foo_action(self, payload: dict[str, Any]) -> dict[str, Any]:
    node = self._require_node()
    value = payload.get("value")
    if value is None:
        raise HTTPException(status_code=400, detail="缺少 value")

    result, error = node.foo_action(value)
    if error:
        raise HTTPException(status_code=500, detail=error)
    return result
```

这里 service 层主要负责：

- 参数校验
- 错误转换
- 文件处理
- 数据格式转换
- 调用 bridge

### 第 4 步：如果需要 ROS2，就在 `node_runtime.py` 增加对应能力

如果这个接口最终要调 ROS2 服务，那就在：

- `visual_pose_estimation_python/web/ros_bridge/node_runtime.py`

里增加对应方法。

例如：

```python
def foo_action(self, value, timeout=10.0):
    if not self.foo_client.wait_for_service(timeout_sec=5.0):
        return None, "foo 服务不可用"

    request = FooService.Request()
    request.value = value
    future = self.foo_client.call_async(request)
    self._spin_future(future, timeout_sec=timeout)

    if not future.done():
        return None, "foo 服务调用超时"

    response = future.result()
    if response is None:
        return None, "foo 服务调用失败"

    return {"success": bool(response.success)}, None
```

### 第 5 步：如果有公共逻辑，再考虑抽到支持层

如果你发现新增的逻辑属于这些类型：

- 配置读取
- 路径解析
- 姿态归一化
- 通用参数转换
- 通用文件处理

那不要继续堆在 `native_api.py` 里。

应考虑放到：

- `runtime_support.py`
- `resources.py`
- `params_manager.py`

### 第 6 步：立刻补测试

不要等功能做完很久之后才补测试。

推荐在功能刚写完时同步补：

- 成功路径
- 参数错误路径
- 外部依赖失败路径
- 副作用验证

---

## 4. 新增一个 ROS2 能力，应该怎么做

有些功能不是单纯加个 HTTP 接口，而是底层还要接新的 ROS2 服务、消息或话题。

这时建议按下面顺序做。

### 第 1 步：先确认能力类型

先明确这到底是哪种 ROS2 能力：

- 服务调用
- 话题订阅
- 话题发布
- 长时间状态监听
- 组合操作

不同类型接法不同。

### 第 2 步：把 ROS2 交互收口到 `ROS2Node`

不要在 `NativeWebService` 里直接散落写 `rclpy` 逻辑。

正确方式是：

- 所有 ROS2 调用都尽量收口到 `node_runtime.py`

为什么？

- 生命周期更容易统一管理
- 错误定位更清晰
- 测试时更容易用 `DummyNode` 替代

### 第 3 步：服务调用推荐模式

推荐保持和当前项目一致：

1. `create_client(...)`
2. `wait_for_service(...)`
3. `call_async(...)`
4. `_spin_future(...)`
5. 统一转换为 Python 结果

这样做的好处是：

- 代码风格一致
- 失败处理一致
- 超时处理一致

### 第 4 步：话题订阅推荐模式

如果需要订阅话题，先判断是：

- 常驻订阅
- 临时订阅

当前相机采图使用的是：

- 临时订阅

因为它只在采图时短暂需要数据。

这种方式的好处是：

- 节省带宽
- 避免无意义持续消费
- 更适合“触发一次拿一帧”的场景

如果以后有视频流预览、状态流订阅等需求，再考虑常驻订阅。

### 第 5 步：如果是长任务，优先考虑 WebSocket 联动

例如：

- 模型推理要 20 秒
- 批量模板处理要 1 分钟
- 机器人抓取链路要持续反馈状态

这时不要只返回一个“正在处理中”。

更推荐：

- HTTP 用于发起任务
- WebSocket 用于返回进度、状态、结果

这也是为什么当前项目提前保留了 `/ws` 和 `WebSocketManager`。

---

## 5. 新增接口时，文件应该改哪里

这里给你一个实战映射表。

### 场景 1：新增一个纯数据查询接口

例如：

- 查询当前任务状态
- 查询配置摘要

通常会改：

- `routers/*.py`
- `services/native_api.py`

如果不涉及 ROS2，不需要动 bridge。

### 场景 2：新增一个 ROS2 服务调用接口

例如：

- 新的抓取控制接口
- 新的机器人动作接口

通常会改：

- `routers/*.py`
- `services/native_api.py`
- `ros_bridge/node_runtime.py`

### 场景 3：新增文件读写能力

例如：

- 新增模板导出
- 新增历史记录保存

通常会改：

- `services/native_api.py`
- 必要时 `runtime_support.py`

### 场景 4：新增实时推送能力

例如：

- 任务进度广播
- 算法状态广播

通常会改：

- `routers/system.py` 或新增路由
- `ws/manager.py`
- 相关 service / task 管理逻辑

---

## 6. 新增功能时，推荐的开发顺序

很多人开发时容易直接从最底层开始写，最后发现 URL 没接好，或者测试不好写。

推荐顺序是：

1. 先想清楚接口输入输出
2. 先决定放哪个路由域
3. 先写 service 方法签名
4. 再写 bridge 方法
5. 再回到路由完成接线
6. 最后补测试

为什么这样更好？

因为这样你会先明确“接口契约”，而不是一开始就陷进 ROS2 细节里。

---

## 7. 如何给新增接口补测试

### 第 1 步：先看它依赖什么

例如你的新 service 里调用了：

- `node.foo_action(value)`

那就先在 `DummyNode` 里补一个最小实现：

```python
def foo_action(self, value):
    return {"success": True, "value": value}, None
```

### 第 2 步：写成功路径

```python
def test_foo_action_route():
    with create_test_client() as client:
        response = client.post("/api/foo_action", json={"value": 123})

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["value"] == 123
```

### 第 3 步：补失败路径

```python
def test_foo_action_route_missing_value():
    with create_test_client() as client:
        response = client.post("/api/foo_action", json={})

    assert response.status_code == 400
```

### 第 4 步：如果有副作用，就验证副作用

例如：

- 写文件
- 更新参数
- 改变状态

不要只断言响应，要断言副作用结果。

---

## 8. 怎样判断某段逻辑该不该抽出来

如果你在写代码时发现自己在 `native_api.py` 里开始反复做这些事情：

- 反复拼路径
- 反复读配置
- 反复做姿态归一化
- 反复处理模板目录
- 反复做数据格式转换

就说明这段逻辑可能应该抽到支持层。

判断标准不是“代码行数多少”，而是：

- 这段逻辑会不会被复用
- 这段逻辑是不是和具体业务接口无关

如果答案是“会复用”或“和业务无关”，就考虑抽。

---

## 9. 怎样避免重新回到“单体大文件”

这是后续开发里最需要警惕的问题。

你可以用下面这张清单提醒自己。

### 不建议做的事

- 在某个路由函数里直接写大量业务逻辑
- 在多个路由里重复写同样的数据处理
- 在 `native_api.py` 里直接散落写 `rclpy`
- 临时想到什么就往 `app.py` 塞
- 直接在真实目录里做测试

### 推荐做的事

- 路由只做接线
- 业务逻辑进 service
- ROS2 交互进 bridge
- 公共逻辑进 support
- 测试用 dummy 隔离真实环境

---

## 10. 新增功能后的最小自检清单

每次你新增一个接口，最少做下面几项检查。

### 10.1 代码层检查

- 路由是否注册了
- service 是否接通了
- bridge 是否有对应方法
- 错误分支是否处理了

### 10.2 测试层检查

- 至少有一个成功路径测试
- 至少有一个失败路径测试
- 有副作用时验证副作用

### 10.3 启动层检查

- 服务能否正常启动
- `/health` 是否正常
- `/status` 是否正常

### 10.4 前端联调检查

- 旧 UI 或调用方是否已接上新接口
- 字段名是否一致
- 返回结构是否和预期一致

---

## 11. 如果以后要接入 YOLO / 大模型 / 强化学习，应该怎么放

当前结构已经为这些能力预留了位置。

### YOLO / 模型推理

建议：

- 新增 `routers/model.py`
- 在 `services/` 增加模型 service
- 必要时再加独立推理管理模块

### 强化学习训练/评估

建议：

- HTTP 用于启动任务
- WebSocket 用于状态更新
- 训练任务不要直接塞进路由函数里同步跑

### 大模型对话/规划

建议：

- 单独拆成新 service
- 保持和视觉控制、机器人控制逻辑解耦

也就是说，未来新能力应该“加模块”，而不是“加判断分支塞进旧函数”。

---

## 12. 如果以后要逐步替换前端，后端需要改吗

当前后端已经是：

- API 化
- 路由化
- 结构化

所以以后如果前端从旧静态页面逐步换成：

- Vue
- React
- 其他前端框架

理论上后端不需要大改。

这正是当前迁移的一个重要收益：

- 先把后端边界稳定住
- 前端以后可以单独演进

---

## 13. 新成员接手开发时，建议怎么读代码

如果以后有新成员接手开发，建议按下面顺序熟悉：

1. `web/app.py`
2. `web/server.py`
3. `web/routers/system.py`
4. `web/services/native_api.py`
5. `web/ros_bridge/manager.py`
6. `web/ros_bridge/node_runtime.py`
7. `test/test_web_app.py`
8. `web_ui/docs/FASTAPI_TESTING_GUIDE.md`

这个顺序的目的就是：

- 先看整体入口
- 再看业务
- 再看底层
- 最后看测试如何兜底

---

## 14. 给后续开发的一个固定模板

如果你后面每次开发都按下面模板走，基本不会偏。

### 新增功能模板

1. 明确接口输入输出
2. 放到正确的 router 域
3. 在 `NativeWebService` 增加方法
4. 必要时在 `node_runtime.py` 增加 ROS2 能力
5. 必要时抽公共逻辑到 support 层
6. 在 `DummyNode` / `DummyRosBridge` 里补测试替身
7. 在 `test/test_web_app.py` 补成功路径
8. 补失败路径
9. 验证副作用
10. 运行测试回归

---

## 15. 一句话总结

这套 FastAPI 架构真正的价值，不只是“现在能跑”，而是：

- 以后新增功能时，你知道该把代码放哪
- 以后接 ROS2 新能力时，你知道该从哪一层接
- 以后写测试时，你知道该怎么隔离外部环境

如果你一直按这份文档的方式扩展，系统会继续保持清晰，而不会重新长回旧的单体结构。

## 延伸阅读

- `FASTAPI_INTERFACE_TEMPLATE.md`
  - 直接照着新增接口、Service、bridge 和测试模板
- `FASTAPI_ARCHITECTURE_DIAGRAMS.md`
  - 用图看整体结构、请求时序和测试分层
