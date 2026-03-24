# 从 0 到新增一个完整接口示例

这份文档演示一个完整的端到端开发流程：

- 新增一个 Web 接口
- 接到 Service
- 接到 ROS2 bridge
- 补 Dummy
- 补测试
- 做自检

这里用一个“教学示例接口”来讲：

- `POST /api/check_bridge_ready`

这个接口的目标很简单：

- 返回当前 ROS2 bridge 是否 ready
- 返回 bridge 模块名

之所以选这个例子，是因为它足够简单，能把结构讲清楚，又不会引入太多外部复杂性。

---

## 1. 先明确这个接口应该放在哪里

这个接口是：

- 系统状态类接口
- 不是相机
- 不是模板
- 不是抓取
- 不是 debug

所以应该放在：

- `visual_pose_estimation_python/web/routers/system.py`

这就是第一步最重要的判断：

- 先分层，再写代码

---

## 2. 路由层怎么写

目标接口：

- `POST /api/check_bridge_ready`

写法示例：

```python
@router.post("/api/check_bridge_ready")
def check_bridge_ready(
    ros_bridge: RosBridgeManager = Depends(get_ros_bridge),
):
    return {
        "success": True,
        "ready": ros_bridge.is_ready,
        "bridge_module": ros_bridge.status().get("bridge_module"),
    }
```

这里你会发现：

- 这个接口足够简单，甚至可以不走 `NativeWebService`

但从长期维护角度，更推荐仍然走 service 层。

因为这样未来如果逻辑变复杂，不需要再把接口从路由里搬走。

所以更推荐下面这种做法。

---

## 3. 推荐写法：仍然走 Service 层

### 第一步：路由层接线

```python
@router.post("/check_bridge_ready")
def check_bridge_ready(
    service: NativeWebService = Depends(get_native_service),
):
    return service.check_bridge_ready()
```

这里路由层只负责：

- 定义 URL
- 调 service

---

## 4. Service 层怎么写

在：

- `visual_pose_estimation_python/web/services/native_api.py`

里新增：

```python
def check_bridge_ready(self) -> dict[str, Any]:
    return {
        "success": True,
        "ready": self._ros_bridge.is_ready,
        "startup_error": self._ros_bridge.startup_error,
        "bridge_module": self._ros_bridge.status().get("bridge_module"),
    }
```

这个示例体现了一个关键原则：

- 如果逻辑只是从共享对象里读取状态，Service 就负责统一组织返回结构

好处是：

- 路由层更轻
- 后续字段变化时，只改 Service

---

## 5. 如果这个接口需要接 ROS2 bridge 方法，该怎么做

假设未来你想把它改成：

- 不只是读 `ros_bridge.is_ready`
- 而是调用 `node_runtime.py` 里的一个方法 `node.check_bridge_ready()`

那做法就是：

### 5.1 在 `node_runtime.py` 增加方法

```python
def check_bridge_ready(self):
    return {
        "success": True,
        "node_name": self.get_name(),
        "executor_running": self.executor_running,
    }, None
```

### 5.2 在 `NativeWebService` 里调它

```python
def check_bridge_ready(self) -> dict[str, Any]:
    node = self._require_node()
    result, error = node.check_bridge_ready()
    if error:
        raise HTTPException(status_code=500, detail=error)
    return result
```

这个流程就是：

- 路由层不直接碰 ROS2
- Service 调 bridge
- bridge 调 ROS2 / Node 本体

---

## 6. Dummy 测试替身怎么补

如果你的 `NativeWebService.check_bridge_ready()` 是直接读 `self._ros_bridge.is_ready`，那可能不需要改 `DummyNode`。

但如果它调用了：

- `node.check_bridge_ready()`

那你必须同步补：

```python
class DummyNode:
    def check_bridge_ready(self):
        return {
            "success": True,
            "node_name": "dummy_node",
            "executor_running": False,
        }, None
```

这一步非常重要。

因为测试之所以能隔离真实 ROS2，就是靠 `DummyNode` 保持“接口一致”。

---

## 7. 测试怎么写

### 7.1 成功路径测试

```python
def test_check_bridge_ready_route():
    with create_test_client() as client:
        response = client.post("/api/check_bridge_ready")

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["ready"] is True
```

### 7.2 如果有错误路径，也要补

假设你未来把这个接口改成必须要求 bridge 已初始化，否则报错，那也要补失败路径。

例如：

```python
def test_check_bridge_ready_route_unavailable():
    app = create_app()
    app.state.ros_bridge = DummyRosBridge()
    app.state.ros_bridge.is_ready = False
    app.state.native_service = NativeWebService(app.state.ros_bridge)

    with TestClient(app) as client:
        response = client.post("/api/check_bridge_ready")

    assert response.status_code == 200
    assert response.json()["ready"] is False
```

这里的要点是：

- 成功和失败，不一定都必须用 HTTP 4xx/5xx
- 有时候“ready=false”也是一种正常业务结果

所以测试要跟业务语义一致，而不是只追求某种固定状态码。

---

## 8. 如果新增的是一个真正业务接口，完整流程也是一样的

上面的例子比较简单，下面用一个更贴近真实业务的虚拟例子：

- 新增 `POST /api/get_debug_summary`

目标：

- 返回当前 debug 参数
- 返回当前 feature 数量

### 开发步骤

1. 判断业务域
   - 这是 debug 相关
   - 所以放 `routers/debug.py`

2. 路由层加：

```python
@router.post("/debug/get_summary")
def debug_get_summary(
    service: NativeWebService = Depends(get_native_service),
):
    return service.debug_get_summary()
```

3. Service 层加：

```python
def debug_get_summary(self) -> dict[str, Any]:
    node = self._require_node()
    params = node.config_reader.load_debug_thresholds()
    features = getattr(node, "_debug_features", [])
    return {
        "success": True,
        "param_count": len(params),
        "feature_count": len(features),
        "params": params,
    }
```

4. 如果 `DummyNode` 没有相关属性，就补：

```python
class DummyNode:
    def __init__(self):
        ...
        self._debug_features = [1]
```

5. 测试：

```python
def test_debug_get_summary_route():
    with create_test_client() as client:
        response = client.post("/api/debug/get_summary")

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["feature_count"] == 1
```

你会发现，不管接口简单还是复杂，流程都一样：

- 定位业务域
- 路由接线
- Service 实现
- 必要时 bridge 实现
- Dummy 补齐
- 测试补齐

---

## 9. 编写过程中最容易犯的错误

### 错误 1：直接在路由里堆业务逻辑

例如在 `routers/*.py` 里直接写：

- 文件读写
- 大段参数处理
- ROS2 调用

这样做短期看快，长期一定会变乱。

### 错误 2：Service 直接乱写 `rclpy`

如果在 `native_api.py` 到处散落 ROS2 逻辑，会把 bridge 层意义破坏掉。

### 错误 3：新增功能不补 Dummy

如果改了 Service 或 bridge，但忘了改 `DummyNode`，测试很快就会失效。

### 错误 4：只补成功路径

这样最容易漏掉参数错误和边界条件。

---

## 10. 每次新增接口后应该怎么验证

建议固定执行下面这套最小验证流程。

### 第一步：跑 Web 测试

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python
pytest test/test_web_app.py
```

### 第二步：只跑你新增相关测试

例如：

```bash
pytest test/test_web_app.py -k bridge
pytest test/test_web_app.py -k debug
```

### 第三步：人工读一遍接口层级

问自己三个问题：

1. 路由是不是只做了接线？
2. Service 是不是承担了业务逻辑？
3. ROS2 交互是不是仍然收口在 bridge？

这一步虽然不是自动化测试，但非常重要。

---

## 11. 这个示例真正想让你学会什么

这份文档不是想教你写某一个具体接口，而是想让你掌握一种固定模式：

### 固定模式

1. 明确接口属于哪个业务域
2. 路由只接线
3. Service 负责业务
4. bridge 负责 ROS2
5. Dummy 负责测试隔离
6. pytest 负责回归验证

只要你以后始终按这个模式开发，系统就能继续保持清晰。

---

## 12. 一句话总结

新增一个接口，真正重要的不是“把它写出来”，而是：

- 把它写进正确的层
- 给它补齐正确的测试
- 让它不会破坏现有结构

这才是当前这套 FastAPI 架构最想教会你的能力。
