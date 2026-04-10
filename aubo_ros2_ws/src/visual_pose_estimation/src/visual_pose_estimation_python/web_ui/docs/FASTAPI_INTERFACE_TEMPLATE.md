# FastAPI 新增接口模板

这份文档给你一套可以直接照着抄的模板，用于后续继续给当前项目新增 Web 接口。

适用场景：

- 新增一个普通 JSON 接口
- 新增一个需要调用 ROS2 的接口
- 新增一个带文件副作用的接口
- 新增一个需要补测试的接口

---

## 1. 先决定接口属于哪个模块

新增接口前，先判断它属于哪个业务域：

- 相机相关：`routers/camera.py`
- 姿态估计相关：`routers/pose.py`
- 模板相关：`routers/templates.py`
- 机器人相关：`routers/robot.py`
- 抓取相关：`routers/grasp.py`
- Debug 相关：`routers/debug.py`
- 系统/状态相关：`routers/system.py`

如果不属于任何已有域，再考虑新建一个新的路由文件。

---

## 2. 最小新增接口模板

假设要新增接口：

- `POST /api/foo_action`

### 第一步：在路由层接线

文件示例：

- `visual_pose_estimation_python/web/routers/robot.py`

模板：

```python
from fastapi import APIRouter, Body, Depends

from ..dependencies import get_native_service
from ..services import NativeWebService

router = APIRouter(prefix="/api", tags=["robot"])


@router.post("/foo_action")
def foo_action(
    payload: dict = Body(...),
    service: NativeWebService = Depends(get_native_service),
):
    return service.foo_action(payload)
```

路由层只负责：

- 接收请求
- 调用 service

不要在这里写复杂业务逻辑。

---

## 3. Service 层模板

文件位置：

- `visual_pose_estimation_python/web/services/native_api.py`

### 场景 A：纯参数处理 + 返回

```python
from fastapi import HTTPException

def foo_action(self, payload: dict[str, Any]) -> dict[str, Any]:
    value = payload.get("value")
    if value is None:
        raise HTTPException(status_code=400, detail="缺少 value")

    return {
        "success": True,
        "value": value,
    }
```

### 场景 B：调用 ROS2 bridge

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

### 场景 C：带文件副作用

```python
def foo_save(self, payload: dict[str, Any]) -> dict[str, Any]:
    name = str(payload.get("name", "")).strip()
    if not name:
        raise HTTPException(status_code=400, detail="name 不能为空")

    target_dir = self.templates_dir / "foo"
    target_dir.mkdir(parents=True, exist_ok=True)
    file_path = target_dir / f"{name}.json"

    with open(file_path, "w", encoding="utf-8") as file_obj:
        json.dump(payload, file_obj, ensure_ascii=False, indent=2)

    return {
        "success": True,
        "file_path": str(file_path),
    }
```

Service 层建议负责：

- 参数校验
- 业务逻辑
- 文件读写
- 调 ROS2 bridge
- 把异常转成 `HTTPException`

---

## 4. ROS2 bridge 层模板

如果接口需要接 ROS2，请在：

- `visual_pose_estimation_python/web/ros_bridge/node_runtime.py`

里增加方法。

### 4.1 ROS2 服务调用模板

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

    return {
        "success": bool(response.success),
        "message": str(response.message),
    }, None
```

### 4.2 话题订阅模板

如果后续新增的是短时订阅能力，可以参考当前采图逻辑：

```python
def foo_subscribe_once(self, timeout=5.0):
    result_holder = {"value": None}

    def callback(msg):
        result_holder["value"] = msg.data

    sub = self.create_subscription(FooMsg, "/foo_topic", callback, 10)
    start_time = time.time()
    try:
        while time.time() - start_time < timeout:
            if result_holder["value"] is not None:
                return {"success": True, "value": result_holder["value"]}, None
            time.sleep(0.05)
        return None, "订阅超时"
    finally:
        self.destroy_subscription(sub)
```

---

## 5. Dummy 测试替身模板

如果你的新接口依赖 `DummyNode`，就要同步补：

文件位置：

- `test/test_web_app.py`

### 最小 Dummy 模板

```python
class DummyNode:
    def foo_action(self, value):
        return {"success": True, "value": value}, None
```

如果接口有状态变化：

```python
class DummyNode:
    def __init__(self):
        self.last_value = None

    def foo_action(self, value):
        self.last_value = value
        return {"success": True, "value": value}, None
```

---

## 6. 测试模板

### 6.1 成功路径

```python
def test_foo_action_route():
    with create_test_client() as client:
        response = client.post("/api/foo_action", json={"value": 123})

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["value"] == 123
```

### 6.2 缺参数路径

```python
def test_foo_action_route_missing_value():
    with create_test_client() as client:
        response = client.post("/api/foo_action", json={})

    assert response.status_code == 400
```

### 6.3 文件副作用路径

```python
def test_foo_save_route():
    with create_test_client() as client:
        response = client.post("/api/foo_save", json={"name": "demo", "value": 1})
        file_path = client.app.state.ros_bridge.templates_dir / "foo" / "demo.json"

    assert response.status_code == 200
    assert file_path.exists()
```

### 6.4 patch 模板

如果你要验证某个函数有没有被调用：

```python
from unittest.mock import patch


def test_foo_calls_bar():
    with patch("visual_pose_estimation_python.web.routers.system.schedule_exit") as mock_fn:
        with create_test_client() as client:
            response = client.post("/exit")

    assert response.status_code == 200
    mock_fn.assert_called_once_with()
```

---

## 7. 最推荐的新增接口工作流

以后每次加接口，可以直接按下面流程做：

1. 先确定属于哪个路由域
2. 先写路由接线
3. 再写 `NativeWebService` 方法
4. 如需 ROS2，再写 `node_runtime.py`
5. 同步补 `DummyNode`
6. 先补成功路径测试
7. 再补失败路径测试
8. 如果有文件副作用，再补副作用测试
9. 跑 `pytest test/test_web_app.py`

---

## 8. 最小自检清单

每次新增接口后，至少确认：

- 路由已注册
- Service 已接通
- ROS2 bridge 方法存在
- 参数错误时能返回 400
- 正常路径返回结构正确
- 如果有副作用，副作用已验证

---

## 9. 一句话总结

新增接口时，记住一条最重要的规则：

- 路由负责接线
- Service 负责业务
- bridge 负责 ROS2
- Dummy 负责隔离测试

只要一直按这个模板扩展，系统就不会重新退化成旧的单体结构。
