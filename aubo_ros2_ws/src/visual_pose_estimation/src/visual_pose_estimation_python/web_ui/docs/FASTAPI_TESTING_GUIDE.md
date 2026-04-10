# 零基础学习如何编写测试验证代码

这份文档专门解释一件事：

- 这个项目里的 FastAPI 代码，应该如何做测试验证
- 测试代码为什么要这样写
- 怎样从 0 写出一套能长期维护的测试

这份文档会结合当前真实测试文件：

- `test/test_web_app.py`

来讲，而不是只讲抽象概念。

---

## 1. 为什么一定要写测试验证代码

对这个项目来说，测试不是“可选项”，而是迁移是否成功的证明。

原因很简单：

- 这个系统既有 Web，又有 ROS2
- 既有文件读写，又有图像处理
- 既有机器人控制，又有调试接口

如果没有测试，迁移时会出现这些问题：

- 改了一个接口，不知道其他接口是否被带坏
- 删除 legacy 代码后，不知道行为是否保持一致
- 调整路径解析后，不知道模板读写是否正常
- 改动 WebSocket 后，不知道前端连接是否还通

所以测试的真正作用不是“追求覆盖率数字”，而是：

- 给重构兜底
- 给迁移提供回归保证
- 给以后继续加功能留安全网

---

## 2. 这个项目里测试最难的地方是什么

初学者最容易低估的一点是：

这个项目的难点不是“HTTP 接口很多”，而是“HTTP 背后牵着 ROS2 和真实硬件”。

如果直接拿真实环境测：

- 需要相机在线
- 需要 ROS2 服务在线
- 需要机械臂状态正常
- 需要模板目录、配置文件、图像都准备好

这样会导致测试：

- 慢
- 脆弱
- 难在开发机和 CI 上复现

所以当前测试策略是：

- Web 层单元/集成测试优先隔离外部环境
- 用假的 `RosBridge` 和假的 `ROS2Node` 代替真实 ROS2
- 用临时目录代替真实模板目录
- 用假图像数据代替真实相机数据

这就是为什么 `test/test_web_app.py` 里有很多 `Dummy*` 类。

---

## 3. 当前测试文件在做什么

当前核心测试文件：

- `test/test_web_app.py`

它的任务可以概括为三类：

### 3.1 验证系统基础能力

例如：

- `/health`
- `/status`
- `/index.html`
- `/ws`
- `/exit`

这些接口验证的是：

- 应用能不能启动
- 路由有没有挂对
- WebSocket 是否可用
- 退出逻辑有没有被正确触发

### 3.2 验证业务接口行为

例如：

- `/api/capture_image`
- `/api/estimate_pose`
- `/api/list_templates`
- `/api/save_template_pose`
- `/api/get_robot_status`
- `/api/execute_single_grasp`
- `/api/debug/*`

这些测试关注的是：

- 输入参数给进去后，返回值是否符合预期
- 路由有没有正确调到服务层
- 服务层有没有正确调用 bridge 层

### 3.3 验证副作用

例如：

- 是否生成了模板文件
- 返回的图像 `content-type` 是否正确
- 保存的 JSON 内容是否被规范化
- debug 参数保存逻辑是否执行

也就是说，这个测试文件不只是测“HTTP 200”，而是在测“这个接口的关键业务结果是否成立”。

---

## 4. 为什么测试里要造假对象 Dummy

当前测试里有这些关键假对象：

- `DummyRosBridge`
- `DummyNode`
- `DummyConfigReader`
- `DummyParamsManager`
- `DummyPreprocessor`
- `DummyFeatureExtractor`

### 4.1 `DummyRosBridge` 的作用

它模拟真实的 `RosBridgeManager`，但不会真的启动 ROS2。

它主要提供：

- `is_ready`
- `startup_error`
- `templates_dir`
- `pose_list_dir`
- `camera_pose_fixed_orientation`
- `node`
- `status()`

为什么要这样做？

因为 Web 层并不关心这是不是“真实 ROS2 管理器”，它只关心：

- 有没有这些属性
- 调用这些方法时会不会得到预期结果

这就是测试隔离的核心思想：

- 只模拟当前测试真正需要的行为
- 不模拟多余的复杂性

### 4.2 `DummyNode` 的作用

它模拟真实 `ROS2Node` 的关键方法，例如：

- `capture_image`
- `estimate_pose`
- `list_templates`
- `standardize_template`
- `get_robot_status`
- `move_to_pose`
- `set_robot_io`
- `call_execute_single_grasp`
- `call_loop_grasp_control`
- `call_publish_grasps_loop_control`
- `call_run_gripper_swap`

为什么它这么重要？

因为 `NativeWebService` 的业务逻辑，很多都依赖：

- `self._ros_bridge.node`

所以只要你把 `node` 替换成一个“接口一样、行为可控”的假对象，就能把整个 Web 层从真实 ROS2 环境里解耦出来。

### 4.3 `DummyConfigReader` / `DummyParamsManager`

这两个对象的作用是让 debug 路由可以测试：

- 获取参数
- 更新参数
- 保存参数

但又不必依赖真实配置文件和真实 ROS2 参数更新链。

### 4.4 `DummyPreprocessor` / `DummyFeatureExtractor`

这两个对象的作用是让 debug 图像处理链条有最小可用实现。

这样测试时：

- 可以得到一个假的连通域
- 可以得到一个假的 feature
- 从而让 debug 返回结构完整

否则 `/api/debug/get_images` 这类接口就没法在纯测试环境里跑起来。

---

## 5. 为什么 `create_test_client()` 很关键

测试里有一个核心入口：

```python
def create_test_client():
    app = create_app()
    app.state.ros_bridge = DummyRosBridge()
    app.state.native_service = NativeWebService(app.state.ros_bridge)
    return TestClient(app)
```

它做了三件最关键的事：

1. 用真实 `create_app()` 创建 FastAPI 应用
2. 把应用里原本的 `ros_bridge` 换成假的
3. 基于假的 bridge 重新创建 `native_service`

这样做的意义非常大：

- 你测到的是真实路由
- 你测到的是真实依赖注入路径
- 你测到的是真实服务层逻辑
- 但你不需要真实 ROS2 环境

这就是一种很典型的“轻量集成测试”写法。

它不是只测某一个函数，也不是全量拉真实系统，而是：

- 保留应用整体结构
- 替换最重的外部依赖

---

## 6. FastAPI 里如何测试接口

FastAPI 官方常用方式就是：

- `fastapi.testclient.TestClient`

使用方法非常像真实前端请求：

```python
with create_test_client() as client:
    response = client.post("/api/capture_image", json={"camera_id": "cam_01"})
```

然后再断言：

```python
assert response.status_code == 200
assert response.json()["success"] is True
```

这种写法的好处是：

- 很接近真实调用方式
- 初学者容易理解
- 后续排查问题时直观

---

## 7. 这个项目里的测试是怎么分类写的

当前测试写法其实已经体现了一种很好的分类方式。

### 7.1 系统基础接口测试

例如：

- `test_health_endpoint`
- `test_status_endpoint`
- `test_index_alias_exists`

这类测试关注：

- 能不能访问
- 返回结构对不对
- 基本状态字段对不对

### 7.2 普通业务接口测试

例如：

- `test_native_capture_image_route`
- `test_native_estimate_pose_route`
- `test_native_robot_routes`
- `test_native_grasp_routes`

这类测试关注：

- 某个接口对应的业务是否正常

### 7.3 文件副作用测试

例如：

- `test_native_template_routes`
- `test_native_save_template_pose_and_standardize`
- `test_native_execute_pose_sequence_route`

这类测试关注：

- 文件有没有被创建
- 内容有没有按预期写进去

### 7.4 特殊行为测试

例如：

- `test_exit_endpoint_schedules_shutdown`
- `test_websocket_status_and_ping`

这类测试关注：

- WebSocket 连接是否正常
- 某些副作用函数是否被调用

这种分类方式很适合继续扩展。

---

## 8. 为什么要验证“结果”，而不是只看状态码

很多初学者写测试时容易只写：

```python
assert response.status_code == 200
```

但这远远不够。

因为：

- 状态码是 200，不代表逻辑一定对
- 接口可能返回了空数据、错数据、假数据

所以当前测试会继续验证关键结果，例如：

### 8.1 图像接口

```python
assert payload["depth_image_base64"].startswith("data:image/png;base64,")
assert payload["color_image_base64"].startswith("data:image/jpeg;base64,")
```

这里验证的是：

- 返回的不只是成功
- 而且返回值格式也正确

### 8.2 模板文件保存

```python
saved_payload = json.loads(saved_json)
assert saved_payload["cartesian_position"]["orientation"]["x"] == CAMERA_POSE_FIXED_ORIENTATION["orientation"]["x"]
```

这里验证的是：

- 不只是文件写了
- 而且写进去的内容经过了姿态归一化和固定姿态覆盖

### 8.3 WebSocket

```python
assert initial_message["type"] == "status"
assert pong_message == {"type": "pong"}
```

这里验证的是：

- WebSocket 连接成功
- 协议交互也符合预期

这才是真正有价值的测试。

---

## 9. 为什么测试里要用临时目录

例如模板测试里使用：

- `tempfile.mkdtemp(prefix="vpe_templates_")`
- `tempfile.mkdtemp(prefix="vpe_pose_list_")`

作用是：

- 每次测试都有独立目录
- 不污染真实项目数据
- 测试互相之间不影响

如果不用临时目录，而是直接写真实目录，会出现：

- 上一次测试遗留文件影响下一次
- 本地真实模板被覆盖
- 测试结果不稳定

所以“涉及文件副作用的测试”最好都走临时目录。

---

## 10. 为什么 `unittest.mock.patch` 也要用

例如：

```python
with patch("visual_pose_estimation_python.web.routers.system.schedule_exit") as schedule_exit_mock:
    ...
    schedule_exit_mock.assert_called_once_with()
```

这个测试为什么不能真的让进程退出？

因为测试的目标不是：

- 真把当前测试进程杀掉

而是：

- 验证路由是否正确调用了“退出调度函数”

这就是 patch 的意义：

- 把危险或不可控的真实行为替换成可断言的假行为

所以什么时候适合 patch？

- 调用退出
- 调用真实外部服务
- 调用系统命令
- 调用可能导致副作用过大的逻辑

---

## 11. 这个项目里如何逐步编写一个新接口的测试

假设你新增了一个接口：

- `/api/foo`

推荐按这个顺序写测试。

### 第 1 步：先想清楚它依赖什么

先问自己：

- 它会不会调用 `ros_bridge.node`？
- 它会不会读写文件？
- 它会不会依赖配置？
- 它会不会有异常分支？

### 第 2 步：如果依赖外部环境，就先补 Dummy 行为

例如新增 service 里调用：

- `node.foo_action(value)`

那就先在 `DummyNode` 里补一个：

```python
def foo_action(self, value):
    return {"success": True, "value": value}, None
```

### 第 3 步：用 `TestClient` 发请求

```python
def test_foo_route():
    with create_test_client() as client:
        response = client.post("/api/foo", json={"value": 123})

    assert response.status_code == 200
    assert response.json()["success"] is True
    assert response.json()["value"] == 123
```

### 第 4 步：如果有副作用，就验证副作用

如果会写文件，就检查文件是否存在。

如果会改参数，就检查结果是否真的变了。

如果会调某个内部函数，就用 patch 检查它是否被调用。

### 第 5 步：补错误分支

例如缺参数时：

```python
def test_foo_route_missing_value():
    with create_test_client() as client:
        response = client.post("/api/foo", json={})

    assert response.status_code == 400
```

这一步非常重要。

很多人只测“成功路径”，不测“失败路径”，结果代码一上线，最容易炸的恰恰是异常分支。

---

## 12. 建议怎样设计测试用例

给一个接口写测试时，建议最少考虑 4 类情况：

### 12.1 正常成功路径

- 输入合法
- 返回成功

### 12.2 参数错误路径

- 缺参数
- 参数类型不对
- 参数值非法

### 12.3 外部依赖失败路径

例如：

- ROS2 服务失败
- 返回 `None`
- 文件不存在

### 12.4 副作用结果路径

例如：

- 文件被创建
- 配置被更新
- WebSocket 返回特定消息

如果你每次写测试都按这四类想，质量会高很多。

---

## 13. 对这个项目最推荐的测试层次

这个项目建议采用三层测试思路。

### 第一层：Web 层隔离测试

就是现在 `test/test_web_app.py` 这种。

特点：

- 快
- 稳
- 适合日常开发回归

这是最应该优先补齐的一层。

### 第二层：服务层单元测试

如果后面 `NativeWebService` 越来越复杂，可以进一步单独对它某些方法做纯 Python 单元测试。

适合：

- 文件路径处理
- 参数规范化
- 错误转换逻辑

### 第三层：真实集成验证

在真实 ROS2 环境、真实相机、真实机械臂链路下做完整联调。

这层不是用来替代前两层，而是用来补充：

- 验证真实环境兼容
- 验证设备联动

所以不要把所有验证压力都压到真实环境联调上。

---

## 14. 如何运行当前测试

在包目录下运行：

```bash
cd /home/mu/IVG2.0/aubo_ros2_ws/src/visual_pose_estimation/src/visual_pose_estimation_python
pytest test/test_web_app.py
```

如果你只想跑某一个测试，例如：

```bash
pytest test/test_web_app.py -k websocket
```

或：

```bash
pytest test/test_web_app.py -k debug
```

这样在开发某一个模块时更高效。

---

## 15. 测试编写时最常见的错误

### 15.1 只测状态码，不测内容

问题：

- 接口返回错数据也测不出来

### 15.2 测试依赖真实环境

问题：

- 一换机器就挂
- 没设备就跑不了

### 15.3 不隔离文件副作用

问题：

- 测试污染真实目录
- 多次运行结果不稳定

### 15.4 不测失败路径

问题：

- 真正线上最容易坏的分支没被覆盖

### 15.5 Dummy 造得太少或太多

太少：

- 接口根本跑不起来

太多：

- 测试实现太复杂，自己也不好维护

正确做法是：

- 只模拟当前测试必须的行为

---

## 16. 未来继续补测试时的建议

如果你后续继续演进这套 Web，推荐优先补这些测试：

### 16.1 错误路径测试

当前已有很多成功路径，后面可以继续补：

- `capture_image` 失败时的 500
- `estimate_pose` 缺图像时的 400
- `save_template_pose` 缺字段时的 400
- `get_template_image` 文件不存在时的 404

### 16.2 路径与配置测试

例如：

- `app_config.json` 中 `template_root` 生效
- `camera_pose_fixed_orientation` 覆盖正确

### 16.3 WebSocket 扩展测试

如果后面增加：

- 进度推送
- 日志推送
- 长任务状态推送

就继续补 WebSocket 消息协议测试。

### 16.4 启动与装配测试

例如：

- `create_app()` 是否完成路由注册
- `/legacy-ui` 是否可挂载

---

## 17. 一句话记住怎么写测试

对这个项目来说，一条高质量测试的标准不是“写得多复杂”，而是：

- 能隔离 ROS2/硬件
- 能保留真实 FastAPI 路径
- 能验证关键业务结果
- 能在重构时真正兜底

也就是说：

先搭真实应用骨架，再替换最重的外部依赖，然后验证最关键的行为结果。

这就是当前 `test/test_web_app.py` 的核心方法论。

## 延伸阅读

- `FASTAPI_BEGINNER_GUIDE.md`
  - 从 0 理解当前 FastAPI 架构
- `FASTAPI_MIGRATION_GUIDE.md`
  - 理解为什么要这样迁移
- `FASTAPI_EXTENSION_GUIDE.md`
  - 学会后续如何新增接口、接 ROS2 能力、继续扩展系统
