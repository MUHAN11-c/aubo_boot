# 零基础学习迁移替换全过程

这份文档专门解释本项目为什么要从旧 Web 方案迁移到 FastAPI、迁移过程中做了什么、为什么这样做、这样做的好处是什么，以及最终形成了什么样的文件结构和逻辑结构。

这份文档适合：

- 想从 0 理解“为什么要迁移”
- 想知道“迁移不是删旧代码重写，而是如何分阶段落地”
- 想复盘整个替换过程
- 想以后继续扩展这套架构

---

## 1. 迁移前的问题是什么

迁移前，旧 Web 方案的核心问题不是“它不能用”，而是“它不适合继续长大”。

旧方案的典型特点：

- Web 接口逻辑集中在一个单体大文件里
- HTTP 处理、业务逻辑、ROS2 调用、模板文件操作、Debug 逻辑混在一起
- 新增功能时，容易继续把逻辑堆到一个文件里
- 不利于长期演进到更复杂的系统

对于当前项目目标来说，这些问题会越来越明显。

因为项目未来不仅要做：

- 当前的视觉姿态估计

还要继续容纳：

- ROS2 节点协作
- OpenCV 图像处理
- 深度学习模型推理
- 强化学习相关流程
- YOLO
- 大模型
- 长耗时任务
- 实时状态推送

也就是说，这不是一个“单页小工具”的后端，而是一个“机器人 + AI + Web”综合系统的后端。

旧结构不够适合做这种长期演进。

---

## 2. 为什么选择 FastAPI，而不是继续修旧 HTTP 服务

可以把这次选择理解成两个方案：

### 方案 A：继续在旧 HTTP 文件上修补

优点：

- 上手快
- 改一两个接口成本低

缺点：

- 结构继续恶化
- 功能越多越难维护
- WebSocket、生命周期管理、依赖注入都不自然
- 测试和拆分都越来越难

### 方案 B：迁移到 FastAPI

优点：

- 天然适合 API 工程化
- 原生支持 WebSocket
- 更容易分层
- 更容易做测试
- 更适合与 ROS2 桥接解耦

缺点：

- 一开始需要做结构设计
- 迁移过程需要分步骤实施

最终选择 FastAPI 的本质原因是：

- 不是为了“换技术栈而换”
- 而是为了“给未来功能增长提前搭好骨架”

---

## 3. 迁移的总体策略是什么

这次迁移不是“一步到位推倒重来”，而是采用了：

- 先替换入口
- 再迁移接口
- 再拆掉遗留依赖
- 最后清理文档与无用代码

这是一个非常重要的工程思路。

如果一开始就直接：

- 删旧文件
- 全量重写
- 前后端同时大改

风险会非常高：

- 功能难以快速回归
- 一旦某个环节出错，不容易定位
- 启动链路容易断
- ROS2 运行时更难排查

所以这里采用的是“分阶段渐进式迁移”。

---

## 4. 迁移为什么要分阶段做

因为这个系统同时包含三种复杂性：

### 4.1 Web 复杂性

- 接口多
- 页面要兼容
- 还要保留静态资源访问

### 4.2 ROS2 复杂性

- 需要 `rclpy`
- 有服务调用
- 有订阅/发布
- 有节点生命周期

### 4.3 业务复杂性

- 相机触发
- 模板读写
- 机器人控制
- 抓取流程
- Debug 预览

如果三种复杂性一起重构，风险太高。

所以分阶段的目的就是：

- 每一阶段只解决一类主要问题
- 每完成一层，就立刻验证
- 降低整体迁移风险

---

## 5. 迁移的完整过程

下面按时间顺序解释这次替换过程。

### 第 1 阶段：建立 FastAPI 骨架

先做的不是改业务，而是先搭框架。

新建核心入口：

- `visual_pose_estimation_python/web/app.py`
- `visual_pose_estimation_python/web/server.py`

这一阶段做了什么：

- 创建 FastAPI 应用
- 建立 `lifespan`
- 挂载静态资源
- 支持 `/`
- 支持 `/status`
- 支持 `/health`
- 支持 `/ws`

为什么先做这一层？

因为必须先让系统具备：

- 可启动
- 可访问
- 可检查状态
- 可承载后续路由

也就是说，先建“地基”，再搬业务。

### 第 2 阶段：保留旧 UI，先替换后端入口

这一阶段没有先重写前端，而是保留旧 UI 页面。

做法是：

- 根路径 `/` 由 FastAPI 提供
- 默认跳转到 `/legacy-ui/index.html`
- 原有 `web_ui` 静态页面继续可访问

为什么这样做？

因为用户明确不希望此时再引入 Vue/React 之类的新前端框架。

所以要先保证：

- 页面继续能用
- 用户不需要同时适应新前端
- 后端先稳定

这就是“后端先替换，前端先兼容”的策略。

### 第 3 阶段：先支持 WebSocket

在骨架阶段就加上了：

- `ws/manager.py`
- `/ws`

为什么不是以后再加？

因为当前项目未来非常明确需要：

- 长任务进度
- 状态流
- 日志流
- 实时消息

如果现在不把 WebSocket 作为架构一部分，后面再加时会打断已有结构。

所以这里提前把实时通信能力放进骨架里。

### 第 4 阶段：引入 ROS2 bridge 生命周期管理

FastAPI 只是 Web 层，真正业务还依赖 ROS2。

因此引入：

- `ros_bridge/manager.py`

它的作用是：

- 在 FastAPI 启动时启动 ROS2 bridge
- 在 FastAPI 关闭时停止 ROS2 bridge
- 后台线程持续 `spin_once`

为什么不能让每个接口自己创建 ROS2Node？

因为那样会导致：

- `rclpy.init()` 重复调用
- 节点生命周期混乱
- 线程/资源管理失控

所以必须把 ROS2 生命周期集中管理。

### 第 5 阶段：先把旧逻辑“接进来”，而不是立即全部重写

迁移初期，采用过一个过渡方案：

- 用适配层把 FastAPI 请求转发到旧逻辑

这样做的原因不是因为这种方式更优，而是因为：

- 它能让新入口先跑起来
- 它能降低第一次切换风险
- 它能让迁移从“全量重写”变成“渐进替换”

这一阶段的核心价值是：

- 让 FastAPI 先成为唯一入口
- 但底层旧逻辑还能暂时复用

这是典型的“适配器过渡方案”。

### 第 6 阶段：逐个接口原生迁移

接下来，不再依赖旧 HTTP 分发，而是把接口按业务域逐步迁成 FastAPI 原生实现。

迁移顺序大致包括：

- `camera`
- `pose`
- `templates`
- `robot`
- `grasp`
- `debug`

这些接口最终都汇总到：

- `services/native_api.py`

为什么要按业务域拆？

因为这样更符合长期维护方式：

- 相机相关放一起
- 模板相关放一起
- 机器人相关放一起
- Debug 相关放一起

这样后面新增接口时不会再回到“大文件堆逻辑”的状态。

### 第 7 阶段：删除未实现功能

迁移过程中，明确删除了本来就未实现、但前端还残留调用的内容。

例如：

- 未实现的可视化接口
- 未实现的 debug 单步处理接口
- PLC 路由

为什么不保留这些“空壳”？

因为空壳会造成三种坏处：

- 前端误以为接口存在
- 后端保留死代码
- 未来阅读代码的人很难分清“哪些是真的、哪些只是遗留”

迁移不仅是“搬家”，还包括“清垃圾”。

### 第 8 阶段：删除 legacy 转发层

当大部分接口都已经原生化之后，就可以删除：

- `LegacyHttpBridgeService`

为什么这一步重要？

因为如果还留着 legacy 转发层：

- 会让系统长期处于“双轨”
- 新旧逻辑同时存在，认知负担更高
- 后续开发时容易误入旧链路

删除它的意义是：

- FastAPI 不再只是“壳”
- 而是真正成为原生业务入口

### 第 9 阶段：把旧工具能力迁到新模块

并不是只有接口要迁移，旧文件里的公共工具能力也要搬出来。

因此新增了：

- `runtime_support.py`
- `params_manager.py`

迁移内容包括：

- 模板根目录解析
- app_config 读取
- 固定拍照姿态配置
- 姿态归一化
- rembg 运行时支持
- debug 参数管理

为什么这一步重要？

因为如果业务接口虽然迁了，但底层公共能力还一直依赖旧大文件，那么“表面迁移了，实际上没脱钩”。

### 第 10 阶段：把 ROS2Node 独立出来

这是最关键的一步之一。

最终新增了：

- `ros_bridge/node_runtime.py`

把真正的 `ROS2Node` 从旧大文件里独立出来。

这里承接了：

- 相机触发
- 图像订阅
- 姿态估计服务调用
- 模板标准化服务调用
- 机器人位姿控制
- IO 控制
- 单次抓取
- 循环抓取控制
- 夹爪切换

为什么这一步特别重要？

因为只要 `RosBridgeManager` 还依赖旧 `http_bridge_server.py`，就说明“核心脏依赖还没断干净”。

把 `ROS2Node` 抽出来之后，FastAPI 才真正做到：

- 启动层独立
- 运行层独立
- 服务层独立

### 第 11 阶段：删除旧源码

最后删除了：

- `web_ui/scripts/http_bridge_server.py`
- `web_ui/scripts/params_manager.py`

删除这一步的意义在于：

- 防止未来误用旧入口
- 防止“双实现”继续共存
- 明确新结构才是唯一真相

如果不删，团队成员以后很容易：

- 继续改旧文件
- 看错入口
- 产生重复维护

### 第 12 阶段：更新启动脚本与文档

最后更新了：

- `start_IVG_graspnet_points_fastapi.sh`
- `web_ui/start_web_ui.sh`
- `web_ui/stop_web_ui.sh`
- `web_ui/check_installation.sh`
- 相关 README 与文档

为什么最后才做？

因为文档和脚本是“最终对外描述”。

只有代码结构稳定后，文档和脚本才能写得准。

---

## 6. 当前迁移后的最终文件架构

核心后端结构如下：

### 6.1 FastAPI 主结构

- `visual_pose_estimation_python/web/server.py`
  - Web 启动入口
- `visual_pose_estimation_python/web/app.py`
  - 应用装配
- `visual_pose_estimation_python/web/dependencies.py`
  - 依赖注入

### 6.2 路由层

- `visual_pose_estimation_python/web/routers/system.py`
- `visual_pose_estimation_python/web/routers/camera.py`
- `visual_pose_estimation_python/web/routers/pose.py`
- `visual_pose_estimation_python/web/routers/templates.py`
- `visual_pose_estimation_python/web/routers/robot.py`
- `visual_pose_estimation_python/web/routers/grasp.py`
- `visual_pose_estimation_python/web/routers/debug.py`

### 6.3 业务服务层

- `visual_pose_estimation_python/web/services/native_api.py`

### 6.4 ROS2 bridge 层

- `visual_pose_estimation_python/web/ros_bridge/manager.py`
- `visual_pose_estimation_python/web/ros_bridge/node_runtime.py`

### 6.5 Web 公共支持层

- `visual_pose_estimation_python/web/resources.py`
- `visual_pose_estimation_python/web/runtime_support.py`
- `visual_pose_estimation_python/web/params_manager.py`
- `visual_pose_estimation_python/web/ws/manager.py`

### 6.6 兼容 UI 资源层

- `web_ui/index.html`
- `web_ui/static/index.html`
- `web_ui/assets/*`
- `web_ui/scripts/app.js`
- `web_ui/configs/*`

注意：

- 旧 UI 资源还保留
- 但旧 Python 后端源码已经移除

---

## 7. 当前请求处理逻辑分析

现在一条请求的真实路径是：

1. 浏览器发请求
2. FastAPI 路由接收请求
3. 路由通过 `Depends(...)` 拿到 `NativeWebService`
4. `NativeWebService` 调用 `RosBridgeManager.node`
5. `ROS2Node` 调用 ROS2 服务或处理话题
6. 结果返回到 FastAPI
7. FastAPI 返回 JSON 给前端

这条链相比旧方案的最大变化是：

- HTTP 分发和业务逻辑分开了
- Web 层和 ROS2 层分开了
- 公共配置和路径逻辑分开了

---

## 8. 为什么要这样迁移，而不是其他迁法

这次迁移的核心原则是：

- 先稳定入口
- 再逐层替换
- 每一步都能回归验证

为什么这样合理？

### 8.1 风险低

如果某一步出问题，只影响局部层，不至于整套系统全崩。

### 8.2 便于验证

每迁完一块，就能跑测试、看启动链、看实际页面。

### 8.3 更容易让旧系统平滑退出

旧 UI 先保留，后端先替换，可以减少使用上的断层。

### 8.4 更适合真实工程

现实项目里，最怕的不是“技术不先进”，而是“切换一次把整套系统弄坏”。

这次迁移选择的是工程上更稳妥的路径。

---

## 9. 这次迁移带来的好处

### 9.1 结构更清晰

现在看到文件就大概知道职责：

- `routers/`：路由
- `services/`：业务
- `ros_bridge/`：ROS2 交互
- `ws/`：WebSocket

### 9.2 更适合继续扩展

以后新增：

- 新模型接口
- 批处理任务
- 训练任务状态
- 日志流
- 管理接口

都能按模块自然添加。

### 9.3 更容易写测试

现在已经能对 FastAPI 路由做隔离测试。

### 9.4 更容易定位问题

出问题时可以先判断在哪一层：

- 路由层
- 服务层
- ROS2 bridge 层
- ROS2 算法层

不像以前那样都塞在一个大文件里。

### 9.5 为未来的 AI/长任务功能预留空间

WebSocket、lifespan、bridge 分层，这些都不是“现在可有可无”的装饰，而是未来扩展的基础设施。

---

## 10. 当前启动链分析

当前推荐启动方式：

- `ros2 launch visual_pose_estimation_python visual_pose_estimation_web.launch.py`
- 或 `ros2 run visual_pose_estimation_python visual_pose_estimation_web`

大型整体启动脚本：

- `/home/mu/IVG2.0/aubo_ros2_ws/start_IVG_graspnet_points_fastapi.sh`

它的作用是把整套系统按顺序拉起：

- 机械臂驱动
- 相机
- 相机控制
- 图像桥接
- 手眼标定
- 视觉姿态估计节点
- 抓取相关 worker
- FastAPI Web 服务

为什么启动脚本也必须更新？

因为如果代码已经迁了，但启动脚本还指向旧入口，那么实际使用时还是会跑旧链路，迁移就没有真正完成。

---

## 11. 迁移过程中删掉了什么，为什么删

### 已删内容

- 旧 HTTP bridge 源码
- 旧 ParamsManager 源码
- PLC 路由
- 未实现的前端调用
- legacy 转发层
- 一批围绕旧桥接的过时专题文档

### 为什么必须删

工程迁移里，“删”不是破坏，而是明确边界。

如果不删：

- 新人不知道哪个入口是真的
- 旧链路可能被误启动
- 文档和代码互相打架
- 维护成本持续上涨

---

## 12. 对初学者最重要的理解

如果你只记住一句话，请记住这一句：

这次迁移的真正目标，不是“把 `http.server` 改成 FastAPI”，而是：

把原来耦合在一起的 Web、业务、ROS2、配置、实时通信，拆成可以长期扩展的工程结构。

所以它是一场“架构迁移”，不是简单的“接口重写”。

---

## 13. 后续还能怎么继续演进

在当前结构上，后续很自然可以继续做：

- 新增模型推理路由
- 增加任务队列
- 增加实时日志推送
- 增加鉴权
- 增加任务记录和历史结果管理
- 在保留当前后端的前提下，未来再逐步替换前端

这就是这次迁移最核心的价值：

- 当前功能能跑
- 未来增长不乱

---

## 14. 总结

完整迁移过程可以总结为：

1. 先搭 FastAPI 骨架
2. 保留旧 UI 做兼容
3. 提前纳入 WebSocket
4. 建立 ROS2 bridge 生命周期
5. 先适配旧逻辑过渡
6. 再逐个业务域原生迁移
7. 删除未实现和无价值遗留
8. 拆出公共运行时支持
9. 抽出独立 `ROS2Node`
10. 删除旧源码
11. 更新启动脚本与文档

这样迁移的好处是：

- 稳
- 清晰
- 可验证
- 可扩展
- 适合未来继续承载机器人 + 视觉 + AI 的综合系统

## 延伸阅读

- `FASTAPI_BEGINNER_GUIDE.md`
  - 从 0 理解 FastAPI 基础概念
- `FASTAPI_TESTING_GUIDE.md`
  - 从 0 理解如何为这套 FastAPI 架构编写测试验证代码
