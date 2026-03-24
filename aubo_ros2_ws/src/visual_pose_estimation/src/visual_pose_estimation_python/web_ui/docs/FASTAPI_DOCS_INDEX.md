# FastAPI 文档总览与学习路线

这份文档是当前 FastAPI 学习资料的总入口。

如果你第一次接触这套 Web 架构，不知道先看哪一份文档，就从这里开始。

---

## 1. 文档总览

当前文档体系如下：

- `FASTAPI_WEB.md`
  - 当前 FastAPI Web 服务的总说明
  - 包含启动方式、访问地址、目录职责

- `FASTAPI_BEGINNER_GUIDE.md`
  - 零基础理解 FastAPI
  - 重点解释 FastAPI 在本项目中的角色、层次划分和请求流向

- `FASTAPI_MIGRATION_GUIDE.md`
  - 零基础理解这次迁移替换全过程
  - 重点解释为什么迁移、怎么迁移、为什么这样迁移、迁移后有什么收益

- `FASTAPI_TESTING_GUIDE.md`
  - 零基础理解如何编写测试验证代码
  - 重点解释 Dummy、TestClient、patch、临时目录、副作用验证

- `FASTAPI_EXTENSION_GUIDE.md`
  - 后续开发实战指南
  - 重点解释如何继续扩展、如何新增接口、如何接 ROS2、如何避免重新长回单体结构

- `FASTAPI_INTERFACE_TEMPLATE.md`
  - 新增接口模板文档
  - 可以直接复制模板去扩展功能

- `FASTAPI_ARCHITECTURE_DIAGRAMS.md`
  - 架构图、调用时序图、测试分层图、迁移前后对比图

---

## 2. 推荐学习顺序

### 路线 A：零基础学习路线

适合：

- 第一次接触 FastAPI
- 第一次接触本项目 Web 架构

推荐顺序：

1. `FASTAPI_WEB.md`
2. `FASTAPI_BEGINNER_GUIDE.md`
3. `FASTAPI_ARCHITECTURE_DIAGRAMS.md`
4. `FASTAPI_MIGRATION_GUIDE.md`
5. `FASTAPI_TESTING_GUIDE.md`
6. `FASTAPI_EXTENSION_GUIDE.md`
7. `FASTAPI_INTERFACE_TEMPLATE.md`

### 路线 B：只想理解迁移过程

适合：

- 想复盘为什么从旧 HTTP 方案迁到 FastAPI
- 想看迁移全过程和设计原因

推荐顺序：

1. `FASTAPI_WEB.md`
2. `FASTAPI_MIGRATION_GUIDE.md`
3. `FASTAPI_ARCHITECTURE_DIAGRAMS.md`

### 路线 C：只想学怎么写测试

适合：

- 想补测试
- 想理解 `test/test_web_app.py`

推荐顺序：

1. `FASTAPI_WEB.md`
2. `FASTAPI_TESTING_GUIDE.md`
3. `FASTAPI_INTERFACE_TEMPLATE.md`

### 路线 D：准备继续开发新功能

适合：

- 你接下来要新增接口
- 要接新的 ROS2 能力
- 要继续扩展当前后端

推荐顺序：

1. `FASTAPI_WEB.md`
2. `FASTAPI_EXTENSION_GUIDE.md`
3. `FASTAPI_INTERFACE_TEMPLATE.md`
4. `FASTAPI_ARCHITECTURE_DIAGRAMS.md`
5. `FASTAPI_TESTING_GUIDE.md`

---

## 3. 不同角色该看什么

### 如果你是“使用者/联调者”

优先看：

- `FASTAPI_WEB.md`
- `web_ui/README.md`

### 如果你是“新接手项目的开发者”

优先看：

- `FASTAPI_BEGINNER_GUIDE.md`
- `FASTAPI_ARCHITECTURE_DIAGRAMS.md`
- `FASTAPI_TESTING_GUIDE.md`

### 如果你是“要继续迁移/重构的人”

优先看：

- `FASTAPI_MIGRATION_GUIDE.md`
- `FASTAPI_EXTENSION_GUIDE.md`

### 如果你是“要新增功能的人”

优先看：

- `FASTAPI_EXTENSION_GUIDE.md`
- `FASTAPI_INTERFACE_TEMPLATE.md`

---

## 4. 和代码一起阅读的推荐顺序

如果你想边看代码边看文档，推荐这样配套：

1. 文档：`FASTAPI_WEB.md`
   代码：`visual_pose_estimation_python/web/server.py`

2. 文档：`FASTAPI_BEGINNER_GUIDE.md`
   代码：`visual_pose_estimation_python/web/app.py`

3. 文档：`FASTAPI_ARCHITECTURE_DIAGRAMS.md`
   代码：`visual_pose_estimation_python/web/routers/*.py`

4. 文档：`FASTAPI_EXTENSION_GUIDE.md`
   代码：`visual_pose_estimation_python/web/services/native_api.py`

5. 文档：`FASTAPI_MIGRATION_GUIDE.md`
   代码：`visual_pose_estimation_python/web/ros_bridge/manager.py`

6. 文档：`FASTAPI_TESTING_GUIDE.md`
   代码：`test/test_web_app.py`

---

## 5. 如果时间很少，只看哪三份

如果你时间很少，只建议先看这三份：

1. `FASTAPI_WEB.md`
2. `FASTAPI_BEGINNER_GUIDE.md`
3. `FASTAPI_TESTING_GUIDE.md`

原因是这三份能最快帮助你理解：

- 当前结构是什么
- 代码该去哪里看
- 改完怎么验证

---

## 6. 一句话总结

如果把这些文档看成一个体系：

- `FASTAPI_WEB.md` 负责“看地图”
- `FASTAPI_BEGINNER_GUIDE.md` 负责“学基础”
- `FASTAPI_MIGRATION_GUIDE.md` 负责“懂历史”
- `FASTAPI_TESTING_GUIDE.md` 负责“会验证”
- `FASTAPI_EXTENSION_GUIDE.md` 负责“会继续开发”
- `FASTAPI_INTERFACE_TEMPLATE.md` 负责“直接上手写”
- `FASTAPI_ARCHITECTURE_DIAGRAMS.md` 负责“把结构看清楚”
