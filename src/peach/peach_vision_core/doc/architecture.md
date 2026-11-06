# peach_vision_core 架构

## 职责与源码地图

这是可被感知、重建等包导入的共享算法库，setup.py 没有 console script，包内没有 launch、节点、话题或服务。算法模块不导入 ROS；`clock_adapter.py` 通过鸭子类型接受调用方的 rclpy 时钟。

- `geometry.py`：向量、法线、球/圆柱 RANSAC 与精修、深度单位/内参/反投影、投影、坐标变换、抓取坐标系、四元数、点云与 RGB 打包。
- `bag_landmarks.py`：袋轴、截面、宽底/细颈与遮挡事实；`condition_bag_landmarks` 提供当前曝光退化轴的连续参考，调用方仍须保留原资格下限。
- `tool_budget.py`：逐目标径向/轴向预算、诊断原因及当前 sleeve/cut 布尔值；`domain/budget.py` 将其投影成能力三态。能力与数值余量的关系见 [io.md](io.md)。
- `domain/model_contract.py`：模型执行身份相等/完整性、有效窗与 allowed 汇总；`domain/evidence.py`：精确 TF 身份门、遮挡及走廊证据分类；`domain/__init__.py` 为领域子包标识。
- `runtime.py`：Clock/ManualClock、有界串行 worker、runs 路径、HarvestDataStore 异步落盘和 ScalarEma；`clock_adapter.py` 把 `.now().nanoseconds` 转为 float 秒，不改变被注入时钟的语义。
- `__init__.py` 为包标识；setup.py、setup.cfg、resource 和 package.xml 定义 Python 安装、ament 索引及 numpy/scipy/OpenCV/YAML/peach_common 依赖。`test/` 仅有 ament flake8/pep257 包装。

## 状态与边界

几何、袋关键点、证据门和预算函数只处理传入数据。RANSAC 的有限 LRU 仅缓存确定性索引，不缓存点云、拟合或资格，细节见 [README](../README.md)。

`BoundedWorker` 和 HarvestDataStore 会创建线程；算法库零 ROS 不表示零状态或零 I/O。worker 的满队列策略由调用方选择，close 会等待当前任务结束，不能抢占数值计算。HarvestDataStore 保存 manifest、events、mask/color/debug 图，不写 supervisor 的 ledger.json；文件失败经 stdlib logger 交给节点日志桥接。

Clock 仅表示所注入的秒值时间基准。RclpyClockAdapter 可以包装系统/ROS/仿真时钟，不能保证主机单调性；ManualClock 单独禁止负 advance。会话、曝光和耗时统计的时钟必须由调用方明确区分。

## 验证

参数和现场行为属于调用方包。维护时核对调用方的单位、缺省值与错误映射，并按 [测试指导](../../peach_system_tests/doc/testing.md) 做冻结输入/回放或 mock 证据；本包 lint 不证明几何精度、模型可执行性或现场成功。
