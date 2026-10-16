# peach_observability 架构

## 职责

只读观测聚合、8090调试桥、会话bag与报告；静态页面由顶层peach_web安装。调试请求委托现有ROS服务/动作，不能绕过臂侧门或新增SetIO。

## 源码地图

`observability_node.py`：订阅镜像/生命周期；`http_server.py`、`debug_actions.py`：HTTP与委托；`image_preview.py`：JPEG缓存；`tcp_trajectory.py`：真实TF缓存；`state.py`、`job.py`、`pipeline.py`：状态解释；`recorder.py`、`catch_all_recorder.py`、`retention.py`：录制/回收；`bag_reader.py`、`bag_report.py`：离线报告；`selfcheck/`：启动检查。

## 入口、输入输出与参数

整栈随起，独立用 observability.launch.py；离线 `ros2 run peach_observability peach_bag_report <bag>`。输入事件、模型、状态、diagnostics、TF/关节等；输出8090、作业/指标、自检和runs/session_*产物。参数源config/observability.yaml+attach；详细API见 [io.md](io.md)。

## 生命周期、并发与失败

Lifecycle节点自激活，不进系统四节点名单；双线程executor外有HTTP、编码、发现等工作线程。HTTP只读缓存图像/TF，不阻塞ROS获取新帧；每流最新槽、公平编码总计最多5Hz。cleanup/destroy停止线程、订阅、timer并封口bag。自动报告在封口后用当前Python启动子进程，继承launch进程组；默认等30秒，超时先terminate、2秒后kill并回收，保留已封口bag供离线重算，报告失败不伪装成功。accepted仅代表受理，202仍在途，网络/超时是结果未知，POST不自动重发。allowed不证明臂侧许可，level6不是level7；陈旧/身份缺失明确标记，不能推断成有效模型。

## 验证与限制

`colcon test --packages-select peach_observability --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。

## 状态投影与静态资源

原始精化诊断、绑定来源及typed/raw/decision合并集中在既有pipeline.py，节点ROS回调负责接线，避免多处解释身份。state.py 的投影只解释已有身份与诊断标记：嵌套 candidate/fitting 空身份或与外层 target_id 不符即投影为 null，BagFitting 负值与非有限标量转 null（零仍是合法比例/点数/枚举），`anchor_from_memory` 保留历史关键点但置信度、相机距离、袋径与行程置 null——不改 ROS 原消息或许可门。HTTP静态白名单包含ui-common.js、monitor-renderers.js、trajectory-renderers.js、request-runtime.js；它们由peach_web安装，与app.js协作，不新增ROS控制面。
