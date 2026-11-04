# peach_observability 架构

## 职责

只读观测聚合、8090调试桥、会话bag与报告；静态页面由peach_web安装。调试请求委托现有ROS服务/动作，不能绕过臂侧门或新增SetIO。

## 源码地图

`observability_node.py`：订阅镜像/生命周期；`http_server.py`、`debug_actions.py`：HTTP与委托；`image_preview.py`：JPEG缓存；`tcp_trajectory.py`：真实TF缓存；`state.py`、`job.py`、`pipeline.py`：状态解释；`recorder.py`、`catch_all_recorder.py`、`retention.py`：录制/回收；`bag_reader.py`、`bag_report.py`：离线报告；`selfcheck/`：启动检查。

## 入口、输入输出与参数

整栈随起，独立用 observability.launch.py；离线 `ros2 run peach_observability peach_bag_report <bag>`。输入事件、模型、状态、diagnostics、TF/关节等；输出8090、作业/指标、自检和runs/session_*产物。参数声明由config/observability_parameters.yaml生成ParamListener，部署值在config/observability.yaml，ObservabilityParams.attach维护运行期快照；详细API见 [io.md](io.md)。

## 生命周期、并发与失败

Lifecycle节点自激活，不进系统四节点名单；双线程executor外有HTTP、编码、发现等工作线程。HTTP只读缓存图像/TF，不阻塞ROS获取新帧；每流最新槽、公平编码总计最多5Hz。cleanup/shutdown/destroy停止HTTP、线程、订阅与配置期发布器并封口bag；远程shutdown迁移也立即收尾。诊断Updater在节点进程内只创建一次，cleanup取消其timer，重新configure时复用并恢复，避免重复声明参数和累积任务。自动报告在封口后用当前Python启动子进程，继承launch进程组；默认等30秒，超时先terminate、2秒后kill并回收，保留已封口bag供离线重算，报告失败不伪装成功。accepted仅代表受理，202仍在途，网络/超时是结果未知，POST不自动重发。allowed不证明臂侧许可，level6不是level7；陈旧/身份缺失明确标记，不能推断成有效模型。

## 验证与限制

`colcon test --packages-select peach_observability --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。

## 状态投影与静态资源

原始精化诊断、绑定来源及 typed/raw/decision 合并集中在 pipeline.py，节点 ROS 回调负责接线。state.py 按已有身份与诊断标记投影。嵌套 candidate/fitting 空身份或与外层 target_id 不符时为 null。BagFitting 负值与非有限标量转 null，合法零值保留。`anchor_from_memory` 保留历史关键点，置信度、相机距离、袋径和行程置 null。

8090 按扩展名托管 peach_web 安装的页面。SDK 和许可使用固定 URL，从 ament 索引定位的 ros2_web_link 安装目录取文件。ROS 投影按各自订户数计算，独立 HTTP 读取照常取当前快照。自检跟踪最近一次托管状态，真机要求 robot_status 时未收过一帧即失败，关闭重建进程时自检跳过 Build 动作。普通面板写参后继续沿用启动事实决定的探针范围。


记录器是会话身份所有者：原子session.json声明写者与状态，8090 力页与调度归档不再用 bag mtime 判断存活。原子/force/sample在所有录制档完整入袋；固定订阅与发现订阅不重叠写入。Recorder控制屏障不可丢，停止接收后排队封口，只有确认sealed才启动自动报告。离线保留数值数组与源ns时间；旧mtime时间推算明确标estimated，时间重叠关联不能替代目标身份。

启动事实仍投影到状态；仅启用录制时启动落盘线程。写者绑定本次Recorder，等待会话目录时可由cleanup/shutdown立即唤醒退出；确认退场后才清句柄，避免旧facts写进重配后的会话。
