# peach_system_tests 架构

## 职责

测试资产包：纯核/基线、隔离 DDS 域的 mock 集成、冻结相机回放及人工复核驱动。

## 源码地图

`CMakeLists.txt`：实际 CTest 注册与域；`test/`：pytest/launch_testing 和基线；`scripts/`：有限时驱动、探针、场景查询和冻结回放；`launch/`、`config/`：回放组合与专用 overlay。

## 入口、输入输出与参数

不进入产品 launch。完整命令与判据见 [testing.md](testing.md)；安装脚本用 ros2 run peach_system_tests，参数先看对应 --help。冻结 FULL 需要 bag/GPU，不属于默认无设备通过证据。

## 生命周期、并发与失败

普通 pytest 不启动默认 DDS 域；ROS 测试使用 CMake 指定隔离域。测试拥有的进程必须 finally/post_shutdown 收口，仍须核对残留。runner 非零、skip、无匹配测试及 action 结果未知均分别记录；不能把进程退出0直接记为采摘成功。

## 验证与限制

`colcon test --packages-select peach_system_tests --return-code-on-test-failure`，随后核对 `colcon test-result --verbose` 中失败、错误、skip及测试数量。系统接线与现场验收见 [测试指导](../../peach_system_tests/doc/testing.md)，系统组合见 [bringup架构](../../peach_bringup/doc/architecture.md)。测试结束清本轮进程；真机运动或SetIO需另授权，驱动只读，numpy固定1.26.4。包测试通过不证明现场精度、功能安全或剪切成功。
