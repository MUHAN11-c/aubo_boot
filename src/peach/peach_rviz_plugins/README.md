# peach_rviz_plugins

ROS 2 Jazzy RViz 的临时兼容插件。`peach_rviz_plugins/Image` 继承官方
`rviz_default_plugins::displays::ImageDisplay`，只修正订阅的重复清理；图像处理、
图像传输、QoS、属性和绘制全部沿用官方实现。不修改系统 ROS 库，也不运行节点。

源码与注册关系见 [架构](doc/architecture.md)，插件入口和配置边界见 [接口](doc/io.md)。

## 适用范围与原因

本机 RViz 14.1.24、message_filters 4.11.17 的 `unsubscribe()` 会断开连接后销毁
SubscriberFilter，但 Connection 仍保存捕获 Signal1 裸地址的断开函数。图像窗口隐藏后
先执行 onDisable，再析构显示器时可能访问已经销毁的 Signal1 mutex。
2026-10-27 的历史 core 已定位到 ImageDisplay 析构中的 removeCallback/system_error。

本插件仅在订阅仍存在时断开连接，然后先清空 Connection，再销毁订阅。
派生析构函数主动执行同一清理，保证官方基类析构再次清理时连接已经为空。
这也覆盖官方 onDisable 使用基类限定调用、提前销毁订阅的情况。

Jazzy 已包含上游 [#1742](https://github.com/ros2/rviz/pull/1742) 的 backport #1745；
该变更解决重复回调，没有使本机的重复断开幂等。上游修复该生命周期问题、发行到部署
版本并通过相同图像窗口关闭与整栈长测后，应删除本包，并恢复官方 Image 插件类名。

## 构建与使用

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select peach_rviz_plugins
source install/setup.bash
colcon test --packages-select peach_rviz_plugins
colcon test-result --test-result-base build/peach_rviz_plugins
```

RViz 配置中的图像显示使用 `Class: peach_rviz_plugins/Image`，其他字段保持原样。
使用该配置的启动包应声明本包的 exec_depend。无额外参数或话题接线。

仅注册官方 ament lint。源码生命周期探针和历史 core 属于 runs 下的过程证据；
RViz 实际加载、图像流和整栈关闭仍须独立 mock 验收，编译/lint 不代表长测通过。

设计依据：[Jazzy pluginlib 官方教程](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Tutorials/Beginner-Client-Libraries/Pluginlib.rst)
及本机 RViz/ImageTransportDisplay、message_filters/Connection 公有 API。
