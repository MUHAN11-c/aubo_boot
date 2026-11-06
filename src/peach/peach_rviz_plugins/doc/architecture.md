# peach_rviz_plugins 架构

本包是 RViz 进程内的共享库，没有节点、launch 或后台服务。`src/image_display.cpp` 仅派生官方 ImageDisplay，重写 unsubscribe，并在派生析构时调用它；图像处理和绘制沿用系统 RViz。

清理顺序为：订阅仍在时 disconnect → 将 Connection 置空 → reset SubscriberFilter。置空用于释放捕获 Signal1 裸地址的旧断开回调，使官方基类析构重复清理时只面对空连接。不能把这个兼容修正解释成相机、DDS 或全部 RViz 退出问题的通用修复。

`plugins.xml` 导出 `peach_rviz_plugins/Image` 到 `rviz_common::Display`，CMake 用 pluginlib 注册描述文件并安装共享库，package.xml 声明 message_filters/pluginlib/RViz 依赖。没有额外头文件或算法配置；测试只注册官方 ament lint。

当前 `peach_bringup/config/harvest.rviz` 和 harvest_app.rviz 都引用此类，bringup/package.xml 也声明 exec_depend。因此它虽为临时兼容包，仍是现行应用依赖，不可直接归档。恢复官方插件的条件和历史定位见 [README](../README.md)。
