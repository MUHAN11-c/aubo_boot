# peach_rviz_plugins 接口

唯一导出类名为 `peach_rviz_plugins/Image`，C++ 类型 `peach_rviz_plugins::ImageDisplay`，基类 pluginlib 契约为 rviz_common::Display。RViz 配置的图像 Display 将 Class 设置为该名字即可，插件由 RViz 按 ament/pluginlib 索引加载。

该类继承官方 ImageDisplay 的图像话题、Image Transport、QoS、属性与渲染，没有新增 ROS 参数、topic、service、action 或命令入口。插件描述中的 message_type 为 sensor_msgs/msg/Image；实际输入话题由使用它的 RViz 配置决定。

构建和实际加载验证见 [README](../README.md)。依赖本类的启动包需声明 exec_depend 并 source 安装空间；找不到插件时不能以删除图像显示替代加载验收。源码工作区新增的 doc 页面用于维护，当前 CMake 只显式安装 README 与插件库/注册元数据。
