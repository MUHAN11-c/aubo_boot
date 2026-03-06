# aubo_robot_simulator_ros2

ROS2 轨迹插值节点：订阅 `joint_path_command`，5 次多项式插值后发布 `moveItController_cmd`（经 ros1_bridge 到 ROS1 aubo_driver）。

## 构建时 stderr 说明

构建时若出现：

```
PkgResourcesDeprecationWarning: 1.12.1-git20200711.33e2d80-dfsg1-0.6 is an invalid version...
```

这是**系统** `pkg_resources`（setuptools）的告警，与本包代码无关，**不影响构建和运行**，可忽略。若要减少输出，可在构建前执行：

```bash
export PYTHONWARNINGS=ignore::DeprecationWarning
colcon build --packages-select aubo_robot_simulator_ros2 --symlink-install
```

（若 colcon 子进程未继承环境，告警仍可能出现，属正常现象。）
