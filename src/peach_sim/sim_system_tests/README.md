# sim_system_tests

仿真子工作空间里留给官方 ament lint 的包。控制器是否起来，手工起 `sim_bringup` 的 launch，看 `/joint_states` 和控制器日志。

同机还有别的 Gazebo 时，给这次单独设 `GZ_PARTITION` 和 `ROS_DOMAIN_ID`，避免实体建进别人的世界。多栈并存时 FastDDS 共享内存端口会抢，可设 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。

```bash
colcon test --packages-select sim_system_tests
```

跑完停栈并核对进程。
