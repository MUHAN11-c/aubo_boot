# sim_perception 接口

本包不定义 IDL，也不重命名共享能力的节点或话题。相机、目标观测、模型动作
的完整字段契约以 `peach_interfaces` 和共享能力包的 `doc/io.md` 为准。

## 输入

- `/camera/color/image_raw`、`/camera/depth/image_raw`：`sensor_msgs/Image`。
- `/camera/color/camera_info`：`sensor_msgs/CameraInfo`。
- `/joint_states`：`sensor_msgs/JointState`，固定 AUBO 六关节顺序。
- `/tf`、`/tf_static` 与 `/clock`：同一仿真图；`use_sim_time=true`。
- 感知与重建根据共享任务状态契约选择目标，组合包负责接线。

## 作业接口

- `/peach_scene_perception_node/begin_scene`：`peach_interfaces/BeginScene`。
- `/peach/perception/target_observations`：`PeachTargetObservationArray`。
- `/peach_target_reconstruction_node/build_target_model`：`BuildTargetModel` action。
- `/peach/reconstruction/grasp_decision`：`GraspDecision`，共享能力提供冻结模型身份、有效期和分阶段许可，供仿真执行边界复核。
- `/diagnostics`：共享能力提供的标准诊断。

兼容 console scripts 名称仍带 `sim_` 前缀；底层节点名保持 Peach 原名，必须在
项目运行器分配的独立 ROS domain 中运行。传感器 QoS 与 TF 时间一致性由共享
能力与相机桥共同保证，不能仅凭静态配置宣称实际端点已兼容。

## 文件输出

重建记录写 `session.root_dir`，launch 必须设为项目内 `run_directory/vision`。
本包 overlay 要求 `PEACH_SIM_RUN_DIRECTORY` 环境变量，防止落到共享包的
工作区根 `runs/` 默认目录。模型仅从既有 `peach_scene_perception/model` 读取。
