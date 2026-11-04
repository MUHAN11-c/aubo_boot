# peach_target_reconstruction

Peach target reconstruction lifecycle node.

RGB-D 进入帧环前核对深度曝光时间、单通道布局、图像与标定尺寸以及针孔内参。
零时间戳会被拒绝，以免 tf2 将其解释为请求最新位姿。
采集和 ICP 整数下限在启动及运行时均校验。被拒绝的修改保留上一组有效参数。

## 开关语义

- `capture.auto_mode`：本节点自动采帧开关，运行期可改；false 只停自动采帧，手动 BuildTargetModel 仍可用。
- 显式 Build 复用相同的曝光时刻 TF、静止、掩膜和质量门；结束后保留该次 READY 模型，直到 reset 或换目标。它不修改自动采帧参数。
- 进程是否启动由 bringup 的 `reconstruction_enabled` 控制，不在本包参数中伪装成“关闭模块”。
- `skip_reconstruction` 属于 supervisor 作业策略，不会跨包改写本节点参数。
- 部署通过 `calibration_version` 标识当前相机内外参。未配置时仍可查看拟合结果，BuildTargetModel 的回执会提示缺少标定版本，臂侧模型身份门会拒绝执行。

cleanup 在销毁 ROS 实体前停止并等待采帧 worker。诊断 Updater 在进程内复用，cleanup 暂停定时器，再 configure 时恢复，不重复声明参数或注册任务。
直接 lifecycle shutdown 复用相同收尾，唤醒等待中的 Build、拒绝新请求；未 configure 或已 cleanup 时也可安全调用，迟到回调不会再发布到已销毁实体。
