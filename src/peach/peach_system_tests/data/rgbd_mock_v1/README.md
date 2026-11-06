# peach-rgbd-mock-v1

一组真实 PS800 RGB-D 固定机位数据，来自本项目 2026-10-28 的相机＋mock 会话。
它选取原录包 RGB 索引14；本精简包使用索引0。数据为本项目用户授权随仓发布的采集结果，使用时保留本说明、仓库许可及来源权益。

## 内容与完整性

- `bag/`：ROS 2 Jazzy MCAP；RGB、registered depth、两路 CameraInfo、registered PointCloud2 各一条。
- `provenance.json`：原录包与文件 SHA-256、原索引、尺寸、内参、frame_id、曝光/录制时间、消息 CDR SHA-256。
- `SHA256SUMS`：交付数据校验和。运行 `sha256sum -c SHA256SUMS`。

所有消息均直接复制原始 CDR 字节和录制时间；未改图像、深度、点云、内参或曝光时间戳。
原数据中三路曝光 stamp 相同；CameraInfo 是静态标定，不能作为独立曝光同步证据。
冻结发布器把整个曝光组平移至实时时间，保留组内偏差。

## 使用

从仓库根完成[部署](../../../../../docs/deployment.md)后：

```bash
source scripts/setup_env.bash
ros2 run peach_system_tests run_frozen_grasp.py --rviz
```

脚本自动查找安装后的本数据；也可显式传 `--bag .../rgbd_mock_v1/bag --index 0`。
保持真实机械臂与工具 IO 关闭；本入口只启动 GenericSystem mock。
Web http://127.0.0.1:8090/；RViz 由 `--rviz` 开启。

## 判定与局限

本样例用于固定拍照位的感知、单视重建、调度、两段去程、套入和撤退软件验证。
`tool=false`，目标为单果 L6；驱动在第一颗终局后主动取消批次，不能把 action CANCELLED 写成整批自然成功。
没有动态相机、多视覆盖、真实力或剪切真值；测试 overlay 的宽松单视预算不得移植到 real。
复测结果见仓库 `docs/releases/2026-11-06.md`。
