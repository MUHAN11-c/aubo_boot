# aubo_hand_eye_calibration 架构

本包提供独立手眼标定与彩色内参落盘工具，不进入采摘 FSM，也不启动驱动或相机。
默认安装入口由 `setup.py` 声明：`calibration_server`、`extrinsics_publisher`、
`web_gateway`、`apply_intrinsics`。操作说明见 [README](../README.md)，端点见 [io.md](io.md)。

## 源码与流程

- `server.py`：普通 ROS Node，四线程 executor；订阅图像/CameraInfo/关节，
  调用 MoveGroup 的 `/move_action`，提供 `~/run` 和 `~/activate`。
- `board.py`、`detector.py`：棋盘格几何、角点与 PnP；`stats.py`：MAD 筛选。
- `solver.py`：OpenCV 五种 hand-eye 方法评分、异常样本筛选及 Huber 精化；
  `transforms.py`：变换与 SE(3) 运算。
- `viewpoints.py`：按实时 K/D 和图像尺寸生成视点、检查投影边界与旋转跨度。
  只约束候选视点的板可见性，路径仍由 MoveIt 规划。
- `joint_calib.py`：K/D 初始化与内外参联合优化；只优化 k1/k2/p1/p2，k3 固定为零。
- `storage.py`：候选、质量门激活与原子 YAML；`intrinsics.py`：读取 OST 或联合
  内参节、校验并写入项目内参文件。
- `extrinsics_publisher.py`：单独发布腕到相机根的静态 TF；
  `web_gateway.py` / `web_goal.py`：回环 HTTP 调试客户端及枚举校验。
  `defaults.py` 只共享默认帧名。

`hand_eye_calibration.launch.py` 默认起标定服务、外参发布器和 Web；launch 不发 goal。
`intrinsics_calibration.launch.py` 调用外部 apt `camera_calibration` 的交互 GUI，
板参数从本包 YAML 读取。driver bringup 的 `hand_eye_enabled` 默认关闭，与独立
launch 的默认组成不同；不要因为启了外参发布器就认为正在自动标定。

## 采集、求解与激活

默认 poses 文件含 17 个腕部位姿。auto 模式用已激活的腕到光学系矩阵定位固定板，
生成冗余视点并 plan-only 预检。plan_only goal 到计划结果即结束；实际采集
使用 MoveGroup 执行，再等待静止与足量有效观测，求解后写 candidate，不自动激活。

图像处理保存 monotonic 到达处理时刻和检测结果，采集器为每条结果读取 latest
腕部 TF；不保留图像 stamp 做精确 TF 查询。当前依靠到位静止降低时间差影响，
不能称为曝光时刻同步配对，也不能从采摘重建的精确时间查询规则推导本包行为。

候选包含 `quality_passed`、frames、三组变换、样本与求解明细；只有通过质量门的
候选可由激活服务写 active。磁盘替换后才请求外参 reload，两步不构成事务：
reload 失败时磁盘可能已变、当前 TF 仍旧，须分别核对服务消息与 TF。

存储默认优先 `AUBO_HAND_EYE_DIR`，再查源码 `hand_eye/`，无源码树时用
`~/.ros/aubo_e5/hand_eye`。内参工具则默认查源码 percipio 配置，找不到时要求
`--output`；不会静默写安装树。YAML 的写入不修改设备 Flash 或重新求解深度配准。

## 验证边界

本包 `test/` 仅有官方 flake8/pep257；静态检查不证明标定收敛。
采集会调用独立 MoveGroup 执行路径，运行前须有对应的操作员授权，不能将采摘
操作台的执行开关当成本包授权。真实几何质量按本轮样本、残差、跨度和现场复核判定。
