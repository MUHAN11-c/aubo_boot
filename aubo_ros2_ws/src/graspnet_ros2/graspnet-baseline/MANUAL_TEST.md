# MoveItPy 手动测试说明

本目录提供了可直接执行的手动测试脚本，用于测试：

- `grasp_motion_controller_moveit2.py` 的 `get_current_ee_pose`
- `move_to_joints`
- `move_to_pose`

## 前置条件

1. 启动仿真和 MoveIt（建议用你现有 launch）  
2. `enable_graspnet_node:=false`（仅测机械臂链路）

## 脚本位置

- 主测试脚本：`manual_moveitpy_test.py`
- 一键启动脚本：`run_manual_moveitpy_test.sh`

## 命令示例

### 1) 关节测试（推荐）

```bash
cd /home/wjz/aubo_boot/aubo_ros2_ws/src/graspnet_ros2/graspnet-baseline
chmod +x run_manual_moveitpy_test.sh
./run_manual_moveitpy_test.sh joints
```

自定义偏移关节和角度：

```bash
./run_manual_moveitpy_test.sh joints --joint-index 2 --joint-delta 0.04
```

### 2) 位姿测试

```bash
./run_manual_moveitpy_test.sh pose --pose-dz 0.005
```

如果你当前规划参考系为 `world`，可显式指定：

```bash
./run_manual_moveitpy_test.sh pose --base-frame world --pose-dz 0.005
```

## 结果判定

- 终端输出：
  - `... result=True` 表示成功
  - `... result=False` 表示规划/执行失败
- 脚本退出码：
  - `0` 成功
  - `1` 规划/执行失败
  - `2` 环境或输入问题（如拿不到 `/joint_states`）

