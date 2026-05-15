#!/usr/bin/env python3
"""
latte_imitation 文本测试脚本 — 无 GUI 环境时的备用方案喵~

用法:
  cd /home/mu/IVG2.0/aubo_ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 src/latte_imitation/scripts/test_replay_service.py

推荐使用可视化面板: latte_debug_panel.py (PySide6 3D + 欧拉角控制) 喵~
"""

import subprocess
import sys
import os as _os
import yaml as _yaml
_sys_path = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _sys_path not in sys.path:
    sys.path.insert(0, _sys_path)
from latte_imitation.trajectory_transform import euler_deg_to_quat

# 上次使用的旋转角度 (方便反复调) 喵~
_last_rpy = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

# ── 命令构建 ──────────────────────────────────────────────────

def build_yaml(ep, arm, spd, mode, col_str, px, py, pz, qx, qy, qz, qw):
    """用 yaml.dump 安全构建 YAML，避免字符串拼接注入风险喵~"""
    data = {
        "episode_idx": int(ep),
        "arm": str(arm),
        "speed_scale": float(spd),
        "mode": str(mode),
        "collision_check": bool(col_str),
        "start_pose": {
            "position": {"x": float(px), "y": float(py), "z": float(pz)},
            "orientation": {"x": float(qx), "y": float(qy), "z": float(qz), "w": float(qw)},
        },
    }
    return _yaml.dump(data, default_flow_style=True, sort_keys=False).strip().replace("\n", "")


def _cmd(ep, arm, spd, mode, col, px, py, pz, qx, qy, qz, qw):
    """构建 ros2 service call 命令喵~"""
    yaml = build_yaml(ep, arm, spd, mode, col, px, py, pz, qx, qy, qz, qw)
    return (
        "ros2 service call /latte_imitation/replay_trajectory "
        "ivg_interfaces/srv/ReplayLatteTrajectory "
        f"'{yaml}'"
    )


# ── 测试用例定义 ────────────────────────────────────────────────


# 默认参数: (ep, arm, spd, mode, col, px, py, pz, qx, qy, qz, qw)
_T = lambda *a: _cmd(*a)

TESTS = [
    {
        "id": 1,
        "name": "Debug — 预览轨迹 (当前 EE, 不旋转)",
        "desc": "position=零→TF查EE / orientation=identity→纯平移, 只发布话题",
        "cmd": _T("0", "right", "1.0", "debug", "false",
                  "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0"),
        "expect": "success=true, num_frames=400, path_length~1.53",
    },
    {
        "id": 2,
        "name": "Action — 完整管线 (当前 EE, 不旋转, 2倍速)",
        "desc": "computeCartesianPath→execute, 轨迹保持原始朝向",
        "cmd": _T("0", "right", "2.0", "action", "true",
                  "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0"),
        "expect": "success=true, ik_success_count≥380, collision_count=0",
    },
    {
        "id": 3,
        "name": "旋转 Z+90° (位置=当前EE, yaw=90°)",
        "desc": "position=零→TF / orientation=euler(0,0,90°), 轨迹绕Z轴旋转",
        "cmd": _T("0", "right", "1.0", "action", "true",
                  "0.0", "0.0", "0.0", "0.0", "0.0", "0.707", "0.707"),
        "expect": "success=true, 轨迹绕EE位置旋转90°, 运动方向改变",
    },
    {
        "id": 4,
        "name": "旋转 Z+180° (位置=当前EE, yaw=180°)",
        "desc": "position=零→TF / orientation=euler(0,0,180°), 轨迹完全翻转",
        "cmd": _T("0", "right", "1.0", "action", "true",
                  "0.0", "0.0", "0.0", "0.0", "0.0", "1.0", "0.0"),
        "expect": "success=true, 轨迹绕EE位置翻转, X/Y方向反转",
    },
    {
        "id": 5,
        "name": "指定位置+旋转 (杯子位姿)",
        "desc": "position=(0.45,0,0.50)+yaw=90°, 杯子位姿全指定",
        "cmd": _T("0", "right", "1.0", "action", "true",
                  "0.45", "0.0", "0.50", "0.0", "0.0", "0.707", "0.707"),
        "expect": "success=true, 起点=(0.45,0,0.50), 路径长度不变",
    },
    {
        "id": 6,
        "name": "错误处理 — 不存在 episode",
        "desc": "请求 episode=999, 验证错误信息",
        "cmd": _T("999", "right", "1.0", "debug", "false",
                  "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0"),
        "expect": 'success=false, message="episode_000999.npz ... 未找到"',
    },
    {
        "id": 7,
        "name": "左臂持杯轨迹",
        "desc": "arm='left', 几乎静止, 路径~0.31m (右臂的1/5)",
        "cmd": _T("0", "left", "1.0", "debug", "false",
                  "0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "1.0"),
        "expect": "success=true, num_frames=400, path_length~0.31",
    },
    {
        "id": 8,
        "name": "自定义 — 欧拉角控制旋转",
        "desc": "逐项输入 episode/arm/speed/mode/pos/(roll,pitch,yaw)°, 记住上次值",
        "interactive": True,
    },
]

# ── 菜单渲染 ────────────────────────────────────────────────────

def print_menu():
    print()
    print("=" * 62)
    print("  latte_imitation 测试菜单")
    print("=" * 62)
    for t in TESTS:
        print(f"  [{t['id']}] {t['name']}")
        print(f"      {t['desc']}")
    print(f"  [0] 退出")
    print("-" * 62)
    print(f"  预期结果速查: 每条测试后附预期返回值喵~")
    print("=" * 62)


def run_cmd(cmd: str) -> int:
    """执行 ros2 service call 命令并打印输出喵~"""
    print()
    print(f"\033[1;34m>>> {cmd[:80]}...\033[0m")
    print()
    result = subprocess.run(cmd, shell=True, capture_output=False)
    return result.returncode


def run_interactive():
    """自定义参数 — 欧拉角控制旋转, 记住上次值方便反复调喵~"""
    global _last_rpy
    try:
        ep   = input(f"  episode_idx (0~39, 默认 0): ").strip() or "0"
        arm  = input(f"  arm (right/left, 默认 right): ").strip() or "right"
        spd  = input(f"  speed_scale (默认 1.0): ").strip() or "1.0"
        mode = input(f"  mode (debug/action, 默认 action): ").strip() or "action"
        print("  collision_check [已废弃, 跳过]")
        col_str = "true"
        px = input(f"  position.x (0=自动TF, 默认 0): ").strip() or "0.0"
        py = input(f"  position.y (0=自动TF, 默认 0): ").strip() or "0.0"
        pz = input(f"  position.z (0=自动TF, 默认 0): ").strip() or "0.0"
        print("  --- 欧拉角旋转 (度) 上次: roll={:.0f} pitch={:.0f} yaw={:.0f} ---".format(
            _last_rpy["roll"], _last_rpy["pitch"], _last_rpy["yaw"]))
        r = input(f"  roll  / X轴 (默认 {_last_rpy['roll']:.0f}): ").strip()
        p = input(f"  pitch / Y轴 (默认 {_last_rpy['pitch']:.0f}): ").strip()
        y = input(f"  yaw   / Z轴 (默认 {_last_rpy['yaw']:.0f}): ").strip()
        roll  = float(r) if r else _last_rpy["roll"]
        pitch = float(p) if p else _last_rpy["pitch"]
        yaw   = float(y) if y else _last_rpy["yaw"]
        _last_rpy = {"roll": roll, "pitch": pitch, "yaw": yaw}

        q = euler_deg_to_quat(roll, pitch, yaw)
        print(f"  → 四元数 (xyzw): [{q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f}]")

        yaml = build_yaml(ep, arm, spd, mode, col_str, px, py, pz, q[0], q[1], q[2], q[3])
        cmd = (
            "ros2 service call /latte_imitation/replay_trajectory "
            "ivg_interfaces/srv/ReplayLatteTrajectory "
            f"'{yaml}'"
        )
        run_cmd(cmd)
    except (KeyboardInterrupt, EOFError):
        print()
    except ValueError as e:
        print(f"\033[1;31m输入错误: {e}\033[0m")


# ── 主循环 ──────────────────────────────────────────────────────

def main():
    print("\033[1;32mlatte_imitation 交互式测试工具\033[0m")
    print("请确保仿真(终端1)和服务节点(终端2)已就绪")

    while True:
        print_menu()
        try:
            choice = input("\n输入编号 (0~8): ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n退出喵~")
            break

        if choice == "0":
            print("退出喵~")
            break

        if choice == "8":
            run_interactive()
            continue

        try:
            idx = int(choice)
            test = next((t for t in TESTS if t["id"] == idx), None)
        except ValueError:
            print("\033[1;31m无效输入, 请输入数字 0~8\033[0m")
            continue

        if test is None:
            print("\033[1;31m未找到该测试\033[0m")
            continue

        print(f"\n\033[1;33m执行: {test['name']}\033[0m")
        print(f"\033[0;36m预期: {test['expect']}\033[0m")

        ret = run_cmd(test["cmd"])
        if ret != 0:
            print(f"\033[1;31m命令执行失败 (exit={ret})\033[0m")


if __name__ == "__main__":
    main()
