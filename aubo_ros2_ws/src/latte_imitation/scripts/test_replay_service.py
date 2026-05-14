#!/usr/bin/env python3
"""
latte_imitation 交互式测试脚本 — 菜单选择, 无需手敲长命令喵~

用法:
  cd /home/mu/IVG2.0/aubo_ros2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 src/latte_imitation/scripts/test_replay_service.py
"""

import subprocess
import sys

# ── 命令构建 ──────────────────────────────────────────────────

def build_yaml(ep, arm, spd, mode, col_str, px, py, pz, qz, qw):
    """用普通字符串拼接构建 YAML, 避开 f-string 的花括号转义地狱喵~"""
    return (
        "{"
        f"episode_idx: {ep}, "
        f"arm: {arm}, "
        f"speed_scale: {spd}, "
        f"mode: {mode}, "
        f"collision_check: {col_str}, "
        "start_pose: {"
        f"position: {{x: {px}, y: {py}, z: {pz}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}"
        "}"
        "}"
    )


def _cmd(ep, arm, spd, mode, col, px, py, pz, qz, qw):
    """构建 ros2 service call 命令喵~"""
    yaml = build_yaml(ep, arm, spd, mode, col, px, py, pz, qz, qw)
    return (
        "ros2 service call /latte_imitation/replay_trajectory "
        "ivg_interfaces/srv/ReplayLatteTrajectory "
        f"'{yaml}'"
    )


# ── 测试用例定义 ────────────────────────────────────────────────


# 测试 1~6 的默认参数: (ep, arm, spd, mode, col, px, py, pz, qz, qw)
_T = lambda *a: _cmd(*a)

TESTS = [
    {
        "id": 1,
        "name": "Debug — 预览轨迹",
        "desc": "加载 episode 0 右臂, 发布 PoseStamped+Path, 不驱动机器人",
        "cmd": _T("0", "right", "1.0", "debug", "false",
                  "0.0", "0.0", "0.0", "0.0", "1.0"),
        "expect": "success=true, num_frames=400, path_length~1.53",
    },
    {
        "id": 2,
        "name": "Action + 碰撞检测",
        "desc": "完整管线: IK→碰撞→JointTrajectory→机械臂执行 (2倍速, ~10秒)",
        "cmd": _T("0", "right", "2.0", "action", "true",
                  "0.0", "0.0", "0.0", "0.0", "1.0"),
        "expect": "success=true, ik_success_count~400 (fraction*400), collision_count=0 (MoveIt2 内置)",
    },
    {
        "id": 3,
        "name": "start_pose 纯平移",
        "desc": "杯子在 camera_pose (0.45,0,0.50), 杯口Z轴朝上 (远离底座防碰撞)",
        "cmd": _T("0", "right", "1.0", "action", "true",
                  "0.45", "0.0", "0.50", "0.0", "1.0"),
        "expect": "success=true, 轨迹起点=杯子上方45cm, 整条轨迹高于桌面",
    },
    {
        "id": 4,
        "name": "start_pose 平移+旋转",
        "desc": "杯子在 camera_pose, 绕Z轴90° (杯口朝向变了) (z=0.707,w=0.707)",
        "cmd": _T("0", "right", "1.0", "action", "true",
                  "0.45", "0.0", "0.50", "0.707", "0.707"),
        "expect": "success=true, 路径长度不变(刚性保距), 碰撞结果可能不同",
    },
    {
        "id": 5,
        "name": "错误处理 — 不存在 episode",
        "desc": "请求 episode=999, 验证错误信息",
        "cmd": _T("999", "right", "1.0", "debug", "false",
                  "0.0", "0.0", "0.0", "0.0", "1.0"),
        "expect": 'success=false, message="episode_000999.npz ... 未找到"',
    },
    {
        "id": 6,
        "name": "左臂持杯轨迹",
        "desc": "arm='left', 几乎静止, 路径~0.31m (右臂的1/5)",
        "cmd": _T("0", "left", "1.0", "debug", "false",
                  "0.0", "0.0", "0.0", "0.0", "1.0"),
        "expect": "success=true, num_frames=400, path_length~0.31",
    },
    {
        "id": 7,
        "name": "自定义 — 手动输入参数",
        "desc": "逐项输入 episode/arm/speed/mode/collision/start_pose",
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
    """自定义 episode 测试喵~"""
    try:
        ep   = input("  episode_idx (0~39, 默认 0): ").strip() or "0"
        arm  = input("  arm (right/left, 默认 right): ").strip() or "right"
        spd  = input("  speed_scale (默认 1.0): ").strip() or "1.0"
        mode = input("  mode (debug/action, 默认 action): ").strip() or "action"
        col  = input("  collision_check (y/n, 默认 y): ").strip().lower()
        col_str = "true" if col in ("y", "yes", "") else "false"
        px = input("  start_pose.x (默认 0.0): ").strip() or "0.0"
        py = input("  start_pose.y (默认 0.0): ").strip() or "0.0"
        pz = input("  start_pose.z (默认 0.0): ").strip() or "0.0"
        qz = input("  orientation.z (0=直立, 0.707=90°, 默认 0): ").strip() or "0.0"
        qw = input("  orientation.w (默认 1.0): ").strip() or "1.0"

        yaml = build_yaml(ep, arm, spd, mode, col_str, px, py, pz, qz, qw)
        cmd = (
            "ros2 service call /latte_imitation/replay_trajectory "
            "ivg_interfaces/srv/ReplayLatteTrajectory "
            f"'{yaml}'"
        )
        run_cmd(cmd)
    except (KeyboardInterrupt, EOFError):
        print()


# ── 主循环 ──────────────────────────────────────────────────────

def main():
    print("\033[1;32mlatte_imitation 交互式测试工具\033[0m")
    print("请确保仿真(终端1)和服务节点(终端2)已就绪")

    while True:
        print_menu()
        try:
            choice = input("\n输入编号 (0~7): ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n退出喵~")
            break

        if choice == "0":
            print("退出喵~")
            break

        if choice == "7":
            run_interactive()
            continue

        try:
            idx = int(choice)
            test = next((t for t in TESTS if t["id"] == idx), None)
        except ValueError:
            print("\033[1;31m无效输入, 请输入数字 0~7\033[0m")
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
