#!/usr/bin/env python3
"""
latte_imitation 交互式预览/执行工具 — shell 控制面板喵~

用法:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 src/latte_imitation/scripts/test_latte_pour.py

功能:
  - 调整 Episode / RPY / 速度 / 工具等参数
  - Preview 模式: 发送 service call → RViz2 即时更新预览
  - Action 模式: 真机执行轨迹
  - 参数记忆: RPY 角度在会话期间保留
"""

import subprocess
import sys
import os as _os

# 将包路径加入 sys.path
_sys_path = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _sys_path not in sys.path:
    sys.path.insert(0, _sys_path)


def euler_deg_to_quat(roll, pitch, yaw):
    """行内四元数计算, 避免 ros2 run 环境依赖问题喵~"""
    import numpy as np
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr = np.cos(r*0.5), np.sin(r*0.5)
    cp, sp = np.cos(p*0.5), np.sin(p*0.5)
    cy, sy = np.cos(y*0.5), np.sin(y*0.5)
    return [
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    ]


def build_yaml(ep, arm, spd, mode, roll, pitch, yaw, tool_id, px, py, pz):
    """用 yaml.dump 安全构建 YAML 喵~"""
    import yaml as _yaml
    q = euler_deg_to_quat(roll, pitch, yaw)
    data = {
        "episode_idx": int(ep),
        "arm": str(arm),
        "speed_scale": float(spd),
        "mode": str(mode),
        "roll_deg": float(roll),
        "pitch_deg": float(pitch),
        "yaw_deg": float(yaw),
        "tool_offset_id": str(tool_id),
        "pos_only": False,
        "collision_check": False,
        "start_pose": {
            "position": {"x": float(px), "y": float(py), "z": float(pz)},
            "orientation": {"x": q[0], "y": q[1], "z": q[2], "w": q[3]},
        },
    }
    return _yaml.dump(data, default_flow_style=True, sort_keys=False).strip().replace("\n", "")


def call_service(yaml_str: str):
    """发送 ros2 service call 喵~"""
    cmd = (
        "ros2 service call /latte_imitation/replay_trajectory "
        "ivg_interfaces/srv/ReplayLatteTrajectory "
        f"'{yaml_str}'"
    )
    print(f"\n\033[1;34m>>> {cmd[:90]}...\033[0m\n")
    result = subprocess.run(cmd, shell=True, capture_output=False)
    return result.returncode


# ── 状态管理 ──────────────────────────────────────────────────

class State:
    def __init__(self):
        self.ep = 0
        self.arm = "right"
        self.spd = 1.0
        self.mode = "preview"
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.tool_id = "default"
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0


# ── 菜单渲染 ──────────────────────────────────────────────────

def print_menu(s: State):
    print()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║       latte_imitation 交互式预览/执行工具                     ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print(f"║  episode: {s.ep:<3d}   arm: {s.arm:<6s}  speed: {s.spd:.1f}x"
          f"   mode: {s.mode:<8s} ║")
    print(f"║  roll: {s.roll:6.0f}°  pitch: {s.pitch:6.0f}°  yaw: {s.yaw:6.0f}°"
          f"            ║")
    print(f"║  tool: {s.tool_id:<8s}  cup_pos: "
          f"({'auto TF' if s.px==0 and s.py==0 and s.pz==0 else f'({s.px:.2f},{s.py:.2f},{s.pz:.2f})':<20s} ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  [1] 选择 Episode      [2] 切换手臂 (right/left)             ║")
    print("║  [3] 设置 RPY 旋转     [4] 调整速度倍率                       ║")
    print("║  [5] 选择工具偏移      [6] 手动设置杯子位姿                   ║")
    print("║  [7] 切换模式          [8] 快速预设                           ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print("║  [p] 刷新预览 (发送 service, 更新 RViz2)                     ║")
    print("║  [e] 执行轨迹 (action 模式, 真机运动)                         ║")
    print("║  [r] 重置参数           [q] 退出                              ║")
    print("╚══════════════════════════════════════════════════════════════╝")


# ── 快速预设 ──────────────────────────────────────────────────

PRESETS = {
    "1": ("默认 (不旋转)", 0, 0.0, 0.0, 0.0),
    "2": ("Yaw +90°", 0, 0.0, 0.0, 90.0),
    "3": ("Yaw +180°", 0, 0.0, 0.0, 180.0),
    "4": ("Yaw -90°", 0, 0.0, 0.0, -90.0),
    "5": ("Roll +10° 前倾", 0, 10.0, 0.0, 0.0),
}


# ── 主循环 ────────────────────────────────────────────────────

def main():
    s = State()
    print("\033[1;32m latte_imitation 交互式预览/执行工具\033[0m")
    print("确保 latte_imitation 节点和 RViz2 已启动")
    print("提示: 在 RViz2 中添加以下 Display:")
    print("  Marker (C++) → /rviz_visual_tools  (EE 轨迹线, moveit_visual_tools)")
    print("  Marker     → /latte_imitation/preview/spout_path")
    print("  Marker     → /latte_imitation/preview/cup_pose")
    print("  Marker     → /latte_imitation/preview/workspace_bounds")

    while True:
        print_menu(s)
        try:
            choice = input("\n选择 > ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\n退出喵~")
            break

        if choice == "q":
            print("退出喵~")
            break

        elif choice == "1":
            try:
                s.ep = int(input(f"  Episode (0-39, 当前 {s.ep}): ").strip() or s.ep)
            except ValueError:
                print("\033[1;31m无效数字\033[0m")

        elif choice == "2":
            s.arm = "left" if s.arm == "right" else "right"
            print(f"  → arm = '{s.arm}'")

        elif choice == "3":
            try:
                r = input(f"  roll  / X轴 度 (当前 {s.roll:.0f}): ").strip()
                p = input(f"  pitch / Y轴 度 (当前 {s.pitch:.0f}): ").strip()
                y = input(f"  yaw   / Z轴 度 (当前 {s.yaw:.0f}): ").strip()
                if r: s.roll = float(r)
                if p: s.pitch = float(p)
                if y: s.yaw = float(y)
            except ValueError:
                print("\033[1;31m无效数字\033[0m")

        elif choice == "4":
            try:
                s.spd = float(input(f"  速度倍率 (当前 {s.spd:.1f}x): ").strip() or s.spd)
            except ValueError:
                print("\033[1;31m无效数字\033[0m")

        elif choice == "5":
            s.tool_id = input(f"  工具 ID (当前 '{s.tool_id}'): ").strip() or s.tool_id

        elif choice == "6":
            try:
                px = input(f"  cup X (0=auto TF, 当前 {s.px:.2f}): ").strip()
                py = input(f"  cup Y (0=auto TF, 当前 {s.py:.2f}): ").strip()
                pz = input(f"  cup Z (0=auto TF, 当前 {s.pz:.2f}): ").strip()
                if px: s.px = float(px)
                if py: s.py = float(py)
                if pz: s.pz = float(pz)
            except ValueError:
                print("\033[1;31m无效数字\033[0m")

        elif choice == "7":
            modes = ["preview", "debug", "action"]
            try:
                idx = modes.index(s.mode)
                s.mode = modes[(idx + 1) % len(modes)]
            except ValueError:
                s.mode = "preview"
            print(f"  → mode = '{s.mode}'")

        elif choice == "8":
            print("\n  快速预设:")
            for k, (desc, ep, r, p, y) in PRESETS.items():
                print(f"    [{k}] {desc}: ep={ep}, rpy=({r:.0f},{p:.0f},{y:.0f})")
            pk = input("  选择预设 > ").strip()
            if pk in PRESETS:
                _, s.ep, s.roll, s.pitch, s.yaw = PRESETS[pk]
                print(f"  → 已加载: {PRESETS[pk][0]}")

        elif choice == "p":
            yaml_str = build_yaml(
                s.ep, s.arm, s.spd, "preview",
                s.roll, s.pitch, s.yaw, s.tool_id,
                s.px, s.py, s.pz,
            )
            call_service(yaml_str)

        elif choice == "e":
            print("\033[1;33m⚠ 即将在真机上执行轨迹! 确认? (输入 yes 继续)\033[0m")
            confirm = input("> ").strip()
            if confirm.lower() == "yes":
                yaml_str = build_yaml(
                    s.ep, s.arm, s.spd, "action",
                    s.roll, s.pitch, s.yaw, s.tool_id,
                    s.px, s.py, s.pz,
                )
                call_service(yaml_str)
            else:
                print("  已取消")

        elif choice == "r":
            s = State()
            print("  → 参数已重置为默认值")

        else:
            print(f"\033[1;31m未知命令: '{choice}'\033[0m")


if __name__ == "__main__":
    main()
