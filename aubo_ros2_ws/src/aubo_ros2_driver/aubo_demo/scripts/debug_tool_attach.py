#!/usr/bin/env python3
"""
调试工具：检查末端工具与 kuaihuan_Link 的连接适配。

用法:
  python3 debug_tool_attach.py list       # 列出可用工具
  python3 debug_tool_attach.py check      # 检查 kuaihuan_Link 当前位姿 (需先 source + 启动机械臂)
  python3 debug_tool_attach.py info       # 显示 URDF 中 kuaihuan_Link 的固定偏移
"""

import sys
import os

TOOLS = {
    "gripper0":  {"mesh": "gripper0_link.stl",          "desc": "气动夹爪 40"},
    "gripper1":  {"mesh": "gripper1_link.stl",          "desc": "电动夹爪 A"},
    "gripper2":  {"mesh": "gripper2_link.stl",          "desc": "电动夹爪 60"},
    "coffee_cup":{"mesh": "gripper1coffeecup_link.stl",  "desc": "咖啡杯工具"},
    "milk_cup":  {"mesh": "gripper1milkcup_link.stl",    "desc": "奶杯工具"},
}

MESHES = "/home/mu/IVG2.0/aubo_ros2_ws/src/aubo_ros2_driver/aubo_description/meshes"


def cmd_list():
    print("=== 可用末端工具 ===")
    for name, t in TOOLS.items():
        mesh_path = os.path.join(MESHES, "visual", t["mesh"])
        size = os.path.getsize(mesh_path) if os.path.exists(mesh_path) else 0
        print(f"  {name:12s} {t['desc']:14s}  {t['mesh']:35s}  ({size/1024:.0f}KB)")
    print(f"\n挂载点: kuaihuan_Link (快换盘)")

def cmd_info():
    """显示 URDF 中 kuaihuan_Link 的固定偏移"""
    print("=== kuaihuan_Link 在 URDF 中的固定链 ===")
    print("""
  wrist3_Link
    │ camera_joint [0, 0, 0.020]
    ▼
  camera_Link
    │ kuaihuan_joint [0, 0, 0.0215]  rpy=[0, 0, π]
    ▼
  kuaihuan_Link  ← 工具在此挂载 (Z轴朝外)
""")

def cmd_check():
    """通过 TF 查询 kuaihuan_Link 当前位置"""
    import subprocess
    print("=== 查询 kuaihuan_Link 当前位姿 (需要机械臂驱动已启动) ===")
    print()

    # Try tf2_echo
    try:
        result = subprocess.run(
            ["ros2", "run", "tf2_ros", "tf2_echo", "base_link", "kuaihuan_Link", "--once"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            for line in result.stdout.split('\n'):
                if 'Translation' in line or 'Rotation' in line:
                    print(f"  {line.strip()}")
        else:
            print("  无法获取 TF。请确保:")
            print("    1. 机械臂驱动或仿真已启动")
            print("    2. robot_state_publisher 正在运行")
    except FileNotFoundError:
        print("  ros2 命令不可用，请先 source setup.bash")
    except subprocess.TimeoutExpired:
        print("  查询超时")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "list"
    {"list": cmd_list, "info": cmd_info, "check": cmd_check}.get(cmd, cmd_list)()
