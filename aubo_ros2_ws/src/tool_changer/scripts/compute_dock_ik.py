#!/usr/bin/env python3
"""
通过 AUBO Dashboard GetIK 服务，从 dock_above 笛卡尔位置反解关节角度喵~

用法:
  python3 compute_dock_ik.py [--dry-run]

  --dry-run  仅打印请求内容，不调用服务（机械臂未启动时可用）

该脚本读取 tools.yaml 中需要 IK 求解的工具条目，调用 /aubo_dashboard/get_ik
服务进行逆运动学求解。ref_joint 默认使用 gripper0 的已知关节角（dock 位相邻）喵~
"""

import argparse
import sys
import os

try:
    import rclpy
    from rclpy.node import Node
    from ivg_interfaces.srv import GetIK
    import yaml
except ImportError as e:
    print(f"[WARN] 缺少依赖: {e}")
    print("  在 aubo_ros2_ws 下执行: source install/setup.bash && python3 ...")
    sys.exit(1)


def load_tools_yaml():
    """加载 tools.yaml，返回工具字典喵~"""
    script_dir = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(script_dir, "..", "config", "tools.yaml")
    yaml_path = os.path.normpath(yaml_path)
    if not os.path.isfile(yaml_path):
        # 尝试 install 目录
        from ament_index_python.packages import get_package_share_directory
        yaml_path = os.path.join(
            get_package_share_directory("tool_changer"), "config", "tools.yaml")
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    return data.get("tools", {})


def compute_ik(node, ref_joint, pos, ori):
    """调用 GetIK 服务，返回 6 个关节角度喵~"""
    client = node.create_client(GetIK, "/aubo_dashboard/get_ik")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("GetIK 服务不可达（机械臂未启动或 Dashboard 未激活）")
        return None

    req = GetIK.Request()
    req.ref_joint = list(ref_joint)
    req.pos = list(pos)
    req.ori = list(ori)
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    if not future.done():
        node.get_logger().error("GetIK 调用超时")
        return None
    try:
        resp = future.result()
        node.get_logger().info(
            f"IK 结果: [{resp.joint[0]:.6f}, {resp.joint[1]:.6f}, "
            f"{resp.joint[2]:.6f}, {resp.joint[3]:.6f}, "
            f"{resp.joint[4]:.6f}, {resp.joint[5]:.6f}]")
        return list(resp.joint)
    except Exception as e:
        node.get_logger().error(f"GetIK 异常: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(description="从 dock_above 位置计算 dock_approach_joints")
    parser.add_argument("--dry-run", action="store_true",
                        help="仅打印请求内容，不调用服务")
    args = parser.parse_args()

    tools = load_tools_yaml()
    if not tools:
        print("[ERROR] tools.yaml 未找到或为空")
        sys.exit(1)

    # gripper0 参考关节角（已验证可到达其 dock_above）
    ref_joint = [1.137820, 0.222690, 1.598043, -0.194970, 1.571688, 1.136957]

    # 默认方向四元数 (w,x,y,z) — 末端垂直向下，与 gripper0 dock 姿态一致
    default_ori = [1.0, 0.0, 0.0, 0.0]

    for tid, tcfg in tools.items():
        if "dock_approach_joints" in tcfg:
            continue  # 已有关节角，跳过
        if "dock_above" not in tcfg:
            continue  # 无 dock_above，跳过

        da = tcfg["dock_above"]
        pos = [float(da["x"]), float(da["y"]), float(da["z"])]
        ori = default_ori

        print(f"\n{'='*60}")
        print(f"工具: {tid} ({tcfg.get('name', '?')})")
        print(f"  dock_above 位置: x={pos[0]:.5f} y={pos[1]:.5f} z={pos[2]:.5f}")
        print(f"  ref_joint (gripper0): [{', '.join(f'{j:.6f}' for j in ref_joint)}]")

        if args.dry_run:
            print(f"  [DRY-RUN] 跳过服务调用。启动机械臂后运行: python3 {sys.argv[0]}")
            print(f"  预期 YAML 条目:")
            print(f"    dock_approach_joints: "
                  f"[TBD, TBD, TBD, TBD, TBD, TBD]  # ← 需实际 IK 求解")
            continue

        rclpy.init(args=[])
        node = Node("dock_ik_computer")
        result = compute_ik(node, ref_joint, pos, ori)
        node.destroy_node()
        rclpy.shutdown()

        if result:
            print(f"  求解成功!")
            print(f"  请在 tools.yaml 的 {tid} 下添加:")
            print(f"    dock_approach_joints: "
                  f"[{result[0]:.6f}, {result[1]:.6f}, {result[2]:.6f}, "
                  f"{result[3]:.6f}, {result[4]:.6f}, {result[5]:.6f}]")
        else:
            print(f"  求解失败，请检查机械臂状态")


if __name__ == "__main__":
    main()
