#!/usr/bin/env python3
"""FK/IK 一致性检查工具（参考 aubo_dashboard 服务接口，仅 real 模式可用）。

流程：读当前 /joint_states → GetFK 求末端位姿 → 以当前关节为参考解 GetIK →
比较 IK 解与当前关节角的偏差。三者应自洽（误差 ~1e-4 rad 量级），
用于验证 SDK 运动学链与 ROS 侧状态的一致性。
用法:
  python3 tools/fk_ik_check.py
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node

from aubo_msgs.srv import GetFK, GetIK
from sensor_msgs.msg import JointState

JOINTS = [
    "shoulder_joint", "upperArm_joint", "foreArm_joint",
    "wrist1_joint", "wrist2_joint", "wrist3_joint",
]


class FkIkCheck(Node):
    def __init__(self):
        super().__init__("fk_ik_check")
        self._js = None
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._fk = self.create_client(GetFK, "/aubo_dashboard/get_fk")
        self._ik = self.create_client(GetIK, "/aubo_dashboard/get_ik")

    def _on_js(self, msg):
        try:
            self._js = [msg.position[msg.name.index(j)] for j in JOINTS]
        except ValueError:
            pass

    def run(self):
        end = time.time() + 5.0
        while self._js is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._js is None:
            raise RuntimeError("未收到 /joint_states")
        for cli, name in ((self._fk, "get_fk"), (self._ik, "get_ik")):
            if not cli.wait_for_service(timeout_sec=3.0):
                raise RuntimeError(
                    f"/aubo_dashboard/{name} 不可用——dashboard 仅 real 模式启动 "
                    "(hardware_mode:=real)")

        fk_req = GetFK.Request()
        fk_req.joint = [float(q) for q in self._js]
        fk = self._fk.call_async(fk_req)
        rclpy.spin_until_future_complete(self, fk)
        resp = fk.result()
        pos, ori = list(resp.pos), list(resp.ori)
        print(f"当前关节: {[round(q, 4) for q in self._js]}")
        print(f"FK 位姿:  pos={[round(p, 4) for p in pos]}  ori(rpy)={[round(o, 4) for o in ori]}")

        ik_req = GetIK.Request()
        ik_req.ref_joint = [float(q) for q in self._js]
        ik_req.pos = pos
        ik_req.ori = ori
        ik = self._ik.call_async(ik_req)
        rclpy.spin_until_future_complete(self, ik)
        sol = list(ik.result().joint)
        errs = [abs(a - b) for a, b in zip(sol, self._js)]
        print(f"IK 解:   {[round(q, 4) for q in sol]}")
        print(f"偏差:    max={max(errs):.6f} rad  mean={sum(errs) / len(errs):.6f} rad")
        ok = max(errs) < 1e-2
        print("FK/IK 自洽性: " + ("OK" if ok else "FAIL（超过 0.01 rad）"))
        return ok


def main():
    rclpy.init()
    node = FkIkCheck()
    ok = False
    try:
        ok = node.run()
    except RuntimeError as e:
        print(f"检查失败: {e}", file=sys.stderr)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
