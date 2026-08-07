"""
eval_refinement — 曲线轨迹+三维重建的效果评估节点（三路对比）.

订阅三路结果在 pick 弧段期间累积，结束时输出对比表 + JSON：
  A 单帧直接估计（/peach_pose_node/grasp_candidates 最优候选）
  B 时序融合（/peach_pose_node/refined_candidates）
  C 三维重建精化（/peach_pose_node/recon_refined，ROI 云重拟合）

用法：
    ros2 run peach_recon eval_refinement
    # 弧段跑完后 Ctrl+C，自动打印对比表并写 peach_records/eval_<时间戳>.json
"""
import json
from pathlib import Path
import time

import numpy as np
from peach_pose_msgs.msg import BagGraspCandidateArray, FusionStats
import rclpy
from rclpy.node import Node
import rclpy.qos


def _best(arr):
    """取最优候选：ACCEPT 优先，其余按置信度降序；无候选返回 None."""
    return max(arr.candidates,
               key=lambda c: (int(c.status) == 0, float(c.confidence)),
               default=None)


def _axis(cand):
    d = cand.translation_direction
    return np.array([d.x, d.y, d.z], dtype=float)


def _pos(cand):
    p = cand.entry_pose.position
    return np.array([p.x, p.y, p.z], dtype=float)


def _axis_angle_deg(a, b):
    """两轴向无向夹角（度，忽略 ± 符号）."""
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return float('nan')
    cos = abs(float(np.dot(a, b)) / (na * nb))
    return float(np.degrees(np.arccos(min(1.0, max(-1.0, cos)))))


class RefinementEval(Node):
    """三路估计收集器：弧段期间累积，收尾时算对比指标."""

    def __init__(self):
        """建订阅与缓存；output_dir 参数决定 JSON 输出目录."""
        super().__init__('refinement_eval')
        self.declare_parameter('output_dir', 'peach_records')
        # 方向真值参考：base_link 下重力方向（套袋桃袋轴应逆重力）
        self.declare_parameter('gravity_base_xyz', '0,0,-1')
        # 可选定位真值：实测目标位置 "x,y,z"（米，base_link），空=不测绝对误差
        self.declare_parameter('gt_position', '')
        self.output_dir = Path(str(self.get_parameter('output_dir').value))
        gh = str(self.get_parameter('gravity_base_xyz').value)
        self.gravity = np.array([float(v) for v in gh.split(',')], dtype=float)
        self.gravity /= np.linalg.norm(self.gravity)
        gt = str(self.get_parameter('gt_position').value).strip()
        self.gt_pos = (np.array([float(v) for v in gt.split(',')])
                       if gt else None)
        qos = rclpy.qos.QoSProfile(
            depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST)
        self.A, self.B, self.C = [], [], []
        self.last_stats = None
        self.create_subscription(
            BagGraspCandidateArray, '/peach_pose_node/grasp_candidates',
            lambda m: self._append(self.A, m), qos)
        self.create_subscription(
            BagGraspCandidateArray, '/peach_pose_node/refined_candidates',
            lambda m: self._append(self.B, m), qos)
        self.create_subscription(
            BagGraspCandidateArray, '/peach_pose_node/recon_refined',
            self._on_recon, qos)
        self.create_subscription(
            FusionStats, '/peach_pose_node/fusion_stats',
            lambda m: setattr(self, 'last_stats', m), qos)
        self.get_logger().info(
            '评估节点就绪：收集中（A 单帧 / B 融合 / C 重建精化），'
            'Ctrl+C 出对比报告')

    def _append(self, buf, msg):
        c = _best(msg)
        if c is None:
            return
        buf.append((_pos(c), _axis(c), float(c.confidence)))

    def _on_recon(self, msg):
        c = _best(msg)
        if c is None:
            return
        rms = next((f for f in c.diagnostic_flags if f.startswith('rms=')), '')
        npts = next((f for f in c.diagnostic_flags
                     if f.startswith('n_points=')), '')
        self.C.append((_pos(c), _axis(c), float(c.confidence), rms, npts))

    def report(self):
        """计算并打印三路对比指标，写 JSON 到 output_dir/eval_<ts>.json."""
        n_a, n_b, n_c = len(self.A), len(self.B), len(self.C)
        print(f'\n===== 曲线轨迹+三维重建 效果评估（A={n_a} 帧, B={n_b} 帧, C={n_c} 次）=====')
        if n_a < 2:
            print('单帧样本不足（<2 帧），无法评估。')
            return
        posA = np.array([p for p, _, _ in self.A])
        axisA = np.array([a for _, a, _ in self.A])
        meanA = posA.mean(axis=0)
        jitA_pos = float(np.sqrt(posA.var(axis=0).sum()))
        angA = np.degrees(np.arccos(np.clip(
            axisA @ (axisA.mean(axis=0) / np.linalg.norm(axisA.mean(axis=0)))
            / np.linalg.norm(axisA, axis=1), -1, 1)))
        jitA_axis = float(np.sqrt(np.nanmean(angA ** 2)))
        print(f'A 单帧直接估计：位置抖动 σ={jitA_pos * 1000:.1f} mm, '
              f'轴向抖动 RMS={jitA_axis:.2f}°')
        # 方向真值参考：袋轴应逆重力，算 A 均值轴与逆重力夹角（越小越准）
        anti_g = -self.gravity
        g_angA = _axis_angle_deg(axisA.mean(axis=0), anti_g)
        print(f'  方向参考：A 均值轴 vs 逆重力夹角 {g_angA:.1f}°'
              f'（套袋桃应接近 0°）')
        out = {'frames': {'A': n_a, 'B': n_b, 'C': n_c},
               'A_single_frame': {'pos_jitter_m': jitA_pos,
                                  'axis_jitter_deg': jitA_axis,
                                  'axis_vs_antigravity_deg': g_angA}}
        if self.gt_pos is not None:
            d_gtA = float(np.linalg.norm(meanA - self.gt_pos))
            print(f'  定位参考：A 均值 vs 实测点误差 {d_gtA * 1000:.1f} mm')
            out['A_single_frame']['dist_to_gt_m'] = d_gtA
        if n_b >= 2:
            posB = np.array([p for p, _, _ in self.B])
            axisB = np.array([a for _, a, _ in self.B])
            finalB = posB[-1]
            jitB_pos = float(np.sqrt(posB.var(axis=0).sum()))
            se_pos = self.last_stats.pos_std_m if self.last_stats else float('nan')
            se_axis = self.last_stats.axis_std_deg if self.last_stats else float('nan')
            d_ab = float(np.linalg.norm(finalB - meanA))
            ang_ab = _axis_angle_deg(axisB[-1], axisA.mean(axis=0))
            print(f'B 时序融合    ：位置标准误 SE={se_pos * 1000:.2f} mm, '
                  f'轴向 SE={se_axis:.2f}°；与 A 均值差 {d_ab * 1000:.1f} mm / '
                  f'{ang_ab:.1f}°')
            out['B_temporal_fused'] = {
                'pos_jitter_m': jitB_pos, 'pos_se_m': float(se_pos),
                'axis_se_deg': float(se_axis),
                'dist_to_A_mean_m': d_ab, 'angle_to_A_mean_deg': ang_ab}
        if n_c >= 1:
            finalC = self.C[-1]
            d_bc = float(np.linalg.norm(finalC[0] - (
                self.B[-1][0] if n_b else meanA)))
            d_ac = float(np.linalg.norm(finalC[0] - meanA))
            g_angC = _axis_angle_deg(finalC[1], anti_g)
            print(f'C 重建精化    ：与 B 终值差 {d_bc * 1000:.1f} mm, '
                  f'与 A 均值差 {d_ac * 1000:.1f} mm'
                  f'（{finalC[3]}, {finalC[4]}）')
            print(f'  方向参考：C 精化轴 vs 逆重力夹角 {g_angC:.1f}°')
            out['C_recon_refined'] = {'dist_to_B_m': d_bc,
                                      'dist_to_A_mean_m': d_ac,
                                      'axis_vs_antigravity_deg': g_angC,
                                      'fit': {'rms': finalC[3], 'n': finalC[4]}}
            if self.gt_pos is not None:
                d_gtC = float(np.linalg.norm(finalC[0] - self.gt_pos))
                print(f'  定位参考：C 精化 vs 实测点误差 {d_gtC * 1000:.1f} mm')
                out['C_recon_refined']['dist_to_gt_m'] = d_gtC
        # 结论
        if n_b >= 2 and jitA_pos > 1e-9:
            ratio = jitA_pos / max(out['B_temporal_fused']['pos_jitter_m'], 1e-9)
            d_bc = out.get('C_recon_refined', {}).get(
                'dist_to_B_m', float('nan'))
            print(f'结论：融合把位置抖动收敛 {ratio:.1f}×；'
                  f'重建精化终值与融合值一致度 {d_bc * 1000:.1f} mm')
            out['conclusion'] = {'jitter_reduction_x': ratio}
        self.output_dir.mkdir(parents=True, exist_ok=True)
        path = self.output_dir / f'eval_{time.strftime("%Y%m%d_%H%M%S")}.json'
        path.write_text(json.dumps(out, ensure_ascii=False, indent=2))
        print(f'报告已写: {path}\n')


def main(args=None):
    """入口：spin 至 Ctrl+C，收尾出报告."""
    rclpy.init(args=args)
    node = RefinementEval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
