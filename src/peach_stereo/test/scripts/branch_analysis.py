#!/usr/bin/env python3
"""枝上深度质量分析：peach_vegetation 枝掩膜 × 相机深度（避障口径）.

用法（venv python）: branch_analysis.py <profile> [n_frames=20]
订 color/depth/branch_mask/overlay；掩膜带源图 stamp，与深度 stamp 就近配对（静态场景
容差 0.3s）。逐帧 JSON 行打 stdout，末行 SUMMARY。存图 /tmp/e2e_live/branch/<profile>/：
  overlay_<i>.png   叶绿枝红叠加（vegetation 原产）
  branchdepth_<i>.png 暗化彩底 + 枝上有效深度 JET 上色 + 枝轮廓红
指标：
  cov/cov_thick/cov_thin  枝/粗枝/细枝像素的窗内(0.3-1.5m)有效深度覆盖
  detail_mm               枝上全有效 3x3 邻域局部 std（细结构可见度）
  mad                     枝上深度空间 MAD（结构展布）
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

OUT_ROOT = '/tmp/e2e_live/branch'


class BranchAnalyzer(Node):
    """按枝掩膜评估深度流对细结构的可用性（避障口径）."""

    def __init__(self, profile, n):
        super().__init__('branch_analyzer')
        self.profile, self.n = profile, n
        self.got = 0
        self.frames = []
        self.latest = {}
        os.makedirs(f'{OUT_ROOT}/{profile}', exist_ok=True)
        q = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE)
        for topic in ('/camera/color/image_raw', '/camera/depth/image_raw',
                      '/peach/vegetation/branch_mask', '/peach/vegetation/overlay'):
            self.create_subscription(
                Image, topic,
                lambda m, t=topic: self.latest.__setitem__(t, m), q)
        self.t0 = None
        self.create_timer(0.7, self.tick)
        print(f'[branch] profile={profile} n={n} 等待流…', flush=True)

    @staticmethod
    def _ns(m):
        return m.header.stamp.sec * 10**9 + m.header.stamp.nanosec

    def tick(self):
        """采样一帧并输出指标与图；够数即汇总退出."""
        if self.t0 is None:
            self.t0 = time.time()
        d = self.latest.get('/camera/depth/image_raw')
        b = self.latest.get('/peach/vegetation/branch_mask')
        c = self.latest.get('/camera/color/image_raw')
        ov = self.latest.get('/peach/vegetation/overlay')
        if d is None or b is None or c is None:
            return
        if abs(self._ns(b) - self._ns(d)) > 6e8:
            return
        dm = np.frombuffer(d.data, np.uint16).reshape(
            d.height, d.width).astype(np.float64) * 0.25
        mask = np.frombuffer(b.data, np.uint8).reshape(
            b.height, b.width) > 127
        if int(mask.sum()) < 500:
            return
        rgb = np.frombuffer(c.data, np.uint8).reshape(
            c.height, c.width, 3).copy()
        valid = (dm >= 300) & (dm <= 1500)
        bv = mask & valid
        thick = cv2.erode(mask.astype(np.uint8),
                          np.ones((3, 3), np.uint8)).astype(bool)
        thin = mask & ~thick
        vf = valid.astype(np.uint8)
        full = cv2.erode(vf, np.ones((3, 3), np.uint8)).astype(bool)
        df = np.where(valid, dm, 0.0)
        ys, xs = np.where(full & bv)
        detail = float('nan')
        if len(ys) > 5:
            step = max(1, len(ys) // 3000)
            detail = float(np.mean([df[y - 1:y + 2, x - 1:x + 2].std()
                                    for y, x in zip(ys[::step], xs[::step])]))
        med = float(np.median(dm[bv])) if bv.any() else 0.0
        rec = dict(i=self.got, branch_px=int(mask.sum()),
                   cov=round(float(bv.sum()) / mask.sum(), 4),
                   cov_thick=round(float((thick & valid).sum()) / max(int(thick.sum()), 1), 4),
                   cov_thin=round(float((thin & valid).sum()) / max(int(thin.sum()), 1), 4),
                   detail_mm=round(detail, 2), med=round(med, 1),
                   mad=round(float(np.median(np.abs(dm[bv] - med))) if bv.any() else 0.0, 2))
        self.frames.append(rec)
        self.got += 1
        i = self.got
        if ov is not None:
            o = np.frombuffer(ov.data, np.uint8).reshape(
                ov.height, ov.width, 3).copy()
            cv2.imwrite(f'{OUT_ROOT}/{self.profile}/overlay_{i:02d}.png', o)
        vis = (rgb * 0.35).astype(np.uint8)
        jet = cv2.applyColorMap(
            np.clip((dm - 300) / 1200 * 255, 0, 255).astype(np.uint8),
            cv2.COLORMAP_JET)
        vis[bv] = jet[bv]
        cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, cnts, -1, (0, 0, 255), 1)
        cv2.imwrite(f'{OUT_ROOT}/{self.profile}/branchdepth_{i:02d}.png', vis)
        print(json.dumps(rec), flush=True)
        if self.got >= self.n:
            agg = {k: round(float(np.nanmean([f[k] for f in self.frames])), 4)
                   for k in ('cov', 'cov_thick', 'cov_thin', 'detail_mm', 'mad')}
            print('SUMMARY ' + json.dumps(dict(profile=self.profile, n=self.n, **agg)),
                  flush=True)
            rclpy.shutdown()


def main():
    profile = sys.argv[1]
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 20
    rclpy.init()
    node = BranchAnalyzer(profile, n)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
