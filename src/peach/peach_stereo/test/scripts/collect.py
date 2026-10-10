#!/usr/bin/env python3
"""A/B live 采集器：活相机 → peach 感知管线（YOLO→SAM→拟合→身份链）→ 结构化 JSONL.

与 /tmp/perception_live.py 同构（含 YOLO 零检测回退 ROI），差异：
  - 固定时长运行（argv[2] 秒，默认 70）后自动干净退出
  - 每帧向 stdout 打一行 JSON（由启动器重定向为 frames.jsonl）：
    检测/确认数、各 target 的 entry/bag_bottom 3D（相机系）、confidence/status、
    mask_depth_ratio、掩膜内深度中位与稳健散布、
    全图有效占比/中位、detect/seg/geom 耗时
  - 每 15 帧存 debug 叠加图、每 10 帧存 depth+mask npz（离线粗糙度复算）
用法: collect.py <run_tag> <duration_s>
  run_tag 仅允许 [A-Za-z0-9_-]（如 hh4 / s3way），产物根固定 /tmp/e2e_live/<run_tag>。
"""
import json
import os
import re
import struct
import sys
import time

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from peach_harvester.vision.scene_perception.params import ScenePerceptionParams
from peach_harvester.vision.scene_perception.pipeline import (
    PerceptionPipeline, SyncedRgbd)
from peach_harvester.yaml_params import (dict_to_ns, expand_share,
                                         load_ros_parameters, package_yaml)

CAM_FRAME = 'camera_color_optical_frame'
K = dict(fx=466.17, fy=465.56, cx=326.07, cy=244.79, width=640, height=480)
FALLBACK_BAG_ROI = (350, 130, 620, 400)
OUT_ROOT = '/tmp/e2e_live'
TAG_RE = re.compile(r'\A[A-Za-z0-9_-]{1,32}\Z')


def log(msg):
    print(f'[collect] {msg}', file=sys.stderr, flush=True)


class _NodeClock:
    def __init__(self, node):
        self._node = node

    def now(self):
        return self._node.get_clock().now().nanoseconds * 1e-9


def struct_f(v):
    return struct.unpack('f', struct.pack('I', v))[0]


def cloud_msg(color, depth_mm, header):
    fx, fy, cx, cy = K['fx'], K['fy'], K['cx'], K['cy']
    m = (depth_mm > 0) & (depth_mm >= 300) & (depth_mm <= 1500)
    vs, us = np.where(m[::2, ::2])
    vs, us = vs * 2, us * 2
    z = depth_mm[vs, us].astype(np.float64) / 1000.0
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    bgr = color[vs, us]
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    pts = []
    for xi, yi, zi, (bb, gg, rr) in zip(x, y, z, bgr):
        rgb_f = struct_f((int(rr) << 16) | (int(gg) << 8) | int(bb))
        pts.append((float(xi), float(yi), float(zi), rgb_f))
    return pc2.create_cloud(header, fields, pts)


def img_msg(bgr, stamp):
    m = Image()
    m.header.frame_id = CAM_FRAME
    m.header.stamp = stamp
    m.height, m.width = bgr.shape[:2]
    m.encoding = 'bgr8'
    m.step = m.width * 3
    m.data = bgr.tobytes()
    return m


def depth_jet(dm):
    """0.3–1.5m 窗 JET 上色矩阵（无效=深灰）；depth_msg 与 GUI composite 共用."""
    m = cv2.inRange(dm, 300, 1500)
    vis = dm.astype(np.float32)
    lo, hi = 300.0, 1500.0
    n = np.clip((vis - lo) / (hi - lo), 0, 1)
    out = cv2.applyColorMap((n * 255).astype(np.uint8), cv2.COLORMAP_JET)
    out[m == 0] = (40, 40, 40)
    return out


def depth_msg(dm, stamp):
    """可视化深度通道：0.3–1.5m 窗 JET 上色（bgr8），无效=深灰（rviz Image 直显）."""
    out = depth_jet(dm)
    mm = Image()
    mm.header.frame_id = CAM_FRAME
    mm.header.stamp = stamp
    mm.height, mm.width = out.shape[:2]
    mm.encoding = 'bgr8'
    mm.step = mm.width * 3
    mm.data = out.tobytes()
    return mm


class Collector(Node):
    def __init__(self, run_tag, duration):
        super().__init__('perception_ab_collector')
        if not TAG_RE.match(run_tag):
            raise SystemExit(f'bad run_tag {run_tag!r}')
        self.outdir = os.path.join(OUT_ROOT, run_tag)
        self.img_dir = os.path.join(self.outdir, 'img')
        self.npz_dir = os.path.join(self.outdir, 'npz')
        self.duration = duration
        os.makedirs(self.img_dir, exist_ok=True)
        os.makedirs(self.npz_dir, exist_ok=True)
        self.n_proc = 0
        # E2E_GUI=1 时开全尺寸 composite 窗（感知叠加图|彩色|深度JET 各 640x480
        # 横排）供 ffmpeg x11grab 直录——rviz 内嵌 Image 面板被缩到 ~300px 宽，
        # 2px 框线/掩膜轮廓在视频里不可见（09-21 末轮翻车主因）
        self.gui = os.environ.get('E2E_GUI') == '1'
        self.gui_placed = False
        latch = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.pub_debug = self.create_publisher(Image, '/demo/debug', latch)
        self.pub_cloud = self.create_publisher(PointCloud2, '/demo/cloud', latch)
        self.pub_markers = self.create_publisher(MarkerArray, '/demo/cyl_markers', latch)
        self.pub_color = self.create_publisher(Image, '/demo/color', latch)
        self.pub_depth = self.create_publisher(Image, '/demo/depth', latch)

        spec = expand_share(load_ros_parameters(
            package_yaml('peach_harvester', 'scene_perception.yaml'),
            'peach_scene_perception_node'))
        params = ScenePerceptionParams.from_params(dict_to_ns(spec))
        # A/B 采集要叠加图（检测框/掩膜/袋轴）直显 rviz 感知叠加图面板；
        # 生产 yaml 默认已 true，采集器仍显式钉死以免被覆盖
        params.publish_debug_image = True
        log('构造 PerceptionPipeline（YOLO+MobileSAM 上 GPU）…')
        t0 = time.time()
        self.pipeline = PerceptionPipeline.from_params(params, _NodeClock(self))
        log(f'管线就绪 {time.time() - t0:.1f}s')
        orig_detect = self.pipeline.engine.detect

        def detect_with_fallback(image):
            dets = orig_detect(image)
            if not dets:
                dets = [dict(bbox=list(FALLBACK_BAG_ROI), class_id=0, conf=0.90)]
            return dets

        self.pipeline.engine.detect = detect_with_fallback

        self.latest = None
        self.t_start = None
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        ts = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, '/camera/color/image_raw'),
             Subscriber(self, Image, '/camera/depth/image_raw')],
            queue_size=10, slop=0.05)
        ts.registerCallback(self.on_pair)
        self.timer = self.create_timer(0.5, self.tick)
        log('等待相机流…')

    def on_pair(self, color, depth):
        rgb = np.frombuffer(color.data, np.uint8).reshape(
            color.height, color.width, 3).copy()
        dmm = np.frombuffer(depth.data, np.uint16).reshape(
            depth.height, depth.width).astype(np.float64) * 0.25
        dmm = np.round(dmm).astype(np.uint16)
        self.latest = (rgb, dmm, depth.header.stamp)

    def tick(self):
        if self.t_start is None:
            self.t_start = time.time()
        if time.time() - self.t_start > self.duration:
            log(f'时长到（{self.duration}s，{self.n_proc} 帧），退出')
            rclpy.shutdown()
            return
        if self.latest is None:
            return
        rgb, dmm, stamp = self.latest
        frame = SyncedRgbd(
            rgb=rgb, depth=dmm, K=K,
            cam_frame=CAM_FRAME, out_frame=CAM_FRAME,
            geometry_stamp=stamp,
            img_header=Header(stamp=stamp, frame_id=CAM_FRAME),
            header=Header(stamp=stamp, frame_id=CAM_FRAME),
            T_out_cam=np.eye(4), tf_status='ok', gravity_hint=None)
        result = self.pipeline.process(frame)
        if result is None:
            return
        self.n_proc += 1
        now = self.get_clock().now().to_msg()
        snap = self.pipeline.timing.snapshot()

        valid = (dmm > 0) & (dmm >= 300) & (dmm <= 1500)
        rec = dict(t=round(time.time() - self.t_start, 3),
                   stamp=stamp.sec * 10**9 + stamp.nanosec,
                   n_proc=self.n_proc,
                   n_kept=len(result.kept),
                   n_conf=len(result.candidates.candidates),
                   valid_ratio=round(float(valid.mean()), 4),
                   depth_med=int(np.median(dmm[valid])) if valid.any() else 0,
                   det_ms=round(snap.get('detect_ms', 0), 1),
                   seg_ms=round(snap.get('segment_ms', 0), 1),
                   geom_ms=round(snap.get('geometry_ms', 0), 1),
                   targets={})

        for tid, payload in result.harvest_payloads.items():
            cand = payload.get('candidate')
            if cand is None:
                continue
            ep = cand.entry_pose.position
            bb = cand.bag_bottom
            mdr = float(payload.get('mask_depth_ratio', 0.0) or 0.0)
            mask = payload.get('mask')
            mstat = {}
            if mask is not None and mask.any():
                mz = dmm[mask > 0].astype(np.float64)
                mz = mz[(mz >= 300) & (mz <= 1500)]
                if mz.size >= 8:
                    med = float(np.median(mz))
                    mstat = dict(n=int(mz.size), med=round(med, 1),
                                 mad=round(float(np.median(np.abs(mz - med))), 2))
            rec['targets'][str(tid)] = dict(
                conf=round(float(cand.confidence), 4),
                status=str(cand.status),
                mask_depth_ratio=round(mdr, 4),
                entry=[round(ep.x, 5), round(ep.y, 5), round(ep.z, 5)],
                bottom=[round(bb.x, 5), round(bb.y, 5), round(bb.z, 5)],
                radius_mm=round(float(cand.bag_diameter_upper_m) * 500.0, 1),
                **mstat)

        print(json.dumps(rec), flush=True)  # stdout=结构化 JSONL（启动器重定向）
        if self.n_proc % 10 == 0:
            log(f"帧 {self.n_proc}: 检测 {rec['n_kept']} 确认 {rec['n_conf']} "
                f"深度中位 {rec['depth_med']}mm")

        # /demo 发布（rviz 可视化）+ 图与 npz 存档
        debug_img = (result.debug if result.debug is not None
                     else result.debug_raw if result.debug_raw is not None else rgb)
        self.pub_debug.publish(img_msg(debug_img, now))
        self.pub_color.publish(img_msg(rgb, now))
        self.pub_depth.publish(depth_msg(dmm, now))
        self.pub_cloud.publish(cloud_msg(rgb, dmm, Header(stamp=now, frame_id=CAM_FRAME)))
        for m in result.markers.markers:
            m.header.stamp = now
        self.pub_markers.publish(result.markers)
        if self.gui:
            try:
                composite = np.hstack([debug_img, rgb, depth_jet(dmm)])
                cv2.imshow('e2e_demo', composite)
                if not self.gui_placed:
                    cv2.moveWindow('e2e_demo', 40, 60)
                    self.gui_placed = True
                    log('GUI composite 窗已开（1920x480：叠加图|彩色|深度JET）')
                cv2.waitKey(1)
            except Exception as exc:
                log(f'GUI 不可用（{exc}），继续无窗采集')
                self.gui = False
        if self.n_proc % 15 == 1:
            cv2.imwrite(os.path.join(self.img_dir, f'debug_{self.n_proc:04d}.png'),
                        debug_img)
        if self.n_proc % 10 == 1:
            np.savez_compressed(
                os.path.join(self.npz_dir, f'frame_{self.n_proc:04d}.npz'),
                depth=dmm, mask=result.mask_canvas.astype(np.uint8))


def main():
    run_tag = sys.argv[1] if len(sys.argv) > 1 else 'run'
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 70.0
    rclpy.init()
    node = Collector(run_tag, duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
