#!/usr/bin/env python3
"""离线 SGBM 深度验证（2026-09-16）：单图案双目 vs 设备 18 图案深度.

前置：stereo_grab ir 30 <dir_ir> 与 stereo_grab depth 30 <dir_dev> 已采集，
且 IR_DIR/DEV_DIR 指向对应目录（calib.txt 在 ir 目录内）。
指标：极线验证(ORB 比值筛选)、时域噪声 k=1/5/10、有效率、与设备深度一致性。
"""
import glob
import sys

import cv2
import numpy as np

IR_DIR = '/tmp/percipio_fps_test/ir_pairs'
DEV_DIR = '/tmp/percipio_fps_test/dev_depth'
ROI = (slice(240, 720), slice(320, 960))


def parse_calib(path):
    blocks, cur = {}, None
    for line in open(path):
        parts = line.split()
        if not parts:
            continue
        if parts[0] == 'calib':
            cur = {'status': int(parts[3]), 'w': int(parts[5].split('x')[0])}
            blocks[parts[1]] = cur
        elif parts[0] in ('intrinsic', 'extrinsic', 'distortion'):
            cur[parts[0]] = np.array([float(v) for v in parts[1:]])
    return blocks


def main():
    calib = parse_calib(f'{IR_DIR}/calib.txt')
    K1 = calib['left_ir']['intrinsic'].reshape(3, 3)
    K2 = calib['right_ir']['intrinsic'].reshape(3, 3)
    D1 = calib['left_ir']['distortion'][:8]
    D2 = calib['right_ir']['distortion'][:8]
    E = calib['right_ir']['extrinsic'].reshape(4, 4)
    R = E[:3, :3]
    T = E[:3, 3]
    print(f'baseline |T| = {np.linalg.norm(T):.1f} mm  (t={T.round(2)})')
    size = (1280, 960)
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        K1, D1, K2, D2, size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    m1x, m1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, size, cv2.CV_32FC1)
    m2x, m2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, size, cv2.CV_32FC1)
    f_rect, b_rect = P1[0, 0], -P2[0, 3] / P1[0, 0]
    print(f'rectified f={f_rect:.1f}px B={b_rect:.1f}mm')

    lfiles = sorted(glob.glob(f'{IR_DIR}/L*.pgm'))
    rfiles = sorted(glob.glob(f'{IR_DIR}/R*.pgm'))
    n = min(len(lfiles), len(rfiles))
    print(f'{n} pairs')

    # 极线验证：ORB + 比值筛选（散斑图裸 BFMatcher 全是错配，勿用）
    l0 = cv2.imread(lfiles[0], cv2.IMREAD_GRAYSCALE)
    r0 = cv2.imread(rfiles[0], cv2.IMREAD_GRAYSCALE)
    l0r = cv2.remap(l0, m1x, m1y, cv2.INTER_LINEAR)
    r0r = cv2.remap(r0, m2x, m2y, cv2.INTER_LINEAR)
    orb = cv2.ORB_create(2000)
    kp1, des1 = orb.detectAndCompute(l0r, None)
    kp2, des2 = orb.detectAndCompute(r0r, None)
    if des1 is not None and des2 is not None:
        knn = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(des1, des2, k=2)
        good = [m for m, s in knn if m.distance < 0.7 * s.distance]
        good = [m for m in good
                if 30 < abs(kp1[m.queryIdx].pt[0] - kp2[m.trainIdx].pt[0]) < 300]
        dys = np.array([kp1[m.queryIdx].pt[1] - kp2[m.trainIdx].pt[1] for m in good])
        if len(dys) > 10:
            print(f'epipolar check(ratio 0.7): {len(dys)} matches, '
                  f'|dy| median={np.median(np.abs(dys)):.2f}px p95={np.percentile(np.abs(dys), 95):.2f}px')
        cv2.imwrite('/tmp/rect_pair.png', np.hstack([l0r, r0r]))

    sgbm = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=256, blockSize=5,
        P1=200, P2=3200, disp12MaxDiff=5, preFilterCap=31,
        uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_HH)

    depths = []
    valids = []
    for lf, rf in zip(lfiles[:n], rfiles[:n]):
        li = cv2.imread(lf, cv2.IMREAD_GRAYSCALE)
        ri = cv2.imread(rf, cv2.IMREAD_GRAYSCALE)
        lr = cv2.remap(li, m1x, m1y, cv2.INTER_LINEAR)
        rr = cv2.remap(ri, m2x, m2y, cv2.INTER_LINEAR)
        disp = sgbm.compute(lr, rr).astype(np.float32) / 16.0
        with np.errstate(divide='ignore', invalid='ignore'):
            z = f_rect * b_rect / disp
        z[disp <= 0] = np.nan
        depths.append(z)
        valids.append(np.isfinite(z[ROI]))
    zstack = np.stack(depths)
    vstack = np.stack(valids)
    print(f'\nSGBM(single pattern): valid rate in ROI = {vstack.mean()*100:.1f}%')

    roi = zstack[:, ROI[0], ROI[1]]
    per_pix_std = np.nanstd(roi, axis=0)
    print(f'temporal noise k=1 : median={np.nanmedian(per_pix_std):.2f}mm '
          f'p90={np.nanpercentile(per_pix_std, 90):.2f}mm')
    med_scene = np.nanmedian(roi)
    print(f'scene median depth = {med_scene:.0f}mm')

    for k in (5, 10):
        m = n // k * k
        blocks = zstack[:m].reshape(m // k, k, *zstack.shape[1:])
        zavg = np.nanmean(blocks, axis=1)[:, ROI[0], ROI[1]]
        std_avg = np.nanstd(zavg, axis=0)
        print(f'temporal noise k={k:2d}: median={np.nanmedian(std_avg):.2f}mm '
              f'p90={np.nanpercentile(std_avg, 90):.2f}mm')

    dfiles = sorted(glob.glob(f'{DEV_DIR}/D*.pgm'))
    devs = []
    for df in dfiles:
        d = cv2.imread(df, cv2.IMREAD_UNCHANGED).astype(np.float32)
        z = np.where(d > 0, d * 0.25, np.nan)
        devs.append(z)
    dstack = np.stack(devs)
    h, w = dstack.shape[1:]
    droi = dstack[:, h // 4: 3 * h // 4, w // 4: 3 * w // 4]
    dstd = np.nanstd(droi, axis=0)
    dvalid = np.isfinite(droi).mean()
    print(f'\nDevice(18 patterns): valid rate in ROI = {dvalid*100:.1f}% '
          f'temporal noise median={np.nanmedian(dstd):.2f}mm '
          f'p90={np.nanpercentile(dstd, 90):.2f}mm')
    print(f'device scene median depth = {np.nanmedian(droi):.0f}mm')
    print(f'\nmedian agreement: sgbm {med_scene:.0f}mm vs device '
          f'{np.nanmedian(droi):.0f}mm, diff {med_scene - np.nanmedian(droi):.0f}mm')

    vis = np.nan_to_num(zstack[0], nan=0)
    vis = np.clip((vis - 300) / 700 * 255, 0, 255).astype(np.uint8)
    cv2.imwrite('/tmp/sgbm_depth_vis.png', vis)


if __name__ == '__main__':
    sys.exit(main())
