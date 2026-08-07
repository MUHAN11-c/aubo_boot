"""
合成圆柱场景共享工具：位姿生成 / 点采样渲染器 / 解析 GT（测试专用）.

场景：base 系下圆柱（轴 [0,0,1]，中心 (0.40,0,0.30)，r=0.035 m，h=0.12 m）；
5 个环绕虚拟相机（0.55 m，相邻平移 ≈43 mm、旋转 ≈12.8°，落在默认
view_filter 窗口内）。点采样 z-buffer 渲染器与解析射线求交 GT 都独立于
cloud_builder 实现（避免循环论证）。
"""
import numpy as np

# ── 场景常量 ────────────────────────────────────────────────
CYL_CENTER = np.array([0.40, 0.0, 0.30])  # 圆柱轴心 [m]
CYL_R = 0.035                             # 半径 [m]
CYL_HALF_H = 0.06                         # 半高 [m]（h=0.12）
K = {'fx': 500.0, 'fy': 500.0, 'cx': 320.0, 'cy': 240.0}
IMG_W, IMG_H = 640, 480
CAM_DIST = 0.55        # 相机到轴心距离 [m]（0.5~0.7 区间）
AZIMUTH_STEP_DEG = 4.5  # 环绕步长 → 相邻平移 ≈43 mm
ROLL_STEP_DEG = 12.0    # 绕光轴滚转步长 → 相邻旋转 ≈12.8°


def look_at_pose(eye, target, roll_deg):
    """构造 base←camera 位姿：+Z 指向 target，再绕光轴滚转 roll_deg."""
    forward = np.asarray(target, dtype=float) - np.asarray(eye, dtype=float)
    forward /= np.linalg.norm(forward)
    up = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(forward, up)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(forward, x_axis)
    R = np.column_stack([x_axis, y_axis, forward])  # 列=相机轴在 base 系
    roll = np.radians(roll_deg)
    R_roll = np.array([[np.cos(roll), -np.sin(roll), 0.0],
                       [np.sin(roll), np.cos(roll), 0.0],
                       [0.0, 0.0, 1.0]])
    T = np.eye(4)
    T[:3, :3] = R @ R_roll  # 绕相机局部 z（光轴）滚转，目标保持在画面中心
    T[:3, 3] = eye
    return T


def make_poses():
    """5 个环绕位姿：方位角 ±9° 内步进 4.5°，滚转步进 12°."""
    poses = []
    for i in range(5):
        az = np.radians(-9.0 + i * AZIMUTH_STEP_DEG)
        eye = CYL_CENTER + CAM_DIST * np.array([np.cos(az), np.sin(az), 0.0])
        poses.append(look_at_pose(eye, CYL_CENTER, roll_deg=i * ROLL_STEP_DEG))
    return poses


def rigid_inv(T):
    """4×4 刚体逆（本模块独立实现，不依赖被测链）."""
    R, t = T[:3, :3], T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def cylinder_samples(n_theta=720, n_z=240):
    """圆柱侧面致密参数采样（base 系）+ 外法线；底面不采（相机同高看不到）."""
    theta = np.linspace(0.0, 2.0 * np.pi, n_theta, endpoint=False)
    zz = np.linspace(-CYL_HALF_H, CYL_HALF_H, n_z)
    T_grid, Z_grid = np.meshgrid(theta, zz)
    pts = np.column_stack([
        CYL_CENTER[0] + CYL_R * np.cos(T_grid).ravel(),
        CYL_CENTER[1] + CYL_R * np.sin(T_grid).ravel(),
        CYL_CENTER[2] + Z_grid.ravel()])
    normals = np.column_stack([np.cos(T_grid).ravel(),
                               np.sin(T_grid).ravel(),
                               np.zeros(T_grid.size)])
    return pts, normals


def render_depth(T_base_camera, pts):
    """
    迷你深度渲染器：表面采样点 → 相机系 → 像素 z-buffer 取最近.

    输出 uint16 毫米深度图（0 = 无效）。
    """
    T_camera_base = rigid_inv(T_base_camera)
    pc = (T_camera_base[:3, :3] @ pts.T).T + T_camera_base[:3, 3]
    z = pc[:, 2]
    ok = z > 0.01
    u = K['fx'] * pc[:, 0] / z + K['cx']
    v = K['fy'] * pc[:, 1] / z + K['cy']
    ok &= (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
    u = np.floor(u[ok]).astype(int)
    v = np.floor(v[ok]).astype(int)
    z = z[ok]
    buf = np.full(IMG_W * IMG_H, np.inf)
    np.minimum.at(buf, v * IMG_W + u, z)  # z-buffer：像素内取最近点
    buf = buf.reshape(IMG_H, IMG_W)
    depth = np.zeros((IMG_H, IMG_W), dtype=np.uint16)
    hit = np.isfinite(buf)
    depth[hit] = np.clip(np.round(buf[hit] * 1000.0), 1, 65534).astype(np.uint16)
    return depth


def raytrace_cylinder_cloud():
    """解析射线-圆柱求交：逐像素精确命中点云（base 系，像素均匀 GT）."""
    us = np.arange(IMG_W) + 0.5
    vs = np.arange(IMG_H) + 0.5
    U, V = np.meshgrid(us, vs)
    dirs = np.stack([(U - K['cx']) / K['fx'],
                     (V - K['cy']) / K['fy'],
                     np.ones_like(U)], axis=-1).reshape(-1, 3)
    hits = []
    for T in make_poses():
        T_camera_base = rigid_inv(T)
        center_c = T_camera_base[:3, :3] @ CYL_CENTER + T_camera_base[:3, 3]
        axis_c = T_camera_base[:3, :3] @ np.array([0.0, 0.0, 1.0])
        # 射线（原点为相机光心）与有限圆柱求交：垂直轴向分量列二次方程
        d_perp = dirs - (dirs @ axis_c)[:, None] * axis_c
        oc = -center_c
        oc_perp = oc - (oc @ axis_c) * axis_c
        a = (d_perp * d_perp).sum(axis=1)
        b = 2.0 * (d_perp * oc_perp).sum(axis=1)
        c = float(oc_perp @ oc_perp) - CYL_R ** 2
        disc = b * b - 4.0 * a * c
        hit = disc > 0
        t = np.full(len(dirs), np.nan)
        t[hit] = (-b[hit] - np.sqrt(disc[hit])) / (2.0 * a[hit])  # 近根
        valid = hit & (t > 0.01)
        p_cam = t[valid, None] * dirs[valid]
        axial = (p_cam - center_c) @ axis_c
        p_cam = p_cam[np.abs(axial) <= CYL_HALF_H]
        hits.append((T[:3, :3] @ p_cam.T).T + T[:3, 3])
    return np.vstack(hits)


def visible_surface_cloud(n_theta=1440, n_z=480):
    """
    致密采样中「对任一相机可见」的侧面点集（面积均匀 GT）.

    可见性 = 法向朝向某相机（dot(n, eye-p) > 0）且投影在该相机视锥内；
    凸圆柱无自遮挡歧义。TSDF 提取云近似面积均匀分布，以此作其质心 GT。
    """
    pts, normals = cylinder_samples(n_theta, n_z)
    visible = np.zeros(len(pts), dtype=bool)
    for T in make_poses():
        eye = T[:3, 3]
        facing = (normals * (eye - pts)).sum(axis=1) > 0.0
        T_camera_base = rigid_inv(T)
        pc = (T_camera_base[:3, :3] @ pts.T).T + T_camera_base[:3, 3]
        z = pc[:, 2]
        u = K['fx'] * pc[:, 0] / z + K['cx']
        v = K['fy'] * pc[:, 1] / z + K['cy']
        in_frustum = ((z > 0.01) & (u >= 0) & (u < IMG_W)
                      & (v >= 0) & (v < IMG_H))
        visible |= facing & in_frustum
    return pts[visible]
