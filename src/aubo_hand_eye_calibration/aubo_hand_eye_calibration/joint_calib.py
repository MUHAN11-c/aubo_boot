"""
联合内参+手眼外参求解 (纯核, 零 ROS).

输入是各视点的腕部 TF 与**检测原始角点**(不经 PnP —— PnP 依赖 K,
联合求解要摆脱它)。三段式:

  a. cv2.calibrateCamera 拟合 K/D 与逐视点位姿 (畸变 flags 与
     intrinsics_calibration.launch.py 的 --k-coefficients 2 同族);
  b. 逐视点位姿喂现有 solver.solve_hand_eye 得 X/B 初值 (五方法+MAD+精化);
  c. 联合抛光: scipy least_squares 同时优化 (fx,fy,cx,cy,k1,k2,p1,p2,
     se3(X), se3(B)) 共 20 参数, 残差为链式位姿
     T_ct = inv(X) inv(A_i) B 投影全部角点与观测之差 (px)。

不实现「calibrateCamera <-> hand-eye 交替迭代」: calibrateCamera 的
输入只有角点, 与 X 无关, 交替是退化的; 正确的联合估计是 c 段的
同时优化 (逐视位姿被链式约束消元)。
"""

from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.optimize import least_squares

from .solver import (
    _consistency,
    CalibrationSample,
    rotation_span,
    solve_hand_eye,
)
from .transforms import (
    from_se3_vector,
    inverse,
    make_transform,
    se3_vector,
)

# plumb_bob 家族: 只放 k1,k2,p1,p2 (k3 固定 0), 与 vendored 标定器
# --k-coefficients 2 的输出同构
_PLUMBOB_FLAGS = (
    cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
    | cv2.CALIB_FIX_K6)


@dataclass(frozen=True)
class JointObservation:
    # T_base_gripper: 腕部 TF (该帧采集时刻)
    base_from_gripper: np.ndarray
    # 检测原始角点 (N,2) px
    corners: np.ndarray
    sample_id: str = ''


@dataclass
class JointResult:
    camera_matrix: np.ndarray       # 3x3
    distortion: np.ndarray          # plumb_bob 5 系数
    gripper_from_camera: np.ndarray
    base_from_target: np.ndarray
    reprojection_rms_px: float
    translation_rms_m: float
    rotation_rms_deg: float
    rotation_span_deg: float
    image_size: tuple
    init_method: str
    polish: dict
    # 逐视明细: {sample_id, rms_px, accepted}
    per_view: list = field(default_factory=list)
    passed: bool = False
    failures: list = field(default_factory=list)

    def intrinsics_document(self, camera_name='camera_color'):
        """等价 camera_calibration_parsers OST 文档 (apply_intrinsics 可直接落盘)."""
        k = self.camera_matrix
        d = np.asarray(self.distortion, dtype=np.float64).reshape(-1)
        return {
            'camera_name': camera_name,
            'image_width': int(self.image_size[0]),
            'image_height': int(self.image_size[1]),
            'distortion_model': 'plumb_bob',
            'camera_matrix': {
                'rows': 3, 'cols': 3,
                'data': [float(value) for value in k.reshape(-1)]},
            'distortion_coefficients': {
                'rows': 1, 'cols': int(d.size),
                'data': [float(value) for value in d]},
            'rectification_matrix': {
                'rows': 3, 'cols': 3,
                'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
            'projection_matrix': {
                'rows': 3, 'cols': 4,
                'data': [
                    float(k[0, 0]), 0.0, float(k[0, 2]), 0.0,
                    0.0, float(k[1, 1]), float(k[1, 2]), 0.0,
                    0.0, 0.0, 1.0, 0.0]},
        }


def _pnp_pose(object_points, corners, camera_matrix, distortion):
    rotation, _ = cv2.Rodrigues(np.zeros(3))
    success, rvec, tvec = cv2.solvePnP(
        object_points, corners, camera_matrix, distortion,
        flags=cv2.SOLVEPNP_SQPNP)
    if not success:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(
        object_points, corners, camera_matrix, distortion, rvec, tvec)
    rotation, _ = cv2.Rodrigues(rvec)
    return make_transform(rotation, tvec)


def _unpack_parameters(vector):
    camera_matrix = np.array([
        [vector[0], 0.0, vector[2]],
        [0.0, vector[1], vector[3]],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    # k3 固定 0 (plumb_bob, 与 _PLUMBOB_FLAGS 一致)
    distortion = np.array(
        [vector[4], vector[5], vector[6], vector[7], 0.0])
    gripper_from_camera = from_se3_vector(vector[8:14])
    base_from_target = from_se3_vector(vector[14:20])
    return camera_matrix, distortion, gripper_from_camera, base_from_target


def solve_joint_intrinsics_hand_eye(
    observations,
    board,
    image_size,
    K0=None,
    D0=None,
    joint_max_reprojection_rms_px=0.8,
    max_translation_rms_m=0.003,
    max_rotation_rms_deg=0.5,
    min_rotation_span_deg=20.0,
    min_views=6,
    huber_px=1.0,
    max_nfev=200,
):
    """联合求解; 质量门不通过时返回 passed=False 与 failures (不抛异常)."""
    observations = list(observations)
    if len(observations) < min_views:
        raise ValueError(
            f'joint solve needs at least {min_views} views, '
            f'got {len(observations)}')
    object_points = np.asarray(
        board.object_points, dtype=np.float32).reshape(-1, 1, 3)
    image_points = [
        np.asarray(o.corners, dtype=np.float32).reshape(-1, 1, 2)
        for o in observations
    ]

    # a. calibrateCamera: K/D 与逐视位姿 (有 K0 时作为初值提示)
    flags = _PLUMBOB_FLAGS
    camera_init = None
    if K0 is not None:
        camera_init = np.asarray(K0, dtype=np.float64).reshape(3, 3).copy()
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    try:
        _, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            [object_points] * len(observations), image_points, image_size,
            camera_init, None, flags=flags)
    except cv2.error as error:
        raise RuntimeError(f'calibrateCamera failed: {error}') from error
    distortion = np.ravel(distortion)[:5].astype(np.float64)

    # b. 逐视位姿 -> 现有 hand-eye 求解器给 X/B 初值
    samples = []
    for index, observation in enumerate(observations):
        rotation, _ = cv2.Rodrigues(rvecs[index])
        samples.append(CalibrationSample(
            observation.base_from_gripper,
            make_transform(rotation, np.ravel(tvecs[index])),
            0.0,
            observation.sample_id or f'view_{index:02d}',
        ))
    init = solve_hand_eye(samples, min_samples=3, method='auto')
    camera_init, target_init = (
        init.gripper_from_camera, init.base_from_target)

    # c. 联合抛光: 20 参数, 残差 = 链式投影与观测角点差 (px)
    initial = np.r_[
        camera_matrix[0, 0], camera_matrix[1, 1],
        camera_matrix[0, 2], camera_matrix[1, 2],
        distortion[0], distortion[1], distortion[2], distortion[3],
        se3_vector(camera_init), se3_vector(target_init),
    ]

    def residual(vector):
        k, d, x, b = _unpack_parameters(vector)
        x_inverse = inverse(x)
        values = []
        for observation in observations:
            camera_from_target = (
                x_inverse @ inverse(observation.base_from_gripper) @ b)
            rotation, _ = cv2.Rodrigues(camera_from_target[:3, :3])
            projected, _ = cv2.projectPoints(
                object_points, rotation, camera_from_target[:3, 3], k, d)
            values.append(
                (projected.reshape(-1, 2)
                 - np.asarray(observation.corners, dtype=np.float64)
                 .reshape(-1, 2)).ravel())
        return np.concatenate(values)

    initial_cost = 0.5 * float(np.sum(residual(initial) ** 2))
    optimized = least_squares(
        residual, initial, loss='huber', f_scale=huber_px,
        max_nfev=max_nfev)
    polish = {
        'converged': bool(optimized.success),
        'initial_cost': initial_cost,
        'final_cost': float(optimized.cost),
        'nfev': int(optimized.nfev),
    }
    if optimized.success and optimized.cost <= initial_cost:
        (camera_matrix, distortion, camera_final, target_final) = (
            _unpack_parameters(optimized.x))
    else:
        # 抛光未收敛或劣于初值: 回退 a/b 段结果, 由质量门裁决
        camera_matrix = np.asarray(camera_matrix, dtype=np.float64)
        distortion = np.asarray(distortion, dtype=np.float64)
        camera_final, target_final = camera_init, target_init

    # 终态指标: 逐视/总重投影 RMS (链式位姿) + 手眼链一致性 (独立 PnP)
    per_view = []
    squared_total, count_total = 0.0, 0
    consistency_samples = []
    for observation in observations:
        camera_from_target = (
            inverse(camera_final)
            @ inverse(observation.base_from_gripper) @ target_final)
        rotation, _ = cv2.Rodrigues(camera_from_target[:3, :3])
        projected, _ = cv2.projectPoints(
            object_points, rotation, camera_from_target[:3, 3],
            camera_matrix, distortion)
        difference = (projected.reshape(-1, 2)
                      - np.asarray(observation.corners,
                                   dtype=np.float64).reshape(-1, 2))
        squared = np.sum(difference ** 2, axis=1)
        rms = float(np.sqrt(np.mean(squared)))
        squared_total += float(np.sum(squared))
        count_total += squared.size
        per_view.append({
            'sample_id': observation.sample_id,
            'rms_px': rms,
            'accepted': None,
        })
        resection = _pnp_pose(
            object_points, np.asarray(
                observation.corners, dtype=np.float32).reshape(-1, 1, 2),
            camera_matrix, distortion)
        if resection is not None:
            consistency_samples.append(CalibrationSample(
                observation.base_from_gripper, resection, rms,
                observation.sample_id))
    total_rms = float(np.sqrt(squared_total / max(count_total, 1)))
    rms_values = sorted(view['rms_px'] for view in per_view)
    median_rms = rms_values[len(rms_values) // 2]
    for view in per_view:
        view['accepted'] = bool(view['rms_px'] <= max(
            3.0 * median_rms, joint_max_reprojection_rms_px))

    _, errors = _consistency(consistency_samples, camera_final)
    translation_rms = float(np.sqrt(np.mean(errors[:, 0] ** 2)))
    rotation_rms = float(np.sqrt(np.mean(errors[:, 1] ** 2)))
    span = rotation_span(observations)

    failures = []
    if len(consistency_samples) < len(observations):
        failures.append(
            f'PnP resection failed on '
            f'{len(observations) - len(consistency_samples)} views')
    if total_rms > joint_max_reprojection_rms_px:
        failures.append(
            f'joint reprojection RMS {total_rms:.3f}px exceeds '
            f'{joint_max_reprojection_rms_px:.3f}px')
    if translation_rms > max_translation_rms_m:
        failures.append(
            f'translation RMS {translation_rms:.6f}m exceeds '
            f'{max_translation_rms_m:.6f}m')
    if rotation_rms > max_rotation_rms_deg:
        failures.append(
            f'rotation RMS {rotation_rms:.3f}deg exceeds '
            f'{max_rotation_rms_deg:.3f}deg')
    if span < min_rotation_span_deg:
        failures.append(
            f'rotation span {span:.1f}deg below '
            f'{min_rotation_span_deg:.1f}deg')

    return JointResult(
        camera_matrix=camera_matrix,
        distortion=distortion,
        gripper_from_camera=camera_final,
        base_from_target=target_final,
        reprojection_rms_px=total_rms,
        translation_rms_m=translation_rms,
        rotation_rms_deg=rotation_rms,
        rotation_span_deg=span,
        image_size=(int(image_size[0]), int(image_size[1])),
        init_method=init.method,
        polish=polish,
        per_view=per_view,
        passed=not failures,
        failures=failures,
    )
