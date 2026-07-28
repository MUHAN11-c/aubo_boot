# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
"""Checkerboard detection and target-to-camera pose estimation."""

from dataclasses import dataclass

import cv2
import numpy as np

from .board import Checkerboard
from .transforms import make_transform


@dataclass(frozen=True)
class CheckerboardObservation:
    camera_from_target: np.ndarray
    corners: np.ndarray
    reprojection_rms_px: float


class CheckerboardDetector:
    def __init__(self, board=None, max_reprojection_rms_px=1.5):
        self.board = board or Checkerboard()
        # 单帧质量门: 超过该重投影 RMS 的观测直接丢弃, 不进入采样缓冲
        self.max_reprojection_rms_px = float(max_reprojection_rms_px)

    @property
    def pattern_size(self):
        return self.board.pattern_size

    def detect(self, image_bgr, camera_matrix, distortion):
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        found, corners = cv2.findChessboardCornersSB(
            gray, self.pattern_size, flags=flags)
        if not found:
            return None
        corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        camera_matrix = np.asarray(
            camera_matrix, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(distortion, dtype=np.float64)
        success, rvec, tvec = cv2.solvePnP(
            self.board.object_points,
            corners,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_SQPNP,
        )
        if not success:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(
            self.board.object_points,
            corners,
            camera_matrix,
            distortion,
            rvec,
            tvec,
        )
        rotation, _ = cv2.Rodrigues(rvec)
        projected, _ = cv2.projectPoints(
            self.board.object_points, rvec, tvec, camera_matrix, distortion)
        residual = projected.reshape(-1, 2) - corners.reshape(-1, 2)
        rms = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
        if rms > self.max_reprojection_rms_px:
            return None
        return CheckerboardObservation(
            camera_from_target=make_transform(rotation, tvec),
            corners=corners,
            reprojection_rms_px=rms,
        )

    def annotate(self, image_bgr, observation):
        annotated = image_bgr.copy()
        if observation is not None:
            cv2.drawChessboardCorners(
                annotated, self.pattern_size, observation.corners, True)
            cv2.putText(
                annotated,
                f'RMS {observation.reprojection_rms_px:.2f}px',
                (16, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            cv2.putText(
                annotated,
                f'board {self.board.columns}x{self.board.rows} not visible',
                (16, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
        return annotated
