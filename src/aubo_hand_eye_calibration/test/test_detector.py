# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
import unittest

from aubo_hand_eye_calibration.board import Checkerboard
from aubo_hand_eye_calibration.detector import CheckerboardDetector
from aubo_hand_eye_calibration.transforms import (
    inverse,
    make_transform,
    transform_error,
)
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def render_checkerboard(board, camera_matrix, camera_from_target,
                        width=960, height=720):
    """合成一张棋盘格图像: 白色背景上投影棋盘角点网格的方格面片。"""
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    # 棋盘物理角点 (比内角点多一圈)
    squares_x, squares_y = board.columns + 1, board.rows + 1
    square = board.square_size_m
    rvec, _ = cv2.Rodrigues(camera_from_target[:3, :3])
    tvec = camera_from_target[:3, 3]
    image_points = {}
    for row in range(squares_y + 1):
        for col in range(squares_x + 1):
            point = np.array([[[col * square, row * square, 0.0]]],
                             dtype=np.float64)
            projected, _ = cv2.projectPoints(
                point, rvec, tvec, camera_matrix, None)
            image_points[(row, col)] = projected.reshape(2)
    # 内角点 (c, r) 位于方格 (c, r) 与 (c+1, r+1) 之间; 棋盘方格
    # 从物理原点外扩一格, 内角点即方格顶点
    for row in range(squares_y):
        for col in range(squares_x):
            if (row + col) % 2 == 0:
                continue
            quad = np.array([
                image_points[(row, col)],
                image_points[(row, col + 1)],
                image_points[(row + 1, col + 1)],
                image_points[(row + 1, col)],
            ], dtype=np.int32)
            cv2.fillConvexPoly(image, quad, (0, 0, 0))
    return image


class DetectorTest(unittest.TestCase):
    def test_recovers_known_pose_from_synthetic_image(self):
        board = Checkerboard(columns=6, rows=5, square_size_m=0.040)
        detector = CheckerboardDetector(board)
        camera_matrix = np.array([
            [700.0, 0.0, 480.0],
            [0.0, 700.0, 360.0],
            [0.0, 0.0, 1.0],
        ])
        camera_from_target = make_transform(
            Rotation.from_euler('xyz', [0.25, -0.15, 0.1]).as_matrix(),
            [-0.12, -0.10, 1.0],
        )
        # 板原点在内角点 (0,0); 物理方格向外扩一圈
        offset = np.eye(4)
        offset[:3, 3] = [-board.square_size_m, -board.square_size_m, 0.0]
        image = render_checkerboard(
            board, camera_matrix, camera_from_target @ offset)

        observation = detector.detect(image, camera_matrix, np.zeros(5))
        self.assertIsNotNone(observation)
        translation_m, rotation_deg = transform_error(
            camera_from_target, observation.camera_from_target)
        self.assertLess(translation_m, 0.01)
        self.assertLess(rotation_deg, 1.0)
        self.assertLess(observation.reprojection_rms_px, 1.0)

    def test_returns_none_without_board(self):
        detector = CheckerboardDetector(Checkerboard(6, 5, 0.040))
        image = np.full((720, 960, 3), 128, dtype=np.uint8)
        observation = detector.detect(
            image, np.eye(3) * 700.0, np.zeros(5))
        self.assertIsNone(observation)


if __name__ == '__main__':
    unittest.main()
