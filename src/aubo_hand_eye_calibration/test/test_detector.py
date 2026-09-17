"""Zero-ROS synthetic-image tests for the checkerboard detector."""

from aubo_hand_eye_calibration.board import Checkerboard
from aubo_hand_eye_calibration.detector import CheckerboardDetector
from aubo_hand_eye_calibration.transforms import inverse, make_transform, transform_error
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

CAMERA_MATRIX = np.array([
    [800.0, 0.0, 640.0],
    [0.0, 800.0, 480.0],
    [0.0, 0.0, 1.0],
])
DISTORTION = np.zeros(5)


def render(board, camera_from_target, width=1280, height=960):
    """
    按已知内参/位姿渲染整块棋盘格 (灰底衬出边界).

    方块索引 -1..columns-1 / -1..rows-1 (共 (columns+1)x(rows+1) 个),
    使内角点阵恰好落在 [0,(columns-1)s]x[0,(rows-1)s], 与
    board.object_points 的网格几何一致, 不多画边框圈 (避免更大的
    角点阵造成检测歧义)。
    """
    image = np.full((height, width, 3), 128, dtype=np.uint8)
    square = board.square_size_m
    squares = []
    for i in range(-1, board.columns):
        for j in range(-1, board.rows):
            color = 0 if (i + j) % 2 else 255
            squares.append((color, np.array([
                [i * square, j * square, 0.0],
                [(i + 1) * square, j * square, 0.0],
                [(i + 1) * square, (j + 1) * square, 0.0],
                [i * square, (j + 1) * square, 0.0],
            ], dtype=np.float32)))
    points = np.vstack([corners for _, corners in squares])
    projected, _ = cv2.projectPoints(
        points,
        camera_from_target[:3, :3],
        camera_from_target[:3, 3],
        CAMERA_MATRIX,
        DISTORTION,
    )
    projected = np.round(projected.reshape(-1, 2)).astype(np.int32)
    for index, (color, _) in enumerate(squares):
        polygon = projected[index * 4:(index + 1) * 4]
        cv2.fillPoly(image, [polygon], (color, color, color))
    return image


def pose_candidates(board):
    """真值位姿与 180° 角点序歧义下的等价位姿."""
    truth = make_transform(np.eye(3), [-0.12, -0.08, 0.35])
    relabel = make_transform(
        Rotation.from_euler('z', 180.0, degrees=True).as_matrix(),
        [
            (board.columns - 1) * board.square_size_m,
            (board.rows - 1) * board.square_size_m,
            0.0,
        ],
    )
    return truth, truth @ inverse(relabel)


def test_detect_recovers_rendered_pose():
    board = Checkerboard()
    image = render(board, pose_candidates(board)[0])
    detector = CheckerboardDetector(board, max_reprojection_rms_px=1.5)
    observation = detector.detect(image, CAMERA_MATRIX, DISTORTION)
    assert observation is not None
    assert observation.reprojection_rms_px < 0.5
    truth, flipped = pose_candidates(board)
    direct = transform_error(truth, observation.camera_from_target)
    ambiguous = transform_error(flipped, observation.camera_from_target)
    recovered = min(direct, ambiguous)
    # 渲染投影取整引入 ~0.5px 量化 (z=0.35 下 ≈0.22mm), 门限放 1mm/0.5°
    assert recovered[0] < 1e-3 and recovered[1] < 0.5, (direct, ambiguous)


def test_detect_rejects_wrong_board_geometry():
    board = Checkerboard()
    other = Checkerboard(columns=9, rows=6)
    image = render(other, make_transform(np.eye(3), [-0.1, -0.06, 0.25]))
    detector = CheckerboardDetector(board, max_reprojection_rms_px=1.5)
    assert detector.detect(image, CAMERA_MATRIX, DISTORTION) is None


def test_per_frame_gate_drops_high_rms_observation():
    board = Checkerboard()
    image = render(board, pose_candidates(board)[0])
    # 阈值压到 0: 任何实测 RMS (>0) 都被单帧门丢弃
    detector = CheckerboardDetector(board, max_reprojection_rms_px=0.0)
    assert detector.detect(image, CAMERA_MATRIX, DISTORTION) is None


def test_annotate_reports_visibility():
    board = Checkerboard()
    detector = CheckerboardDetector(board)
    blank = np.full((480, 640, 3), 128, dtype=np.uint8)
    annotated = detector.annotate(blank, None)
    assert annotated is not blank
