"""session 落盘的防覆盖与写盘失败测试."""
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from peach_reconstruction_ros2.session_io import save_session
import yaml


def _frame():
    """构造最小 CapturedFrame 鸭子类型."""
    return SimpleNamespace(
        rgb=np.zeros((2, 2, 3), dtype=np.uint8),
        depth_mm=np.ones((2, 2), dtype=np.uint16),
        stamp=1.0,
        camera_K={
            'width': 2, 'height': 2,
            'fx': 100.0, 'fy': 100.0, 'cx': 1.0, 'cy': 1.0,
        },
        T_base_camera=np.eye(4),
        T_base_camera_fk=np.eye(4),
        camera_position_base=np.zeros(3),
        valid_depth_ratio=1.0,
        diagnostic_flags=[],
        registration={
            'mode': 'icp', 'fitness': 0.8, 'rmse_m': 0.003,
            'translation_m': 0.002, 'rotation_deg': 0.2,
            'reason': 'accepted',
        },
    )


class SessionIoTest(unittest.TestCase):
    """会话目录唯一性与 OpenCV 错误传播."""

    def test_back_to_back_saves_use_distinct_directories(self):
        """连续保存不能覆盖前一次会话."""
        with tempfile.TemporaryDirectory() as root:
            first = save_session(root, [_frame()], {})
            second = save_session(root, [_frame()], {})
            self.assertNotEqual(first, second)
            self.assertTrue((first / 'frame_00_rgb.png').is_file())
            self.assertTrue((second / 'frame_00_rgb.png').is_file())
            pose = yaml.safe_load((first / 'frame_00_T_base_camera.yaml')
                                  .read_text(encoding='utf-8'))
            self.assertIn('T_base_camera_fk', pose)
            self.assertIn('T_base_camera_used', pose)
            self.assertEqual(pose['registration']['mode'], 'icp')

    def test_png_write_failure_raises(self):
        """cv2.imwrite 返回 false 时必须向调用方报告失败."""
        with tempfile.TemporaryDirectory() as root:
            with patch(
                    'peach_reconstruction_ros2.session_io.cv2.imwrite',
                    return_value=False):
                with self.assertRaisesRegex(OSError, 'RGB PNG 写出失败'):
                    save_session(Path(root), [_frame()], {})


if __name__ == '__main__':
    unittest.main()
