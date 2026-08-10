"""validation：标注 JSONL 加载与 bbox IoU."""
import json
from pathlib import Path
import tempfile
import unittest

from peach_pose_ros2.peach_pose.offline.validation import (
    bbox_iou,
    load_annotations,
)


class ValidationSchemaTest(unittest.TestCase):
    def test_jsonl_schema_loads_and_groups_frames(self):
        row = {'frame_id': '1', 'target_id': 'bag-1', 'class_id': 0,
               'bbox': [1, 2, 30, 40], 'expected_status': 'REOBSERVE'}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'targets.jsonl'
            path.write_text(json.dumps(row) + '\n', encoding='utf-8')
            grouped = load_annotations(path)
        self.assertEqual(grouped['1'][0]['target_id'], 'bag-1')

    def test_bbox_iou(self):
        self.assertEqual(bbox_iou((0, 0, 10, 10), (0, 0, 10, 10)), 1.0)
        self.assertEqual(bbox_iou((0, 0, 2, 2), (3, 3, 5, 5)), 0.0)


if __name__ == '__main__':
    unittest.main()
