# Copyright 2026 wjz
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd
from types import SimpleNamespace
import unittest

from aubo_hand_eye_calibration.storage import (
    activate_candidate,
    load_candidate,
    write_candidate,
)
import numpy as np


def result(passed=True):
    return SimpleNamespace(
        passed=passed,
        failures=[] if passed else ['quality failed'],
        method='park',
        accepted_indices=list(range(12)),
        rejected_indices=[],
        translation_rms_m=0.001,
        rotation_rms_deg=0.1,
        reprojection_rms_px=0.2,
        rotation_span_deg=45.0,
    )


class StorageTest(unittest.TestCase):
    def test_candidate_is_versioned_and_activation_is_gated(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as directory:
            tmp_path = Path(directory)
            identity = np.eye(4)
            write_candidate(
                'good', identity, identity, identity, result(), {
                    'base': 'base_link',
                    'wrist': 'wrist3_Link',
                    'camera_root': 'camera_link',
                    'camera_optical': 'camera_color_optical_frame',
                }, {
                    'inner_corners': [11, 8], 'square_size_m': 0.020,
                }, [], tmp_path)
            _, document = load_candidate('good', tmp_path)
            self.assertEqual(document['schema_version'], 1)
            self.assertTrue(activate_candidate('good', tmp_path).is_file())

            write_candidate(
                'bad', identity, identity, identity, result(False), {}, {},
                [], tmp_path)
            with self.assertRaises(ValueError):
                activate_candidate('bad', tmp_path)
