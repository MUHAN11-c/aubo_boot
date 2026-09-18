"""Zero-ROS tests for CandidateEstimator fruit-line gating."""
import numpy as np

from peach_harvester.vision.scene_perception.contracts import BagObservation
from peach_harvester.vision.scene_perception.inference import CandidateEstimator
from peach_harvester.vision.scene_perception.pipeline import filter_detections


def _obs(class_id: int) -> BagObservation:
    depth = np.full((16, 16), 800, dtype=np.uint16)
    rgb = np.zeros((16, 16, 3), dtype=np.uint8)
    return BagObservation(
        rgb=rgb, depth=depth,
        camera_K={'fx': 1.0, 'fy': 1.0, 'cx': 8.0, 'cy': 8.0,
                  'width': 16, 'height': 16},
        detections=[{'class_id': class_id, 'bbox': (2, 2, 10, 10), 'conf': 0.9}],
    )


class _P:
    min_detection_conf = 0.4
    detection_dedup_ios = 0.6
    detection_dedup_area_ratio = 0.5


def test_filter_detections_drops_fruit_when_disabled():
    dets = [
        {'class_id': 0, 'conf': 0.9, 'bbox': (0, 0, 10, 10)},
        {'class_id': 1, 'conf': 0.9, 'bbox': (20, 20, 30, 30)},
    ]
    kept = filter_detections(dets, _P(), enable_fruit=False)
    assert [d['class_id'] for d in kept] == [0]
    kept_on = filter_detections(dets, _P(), enable_fruit=True)
    assert {d['class_id'] for d in kept_on} == {0, 1}


def test_estimator_does_not_route_fruit_to_bag_when_pipeline_none():
    est = CandidateEstimator(fruit_pipeline=None)
    assert est.fruit_pipeline is None
    results = est.estimate_modes(_obs(1), 't0', (2, 2, 10, 10), None)
    pose = results['hybrid_dilated']
    assert pose.target_kind == 'fruit'
    assert pose.grasp_3d.status == 'REOBSERVE'
    assert 'fruit_pipeline_disabled' in pose.grasp_3d.diagnostic_flags
