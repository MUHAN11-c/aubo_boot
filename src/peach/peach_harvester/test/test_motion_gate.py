"""停稳判断：臂在动时不把画面送进检测。"""
from peach_harvester.vision.scene_perception.motion_gate import arm_is_settled


def test_missing_velocity_is_not_motion():
    assert arm_is_settled(None) is True


def test_fast_joint_is_moving():
    assert arm_is_settled([0.0, 0.2, 0.0]) is False


def test_slow_joints_are_settled():
    assert arm_is_settled([0.01, -0.02, 0.0]) is True
