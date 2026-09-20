"""Identity, domain, and W3 split-module imports without sourcing ROS."""
import importlib
from pathlib import Path
import sys

import pytest

_SCENE_DIR = (Path(__file__).resolve().parents[1] / 'peach_harvester'
              / 'vision' / 'scene_perception')


def test_identity_imports_without_rclpy():
    identity_mod = 'peach_harvester.vision.scene_perception.identity'
    assert 'rclpy' not in sys.modules or identity_mod not in sys.modules
    module = importlib.import_module(identity_mod)
    assert hasattr(module, 'TargetRegistry')
    assert 'rclpy' not in sys.modules or not hasattr(module, 'Node')


def test_plan_updater_imports_without_ros_msgs():
    """W3 纯核：计划推进零 ROS msg import（本测试环境无 ROS 即最强证明）."""
    source = (_SCENE_DIR / 'plan_updater.py').read_text(encoding='utf-8')
    for banned in ('geometry_msgs', 'sensor_msgs', 'peach_interfaces',
                   'std_msgs', 'vision_msgs', 'visualization_msgs', 'rclpy'):
        assert banned not in source, banned
    module = importlib.import_module(
        'peach_harvester.vision.scene_perception.plan_updater')
    assert hasattr(module, 'PlanUpdater')
    assert hasattr(module, 'harvest_state_dict')


def test_pose_pipelines_is_pure_core():
    """V5 修复：位姿纯核零 ROS msg import（Quaternion 包装已移 msg_builders）."""
    source = (_SCENE_DIR / 'pose_pipelines.py').read_text(encoding='utf-8')
    assert 'geometry_msgs' not in source
    module = importlib.import_module(
        'peach_harvester.vision.scene_perception.pose_pipelines')
    # W3 公开化 + geometry 迁移 re-export 双锚点
    assert hasattr(module, 'apply_transform_to_reference')
    assert hasattr(module, 'grasp_frame_from_axis')


def test_msg_builders_imports_in_ros_env():
    """ROS 环境下消息组装层可导入且公开面齐备；零 ROS 门按缺依赖跳过."""
    try:
        module = importlib.import_module(
            'peach_harvester.vision.scene_perception.msg_builders')
    except ImportError:
        pytest.skip('ROS msg 环境不可用（msg_builders 需 geometry_msgs 等）')
    for name in ('to_detection2d', 'to_candidate', 'to_candidate_2d',
                 'to_fitting', 'to_markers', 'xyzrgb_to_cloud_msg',
                 'bbox_cloud_xyzrgb', 'best_axis_direction',
                 'TRACKING_STATUS_TO_MSG', 'quat_to_msg'):
        assert hasattr(module, name), name


def test_domain_budget_imports_without_rclpy():
    module = importlib.import_module(
        'peach_harvester.vision.domain.budget')
    assert module.CAPABILITY_VALID == 0


def test_package_xml_depends_on_aubo_description():
    from pathlib import Path
    text = Path(__file__).resolve().parents[1].joinpath(
        'package.xml').read_text(encoding='utf-8')
    assert '<exec_depend>aubo_description</exec_depend>' in text
