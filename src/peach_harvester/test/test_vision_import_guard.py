"""Identity and domain import without sourcing ROS."""
import importlib
import sys


def test_identity_imports_without_rclpy():
    identity_mod = 'peach_harvester.vision.scene_perception.identity'
    assert 'rclpy' not in sys.modules or identity_mod not in sys.modules
    module = importlib.import_module(identity_mod)
    assert hasattr(module, 'TargetRegistry')
    assert 'rclpy' not in sys.modules or not hasattr(module, 'Node')


def test_domain_budget_imports_without_rclpy():
    module = importlib.import_module(
        'peach_harvester.vision.domain.budget')
    assert module.CAPABILITY_VALID == 0


def test_package_xml_depends_on_aubo_description():
    from pathlib import Path
    text = Path(__file__).resolve().parents[1].joinpath(
        'package.xml').read_text(encoding='utf-8')
    assert '<exec_depend>aubo_description</exec_depend>' in text
