"""Executor import without sourcing ROS."""
import importlib
import sys


def test_harvest_fsm_imports_without_rclpy():
    module = importlib.import_module('peach_harvester.supervisor.harvest_fsm')
    assert hasattr(module, 'react')
    assert 'rclpy' not in sys.modules or not hasattr(module, 'Node')


def test_reducer_imports_without_rclpy():
    module = importlib.import_module(
        'peach_harvester.supervisor.domain.reducer')
    assert hasattr(module, 'reduce_event')


def test_observe_imports_without_rclpy():
    """W6-A：fast 档观察纯核（observe.py）零 ROS import."""
    module = importlib.import_module('peach_harvester.supervisor.observe')
    assert hasattr(module, 'fast_observe_loop')
    assert hasattr(module, 'view_signals')
    assert hasattr(module, 'camera_position')
    assert hasattr(module, 'target_anchor')
