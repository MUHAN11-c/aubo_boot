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
