"""Executor import without sourcing ROS."""
import importlib
from pathlib import Path
import sys


def test_harvest_fsm_imports_without_rclpy():
    module = importlib.import_module('peach_executor.harvest_fsm')
    assert hasattr(module, 'react')
    assert 'rclpy' not in sys.modules or not hasattr(module, 'Node')


def test_reducer_imports_without_rclpy():
    module = importlib.import_module('peach_executor.domain.reducer')
    assert hasattr(module, 'reduce_event')


def test_observability_package_is_import_shim():
    root = Path(__file__).resolve().parents[1] / 'peach_executor' / 'observability'
    node = (root / 'observability_node.py').read_text(encoding='utf-8')
    assert 'from peach_observability.observability_node import' in node
    assert 'class ' not in node
    assert 'def main' not in node
