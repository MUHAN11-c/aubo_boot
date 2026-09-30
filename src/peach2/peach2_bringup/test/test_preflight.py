"""Pure tests for peach2_bringup.preflight and peach2_bringup.stack (fake /proc, no rclpy)."""
from pathlib import Path

from peach2_bringup import preflight, stack
import pytest


def _proc(root: Path, pid: int, argv: list[str], env: dict[str, str] | None) -> None:
    d = root / str(pid)
    d.mkdir()
    (d / 'cmdline').write_bytes(b'\x00'.join(a.encode() for a in argv) + b'\x00')
    if env is not None:
        (d / 'environ').write_bytes(
            b'\x00'.join(f'{k}={v}'.encode() for k, v in env.items()) + b'\x00')


PY_NODE = ['/usr/bin/python3',
           '/ws/install/peach2_target_model/lib/peach2_target_model/target_model_node',
           '--ros-args', '-r', '__node:=peach2_target_model']


@pytest.mark.parametrize('argv, expected', [
    (['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher', '--ros-args'],
     'robot_state_publisher'),
    (['/ws/install/peach2_task/lib/peach2_task/peach2_task_node', '--ros-args'],
     'peach2_task_node'),
    (PY_NODE, 'target_model_node'),
    (['/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager', '--ros-args', '-r',
      '__node:=peach2_lifecycle_manager'], 'lifecycle_manager'),
    (['/ws/install/peach_arm/lib/peach_arm/peach_arm'], 'peach_arm'),
])
def test_matched_executable_hits(argv, expected):
    assert preflight.matched_executable(argv) == expected


@pytest.mark.parametrize('argv', [
    [],
    ['/usr/bin/python3', '-m', 'pytest', 'test_move_group.py'],
    ['grep', 'robot_state_publisher'],
    ['/usr/bin/python3', '/usr/bin/colcon', 'test', '--packages-select', 'peach2_task_node'],
    ['/usr/bin/code', '/ws/src/peach2/peach2_task/peach2_task_node'],
    ['/usr/bin/bash', '-c', 'pgrep -af move_group'],
])
def test_matched_executable_misses(argv):
    assert preflight.matched_executable(argv) is None


def test_domain_from_environ():
    assert preflight.domain_from_environ(b'A=1\x00ROS_DOMAIN_ID=95\x00') == '95'
    assert preflight.domain_from_environ(b'ROS_DOMAIN_ID=\x00') == '0'
    assert preflight.domain_from_environ(b'PATH=/usr/bin\x00') == '0'
    assert preflight.domain_from_environ(None) == preflight.UNKNOWN_DOMAIN


def test_conflicts_same_or_unknown_domain():
    assert preflight.conflicts('0', '0')
    assert preflight.conflicts(preflight.UNKNOWN_DOMAIN, '95')
    assert not preflight.conflicts('0', '95')


def test_running_stack_processes_filters_by_domain(tmp_path):
    _proc(tmp_path, 101, ['/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher'],
          {'ROS_DOMAIN_ID': '0'})
    _proc(tmp_path, 102, PY_NODE, {'ROS_DOMAIN_ID': '95'})
    _proc(tmp_path, 103, ['/opt/ros/jazzy/lib/moveit_ros_move_group/move_group'], None)
    _proc(tmp_path, 104, ['/usr/bin/bash'], {'ROS_DOMAIN_ID': '95'})
    _proc(tmp_path, 105, ['/opt/ros/jazzy/lib/tf2_ros/static_transform_publisher'], {})
    (tmp_path / 'self').mkdir()

    in_zero = preflight.running_stack_processes('0', proc_root=str(tmp_path))
    assert [p.pid for p in in_zero] == [101, 103]
    assert in_zero[1].domain_id == preflight.UNKNOWN_DOMAIN

    in_95 = preflight.running_stack_processes('95', proc_root=str(tmp_path))
    assert [p.pid for p in in_95] == [102, 103]

    excluded = preflight.running_stack_processes(
        '95', proc_root=str(tmp_path), exclude_pids=[102])
    assert [p.pid for p in excluded] == [103]


def test_running_stack_processes_missing_root(tmp_path):
    assert preflight.running_stack_processes('0', proc_root=str(tmp_path / 'none')) == []


def test_refusal_message_lists_pids():
    stale = [preflight.StackProcess(i, '0', f'cmd{i}') for i in range(15)]
    text = preflight.refusal_message(stale, limit=12)
    assert 'kill -TERM 0' in text and 'kill -TERM 11' in text
    assert 'kill -TERM 12' not in text
    assert '3 more' in text


def test_managed_node_names_order():
    assert stack.managed_node_names(True) == [
        'peach2_perception', 'peach2_target_model', 'peach2_scene',
        'peach2_manipulation', 'peach2_task']
    assert stack.managed_node_names(False) == [
        'peach2_target_model', 'peach2_manipulation', 'peach2_task']


@pytest.mark.parametrize('enabled, frontend, stereo, aubo', [
    (False, 'stereo', False, False),
    (False, 'percipio', False, False),
    (True, 'stereo', True, False),
    (True, 'percipio', False, True),
])
def test_camera_routing(enabled, frontend, stereo, aubo):
    assert stack.start_stereo(enabled, frontend) is stereo
    assert stack.aubo_camera_enabled(enabled, frontend) is aubo


def test_as_bool():
    assert stack.as_bool('True') and stack.as_bool('1')
    assert not stack.as_bool('false') and not stack.as_bool('0')
    with pytest.raises(ValueError):
        stack.as_bool('maybe')


def test_validate():
    assert stack.validate('mock', 'stereo', 'adaptive_shear_v1', 4.0) == ''
    assert 'hardware_mode' in stack.validate('sim', 'stereo', 'adaptive_shear_v1', 4.0)
    assert 'camera_frontend' in stack.validate('mock', 'kinect', 'adaptive_shear_v1', 4.0)
    assert 'tool_id' in stack.validate('real', 'percipio', 'gripper', 4.0)
    assert 'bond_timeout' in stack.validate('mock', 'stereo', 'shear_v1', -1.0)
    assert 'bond_timeout' in stack.validate('mock', 'stereo', 'shear_v1', float('nan'))
