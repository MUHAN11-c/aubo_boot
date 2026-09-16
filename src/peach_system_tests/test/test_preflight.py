"""Harvest-system preflight helper (no DDS)."""
from pathlib import Path

from peach_bringup.preflight import (
    PREFLIGHT_PATTERNS,
    cmdline_looks_like_stack,
    running_stack_pids,
)


def _src_root():
    here = Path(__file__).resolve()
    for parent in here.parents:
        marker = parent / 'peach_bringup' / 'launch' / 'harvest_system.launch.py'
        if marker.exists():
            return parent
    return None


def test_preflight_patterns_include_executor():
    assert 'peach_executor' in PREFLIGHT_PATTERNS
    assert 'extrinsics_publisher' in PREFLIGHT_PATTERNS


def test_running_stack_pids_skips_missing_proc(tmp_path):
    assert running_stack_pids(str(tmp_path)) == []


def test_preflight_ignores_editor_path_substring():
    cmdline = (
        '/usr/share/cursor/cursor\x00'
        '/home/mu/Desktop/aubo_e5_jazzy_ws/src/peach_executor/executor_node.py')
    assert not cmdline_looks_like_stack(cmdline)


def test_preflight_matches_installed_executable():
    cmdline = (
        '/home/ws/install/peach_executor/lib/peach_executor/peach_executor')
    assert cmdline_looks_like_stack(cmdline)


def test_preflight_matches_python_wrapping_installed_node():
    cmdline = (
        '/usr/bin/python3\x00'
        '/home/ws/install/peach_executor/lib/peach_executor/peach_executor')
    assert cmdline_looks_like_stack(cmdline)


def test_preflight_ignores_colcon_package_select():
    cmdline = (
        '/usr/bin/python3\x00-m\x00colcon\x00test\x00'
        '--packages-select\x00peach_executor')
    assert not cmdline_looks_like_stack(cmdline)


def test_preflight_matches_observability_package_path():
    cmdline = (
        '/home/ws/install/peach_observability/lib/'
        'peach_observability/peach_observability')
    assert cmdline_looks_like_stack(cmdline)


def test_preflight_skips_harvest_system_launch_process():
    cmdline = (
        'python3\x00/opt/ros/jazzy/bin/ros2\x00launch\x00'
        'peach_bringup\x00harvest_system.launch.py')
    assert not cmdline_looks_like_stack(cmdline)


def test_bringup_launch_does_not_auto_run_harvest():
    root = _src_root()
    assert root is not None
    text = (
        root / 'peach_bringup' / 'launch' / 'harvest_system.launch.py'
        ).read_text(encoding='utf-8')
    assert 'ros2 action send_goal' not in text
    assert 'send_goal(' not in text


def test_executor_enable_default_off():
    root = _src_root()
    assert root is not None
    text = (
        root / 'peach_executor' / 'config' / 'peach_executor.yaml'
        ).read_text(encoding='utf-8')
    assert 'execution_enabled: false' in text


def test_observability_launch_owns_node():
    root = _src_root()
    assert root is not None
    text = (
        root / 'peach_observability' / 'launch' / 'observability.launch.py'
        ).read_text(encoding='utf-8')
    assert "package='peach_observability'" in text
    assert "executable='peach_observability'" in text
    assert "launch', 'observability.launch.py'" not in text


def test_executor_observability_launch_forwards():
    root = _src_root()
    assert root is not None
    text = (
        root / 'peach_executor' / 'launch' / 'observability.launch.py'
        ).read_text(encoding='utf-8')
    assert "FindPackageShare('peach_observability')" in text
    assert "package='peach_executor'" not in text


def test_observability_setup_owns_entry_points():
    root = _src_root()
    assert root is not None
    text = (root / 'peach_observability' / 'setup.py').read_text(
        encoding='utf-8')
    assert 'peach_observability.observability_node:main' in text
    assert 'peach_observability.bag_report:main' in text
    assert "glob('config/*.yaml')" not in text


def test_executor_setup_does_not_install_observability():
    root = _src_root()
    assert root is not None
    text = (root / 'peach_executor' / 'setup.py').read_text(encoding='utf-8')
    assert 'peach_observability =' not in text
    assert 'peach_bag_report =' not in text
    assert "glob('web/*')" not in text


def test_executor_does_not_ship_web_assets():
    root = _src_root()
    assert root is not None
    assert not (root / 'peach_executor' / 'web').exists()
    assert (root / 'peach_observability' / 'web' / 'index.html').is_file()
    assert (root / 'peach_observability' / 'web' / 'app.js').is_file()
    assert (root / 'peach_observability' / 'web' / 'app.css').is_file()


def test_industrial_ci_ignores_vendor_sidecars():
    root = _src_root()
    assert root is not None
    text = (root.parent / '.github' / 'workflows' / 'jazzy.yaml').read_text(
        encoding='utf-8')
    assert 'src/imu_follow/COLCON_IGNORE' in text
    assert 'src/percipio_camera/COLCON_IGNORE' in text
    assert 'src/ivg_graspnet/COLCON_IGNORE' in text
