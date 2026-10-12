"""Expand the same parameterized description for every IVG consumer."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import xacro

GRIPPERS = ('none', 'gripper0', 'gripper1', 'gripper2',
            'gripper1coffeecup', 'gripper1milkcup')


def share_directory():
    """Locate installed assets; also support source-only model tests."""
    source = Path(__file__).resolve().parents[1]
    if (source / 'urdf/aubo_e5.urdf.xacro').exists():
        return source
    return Path(get_package_share_directory('ivg_description'))


def build_description(gripper='none', hardware_mode='mock', robot_ip='169.254.10.98',
                      include_workcell=True):
    """Expand a deployment choice; unsupported models fail before launching nodes."""
    if gripper not in GRIPPERS:
        raise ValueError(f'Unknown IVG gripper: {gripper}')
    if hardware_mode not in ('mock', 'real', 'none'):
        raise ValueError(f'Unknown IVG hardware mode: {hardware_mode}')
    return xacro.process_file(
        str(share_directory() / 'urdf/aubo_e5.urdf.xacro'), mappings={
            'gripper': gripper, 'hardware_mode': hardware_mode,
            'robot_ip': robot_ip, 'include_workcell': str(include_workcell).lower(),
        }).toxml()
