"""Run the shared Peach vision capabilities with simulation-only parameter overlays."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.frontend.parse_substitution import parse_substitution
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from peach_common.grasp_standoffs import reconstruction_overlay, scene_overlay
from peach_common.tool_profiles import reconstruction_tool_params, scene_tool_params
import yaml


def _runtime_default():
    """Find the project-local runtime directory for standalone vision bringup."""
    for directory in Path(__file__).resolve().parents:
        if (directory / 'sim_description' / 'package.xml').is_file():
            return str(directory / '.runtime' / 'run')
    raise RuntimeError('Set runtime_dir explicitly for a relocated installation')


def _resolve_file(context, source, destination):
    """Evaluate official launch substitutions into a project-local parameter file."""
    expanded = perform_substitutions(
        context, parse_substitution(source.read_text(encoding='utf-8')))
    data = yaml.safe_load(expanded)
    destination.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return str(destination)


def _setup(context):
    """Compose current upstream capabilities without importing the old vision fork."""
    if LaunchConfiguration('use_sim_time').perform(context).lower() != 'true':
        raise RuntimeError('The simulator vision stack requires use_sim_time:=true')
    runtime = Path(LaunchConfiguration('runtime_dir').perform(context)).expanduser().resolve()
    runtime.mkdir(parents=True, exist_ok=True)
    context.environment['PEACH_SIM_RUN_DIRECTORY'] = str(runtime)
    context.environment['AUBO_RUNS_DIR'] = str(runtime / 'vision')
    profile = LaunchConfiguration('tool_profile')
    local = Path(get_package_share_directory('sim_perception'))
    specifications = [
        ('peach_scene_perception', 'peach_scene_perception_node', 'scene_perception.yaml',
         'sim_scene_perception_node', {**scene_overlay(), **scene_tool_params(profile)}),
        ('peach_target_reconstruction', 'peach_target_reconstruction_node',
         'target_reconstruction.yaml', 'sim_target_reconstruction_node',
         {**reconstruction_overlay(), **reconstruction_tool_params(profile),
          'session.root_dir': str(runtime / 'vision')}),
    ]
    nodes = []
    for package, name, filename, executable, overrides in specifications:
        share = Path(get_package_share_directory(package))
        base = _resolve_file(context, share / 'config' / filename, runtime / f'base_{filename}')
        overlay = _resolve_file(
            context, local / 'config' / filename, runtime / f'sim_{filename}')
        evaluated = {key: item.evaluate(context) if isinstance(item, ParameterValue) else item
                     for key, item in overrides.items()}
        evaluated['use_sim_time'] = True
        nodes.append(LifecycleNode(
            package='sim_perception', executable=executable, name=name, namespace='',
            output='screen', parameters=[base, overlay, evaluated]))
    nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='sim_perception_lifecycle_manager', output='screen', parameters=[{
            'node_names': [item[1] for item in specifications], 'autostart': True,
            'bond_timeout': float(LaunchConfiguration('bond_timeout').perform(context)),
            'use_sim_time': True,
        }]))
    return nodes


def generate_launch_description():
    """Declare the vision overlay settings."""
    return LaunchDescription([
        DeclareLaunchArgument('runtime_dir', default_value=_runtime_default()),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('tool_profile', default_value='bite_shear_v1'),
        DeclareLaunchArgument('bond_timeout', default_value='8.0'),
        OpaqueFunction(function=_setup),
    ])
