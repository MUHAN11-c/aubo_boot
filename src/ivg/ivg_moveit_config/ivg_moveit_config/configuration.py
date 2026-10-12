"""Thin composition of the official MoveIt configuration builder."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ivg_description.description import build_description, share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config(gripper='none', hardware_mode='mock', robot_ip='169.254.10.98',
                        robot_description=None, srdf=None):
    """Share IVG URDF/SRDF; reuse only the driver's arm limits and planners."""
    common = Path(get_package_share_directory('aubo_e5_moveit_config'))
    share = Path(get_package_share_directory('ivg_moveit_config'))
    description = robot_description if robot_description is not None else build_description(
        gripper, hardware_mode, robot_ip)
    controllers = 'controllers.yaml' if hardware_mode == 'real' else 'controllers_mock.yaml'
    config = (MoveItConfigsBuilder('aubo_e5', package_name='ivg_moveit_config')
              .robot_description(file_path=share_directory() / 'urdf/aubo_e5.urdf.xacro',
                                 mappings={'gripper': gripper, 'hardware_mode': hardware_mode,
                                           'robot_ip': robot_ip})
              .robot_description_semantic(file_path=share / 'config/aubo_e5.srdf.xacro',
                                          mappings={'gripper': gripper})
              .robot_description_kinematics(file_path=common / 'config/kinematics.yaml')
              .joint_limits(file_path=common / 'config/joint_limits.yaml')
              .pilz_cartesian_limits(file_path=common / 'config/pilz_cartesian_limits.yaml')
              .planning_pipelines(
                  pipelines=['ompl', 'pilz_industrial_motion_planner', 'stomp'],
                  default_planning_pipeline='pilz_industrial_motion_planner')
              .trajectory_execution(file_path=common / 'config' / controllers,
                                    moveit_manage_controllers=False)
              .planning_scene_monitor(publish_planning_scene=True,
                                      publish_geometry_updates=True,
                                      publish_state_updates=True,
                                      publish_transforms_updates=True))
    result = config.to_moveit_configs()
    result.robot_description = {'robot_description': description}
    if srdf is not None:
        result.robot_description_semantic = {'robot_description_semantic': srdf}
    return result
