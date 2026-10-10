"""ivg_sim MoveIt 参数单源（launch 的 move_group 与 execute_grasp 的 MoveItPy 共用）.

管线/关节限位/IK 复用 aubo_e5_moveit_config 的官方 Builder 装载；
robot_description / SRDF / 控制器映射替换为本包 gz 机器人（厂商臂+sim 夹爪）。
JTC 名字保持 joint_trajectory_controller（与 controllers_mock.yaml 对齐，
MoveIt 侧零改动）。
"""

from pathlib import Path

import yaml

from ivg_sim.arm_description import ARM_JOINTS, build_arm_urdf, srdf_path


def build_moveit_params() -> dict:
    """组装 move_group / MoveItPy 的完整参数 dict."""
    from moveit_configs_utils import MoveItConfigsBuilder

    cfg = (
        MoveItConfigsBuilder('aubo_e5', package_name='aubo_e5_moveit_config')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner', 'stomp'],
            default_planning_pipeline='ompl')
        .to_moveit_configs()
    )

    cfg.robot_description = {'robot_description': build_arm_urdf()}
    cfg.robot_description_semantic = {
        'robot_description_semantic': srdf_path().read_text(encoding='utf-8')}
    cfg.trajectory_execution = {
        'moveit_manage_controllers': False,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 5.0,
            'allowed_goal_duration_margin': 10.0,
            'allowed_start_tolerance': 0.15,
            # 与 aubo mock 口径一致（决策 0029）：TEM 关，执行超时由调用方管
            'execution_duration_monitoring': False,
        },
        'moveit_simple_controller_manager': {
            'controller_names': ['joint_trajectory_controller'],
            'joint_trajectory_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True,
                'joints': list(ARM_JOINTS),
            },
        },
    }
    params = cfg.to_dict()
    params['use_sim_time'] = True
    return params


def load_manifest(worlds_dir: Path) -> dict:
    """读 GT manifest（对象摆位；抓取执行选目标用，与检测评分同源）."""
    path = worlds_dir / 'ivg_table.manifest.yaml'
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)
