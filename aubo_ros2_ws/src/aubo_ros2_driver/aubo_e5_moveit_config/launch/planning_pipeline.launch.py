from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    pipeline_arg = DeclareLaunchArgument(
        'pipeline',
        default_value='ompl',
        description='Planning pipeline name'
    )

    # Include the appropriate planning pipeline
    planning_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('aubo_e5_moveit_config'),
                'launch',
                PythonExpression([
                    "'", LaunchConfiguration('pipeline'), "_planning_pipeline.launch.py'"
                ])
            ])
        ])
    )

    return LaunchDescription([
        pipeline_arg,
        planning_pipeline,
    ])

