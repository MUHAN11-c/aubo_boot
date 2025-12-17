from launch import LaunchDescription


def generate_launch_description():
    # Sensor manager for aubo_e5
    # In ROS2 MoveIt2, sensor manager parameters are typically loaded by move_group node
    return LaunchDescription([
        # Note: Sensor parameters are passed to move_group node
    ])

