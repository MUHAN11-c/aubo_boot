from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('visual_pose_estimation')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'configs', 'default.yaml'),
        description='配置文件路径'
    )
    
    calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value=os.path.join(pkg_share, 'configs', 'calibration.yaml'),
        description='标定文件路径'
    )
    
    template_root_arg = DeclareLaunchArgument(
        'template_root',
        default_value='/home/nvidia/RVG_ws/templates',
        description='模板库根目录'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='是否启用调试模式'
    )
    
    # 使用OpaqueFunction来解析LaunchConfiguration
    def create_node(context):
        config_file = LaunchConfiguration('config_file').perform(context)
        calib_file = LaunchConfiguration('calib_file').perform(context)
        template_root = LaunchConfiguration('template_root').perform(context)
        debug = LaunchConfiguration('debug').perform(context)
        
        # 转换debug字符串为布尔值
        debug_bool = debug.lower() in ('true', '1', 'yes', 'on')
        
        return [
            Node(
                package='visual_pose_estimation',
                executable='visual_pose_estimation_node',
                name='visual_pose_estimation',
                output='screen',
                parameters=[{
                    'config_file': config_file,
                    'calib_file': calib_file,
                    'template_root': template_root,
                    'debug': debug_bool
                }],
                remappings=[
                    ('/image_data', '/camera/image_raw'),
                    ('/pose_estimation_results', '/visual_pose_estimation/results'),
                    ('/visual_pose_estimation_status', '/visual_pose_estimation/status')
                ]
            )
        ]
    
    visual_pose_estimation_node_action = OpaqueFunction(function=create_node)
    
    return LaunchDescription([
        config_file_arg,
        calib_file_arg,
        template_root_arg,
        debug_arg,
        visual_pose_estimation_node_action
    ])

