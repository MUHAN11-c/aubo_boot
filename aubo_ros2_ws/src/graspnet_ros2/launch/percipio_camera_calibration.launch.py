from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    # 声明启动参数
    args = [
        # Topic 名，自定义，多相机情况下可通过该名称区分相机
        DeclareLaunchArgument('camera_name', default_value='camera'),
        
        # 相机 SN 号，可为空；也可填写待连接相机的 SN 号
        DeclareLaunchArgument('serial_number', default_value='"207000152740"'),
        
        # 相机 IP 地址，可为空；也可填写待连接相机的 IP 地址
        DeclareLaunchArgument('device_ip', default_value='169.254.10.91'),

        # 相机触发模式，支持连续出图、硬触发和软触发模式
        # 可选值：trigger_off（连续模式）/ trigger_soft（软触发）/ trigger_hard（硬触发）
        # 如果使用软触发模式，可以参考示例文件 "send_trigger.py" 来发送软触发信号
        DeclareLaunchArgument('device_workmode', default_value='trigger_off'),

        # 重传开关，如果设备工作模式为触发模式，此功能将自动强制开启
        # 可选值：true / false
        DeclareLaunchArgument('gvsp_resend', default_value='true'),

        # 相机掉线重连，可以参考示例文件 "offline_detect.py" 来检测相机离线事件，无论此开关是否开启
        # 可选值：true / false
        DeclareLaunchArgument('device_auto_reconnect', default_value='true'),

        # 彩色图像使能
        # 可选值：true / false
        DeclareLaunchArgument('color_enable', default_value='true'),
        
        # 彩色图像分辨率，设定范围请参考相机的 fetch_config.xml 文件
        DeclareLaunchArgument('color_resolution', default_value='"640x480"'),
        
        # 彩色图像格式，参考 camport_ros2/src/percipio_camera/src/percipio_device.cpp StreamFormatMapper format_mapper_list[]
        # 格式列表:yuv / jpeg / bayer / mono...
        DeclareLaunchArgument('color_format', default_value='"yuv"'),

        # 彩色图自动曝光 ROI，格式为 'x,y,width,height'
        # 其中(x,y)表示 ROI 左上角坐标，width 和 height 分别表示区域的宽度和高度
        # 此设置要求相机本身支持此功能，否则设置无效
        #DeclareLaunchArgument('color_aec_roi', default_value='0.0.1280.960'),
        
        # Color 服务质量
        # 可选值：SYSTEM_DEFAULT / DEFAULT / PARAMETER_EVENTS / SERVICES_DEFAULT / PARAMETERS / SENSOR_DATA
        DeclareLaunchArgument('color_qos', default_value='default'),
        
        # Color camera info 服务质量
        # 可选值：SYSTEM_DEFAULT / DEFAULT / PARAMETER_EVENTS / SERVICES_DEFAULT / PARAMETERS / SENSOR_DATA
        DeclareLaunchArgument('color_camera_info_qos', default_value='default'),

        # 深度图像使能
        # 可选值：true / false
        DeclareLaunchArgument('depth_enable', default_value='true'),
        
        # 深度图像分辨率，设定范围请参考相机的 fetch_config.xml 文件
        DeclareLaunchArgument('depth_resolution', default_value='"640x480"'),

        # 深度图像格式，参考 camport_ros2/src/percipio_camera/src/percipio_device.cpp StreamFormatMapper format_mapper_list[]
        # 格式列表:depth16/xyz48...
        #DeclareLaunchArgument('depth_format', default_value='"xyz48"'),

        # Depth 服务质量
        # 可选值：SYSTEM_DEFAULT / DEFAULT / PARAMETER_EVENTS / SERVICES_DEFAULT / PARAMETERS / SENSOR_DATA
        DeclareLaunchArgument('depth_qos', default_value='default'),
        
        # Depth camera info 服务质量
        # 可选值：SYSTEM_DEFAULT / DEFAULT / PARAMETER_EVENTS / SERVICES_DEFAULT / PARAMETERS / SENSOR_DATA
        DeclareLaunchArgument('depth_camera_info_qos', default_value='default'),

        # Depth to Color 对齐使能
        # 可选值：true / false
        DeclareLaunchArgument('depth_registration_enable', default_value='true'),

        # 深度图斑点滤波器开关
        # 可选值：true / false
        DeclareLaunchArgument('depth_speckle_filter', default_value='true'),
        
        # 斑点滤波器面积阈值，小于此大小的斑点将被移除
        # 默认值：150，取值范围：10-3000
        DeclareLaunchArgument('max_speckle_size', default_value='150'),
        
        # 斑点滤波器聚类，相邻视差像素之间的最大差值
        # 默认值：64，取值范围：5-1500
        DeclareLaunchArgument('max_speckle_diff', default_value='64'),
  
        # 时域滤波开关
        # 可选值：true / false
        DeclareLaunchArgument('depth_time_domain_filter', default_value='false'),
        
        # 时域滤波数量
        # 默认值：3，范围：2-10
        DeclareLaunchArgument('depth_time_domain_num', default_value='3'),

        # 点云图像使能
        # 可选值：true / false
        DeclareLaunchArgument('point_cloud_enable', default_value='true'),

        # 彩色点云图像使能
        # 当启用彩色点云时，depth_registration_enable 将自动设置为 true，point_cloud_enable 将自动设置为 false
        # 可选值：true / false
        DeclareLaunchArgument('color_point_cloud_enable', default_value='true'),
        
        # 点云图服务质量
        # 可选值：SYSTEM_DEFAULT / DEFAULT / PARAMETER_EVENTS / SERVICES_DEFAULT / PARAMETERS / SENSOR_DATA
        DeclareLaunchArgument('point_cloud_qos', default_value='default'),

        # 左 IR 图像使能
        # 可选值：true / false
        DeclareLaunchArgument('left_ir_enable', default_value='false'),
        
        # 左 IR 服务质量
        # 可选值：SYSTEM_DEFAULT / DEFAULT / PARAMETER_EVENTS / SERVICES_DEFAULT / PARAMETERS / SENSOR_DATA
        DeclareLaunchArgument('left_ir_qos', default_value='default'),
        
        # 左 IR camera info 服务质量
        # 可选值：SYSTEM_DEFAULT / DEFAULT / PARAMETER_EVENTS / SERVICES_DEFAULT / PARAMETERS / SENSOR_DATA
        DeclareLaunchArgument('left_ir_camera_info_qos', default_value='default'),

        # ToF 相机深度质量，设定范围请参考相机的相关属性
        # 可选值：basic / medium / high
        DeclareLaunchArgument('tof_depth_quality', default_value='medium'),
        
        # ToF 相机激光调制光强，设定范围请参考相机的相关属性
        DeclareLaunchArgument('tof_modulation_threshold', default_value='-1'),
        
        # ToF 相机抖动过滤，设定范围请参考相机的相关属性
        DeclareLaunchArgument('tof_jitter_threshold', default_value='-1'),
        
        # ToF 相机飞点滤波，设定范围请参考相机的相关属性
        DeclareLaunchArgument('tof_filter_threshold', default_value='-1'),
        
        # ToF 相机调制频率，设定范围请参考相机的相关属性
        DeclareLaunchArgument('tof_channel', default_value='-1'),
        
        # ToF 相机高动态范围比，设定范围请参考相机的相关属性
        DeclareLaunchArgument('tof_HDR_ratio', default_value='-1'),

    ]

    parameters = [{arg.name: LaunchConfiguration(arg.name)} for arg in args]

    compose_node = ComposableNode(
        package='percipio_camera',
        plugin='percipio_camera::PercipioCameraNodeDriver',
        name=LaunchConfiguration('camera_name'),
        namespace='',
        parameters=parameters,
    )

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            compose_node,
        ],
        output='screen',
    )

    ld = LaunchDescription(
        args +
        [
            GroupAction([
                PushRosNamespace(LaunchConfiguration('camera_name')),
                container
            ])
        ]
    )
    return ld
