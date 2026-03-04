deb依赖项:
    ros-$ROS_DISTRO-image-transport
    ros-$ROS_DISTRO-image-publisher
    ros-$ROS_DISTRO-camera-info-manager
    ros-$ROS_DISTRO-diagnostic-updater
    ros-$ROS_DISTRO-diagnostic-msgs

1.build project:
    colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release

2.config system env:
    source ./install/setup.bash

3.publish message:
    ros2 launch percipio_camera percipio_camera.launch.py

4.list topics / services / parameters
    ros2 topic list
    ros2 service list
    ros2 param list

5.run rviz2:
  ros2 run rviz2 rviz2
  a.Create visualization by toptic
  b.Select Camera toptic:
        /camera/color/camera_info
        /camera/color/image_raw
        /camera/depth/camera_info
        /camera/depth/image_raw
        /camera/depth/points
        /camera/depth_registered/points
        /camera/left_ir/camera_info
        /camera/left_ir/image_raw
        /camera/right_ir/camera_info
        /camera/right_ir/image_raw

6. 话题与服务含义说明

   发布的话题（camera 为相机命名空间，默认 camera）：

   | 话题 | 含义 |
   |------|------|
   | /camera/color/camera_info | 彩色相机标定信息（内参、分辨率等），类型 sensor_msgs/msg/CameraInfo |
   | /camera/color/image_raw | 彩色图像流，类型 sensor_msgs/msg/Image |
   | /camera/depth/camera_info | 深度相机标定信息，类型 sensor_msgs/msg/CameraInfo |
   | /camera/depth/image_raw | 深度图，类型 sensor_msgs/msg/Image |
   | /camera/depth_registration/points | 已配准到彩色相机坐标系的点云（与彩色对齐），类型 sensor_msgs/msg/PointCloud2。实现中话题名为 depth_registered/points，需启用 color_point_cloud |
   | /camera/depth/points | 深度点云（仅深度坐标系，未与彩色配准），类型 sensor_msgs/msg/PointCloud2 |
   | /camera/left_ir/image_raw | 左红外图像，类型 sensor_msgs/msg/Image |
   | /camera/device_event | 设备事件（连接/断开/超时），发布 "DeviceConnect\<SN\>"、"DeviceOffline\<SN\>"、"DeviceTimeout\<SN\>"，类型 std_msgs/msg/String |

   订阅的话题：

   | 话题 | 含义 |
   |------|------|
   | /camera/trigger_event | 软件触发：收到内容为 "SoftTrigger" 时执行一次采集（软触发模式下使用） |

   与节点相关的服务/参数：

   | 名称 | 含义 |
   |------|------|
   | /camera/dynamic_config | 节点动态参数配置（如 depth_speckle_filter、depth_time_domain_filter 等可通过 set_parameters 动态更新） |
   | /camera/reset | 若存在则为与相机或节点重置相关的服务/话题，具体以 ros2 service list / ros2 topic list 为准 |
   | /camera/soft_trigger | 软件触发：通过调用 percipio_camera_interface 的 /camera/software_trigger 服务，或向 /camera/trigger_event 发布 "SoftTrigger" 实现 |

   ROS2 系统话题：

   | 话题 | 含义 |
   |------|------|
   | /parameter_events | ROS2 全局参数变更事件 |
   | /rosout | ROS2 全局日志输出 |
   | /tf_static | 静态坐标变换（如相机 link 与 optical_frame） |
