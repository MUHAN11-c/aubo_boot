# 图像数据消息（统一接口）
#
# 用途：提供统一的图像数据传输接口，将 percipio_camera_interface 的相机原始图像
#       （RGB / Depth / IR）和 visual_pose_estimation 的处理结果图像合并为相同消息类型，
#       简化下游节点的订阅逻辑喵~
# 发布者：
#   - percipio_camera_interface 节点（~30Hz 彩色，~30Hz 深度，~15Hz IR）
#   - visual_pose_estimation 节点（推理结果可视化图像，~5-10Hz）喵~
# 订阅者：
#   - Web Dashboard 前端（通过 websocket gateway 转发给浏览器显示）
#   - rosbag 记录（用于离线调试和算法复现）
#   - monitoring dashboard（实时画面监控）喵~
# 注意：本消息由 percipio_camera_interface（原接口）和 visual_pose_estimation/interface 合并而来喵~
#

# 标准消息头：时间戳标记图像采集或处理时刻，frame_id 为相机坐标系名称（如 "camera_color_optical_frame"）喵~
std_msgs/Header header
# 相机标识符字符串，用于区分多相机系统中的不同相机（如 "camera_left", "camera_right"），
# 也用于区分原始图像流和处理结果流（如 "camera_color" vs "vision_result"）喵~
string camera_id
# 图像数据（ROS 2 标准 sensor_msgs/Image），兼容 RGB/RGBA/灰度/浮点深度图等编码类型，
# 由 sensor_msgs/image_encodings.hpp 定义编码格式喵~
sensor_msgs/Image image
