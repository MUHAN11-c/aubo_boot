# 相机状态消息
#
# 用途：提供 Percipio 深度相机（或其他兼容相机）的运行状态和当前参数配置，
#       用于前端相机控制面板的状态显示和参数调节反馈喵~
# 发布者：percipio_camera_interface 节点（~1Hz），每次状态轮询后发布喵~
# 订阅者：Web Dashboard 相机控制面板、visual_pose_estimation 节点（用于检查相机就绪状态）、
#         monitoring dashboard 相机监控面板喵~
#

# 标准消息头：时间戳标记状态采集时刻，frame_id 为相机设备标识（如 "CAM-123456"）喵~
std_msgs/Header header
# 相机唯一标识符，与相机序列号或配置中的 camera_id 一致，用于多相机系统区分喵~
string camera_id
# 相机 USB 连接状态：true=已连接且枚举成功，false=USB 断开或设备未响应喵~
bool is_connected
# 当前分辨率宽度（像素），如 1280、1920 等，受相机型号和配置限制喵~
uint32 resolution_width
# 当前分辨率高度（像素），如 720、1080 等喵~
uint32 resolution_height
# 当前帧率（fps），如 30.0、60.0，受曝光时间和带宽限制喵~
float32 frame_rate
# 当前曝光时间（微秒），值越大低光性能越好但帧率降低且运动模糊增加喵~
float32 exposure_time
# 当前增益值（dB），值越大图像越亮但噪声也越大喵~
float32 gain
# 触发模式：-1=未设置, 0=连续采集（free-run）, 1=软件触发（software trigger）,
# 2=硬触发（硬件外部触发信号同步采集）喵~
int8 trigger_mode
