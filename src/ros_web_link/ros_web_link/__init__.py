"""ros_web_link — Web↔ROS 2 通信层（纯传输，不含 UI 业务）.

对标 rosbridge_suite 分包：
  ros_web_link（本包）      ≈ rosbridge_server + rosbridge_library + roslibjs
  ivg_foxglove_bridge       ≈ ROS 2 协议桥（已有 vendored 包）
  aubo_ros2_web_dashboard   ≈ 应用面板（UI 层，通过 create_app() 组合本包）

网关只做传输（WS 代理/视频代理/mesh/health）；UI 面板的 BFF 路由以 sub_routers 注入。
"""

__version__ = '1.1.0'

__all__ = ['__version__']
