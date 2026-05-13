"""咖啡拉花演示节点：通过 /aubo_driver/set_io 控制 DO2/DO4，订阅 /aubo_driver/io_states 反馈 DI 状态。"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

# 通过 demo_interface 调用 Aubo 驱动 IO
try:
    from ivg_interfaces.srv import SetRobotIO
    from ivg_interfaces.msg import RobotIOStatus
    HAS_DRIVER_IFACE = True
except ImportError:
    SetRobotIO = None
    RobotIOStatus = None
    HAS_DRIVER_IFACE = False


class LatteNode(Node):
    def __init__(self):
        super().__init__('coffee_latte_demo')
        self.di_status_pub = self.create_publisher(String, '/latte_di_status', 10)
        self.set_do2_srv = self.create_service(SetBool, '/set_latte_do2', self.set_do2_callback)
        self.set_do4_srv = self.create_service(SetBool, '/set_latte_do4', self.set_do4_callback)
        self.do2_state = False
        self.do4_state = False
        self.di2_state = False
        self.di3_state = False
        self.di4_state = False

        if HAS_DRIVER_IFACE:
            self.set_io_client = self.create_client(SetRobotIO, '/aubo_driver/set_io')
            self.io_states_sub = self.create_subscription(
                RobotIOStatus, '/aubo_driver/io_states', self.io_states_callback, 10)
            self.get_logger().info('已连接 Aubo 驱动 IO（set_io + io_states 订阅）')
        else:
            self.set_io_client = None
            self.io_states_sub = None
            self.get_logger().warn('未找到 demo_interface，IO 控制仅在内存中生效')

        self.timer = self.create_timer(5.0, self.publish_status)
        self.get_logger().info('咖啡拉花演示节点已就绪')

    # ── Aubo 驱动 IO 反馈 ──
    def io_states_callback(self, msg: RobotIOStatus):
        """从 /aubo_driver/io_states 更新真实 DI 状态。"""
        n_di = len(msg.digital_inputs)
        self.di2_state = msg.digital_inputs[2] if n_di > 2 else False
        self.di3_state = msg.digital_inputs[3] if n_di > 3 else False
        self.di4_state = msg.digital_inputs[4] if n_di > 4 else False

    # ── 硬件 IO 调用 ──
    def _call_set_io(self, io_index: int, value: bool) -> bool:
        """调用 /aubo_driver/set_io 设置 digital_output。失败时返回 False。"""
        if self.set_io_client is None:
            return True  # 无驱动接口时静默跳过
        if not self.set_io_client.service_is_ready():
            self.get_logger().error(f'/aubo_driver/set_io 不可用')
            return False
        req = SetRobotIO.Request()
        req.io_type = 'digital_output'
        req.io_index = io_index
        req.value = 1.0 if value else 0.0
        future = self.set_io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            resp = future.result()
            if not resp.success:
                self.get_logger().error(f'set_io(DO{io_index}) 失败: {resp.message}')
                return False
            return True
        self.get_logger().error(f'set_io(DO{io_index}) 超时')
        return False

    # ── 状态发布 ──
    def publish_status(self):
        msg = String()
        msg.data = (
            f"DO2={'ON' if self.do2_state else 'OFF'} "
            f"DO4={'ON' if self.do4_state else 'OFF'} "
            f"| DI2={'HI' if self.di2_state else 'LO'} "
            f"DI3={'HI' if self.di3_state else 'LO'} "
            f"DI4={'HI' if self.di4_state else 'LO'}"
        )
        self.di_status_pub.publish(msg)

    # ── DO 服务回调 ──
    def set_do2_callback(self, request, response):
        io_index = 2
        if self._call_set_io(io_index, request.data):
            self.do2_state = request.data
            response.success = True
            response.message = "OK"
            self.get_logger().info(f"DO{io_index} (打花开关) -> {'开' if request.data else '关'}")
        else:
            response.success = False
            response.message = f"DO{io_index} 设置失败"
        return response

    def set_do4_callback(self, request, response):
        io_index = 4
        if self._call_set_io(io_index, request.data):
            self.do4_state = request.data
            response.success = True
            response.message = "OK"
            self.get_logger().info(f"DO{io_index} (咖啡开关) -> {'开' if request.data else '关'}")
        else:
            response.success = False
            response.message = f"DO{io_index} 设置失败"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LatteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
