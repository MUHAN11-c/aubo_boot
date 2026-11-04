"""Apply the mobile-manipulation interlock before Nav2's final collision monitor."""

import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from .operations import named_values


class BaseGuard(Node):
    """Reject base velocity while the arm is active, extended, stale or recovering."""

    def __init__(self):
        super().__init__('sim_base_guard')
        self.declare_parameter('require_arm_lock', True)
        self.arm_busy = self.get_parameter('require_arm_lock').value
        self.recovery = False
        self.joints = {}
        self.joint_seen, self.clock_seen, self.command_seen = 0., 0., 0.
        self.command = Twist()
        self.stow = named_values('harvest_stow')
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_interlocked', 10)
        self.ack = self.create_publisher(Bool, '/sim_harvest/base_locked', latch)
        self.create_subscription(Twist, '/cmd_vel_smoothed', self.on_command, 10)
        self.create_subscription(Bool, '/sim_harvest/arm_busy', self.on_busy, latch)
        self.create_subscription(Bool, '/sim_harvest/recovery_required', self.on_recovery, latch)
        self.create_subscription(JointState, '/joint_states', self.on_joints,
                                 qos_profile_sensor_data)
        self.create_subscription(ClockMessage, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_command(self, msg):
        self.command, self.command_seen = msg, time.monotonic()

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        self.joint_seen = time.monotonic()

    def on_clock(self, _):
        self.clock_seen = time.monotonic()

    def on_busy(self, msg):
        self.arm_busy = msg.data
        self.tick()

    def on_recovery(self, msg):
        self.recovery = msg.data
        self.tick()

    def tick(self):
        """Use a steady clock so stopping does not depend on a running /clock."""
        now = time.monotonic()
        stowed = all(name in self.joints and abs(self.joints[name] - value) < .08
                     for name, value in self.stow.items())
        locked = (self.arm_busy or self.recovery or not stowed
                  or now - self.joint_seen > 1. or now - self.clock_seen > 1.)
        allowed = not locked and now - self.command_seen < .5
        self.publisher.publish(self.command if allowed else Twist())
        self.ack.publish(Bool(data=locked))


def main(args=None):
    """Run the interlock; launch shuts the simulation down if this node exits."""
    rclpy.init(args=args)
    node = BaseGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
