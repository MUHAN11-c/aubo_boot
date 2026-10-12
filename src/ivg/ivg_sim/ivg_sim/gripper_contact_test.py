"""Bounded Gazebo contact test; always stops its launch process group.

The box is lifted only by collision/friction, without attaching it to the hand.
Run in an isolated ROS_DOMAIN_ID and Gazebo partition.
"""

import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
import uuid

from ivg_sim.arm_description import FINGER_JOINTS
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


def object_pose():
    """Read the actual dynamic object's pose from Gazebo, not a scene attachment."""
    result = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/ivg_gripper_test/dynamic_pose/info', '-n', '1'],
        capture_output=True, text=True, timeout=8, check=True)
    for block in re.split(r'name:\s*"', result.stdout)[1:]:
        if block.split('"', 1)[0] == 'test_object':
            position = block.split('position', 1)[1].split('}', 1)[0]
            values = dict(re.findall(r'([xyz]):\s*([-\d.eE+]+)', position))
            return tuple(float(values.get(axis, 0)) for axis in 'xyz')
    raise RuntimeError('Gazebo pose stream does not contain test_object')


def main():
    """Open, close, lift, hold, release; exit nonzero on any failed gate."""
    os.environ['ROS_DOMAIN_ID'] = '97'
    os.environ['GZ_PARTITION'] = 'ivg_contact_' + uuid.uuid4().hex
    os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='ivg_ros_logs_')
    os.environ['ROS_HOME'] = tempfile.mkdtemp(prefix='ivg_ros_home_')
    os.environ['GZ_HOMEDIR'] = tempfile.mkdtemp(prefix='ivg_gz_home_')
    log_path = Path(tempfile.gettempdir()) / (os.environ['GZ_PARTITION'] + '.log')
    process = None
    node = None
    with log_path.open('w') as log:
        try:
            process = subprocess.Popen(
                ['ros2', 'launch', 'ivg_sim', 'gripper_contact.launch.py'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            rclpy.init()
            node = rclpy.create_node('ivg_gripper_contact_probe')
            states = {}

            def on_state(msg):
                states.update(zip(msg.name, msg.position))
            node.create_subscription(JointState, '/joint_states', on_state,
                                     qos_profile_sensor_data)
            hand = node.create_publisher(Float64MultiArray,
                                         '/gripper_effort_controller/commands', 10)
            lift = node.create_publisher(Float64, '/ivg_fixture/lift', 10)
            deadline = time.monotonic() + 40
            while not (all(j in states for j in FINGER_JOINTS) and
                       hand.get_subscription_count() and lift.get_subscription_count()):
                if time.monotonic() > deadline or process.poll() is not None:
                    raise RuntimeError(f'Gazebo/controller startup failed; log: {log_path}')
                rclpy.spin_once(node, timeout_sec=0.1)

            def settle(seconds, force):
                end = time.monotonic() + seconds
                while time.monotonic() < end:
                    hand.publish(Float64MultiArray(data=[force] * 4))
                    rclpy.spin_once(node, timeout_sec=0.05)

            settle(1.0, 4.0)
            opened = [states[j] for j in FINGER_JOINTS]
            if min(opened) < 0.04:
                raise RuntimeError(f'Fingers failed to open: {opened}')
            before = object_pose()
            print('OPEN', opened, 'object', before, flush=True)
            settle(3.0, -12.0)
            closed = [states[j] for j in FINGER_JOINTS]
            print('CONTACT', closed, 'object', object_pose(), flush=True)
            if max(closed) >= min(opened) - 0.005:
                raise RuntimeError(f'Fingers did not close: {closed}')
            # A slow commanded lift; no object teleport or weld is involved.
            for i in range(1, 41):
                lift.publish(Float64(data=0.004 * i))
                settle(0.08, -12.0)
            settle(2.0, -12.0)
            held = object_pose()
            print('HELD', held, 'lift_joint', states.get('fixture_lift'),
                  'fingers', [states[j] for j in FINGER_JOINTS], flush=True)
            dz = held[2] - before[2]
            drift = math.hypot(held[0] - before[0], held[1] - before[1])
            if dz < 0.10 or drift > 0.03:
                raise RuntimeError(f'Contact hold failed: dz={dz:.4f}, drift={drift:.4f}')
            settle(3.0, 4.0)
            released = object_pose()
            if released[2] >= held[2] - 0.08:
                raise RuntimeError(f'Object did not release: held={held}, released={released}')
            print(json.dumps({'passed': True, 'open_m': opened, 'contact_m': closed,
                              'before': before, 'held': held, 'released': released,
                              'lift_m': dz, 'drift_m': drift, 'log': str(log_path)}),
                  flush=True)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process is not None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                except ProcessLookupError:
                    pass


if __name__ == '__main__':
    main()
