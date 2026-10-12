"""Real-node regression: display requests must not remove the actual tool ACO."""

import os
import signal
import subprocess
import tempfile
import time

from ivg_interfaces.srv import ChangeTool
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import rclpy


def main():
    """Exercise real services in an isolated ROS domain and clean up the worker."""
    os.environ['ROS_DOMAIN_ID'] = '96'
    os.environ['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='ivg_scene_logs_')
    process = None
    node = None
    try:
        process = subprocess.Popen(
            ['ros2', 'run', 'tool_changer', 'scene_attach_worker_node'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        rclpy.init()
        node = rclpy.create_node('ivg_scene_attach_regression')
        events = []
        node.create_subscription(AttachedCollisionObject, '/attached_collision_object',
                                 events.append, 10)
        clients = {name: node.create_client(ChangeTool, name)
                   for name in ('/scene_attach', '/set_display_tool', '/scene_detach')}
        for client in clients.values():
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError('Scene attach service did not start')

        def call(name, tool):
            future = clients[name].call_async(ChangeTool.Request(tool_id=tool))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if future.result() is None:
                raise RuntimeError('Scene service timed out')
            return future.result()

        assert call('/scene_attach', 'gripper2').success
        assert call('/set_display_tool', 'gripper2').success
        assert not call('/set_display_tool', 'gripper1').success
        assert not call('/set_display_tool', '').success
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        actual = [e for e in events if e.object.id == 'attached_tool_gripper2']
        assert actual and all(e.object.operation == CollisionObject.ADD for e in actual)
        assert not node.get_publishers_info_by_topic('/robot_description')
        assert call('/scene_detach', 'gripper2').success
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert events[-1].object.operation == CollisionObject.REMOVE
        print('PASS: display cannot remove actual tool; explicit detach works; no URDF publisher')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            except ProcessLookupError:
                pass


if __name__ == '__main__':
    main()
