"""Isolated lifecycle debug bridge against a Nav2-typed service."""
import os
import sys
import tempfile
import threading
import time
import unittest

import launch
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import launch_testing
from launch_testing.actions import ReadyToTest
import launch_testing.asserts
from nav2_msgs.srv import ManageLifecycleNodes
from peach_observability.debug_actions import DebugBridge
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

SERVICE = '/peach_lifecycle_manager/manage_nodes'


@pytest.mark.launch_test
def generate_test_description():
    """Run only a fake lifecycle server; no robot or camera driver."""
    runs = tempfile.mkdtemp(prefix='peach_lifecycle_lt_')
    server = ExecuteProcess(
        cmd=[sys.executable, __file__, '--server'], output='screen')
    return launch.LaunchDescription([
        SetEnvironmentVariable('ROS_HOME', os.path.join(runs, 'ros_home')),
        server, ReadyToTest(),
    ]), {'server': server}


class TestLifecycleDebug(unittest.TestCase):
    """The debug client discovers Nav2 and preserves HTTP response semantics."""

    @classmethod
    def setUpClass(cls):
        """Spin client callbacks separately from blocking debug requests."""
        rclpy.init()
        cls.node = Node('lifecycle_debug_test')
        cls.executor = MultiThreadedExecutor(num_threads=2)
        cls.executor.add_node(cls.node)
        cls.worker = threading.Thread(target=cls.executor.spin)
        cls.worker.start()
        cls.bridge = DebugBridge(
            cls.node, {'manage_nodes_service': SERVICE}, 1.0,
            False, lambda _message: None)
        cls.bridge.open_all()

    @classmethod
    def tearDownClass(cls):
        """Destroy clients and join executor before shutting down ROS."""
        cls.bridge.close()
        cls.executor.shutdown(timeout_sec=5.0)
        cls.worker.join(timeout=5.0)
        cls.node.destroy_node()
        rclpy.shutdown()
        if cls.worker.is_alive():
            raise RuntimeError('lifecycle debug executor did not stop')

    def test_nav2_commands_and_boolean_only_response(self):
        """All five existing commands use the service's real wire type."""
        client = self.bridge._clients['manage_nodes_service']
        self.assertTrue(client.wait_for_service(timeout_sec=15.0))
        for command in ('STARTUP', 'PAUSE', 'RESUME', 'RESET', 'SHUTDOWN'):
            with self.subTest(command=command):
                status, body = self.bridge.command(
                    'manage_nodes_service', {'command': command})
                self.assertEqual(status, 200)
                self.assertTrue(body['accepted'])
                self.assertEqual(body['result'], {'success': command != 'PAUSE'})

    def test_unavailable_server_is_503(self):
        """Missing servers fail within the existing discovery timeout."""
        bridge = DebugBridge(
            self.node, {'manage_nodes_service': SERVICE + '_missing'},
            1.0, False, lambda _message: None)
        bridge.open_all()
        try:
            started = time.monotonic()
            status, body = bridge.command(
                'manage_nodes_service', {'command': 'STARTUP'})
            self.assertEqual(status, 503)
            self.assertFalse(body['accepted'])
            self.assertLess(time.monotonic() - started, 5.0)
        finally:
            bridge.close()


@launch_testing.post_shutdown_test()
class TestServerExit(unittest.TestCase):
    """The server must exit normally after launch shutdown."""

    def test_exit_code(self, proc_info, server):
        """SIGINT is handled by the server without a forced kill."""
        launch_testing.asserts.assertExitCodes(proc_info, process=server)


def _server():
    """Serve the official type with both success and rejection responses."""
    rclpy.init()
    node = Node('nav2_lifecycle_fake')

    def manage(request, response):
        response.success = request.command != ManageLifecycleNodes.Request.PAUSE
        return response

    node.create_service(ManageLifecycleNodes, SERVICE, manage)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__' and '--server' in sys.argv:
    _server()
