# Copyright 2026 aubo_e5_ros2_ws authors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""跨进程假能力端可达性。."""

import os
import sys
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
from peach_harvest_msgs.action import RunTargetCycle
import pytest
import rclpy
from rclpy.action import ActionClient


@pytest.mark.launch_test
def generate_test_description():
    """启动跨进程假能力端。."""
    fake = os.path.join(os.path.dirname(__file__), 'fake_capability_node.py')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[fake],
            output='screen',
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestFakeCapabilityReachable(unittest.TestCase):
    """断言假 RunTargetCycle action 可被发现。."""

    def test_run_target_cycle_action_visible(self):
        rclpy.init()
        node = rclpy.create_node('launch_probe')
        client = ActionClient(
            node, RunTargetCycle,
            '/peach_approach_grasp_node/run_target_cycle')
        deadline = time.time() + 20.0
        ready = False
        while time.time() < deadline:
            if client.wait_for_server(timeout_sec=1.0):
                ready = True
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
        self.assertTrue(ready, '跨进程假 RunTargetCycle action 仍不可达')
