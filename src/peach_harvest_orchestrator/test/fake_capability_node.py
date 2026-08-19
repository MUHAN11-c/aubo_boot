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

"""跨进程假能力端：RunTargetCycle 与感知 Trigger。."""

import json
import threading
import time

from aubo_msgs.msg import RobotStatus
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from peach_harvest_msgs.action import RunTargetCycle
from peach_harvest_msgs.msg import ReconstructionStatus
from peach_pose_msgs.msg import PeachTargetObservation
from peach_pose_msgs.msg import PeachTargetObservationArray
from peach_pose_msgs.srv import ReopenTarget
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster


class FakeField(LifecycleNode):
    """假现场：跨进程脚本化能力端。."""

    def __init__(self):
        super().__init__('dispatch_protocol_field')
        self._lock = threading.Lock()
        self.complete_success = True
        self.complete_calls = 0
        self.clear_success = True
        self.clear_calls = 0
        self.reset_calls = 0
        self.pose_order = []
        self.estop = False
        self.targets_locked = True
        self.collecting_count = 0
        self.pending_count = 0
        self.selected = ''
        self.target_ids = ['peach_1']
        self.goals_received = 0
        self.cancels_received = 0
        self.cycle_ids = []
        self.goal_modes = []
        self.reject_observe_only = False
        self.reject_all = False
        self.reject_first_n = 0
        self.estop_on_reject = False
        self.first_outcome_quality = False
        self.finish_delay_ms = 300
        self.peach2_delay_ms = 0
        self.complete_select_seq = None
        self.cancel_reject_ids = set()
        self.on_complete_clear_selected = False
        self.clear_fail_first = 0
        self.publish_static_tf = False
        self._group = ReentrantCallbackGroup()

    def on_configure(self, _state):
        self._cmd_sub = self.create_subscription(
            String, '/fake_field/command', self._on_command, 20,
            callback_group=self._group)
        self._status_pub = self.create_publisher(String, '/fake_field/status', 10)
        self.create_service(
            Trigger, '/peach_pose_node/complete_selected_target',
            self._on_complete)
        self.create_service(
            Trigger, '/peach_pose_node/reset_global_targets', self._on_reset)
        self.create_service(
            Trigger, '/peach_pose_node/clear_target_memory', self._on_clear)
        self.create_service(
            ReopenTarget, '/peach_pose_node/reopen_target', self._on_reopen)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._obs_pub = self.create_publisher(
            PeachTargetObservationArray,
            '/peach/perception/target_observations', 10)
        self._recon_pub = self.create_publisher(
            ReconstructionStatus, '/peach/reconstruction/diagnostics', latched)
        self._robot_pub = self.create_publisher(
            RobotStatus, '/aubo_io_controller/robot_status', 10)
        self.create_timer(0.2, self._tick_obs)
        self.create_timer(0.5, self._tick_recon)
        self.create_timer(0.1, self._tick_robot)
        self.create_timer(0.2, self._tick_status)
        self._tf = StaticTransformBroadcaster(self)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, _state):
        return TransitionCallbackReturn.SUCCESS

    def _on_command(self, msg):
        for raw in msg.data.splitlines():
            line = raw.strip()
            if not line or '=' not in line:
                continue
            key, value = line.split('=', 1)
            with self._lock:
                if key == 'set_selected':
                    self.selected = value
                elif key == 'complete_success':
                    self.complete_success = value not in ('0', 'false', 'False')
                elif key == 'clear_success':
                    self.clear_success = value not in ('0', 'false', 'False')
                elif key == 'locked':
                    self.targets_locked = value not in ('0', 'false', 'False')
                elif key == 'estop':
                    self.estop = value in ('1', 'true', 'True')
                elif key == 'target_ids':
                    self.target_ids = [item for item in value.split(',') if item]
                elif key == 'collecting_count':
                    self.collecting_count = int(value)
                elif key == 'pending_count':
                    self.pending_count = int(value)
                elif key == 'reject_observe_only':
                    self.reject_observe_only = value not in ('0', 'false')
                elif key == 'reject_all':
                    self.reject_all = value not in ('0', 'false')
                elif key == 'reject_first_n':
                    self.reject_first_n = int(value)
                elif key == 'estop_on_reject':
                    self.estop_on_reject = value not in ('0', 'false')
                elif key == 'first_outcome_quality':
                    self.first_outcome_quality = value not in ('0', 'false')
                elif key == 'peach2_delay_ms':
                    self.peach2_delay_ms = int(value)
                elif key == 'complete_select_seq':
                    self.complete_select_seq = value.split(',')
                elif key == 'finish_delay_ms':
                    self.finish_delay_ms = int(value)
                elif key == 'cancel_reject':
                    self.cancel_reject_ids = {
                        item for item in value.split(',') if item}
                elif key == 'on_complete_clear_selected':
                    self.on_complete_clear_selected = value not in ('0', 'false')
                elif key == 'clear_fail_first':
                    self.clear_fail_first = int(value)
                elif key == 'publish_static_tf':
                    self.publish_static_tf = value not in ('0', 'false')

    def _on_complete(self, _req, response):
        with self._lock:
            self.complete_calls += 1
            ok = self.complete_success
            if ok and self.on_complete_clear_selected:
                self.selected = ''
            if ok and self.complete_select_seq is not None:
                idx = self.complete_calls - 1
                if idx < len(self.complete_select_seq):
                    self.selected = self.complete_select_seq[idx]
        response.success = ok
        response.message = '已推进' if ok else '（假）感知拒绝推进'
        return response

    def _on_reset(self, _req, response):
        with self._lock:
            self.reset_calls += 1
            self.pose_order.append('reset')
        response.success = True
        response.message = '已重置'
        return response

    def _on_clear(self, _req, response):
        with self._lock:
            self.clear_calls += 1
            self.pose_order.append('clear')
            ok = self.clear_success
            if self.clear_fail_first and self.clear_calls <= self.clear_fail_first:
                ok = False
        response.success = ok
        response.message = '已清空身份记忆' if ok else '（假）感知拒绝清记忆'
        return response

    def _on_reopen(self, request, response):
        response.success = True
        response.message = 'reopened ' + request.target_id
        return response

    def _tick_obs(self):
        msg = PeachTargetObservationArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            locked = self.targets_locked
            ids = list(self.target_ids)
            selected = self.selected
            collecting = self.collecting_count
            pending = self.pending_count
        msg.target_set_locked = locked
        if locked:
            msg.target_count = len(ids)
            msg.selected_target_id = selected
            msg.collecting_count = len(ids)
            msg.pending_count = 0
            for target_id in ids:
                item = PeachTargetObservation()
                item.header = msg.header
                item.target_id = target_id
                item.confirmed = True
                item.selected = target_id == selected
                msg.observations.append(item)
        else:
            msg.collecting_count = collecting
            msg.pending_count = pending
        self._obs_pub.publish(msg)

    def _tick_recon(self):
        msg = ReconstructionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._recon_pub.publish(msg)

    def _tick_robot(self):
        msg = RobotStatus()
        with self._lock:
            stopped = self.estop
            want_tf = self.publish_static_tf
        msg.e_stopped = 1 if stopped else 0
        msg.drives_powered = 0 if stopped else 1
        msg.motion_possible = 0 if stopped else 1
        self._robot_pub.publish(msg)
        if want_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'base_link'
            tf_msg.child_frame_id = 'camera_link'
            tf_msg.transform.rotation.w = 1.0
            self._tf.sendTransform(tf_msg)

    def _tick_status(self):
        with self._lock:
            payload = {
                'goals_received': self.goals_received,
                'cancels_received': self.cancels_received,
                'cycle_ids': list(self.cycle_ids),
                'goal_modes': list(self.goal_modes),
                'clear_calls': self.clear_calls,
                'reset_calls': self.reset_calls,
                'pose_order': list(self.pose_order),
                'complete_calls': self.complete_calls,
            }
        msg = String()
        msg.data = json.dumps(payload)
        self._status_pub.publish(msg)


class ApproachHelper:
    """普通 Node 上的假 RunTargetCycle，避免 Lifecycle 未激活导致不可见。."""

    def __init__(self, field: FakeField):
        self.field = field
        self.node = Node('peach_approach_grasp_node')
        group = ReentrantCallbackGroup()
        self.node.create_service(
            GetState, 'get_state', self._on_get_state, callback_group=group)
        self.node.create_service(
            Trigger, 'go_to_photo_pose', self._on_photo, callback_group=group)
        self._server = ActionServer(
            self.node, RunTargetCycle, 'run_target_cycle',
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=group)

    def _on_get_state(self, _req, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        response.current_state.label = 'active'
        return response

    def _on_photo(self, _req, response):
        response.success = True
        response.message = '已到拍照位姿'
        return response

    def _goal(self, goal_request):
        field = self.field
        with field._lock:
            field.goals_received += 1
            index = field.goals_received
            field.cycle_ids.append(goal_request.cycle_id)
            field.goal_modes.append(goal_request.mode)
            reject_all = field.reject_all
            reject_obs = field.reject_observe_only
            reject_n = field.reject_first_n
            estop_on_reject = field.estop_on_reject
        if reject_all:
            return GoalResponse.REJECT
        if reject_obs and goal_request.mode == RunTargetCycle.Goal.OBSERVE_ONLY:
            return GoalResponse.REJECT
        if reject_n and index <= reject_n:
            if estop_on_reject:
                with field._lock:
                    field.estop = True
                time.sleep(0.3)
                with field._lock:
                    field.estop = False
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, goal_handle):
        field = self.field
        target_id = goal_handle.request.target_id
        with field._lock:
            field.cancels_received += 1
            reject = target_id in field.cancel_reject_ids
        if reject:
            return CancelResponse.REJECT
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        field = self.field
        target_id = goal_handle.request.target_id
        with field._lock:
            delay_s = field.finish_delay_ms / 1000.0
            if target_id == 'peach_2' and field.peach2_delay_ms:
                delay_s = field.peach2_delay_ms / 1000.0
            index = field.goals_received
            quality = field.first_outcome_quality and index == 1
        deadline = time.monotonic() + delay_s
        result = RunTargetCycle.Result()
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.outcome = RunTargetCycle.Result.CANCELED
                result.reason = '（假）周期已取消'
                return result
            time.sleep(0.02)
        if quality:
            result.outcome = RunTargetCycle.Result.SKIPPED_QUALITY
            result.reason = '（假）质量未过'
            goal_handle.abort()
            return result
        result.outcome = RunTargetCycle.Result.SUCCEEDED
        result.reason = '（假）周期终局'
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    field = FakeField()
    helper = ApproachHelper(field)
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(field)
    executor.add_node(helper.node)
    field.trigger_configure()
    field.trigger_activate()
    try:
        executor.spin()
    finally:
        executor.shutdown()
        helper.node.destroy_node()
        field.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
