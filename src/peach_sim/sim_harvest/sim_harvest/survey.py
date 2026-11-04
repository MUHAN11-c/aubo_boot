"""Provide the simulation camera observation move through a serialized SurveyScene action."""

import math
import time

from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
import numpy as np
from peach_interfaces.action import MoveTo, SurveyScene
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformException, TransformListener

from .operations import OperationError


class SurveyAdapter:
    """Share the grasp server's command lock and check the actual observation TCP."""

    def __init__(self, node, ops, busy, ready_callback):
        self.node, self.ops = node, ops
        self.busy, self.ready = busy, ready_callback
        for name, default in [('survey_camera_position', [.20, -.15, 1.25]),
                              ('survey_look_at', [0., -.60, 1.25])]:
            node.declare_parameter(name, default)
        self.buffer = getattr(node, 'tf', None)
        self.listener = None
        if self.buffer is None:
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, node)
        self.server = ActionServer(
            node, SurveyScene, '/sim_harvest/survey_scene', self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=ops.group)
        self.move_server = ActionServer(
            node, MoveTo, '/sim_harvest/move_to', self.stow,
            goal_callback=self.accept_stow, cancel_callback=self.cancel,
            callback_group=ops.group)

    def accept_stow(self, request):
        """Expose the existing named-motion contract for a serialized return to stow."""
        if request.kind != MoveTo.Goal.KIND_NAMED or request.named_target != 'harvest_stow':
            return GoalResponse.REJECT
        return self.accept(request)

    def stow(self, goal):
        """Return from an empty survey without granting the mission a second arm writer."""
        result = MoveTo.Result()
        try:
            def canceled():
                return goal.is_cancel_requested
            self.node.lock_base(canceled)
            self.node.initialize_scene()
            goal.publish_feedback(MoveTo.Feedback(phase=MoveTo.Feedback.PHASE_MOVING))
            self.ops.move(named='harvest_stow', canceled=canceled)
            from .operations import named_values
            expected = named_values('harvest_stow')
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline:
                if canceled():
                    raise OperationError('canceled')
                current = dict(zip(self.node.joint_state.name, self.node.joint_state.position))
                if all(abs(current.get(k, math.inf) - v) < .04 for k, v in expected.items()):
                    result.arrived, result.detail = True, 'harvest_stow_verified'
                    goal.succeed()
                    return result
                time.sleep(.05)
            raise OperationError('stow_joint_residual')
        except Exception as exc:
            self.ops.cancel_all()
            result.detail = str(exc)
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
            return result
        finally:
            self.node.finish_transaction()
            self.busy.release()

    def accept(self, request):
        """Accept only a ready simulation with exclusive ownership of the arm."""
        del request
        if not self.ready() or not self.busy.acquire(blocking=False):
            return GoalResponse.REJECT
        if not self.ops.acquire_lease():
            self.busy.release()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def cancel(request):
        """Let the execute path propagate cancellation through the bounded motion adapter."""
        del request
        return CancelResponse.ACCEPT

    def _verify(self, goal, expected, forward):
        """Wait briefly for fresh current-state TF and verify the observation position."""
        deadline = time.monotonic() + 3.0
        reason = 'survey_tcp_tf_unavailable'
        while time.monotonic() < deadline:
            if goal.is_cancel_requested or not self.node.context.ok():
                raise OperationError('canceled')
            if not self.ready():
                raise OperationError('survey_simulation_not_ready')
            try:
                # Latest is only a post-motion state check, never an image transform.
                transform = self.buffer.lookup_transform(
                    'platform_base_link', 'camera_color_optical_frame', Time(),
                    timeout=Duration(seconds=0.2))
                stamp = Time.from_msg(transform.header.stamp).nanoseconds
                age = abs(self.node.get_clock().now().nanoseconds - stamp) / 1e9
                point = transform.transform.translation
                residual = math.dist([point.x, point.y, point.z], expected)
                q = transform.transform.rotation
                axis = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0., 0., 1.])
                angle = math.acos(float(np.clip(np.dot(axis, forward), -1., 1.)))
                if (stamp > 0 and age <= 1.0 and math.isfinite(residual)
                        and residual <= 0.025 and angle < math.radians(3)):
                    return residual
                reason = f'survey_tcp_residual:{residual:.4f},tf_age:{age:.3f}'
            except TransformException as exc:
                reason = 'survey_tcp_tf_unavailable:' + str(exc)
            time.sleep(0.05)
        raise OperationError(reason)

    def execute(self, goal):
        """Move to the simulation observation pose, verify it, and release the shared lock."""
        result = SurveyScene.Result(degraded=True)
        request_id = goal.request.request_id
        try:
            if goal.is_cancel_requested:
                raise OperationError('canceled')
            if not self.ready():
                raise OperationError('survey_simulation_not_ready')
            self.node.lock_base(lambda: goal.is_cancel_requested)
            self.node.initialize_scene()
            goal.publish_feedback(SurveyScene.Feedback(status='moving'))
            self.ops.event('survey_moving', request_id=request_id)
            position = np.array(self.node.get_parameter('survey_camera_position').value)
            look_at = np.array(self.node.get_parameter('survey_look_at').value)
            forward = look_at - position
            if not np.all(np.isfinite(forward)) or np.linalg.norm(forward) < .01:
                raise OperationError('invalid_survey_camera_geometry')
            forward /= np.linalg.norm(forward)
            right = np.cross(forward, [0., 0., 1.])
            if np.linalg.norm(right) < 1e-6:
                raise OperationError('survey_look_at_parallel_to_up')
            right /= np.linalg.norm(right)
            down = np.cross(forward, right)
            camera_rotation = Rotation.from_matrix(np.column_stack((right, down, forward)))
            # A fixed mounting transform is independent of the observation
            # exposure. Use the real robot TF chain for the selected tool.
            fixed = self.buffer.lookup_transform(
                'tcp', 'camera_color_optical_frame', Time(), timeout=Duration(seconds=.5))
            q, t = fixed.transform.rotation, fixed.transform.translation
            tcp_camera = Rotation.from_quat([q.x, q.y, q.z, q.w])
            rotation = camera_rotation * tcp_camera.inv()
            xyz = position - rotation.apply([t.x, t.y, t.z])
            q = rotation.as_quat()
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
            pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
            pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])

            def canceled():
                return goal.is_cancel_requested

            request = GetPositionIK.Request()
            request.ik_request.group_name = 'manipulator_e5'
            request.ik_request.ik_link_name = 'tcp'
            request.ik_request.avoid_collisions = True
            request.ik_request.robot_state.is_diff = True
            request.ik_request.robot_state.joint_state = self.node.joint_state
            request.ik_request.timeout.nanosec = 500000000
            request.ik_request.pose_stamped.header.frame_id = 'platform_base_link'
            request.ik_request.pose_stamped.pose = pose
            answer = self.ops.call(self.node.ik, request, canceled=canceled)
            if answer.error_code.val != 1:
                raise OperationError('survey_has_no_collision_free_ik')
            values = dict(zip(answer.solution.joint_state.name,
                              answer.solution.joint_state.position))
            from .operations import JOINTS
            self.ops.move(joints={name: values[name] for name in JOINTS}, canceled=canceled)
            residual = self._verify(goal, position, forward)
            if canceled():
                raise OperationError('canceled')
            result.snapshot_id = f'sim_survey_{time.time_ns()}'
            result.degraded = False
            result.message = 'simulation_observation_tcp_verified'
            self.ops.event('survey_confirmed', request_id=request_id,
                           snapshot_id=result.snapshot_id, residual_m=residual,
                           camera_position=position.tolist(), forward=forward.tolist())
            goal.succeed()
        except Exception as exc:
            self.ops.cancel_all()
            result.degraded = True
            result.message = str(exc)
            self.ops.event('survey_failed', request_id=request_id, reason=result.message)
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
        finally:
            self.node.finish_transaction()
            self.busy.release()
        return result
