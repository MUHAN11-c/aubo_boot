"""Bounded ROS calls executed by action callbacks under a multithreaded executor."""

import fcntl
import json
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from moveit_msgs.msg import OrientationConstraint, PositionConstraint
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

JOINTS = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint')


class OperationError(RuntimeError):
    """A failed, canceled or timed-out dependency."""


class Operations:
    """Thin action/service adapter; callbacks must run in another executor thread."""

    def __init__(self, node):
        self.node = node
        self.group = ReentrantCallbackGroup()
        self.motion = ActionClient(node, MoveGroup, '/move_action', callback_group=self.group)
        self.handles = []
        self.recovery_required = False
        self.lock = threading.Lock()
        self.events = node.create_publisher(String, '/sim_harvest/events', 10)
        self.run_dir = Path(node.get_parameter('run_directory').value).resolve()
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.log_lock = threading.Lock()
        self.lease = (self.run_dir / 'operation.lock').open('a')

    def acquire_lease(self):
        """Use one OS mutex for local navigation and arm action transactions."""
        try:
            fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return True
        except BlockingIOError:
            return False

    def release_lease(self):
        """Release this process's navigation/manipulation transaction lease."""
        fcntl.flock(self.lease, fcntl.LOCK_UN)

    def event(self, stage, **fields):
        """Publish and persist actual stage evidence with wall and ROS timestamps."""
        data = {'stage': stage, 'wall_time': time.time(),
                'ros_time_ns': self.node.get_clock().now().nanoseconds, **fields}
        line = json.dumps(data, ensure_ascii=False)
        self.events.publish(String(data=line))
        with self.log_lock, (self.run_dir / (self.node.get_name() + '.jsonl')).open('a') as out:
            out.write(line + '\n')
        self.node.get_logger().info(line)

    def wait(self, future, timeout, canceled=lambda: False):
        """Wait with a wall deadline; never nest spin or wait on simulated time."""
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        deadline = time.monotonic() + timeout
        while not ready.wait(0.05):
            if canceled() or not self.node.context.ok():
                raise OperationError('canceled')
            if time.monotonic() >= deadline:
                raise OperationError('dependency_timeout')
        error = future.exception()
        if error is not None:
            raise OperationError(str(error))
        return future.result()

    def call(self, client, request, timeout=15.0, canceled=lambda: False):
        """Call an existing ROS service with bounded discovery and response waits."""
        deadline = time.monotonic() + timeout
        while not client.wait_for_service(timeout_sec=0.1):
            if canceled() or time.monotonic() >= deadline:
                raise OperationError('service_unavailable:' + client.srv_name)
        return self.wait(client.call_async(request), max(.1, deadline - time.monotonic()),
                         canceled)

    def action(self, client, goal, timeout=90.0, canceled=lambda: False):
        """Await acceptance/result and cancel active or late goals on every failure."""
        deadline = time.monotonic() + min(timeout, 60.0)
        while not client.wait_for_server(timeout_sec=0.1):
            if canceled() or time.monotonic() >= deadline:
                raise OperationError('action_unavailable')
        accepted = client.send_goal_async(goal)
        try:
            handle = self.wait(accepted, 15.0, canceled)
        except OperationError:
            self.recovery_required = True

            def discard_late(future):
                if future.exception() is None:
                    late = future.result()
                    if late and late.accepted:
                        late.cancel_goal_async()
            accepted.add_done_callback(discard_late)
            raise
        if not handle.accepted:
            raise OperationError('goal_rejected')
        with self.lock:
            self.handles.append(handle)
        terminal = handle.get_result_async()
        try:
            result = self.wait(terminal, timeout, canceled)
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                code = getattr(getattr(result.result, 'error_code', None), 'val', '')
                reason = getattr(result.result, 'termination_reason', '')
                raise OperationError(f'action_status:{result.status},error:{code},reason:{reason}')
            return result.result
        except OperationError:
            if not terminal.done():
                try:
                    self.wait(handle.cancel_goal_async(), 5.0)
                    stopped = self.wait(terminal, 8.0)
                    if stopped.status not in (4, 5, 6):
                        raise OperationError('nonterminal_cancel_result')
                    self.event('cancel_confirmed', status=stopped.status)
                except OperationError:
                    self.recovery_required = True
                    self.event('cancel_unconfirmed')
            raise
        finally:
            with self.lock:
                if terminal.done() and handle in self.handles:
                    self.handles.remove(handle)

    def cancel_all(self):
        """Request cancellation without deadlocking executor shutdown."""
        with self.lock:
            handles = list(self.handles)
        for handle in handles:
            handle.cancel_goal_async()

    def move(self, pose=None, frame='map', named=None, joints=None, linear=False,
             canceled=lambda: False, timeout=90.0):
        """Execute a checked MoveGroup result using standard OMPL or Pilz goals."""
        req = MotionPlanRequest()
        req.group_name = 'manipulator_e5'
        req.pipeline_id = 'pilz_industrial_motion_planner' if linear else 'ompl'
        req.planner_id = 'LIN' if linear else 'RRTConnect'
        req.allowed_planning_time = 8.0
        req.num_planning_attempts = 5 if not linear else 1
        req.max_velocity_scaling_factor = 0.25
        req.max_acceleration_scaling_factor = 0.25
        req.start_state.is_diff = True
        constraint = Constraints()
        if named or joints:
            positions = joints if joints is not None else named_values(named)
            for joint, value in positions.items():
                constraint.joint_constraints.append(JointConstraint(
                    joint_name=joint, position=value, tolerance_above=.01,
                    tolerance_below=.01, weight=1.0))
        else:
            position = PositionConstraint(link_name='tcp', weight=1.0)
            position.header.frame_id = frame
            position.constraint_region.primitives = [SolidPrimitive(
                type=SolidPrimitive.SPHERE, dimensions=[.005])]
            region_pose = Pose()
            region_pose.position = pose.position
            region_pose.orientation.w = 1.0
            position.constraint_region.primitive_poses = [region_pose]
            orientation = OrientationConstraint(
                link_name='tcp', orientation=pose.orientation,
                absolute_x_axis_tolerance=.05, absolute_y_axis_tolerance=.05,
                absolute_z_axis_tolerance=.05, weight=1.0)
            orientation.header.frame_id = frame
            constraint.position_constraints = [position]
            constraint.orientation_constraints = [orientation]
        req.goal_constraints = [constraint]
        goal = MoveGroup.Goal(request=req)
        goal.planning_options.plan_only = False
        result = self.action(self.motion, goal, timeout, canceled)
        if result.error_code.val != 1:
            raise OperationError(f'moveit_error:{result.error_code.val}')
        self.event('motion_completed', named=named or '', linear=linear)
        return result


def named_values(name):
    """Read current SRDF named states instead of maintaining a second joint snapshot."""
    path = Path(get_package_share_directory('aubo_e5_moveit_config')) / 'config/aubo_e5.srdf'
    state = ET.parse(path).find(f".//group_state[@name='{name}']")
    if state is None:
        raise OperationError('unknown_named_state:' + name)
    return {item.get('name'): float(item.get('value')) for item in state.findall('joint')}
