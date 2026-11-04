"""Run a cancelable Nav2-to-Pickup harvesting mission in an isolated simulation."""

import math
import os
from pathlib import Path
import threading
import time

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from moveit_msgs.action import Pickup
from moveit_msgs.msg import Grasp
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import numpy as np
from peach_interfaces.action import MoveTo, RunHarvest, SurveyScene
from peach_interfaces.msg import HarvestState, TargetOutcome
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .operations import named_values, OperationError, Operations


class MissionNode(Node):
    """Keep navigation and arm action execution sequential and evidence driven."""

    def __init__(self):
        super().__init__('sim_mission')
        for name, default in [('manifest_file', ''), ('stations_file', ''),
                              ('run_directory', ''), ('autostart', False),
                              ('max_targets', 1), ('navigation_timeout', 120.0),
                              ('perception_mode', 'ground_truth'),
                              ('tool_profile', 'adaptive_shear_v1')]:
            self.declare_parameter(name, default)
        manifest = yaml.safe_load(Path(self.get_parameter('manifest_file').value).read_text())
        self.targets = {item['id']: item for item in manifest['targets']}
        stations = yaml.safe_load(Path(self.get_parameter('stations_file').value).read_text())
        self.stations = stations['stations']
        self.ops = Operations(self)
        self.group = ReentrantCallbackGroup()
        self.visual = None
        if self.get_parameter('perception_mode').value == 'vision':
            from .vision_target import VisionTarget
            self.visual = VisionTarget(self, self.ops)
        self.lifecycle_clients = [self.create_client(
            GetState, '/' + name + '/get_state', callback_group=self.group)
            for name in ('amcl', 'bt_navigator')]
        self.controllers = self.create_client(
            ListControllers, '/controller_manager/list_controllers', callback_group=self.group)
        self.health_futures = []
        self.health_started = 0.
        self.dependencies_active = False
        self.health_count = 0
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                callback_group=self.group)
        self.survey = ActionClient(self, SurveyScene, '/sim_harvest/survey_scene',
                                   callback_group=self.group)
        self.return_stow = ActionClient(self, MoveTo, '/sim_harvest/move_to',
                                        callback_group=self.group)
        self.pick = ActionClient(self, Pickup, '/sim_harvest/pickup',
                                 callback_group=self.group)
        self.clock_seen, self.joint_seen = 0.0, 0.0
        self.joints = {}
        self.base_still_since = None
        self.odom_seen = 0.
        self.create_subscription(Odometry, '/odom', self.on_odom,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data,
                                 callback_group=self.group)
        self.create_subscription(JointState, '/joint_states', self.on_joints,
                                 qos_profile_sensor_data, callback_group=self.group)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grasp_recovery = False
        self.create_subscription(Bool, '/sim_harvest/recovery_required',
                                 self.on_recovery, latched, callback_group=self.group)
        self.publisher = self.create_publisher(HarvestState, '/sim_harvest/state', latched)
        self.markers = self.create_publisher(MarkerArray, '/sim_harvest/targets', latched)
        self.busy = threading.Lock()
        self.completed = set()
        self.completed_positions = []
        self.completed_visual_ids = {}
        self.state = HarvestState(navigation_enabled=True)
        self.server = ActionServer(self, RunHarvest, '/sim_harvest/run_harvest', self.execute,
                                   goal_callback=self.accept,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        self.auto_client = ActionClient(self, RunHarvest, '/sim_harvest/run_harvest',
                                        callback_group=self.group)
        self.timer = self.create_timer(.5, self.tick, callback_group=self.group)
        self.autostart_pending = self.get_parameter('autostart').value
        self.publish_targets()
        self.emit(HarvestState.WAITING_READY, 'waiting_for_run_harvest')

    def on_recovery(self, msg):
        self.grasp_recovery = msg.data

    def on_clock(self, _):
        self.clock_seen = time.monotonic()

    def on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        self.joint_seen = time.monotonic()

    def on_odom(self, msg):
        """Observe deceleration; navigation success alone does not prove a stopped base."""
        self.odom_seen = time.monotonic()
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        if math.hypot(v.x, v.y) < .01 and abs(w.z) < .02:
            self.base_still_since = self.base_still_since or self.odom_seen
        else:
            self.base_still_since = None

    def wait_stopped(self, canceled):
        """Wait for one second of measured rest before starting any arm operation."""
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if canceled():
                raise OperationError('canceled')
            if (self.base_still_since is not None
                    and time.monotonic() - self.base_still_since > 1.
                    and time.monotonic() - self.odom_seen < 1.):
                self.ops.event('base_stopped')
                return
            time.sleep(.05)
        raise OperationError('base_did_not_stop')

    def ready(self):
        """Require fresh simulated clock, controller state and action dependencies."""
        return (os.environ.get('PEACH_SIM_ISOLATED') == '1'
                and self.get_parameter('use_sim_time').value
                and time.monotonic() - self.clock_seen < 3
                and time.monotonic() - self.joint_seen < 3
                and not self.ops.recovery_required and not self.grasp_recovery
                and self.dependencies_active
                and self.nav.server_is_ready() and self.pick.server_is_ready())

    def stowed(self):
        return all(name in self.joints and abs(self.joints[name] - value) < .08
                   for name, value in named_values('harvest_stow').items())

    def tick(self):
        """Start a requested automatic simulation run only after dependencies are ready."""
        if self.health_futures and all(f.done() for f in self.health_futures):
            try:
                states = [f.result() for f in self.health_futures]
                controllers = {c.name: c.state for c in states[-1].controller}
                self.dependencies_active = (all(s.current_state.id == 3 for s in states[:-1])
                                            and all(controllers.get(name) == 'active' for name in (
                                                'joint_state_broadcaster',
                                                'joint_trajectory_controller')))
            except Exception:
                self.dependencies_active = False
            self.health_futures = []
        if self.health_futures and time.monotonic() - self.health_started > 3.0:
            self.dependencies_active = False
            for client, future in zip((*self.lifecycle_clients, self.controllers),
                                      self.health_futures):
                client.remove_pending_request(future)
                future.cancel()
            self.health_futures = []
        if not self.health_futures and all(c.service_is_ready() for c in (
                *self.lifecycle_clients, self.controllers)):
            self.health_started = time.monotonic()
            self.health_futures = [c.call_async(GetState.Request())
                                   for c in self.lifecycle_clients]
            self.health_futures.append(self.controllers.call_async(ListControllers.Request()))
        self.health_count += 1
        if self.health_count % 10 == 0:
            self.ops.event('readiness', ready=self.ready(), deps=self.dependencies_active,
                           clock_age=time.monotonic() - self.clock_seen,
                           joint_age=time.monotonic() - self.joint_seen,
                           nav=self.nav.server_is_ready(), pickup=self.pick.server_is_ready(),
                           health_done=[f.done() for f in self.health_futures])
        if self.autostart_pending and self.ready():
            self.autostart_pending = False
            self.auto_client.send_goal_async(RunHarvest.Goal(
                request_id=f'auto_{time.time_ns()}', scene_key='harvest_demo'))

    def accept(self, request):
        valid = (self.ready() and bool(request.request_id)
                 and len(self.completed_positions) < 2
                 and request.intent in (RunHarvest.Goal.INTENT_PICK_ALL,
                                        RunHarvest.Goal.INTENT_PICK_SELECTED)
                 and (self.visual is None or (not request.target_ids
                      and request.intent == RunHarvest.Goal.INTENT_PICK_ALL))
                 and all(key in self.targets for key in request.target_ids)
                 and (request.intent != RunHarvest.Goal.INTENT_PICK_SELECTED
                      or bool(request.target_ids)))
        if not valid or not self.busy.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def emit(self, state, message, target='', goal=None):
        """Publish simulation-specific state and action feedback from one snapshot."""
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.state.header.frame_id = 'map'
        self.state.state_seq += 1
        self.state.revision = self.state.state_seq
        self.state.batch_state, self.state.message = state, message
        self.state.target_id = target
        self.state.action_active = state in (HarvestState.RUNNING, HarvestState.NAVIGATING)
        self.publisher.publish(self.state)
        self.publish_targets()
        if goal:
            goal.publish_feedback(RunHarvest.Feedback(state=self.state))
        self.ops.event(message, target=target)

    def publish_targets(self):
        markers = []
        for index, target in enumerate(self.targets.values()):
            marker = Marker(id=index, ns='harvest_targets', type=Marker.CUBE,
                            action=Marker.ADD)
            marker.header.frame_id = 'map'
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                target['collision_center'])
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = target['collision_size']
            marker.color.r, marker.color.g, marker.color.a = .8, .5, .75
            if target['id'] in self.completed:
                marker.action = Marker.DELETE
            markers.append(marker)
        status = Marker(id=100000, ns='mission_state', type=Marker.TEXT_VIEW_FACING,
                        action=Marker.ADD)
        status.header.frame_id = 'platform_base_link'
        status.pose.position.z = 1.8
        status.pose.orientation.w = 1.
        status.scale.z = .12
        status.color.r = status.color.g = status.color.b = status.color.a = 1.
        status.text = self.state.message
        if status.text:
            markers.append(status)
        self.markers.publish(MarkerArray(markers=markers))

    def execute(self, goal):
        """Navigate to each selected target and count only verified completed pickups."""
        result = RunHarvest.Result()
        result.summary.run_id = goal.request.request_id
        self.state.run_id = goal.request.request_id

        def canceled():
            return goal.is_cancel_requested
        start = time.monotonic()
        allowed = set(goal.request.target_ids or self.targets)
        if self.visual is not None:
            plan = list(self.stations)
        else:
            plan = []
            for station in self.stations:
                keys = ([station['target_id']] if 'target_id' in station else
                        [key for key, item in self.targets.items()
                         if item['tree'] == station['tree_id']])
                plan.extend(dict(station, target_id=key) for key in keys
                            if key in allowed and key not in self.completed)
        limit = self.get_parameter('max_targets').value
        capacity_left = 2 - len(self.completed_positions)
        budget = min(limit, capacity_left) if limit > 0 else capacity_left
        if self.visual is None and limit > 0:
            plan = plan[:limit]
        result.summary.discovered = 0 if self.visual else len(plan)
        observed_ids = set()
        previous_station = None
        reason = 'scene_exhausted'
        try:
            if not plan:
                raise OperationError('no_unharvested_targets')
            while plan and result.summary.succeeded < budget:
                station = plan.pop(0)
                key = station.get('target_id', station['id'])
                if canceled():
                    raise OperationError('canceled')
                if previous_station != station['id']:
                    self.navigate(station, key, goal, canceled)
                    previous_station = station['id']
                target_start = time.monotonic()
                target = None if self.visual else self.targets[key]
                if self.visual is not None:
                    observed = self.ops.action(self.survey, SurveyScene.Goal(
                        request_id=goal.request.request_id, scene_key='sim_visual'),
                        90., canceled)
                    if observed.degraded:
                        raise OperationError('survey_failed:' + observed.message)
                    scene_key = 'sim_visual/' + station['id']
                    completed_ids = self.completed_visual_ids.setdefault(scene_key, set())
                    try:
                        target = self.visual.resolve(canceled, scene_key, completed_ids)
                    except OperationError as exc:
                        if str(exc) != 'vision_no_accepted_observed_target':
                            raise
                        observed_ids.update((scene_key, item)
                                            for item in self.visual.locked_ids)
                        result.summary.discovered = len(observed_ids)
                        self.ops.event('survey_no_pickable_target', station=station['id'])
                        self.ops.action(self.return_stow, MoveTo.Goal(
                            kind=MoveTo.Goal.KIND_NAMED, named_target='harvest_stow'),
                            90., canceled)
                        continue
                    key = target['id']
                    observed_ids.update((scene_key, item) for item in target['observed_ids'])
                    result.summary.discovered = len(observed_ids)
                    self.ops.event('visual_model_confirmed', target=key,
                                   source_target_id=target['source_target_id'])
                grasp = Grasp(id=key)
                if self.visual:
                    # Candidate ID binds the standard Pickup to the existing
                    # GraspDecision revision; no parallel permission message.
                    grasp.id = target['grasp_id']
                grasp.grasp_pose.header.frame_id = 'map'
                grasp.grasp_pose.pose.position.x = float(target['bag_bottom'][0])
                grasp.grasp_pose.pose.position.y = float(target['bag_bottom'][1])
                grasp.grasp_pose.pose.position.z = float(target['bag_bottom'][2])
                # A single axis leaves tool roll unspecified in scipy 1.11.
                # Add a projected world direction to define a stable full pose.
                axis = np.asarray(target['axis'], dtype=float)
                axis /= np.linalg.norm(axis)
                reference = np.array([1., 0., 0.] if abs(axis[0]) < .9 else [0., 1., 0.])
                reference -= axis * np.dot(reference, axis)
                reference /= np.linalg.norm(reference)
                rotation, _ = Rotation.align_vectors(
                    [axis, reference], [[0., 0., 1.], [1., 0., 0.]])
                q = rotation.as_quat()
                grasp.grasp_pose.pose.orientation.x = float(q[0])
                grasp.grasp_pose.pose.orientation.y = float(q[1])
                grasp.grasp_pose.pose.orientation.z = float(q[2])
                grasp.grasp_pose.pose.orientation.w = float(q[3])
                pickup = Pickup.Goal(target_name=key, group_name='manipulator_e5',
                                     possible_grasps=[grasp])
                result.summary.attempted += 1
                timeout = goal.request.per_target_timeout_s or 240.0
                picked = self.ops.action(self.pick, pickup, timeout, canceled)
                if picked.error_code.val != 1:
                    raise OperationError(f'pickup_error:{picked.error_code.val}')
                self.completed.add(key)
                self.completed_positions.append(list(target['bag_bottom']))
                if self.visual:
                    completed_ids.add(key)
                result.summary.succeeded += 1
                outcome = TargetOutcome(target_id=key, outcome=TargetOutcome.SUCCEEDED,
                                        reason='gazebo_basket_confirmed')
                elapsed = time.monotonic() - target_start
                outcome.elapsed.sec = int(elapsed)
                result.summary.outcomes.append(outcome)
                self.state.progress = result.summary.succeeded / max(1, budget)
                self.publish_targets()
                self.emit(HarvestState.RUNNING, 'target_completed', key, goal)
                ratio = goal.request.target_harvest_ratio
                if (ratio > 0 and result.summary.succeeded
                        / max(1, result.summary.discovered) >= ratio):
                    reason = 'target_ratio_reached'
                    break
                if self.visual:
                    plan.insert(0, station)
            if result.summary.succeeded >= budget:
                reason = ('basket_capacity_reached' if budget == capacity_left
                          else 'target_limit_reached')
            result.success = result.summary.succeeded > 0
            result.termination_reason = reason if result.success else 'no_targets_succeeded'
            self.emit(HarvestState.COMPLETED, 'mission_completed', goal=goal)
            goal.succeed()
        except Exception as exc:
            self.ops.cancel_all()
            result.success = False
            result.termination_reason = str(exc)
            result.summary.canceled += int(canceled())
            result.summary.failed += int(not canceled())
            self.emit(HarvestState.INTERRUPTED, 'mission_interrupted', goal=goal)
            if canceled():
                goal.canceled()
            else:
                goal.abort()
        finally:
            self.ops.release_lease()
            elapsed = time.monotonic() - start
            result.summary.elapsed.sec = int(elapsed)
            self.busy.release()
            self.ops.event('mission_result', success=result.success,
                           succeeded=result.summary.succeeded,
                           reason=result.termination_reason)
        return result

    def navigate(self, station, key, goal, canceled):
        """Navigate only after the existing shared operation lease and measured stow gate."""
        if not self.ops.acquire_lease():
            raise OperationError('navigation_refused_arm_operation_active')
        try:
            if not self.ready() or not self.stowed():
                raise OperationError('navigation_refused_arm_not_stowed_or_stale_state')
            self.emit(HarvestState.NAVIGATING, 'navigating', key, goal)
            x, y, yaw = station['nav_pose']
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.z, pose.pose.orientation.w = (
                math.sin(yaw / 2), math.cos(yaw / 2))
            self.ops.action(self.nav, NavigateToPose.Goal(pose=pose),
                            self.get_parameter('navigation_timeout').value, canceled)
            self.wait_stopped(canceled)
        finally:
            self.ops.release_lease()
        self.emit(HarvestState.RUNNING, 'navigation_completed', key, goal)


def main(args=None):
    """Run mission callbacks and their action dependencies in separate executor threads."""
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.ops.cancel_all()
        executor.shutdown(timeout_sec=8)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
