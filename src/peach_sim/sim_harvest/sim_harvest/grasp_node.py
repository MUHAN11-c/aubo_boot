"""Execute simulation-only Pickup stages with Gazebo and PlanningScene evidence."""

from copy import deepcopy
from functools import partial
import os
from pathlib import Path
import threading
import time

from geometry_msgs.msg import Pose
from moveit_msgs.action import Pickup
from moveit_msgs.msg import AllowedCollisionEntry, AttachedCollisionObject, CollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionIK, GetStateValidity
from nav_msgs.msg import Odometry
import numpy as np
from peach_interfaces.msg import GraspDecision
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from ros_gz_interfaces.msg import Contacts, Entity
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from sim_worlds.demo import fruit_sdf
from std_msgs.msg import Bool, Empty, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
import yaml

from .operations import JOINTS, OperationError, Operations
from .vision_target import visual_model_identity


class GraspNode(Node):
    """Serialize all simulation arm commands through a standard Pickup action."""

    def __init__(self):
        super().__init__('sim_grasp')
        for name, default in [('manifest_file', ''), ('run_directory', ''),
                              ('robot_description_file', ''), ('perception_mode', 'ground_truth'),
                              ('tool_profile', 'adaptive_shear_v1')]:
            self.declare_parameter(name, default)
        manifest = yaml.safe_load(
            Path(self.get_parameter('manifest_file').value).read_text())
        self.targets = {item['id']: item for item in manifest['targets']}
        self.obstacles = manifest.get('obstacles', [])
        self.ops = Operations(self)
        self.group = ReentrantCallbackGroup()
        self.busy = threading.Lock()
        self.recovery_required = False
        self.clock_seen = 0.0
        self.joint_state = JointState()
        self.still_since = None
        self.odom_seen = 0.
        self.guard_locked = False
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.arm_busy_pub = self.create_publisher(Bool, '/sim_harvest/arm_busy', latch)
        self.recovery_pub = self.create_publisher(Bool, '/sim_harvest/recovery_required', latch)
        self.create_subscription(Bool, '/sim_harvest/base_locked', self._guard,
                                 latch, callback_group=self.group)
        self.create_subscription(JointState, '/joint_states', self._joints,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(Odometry, '/odom', self._odom,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data,
                                 callback_group=self.group)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.states, self.poses = {}, {}
        self.visual_decision = None
        self.create_subscription(GraspDecision, '/peach/reconstruction/grasp_decision',
                                 self._decision, latch, callback_group=self.group)
        self.collected, self.contact_times, self.basket_bounds = set(), {}, {}
        self.pose_frames_logged = False
        self.detach = {}
        for key in self.targets:
            self.create_subscription(String, f'/sim_harvest/{key}/state',
                                     partial(self._state, key),
                                     10, callback_group=self.group)
            self.detach[key] = self.create_publisher(Empty, f'/sim_harvest/{key}/detach', 10)
        self.create_subscription(TFMessage, '/sim_harvest/entity_poses', self._poses,
                                 10, callback_group=self.group)
        self.create_subscription(Contacts, '/sim_harvest/basket_contacts', self._contacts,
                                 10, callback_group=self.group)
        self.spawn = self.create_client(SpawnEntity, '/world/peach_orchard/create',
                                        callback_group=self.group)
        self.remove = self.create_client(DeleteEntity, '/world/peach_orchard/remove',
                                         callback_group=self.group)
        self.ik = self.create_client(GetPositionIK, '/compute_ik',
                                     callback_group=self.group)
        self.validity = self.create_client(GetStateValidity, '/check_state_validity',
                                           callback_group=self.group)
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                        callback_group=self.group)
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene',
                                            callback_group=self.group)
        self.server = ActionServer(self, Pickup, '/sim_harvest/pickup', self.execute,
                                   goal_callback=self.accept,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=self.group)
        from .survey import SurveyAdapter
        self.survey = SurveyAdapter(self, self.ops, self.busy, self.simulation_ready)
        self.arm_busy_pub.publish(Bool(data=False))
        self.recovery_pub.publish(Bool(data=False))
        self.ops.event('pickup_ready', targets=len(self.targets))

    def _guard(self, msg):
        self.guard_locked = msg.data

    def _decision(self, msg):
        self.visual_decision = msg

    def check_visual_permission(self, visual_id, revision, stage):
        """Recheck the shared, bounded decision at every motion/cut boundary."""
        if self.get_parameter('perception_mode').value != 'vision':
            return
        decision = self.visual_decision
        now = self.get_clock().now().nanoseconds
        if (decision is None or not revision or decision.target_id != visual_id
                or not decision.model_revision or visual_model_identity(decision) != revision
                or not decision.harvest_run_id
                or decision.scene_epoch == 0 or not decision.calibration_revision
                or not decision.config_revision
                or decision.tool_profile_id != self.get_parameter('tool_profile').value
                or now <= 0 or rclpy.time.Time.from_msg(decision.valid_until).nanoseconds <= now):
            raise OperationError('visual_decision_identity_or_validity_invalid')
        capabilities = [decision.geometry_capability, decision.pregrasp_capability]
        if stage in ('insert', 'cut'):
            capabilities.append(decision.sleeve_capability)
        if stage == 'cut':
            capabilities.append(decision.cut_capability)
        if any(value != 0 for value in capabilities):
            raise OperationError('visual_stage_not_permitted:' + stage)

    def _joints(self, msg):
        self.joint_state = msg

    def _odom(self, msg):
        self.odom_seen = time.monotonic()
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        if np.hypot(v.x, v.y) < .01 and abs(w.z) < .02:
            self.still_since = self.still_since or self.odom_seen
        else:
            self.still_since = None

    def lock_base(self, canceled):
        """Close the command interlock and observe a stopped base before arm motion."""
        self.arm_busy_pub.publish(Bool(data=True))
        self.wait_until(lambda: self.guard_locked and self.still_since is not None
                        and time.monotonic() - self.still_since > .5
                        and time.monotonic() - self.odom_seen < 1., 10., canceled)

    def finish_transaction(self):
        """Keep the base locked if any physical or motion state remains uncertain."""
        recovery = self.recovery_required or self.ops.recovery_required
        try:
            if self.context.ok():
                self.recovery_pub.publish(Bool(data=recovery))
                self.arm_busy_pub.publish(Bool(data=recovery))
        finally:
            self.ops.release_lease()

    def _state(self, key, msg):
        self.states[key] = msg.data

    def _contacts(self, msg):
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if not any('harvest_basket' in name for name in names):
                continue
            for key in self.targets:
                if any(key + '_picked::' in name for name in names):
                    self.contact_times[key] = time.monotonic()

    def _on_clock(self, _):
        self.clock_seen = time.monotonic()

    def _poses(self, msg):
        stamp = time.monotonic()
        if not self.pose_frames_logged:
            self.pose_frames_logged = True
            self.ops.event('gazebo_pose_frames', frames=[
                [t.header.frame_id, t.child_frame_id] for t in msg.transforms[:12]])
        for item in msg.transforms:
            # Gazebo's bridge provides model names (possibly world-scoped).
            name = item.child_frame_id.split('::')[-1].split('/')[-1]
            self.poses[name] = (item.transform, stamp)

    def simulation_ready(self):
        """Require a fresh, explicitly isolated Gazebo execution chain."""
        model = Path(self.get_parameter('robot_description_file').value)
        return (os.environ.get('PEACH_SIM_ISOLATED') == '1'
                and os.environ.get('PEACH_SIM_IN_SANDBOX') == '1'
                and self.get_parameter('use_sim_time').value
                and time.monotonic() - self.clock_seen < 3.0
                and not self.recovery_required and not self.ops.recovery_required
                and self.ops.motion.server_is_ready()
                and self.spawn.service_is_ready() and self.remove.service_is_ready()
                and model.is_file()
                and 'gz_ros2_control/GazeboSimSystem' in model.read_text())

    def accept(self, request):
        """Refuse non-simulation, stale-clock, unknown, preview or concurrent goals."""
        visual = self.get_parameter('perception_mode').value == 'vision'
        valid = (self.simulation_ready() and bool(request.target_name)
                 and request.group_name == 'manipulator_e5'
                 and (visual or request.target_name in self.targets)
                 and (visual or request.target_name not in self.collected)
                 and len(self.collected) < 2
                 and len(request.possible_grasps) == 1
                 and not request.planning_options.plan_only)
        if not valid or not self.busy.acquire(blocking=False):
            return GoalResponse.REJECT
        if not self.ops.acquire_lease():
            self.busy.release()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def stage(self, goal, name):
        """Publish Pickup stage feedback and a persistent evidence event."""
        goal.publish_feedback(Pickup.Feedback(state=name))
        self.ops.event(name, target=goal.request.target_name)

    def wait_until(self, predicate, timeout, canceled):
        deadline = time.monotonic() + timeout
        while not predicate():
            if canceled() or time.monotonic() >= deadline or not self.context.ok():
                raise OperationError('canceled' if canceled() else 'evidence_timeout')
            time.sleep(.05)

    def apply(self, objects=None, attached=None, matrix=None):
        """Apply planning geometry synchronously before executing the next segment."""
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objects or []
        scene.robot_state.attached_collision_objects = attached or []
        if matrix is not None:
            scene.allowed_collision_matrix = matrix
        result = self.ops.call(self.scene, ApplyPlanningScene.Request(scene=scene))
        if not result.success:
            raise OperationError('planning_scene_rejected')

    def allow_target_contact(self, key):
        """Preserve the robot ACM and permit only this bag against tool contact links."""
        answer = self.ops.call(self.get_scene, GetPlanningScene.Request(
            components=PlanningSceneComponents(
                components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)))
        original = answer.scene.allowed_collision_matrix
        matrix = deepcopy(original)
        links = ['wrist3_Link', 'tcp', 'tool_body_link', 'sleeve_mouth']
        for name in [key, *links]:
            if name in matrix.entry_names:
                continue
            matrix.entry_names.append(name)
            for row in matrix.entry_values:
                row.enabled.append(False)
            matrix.entry_values.append(AllowedCollisionEntry(
                enabled=[False] * len(matrix.entry_names)))
        index = matrix.entry_names.index(key)
        for link in links:
            other = matrix.entry_names.index(link)
            matrix.entry_values[index].enabled[other] = True
            matrix.entry_values[other].enabled[index] = True
        self.apply(matrix=matrix)
        return original

    def collision_object(self, key, center, frame='map', operation=CollisionObject.ADD,
                         dimensions=(.075, .060, .080)):
        obj = CollisionObject(id=key, operation=operation)
        obj.header.frame_id = frame
        if operation == CollisionObject.ADD:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, center)
            pose.orientation.w = 1.0
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                             dimensions=list(dimensions))]
            obj.primitive_poses = [pose]
        return obj

    def initialize_scene(self):
        """Populate the planner with the same obstacle proxies as the Gazebo world."""
        objects = []
        for item in self.obstacles + [{'id': 'ground', 'position': [0, 0, -.06],
                                      'size': [12., 12., .1]}]:
            obj = self.collision_object(item['id'], item['position'])
            if item.get('shape') == 'cylinder':
                obj.primitives[0].type = SolidPrimitive.CYLINDER
                obj.primitives[0].dimensions = [item['length'], item['radius']]
                q = Rotation.from_euler('xyz', item['orientation']).as_quat()
                pose = obj.primitive_poses[0]
                pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
                pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])
            else:
                obj.primitives[0].dimensions = [float(value) for value in item['size']]
            objects.append(obj)
        for key, target in self.targets.items():
            if key not in self.collected:
                objects.append(self.collision_object(
                    key, target['collision_center'], dimensions=target['collision_size']))
        self.apply(objects=objects)
        # Loads remain free rigid bodies during base motion. Refresh their
        # measured poses before planning the next arm operation.
        for key in self.collected:
            self.record_collected_load(key)
        self.ops.event('planning_scene_confirmed', obstacles=len(objects))

    def actual_contact_residual(self, target):
        """Compose actual Gazebo model pose with measured arm TF, independently of AMCL."""
        robot = self.poses.get('harvester')
        if robot is None or time.monotonic() - robot[1] > 2:
            raise OperationError('missing_gazebo_robot_pose')
        local = self.tf.lookup_transform('platform_base_link', 'tcp', rclpy.time.Time())
        point, origin, q = (local.transform.translation, robot[0].translation,
                            robot[0].rotation)
        world = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply(
            [point.x, point.y, point.z]) + np.array([origin.x, origin.y, origin.z])
        residual = float(np.linalg.norm(world - np.array(target['bag_bottom'])))
        self.ops.event('contact_checked', actual_tcp=world.tolist(),
                       expected=target['bag_bottom'], residual_m=residual,
                       robot_position=[origin.x, origin.y, origin.z])
        return residual

    def confirm_in_basket(self, key, after=0):
        """Check actual Gazebo model position in the moving robot's basket bounds."""
        fruit = self.poses.get(key + '_picked')
        robot = self.poses.get('harvester')
        if fruit is None or robot is None:
            return False
        if fruit[1] <= after or time.monotonic() - min(fruit[1], robot[1]) > 2:
            return False
        fp, rp = fruit[0].translation, robot[0].translation
        q = robot[0].rotation
        local = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv().apply(
            np.array([fp.x-rp.x, fp.y-rp.y, fp.z-rp.z]))
        fq = fruit[0].rotation
        inverse = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv()
        relative = inverse * Rotation.from_quat([fq.x, fq.y, fq.z, fq.w])
        target = self.targets[key]
        corners = np.array([[x, y, z] for x in (-.5, .5)
                            for y in (-.5, .5) for z in (-.5, .5)])
        corners = relative.apply(corners * target['collision_size']
                                 + target['collision_center_local']) + local
        low, high = corners.min(axis=0), corners.max(axis=0)
        self.basket_bounds[key] = [low.tolist(), high.tolist()]
        return (low[0] > -.49 and high[0] < -.19 and low[1] > -.14 and high[1] < .14
                and .55 < low[2] < .58 and high[2] < .73
                and self.contact_times.get(key, 0) > after
                and time.monotonic() - self.contact_times.get(key, 0) < .5)

    def confirm_settled(self, key, after, canceled):
        """Require fresh observations inside the basket continuously for one second."""
        stable_since = None
        previous, previous_stamp = None, 0.
        deadline = time.monotonic() + 12.
        while time.monotonic() < deadline:
            if canceled():
                raise OperationError('canceled')
            item = self.poses.get(key + '_picked')
            valid = self.confirm_in_basket(key, after)
            if item and item[1] > previous_stamp:
                p = item[0].translation
                current = np.array([p.x, p.y, p.z])
                speed = (float(np.linalg.norm(current - previous)) /
                         max(.001, item[1] - previous_stamp)) if previous is not None else 1.
                if valid and speed < .04:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 1.0:
                        self.ops.event('load_settled', target=key,
                                       bounds_in_base=self.basket_bounds[key],
                                       contact_age_s=time.monotonic() - self.contact_times[key],
                                       speed_m_s=speed)
                        return
                else:
                    stable_since = None
                previous, previous_stamp = current, item[1]
            time.sleep(.05)
        fruit, robot = self.poses.get(key + '_picked'), self.poses.get('harvester')

        def xyz(item):
            return ([item[0].translation.x, item[0].translation.y, item[0].translation.z]
                    if item else None)
        self.ops.event('basket_evidence_failed', fruit_position=xyz(fruit),
                       robot_position=xyz(robot), last_update=fruit[1] if fruit else 0.)
        raise OperationError('basket_not_settled')

    def record_collected_load(self, key):
        """Replace MoveIt's detach pose with the measured load inside the robot basket."""
        fruit, robot = self.poses[key + '_picked'][0], self.poses['harvester'][0]
        fq, rq = fruit.rotation, robot.rotation
        fruit_rotation = Rotation.from_quat([fq.x, fq.y, fq.z, fq.w])
        inverse = Rotation.from_quat([rq.x, rq.y, rq.z, rq.w]).inv()
        f, r = fruit.translation, robot.translation
        target = self.targets[key]
        center = fruit_rotation.apply(target['collision_center_local']) + np.array(
            [f.x, f.y, f.z])
        center = inverse.apply(center - np.array([r.x, r.y, r.z]))
        obj = self.collision_object(key, center, frame='platform_base_link',
                                    dimensions=target['collision_size'])
        q = (inverse * fruit_rotation).as_quat()
        pose = obj.primitive_poses[0]
        pose.orientation.x, pose.orientation.y = float(q[0]), float(q[1])
        pose.orientation.z, pose.orientation.w = float(q[2]), float(q[3])
        load = AttachedCollisionObject(link_name='harvest_basket', object=obj,
                                       touch_links=['harvest_basket', 'platform_base_link'])
        self.apply(attached=[load])
        self.ops.event('collected_load_registered', target=key, center_in_base=center.tolist())

    def deposit_joints(self, canceled):
        """Find a collision-free upright release configuration using MoveIt's IK service."""
        # Two separated drop sites fit this demo basket. Do not drop a second
        # rigid bag onto the first and count a stack above the rim as in-basket.
        y = -.07 if not self.collected else .07
        # Cardinal-only yaw samples collide with the forearm at the second
        # slot. Include intermediate wrist orientations, keeping collision-aware
        # IK and complete path validation enabled.
        for height in (1.15, 1.25, 1.35, 1.05, .95):
            for yaw in (0., 1.5707963268, 3.1415926536, -1.5707963268,
                        .7853981634, -.7853981634, 2.3561944902, -2.3561944902):
                request = GetPositionIK.Request()
                request.ik_request.group_name = 'manipulator_e5'
                request.ik_request.ik_link_name = 'tcp'
                request.ik_request.avoid_collisions = True
                request.ik_request.robot_state.is_diff = True
                request.ik_request.robot_state.joint_state = self.joint_state
                request.ik_request.timeout.nanosec = 300000000
                pose = request.ik_request.pose_stamped
                pose.header.frame_id = 'platform_base_link'
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = -.34, y, height
                q = Rotation.from_euler('z', yaw).as_quat()
                pose.pose.orientation.x, pose.pose.orientation.y = float(q[0]), float(q[1])
                pose.pose.orientation.z, pose.pose.orientation.w = float(q[2]), float(q[3])
                answer = self.ops.call(self.ik, request, canceled=canceled)
                if answer.error_code.val == 1:
                    joints = dict(zip(answer.solution.joint_state.name,
                                      answer.solution.joint_state.position))
                    self.ops.event('deposit_ik_selected', height=height, yaw=yaw, y=y)
                    return {name: joints[name] for name in JOINTS}
                # A failed collision-aware IK alone does not locate the problem.
                # Query an unconstrained solution only for collision diagnostics;
                # it is never sent to motion execution.
                request.ik_request.avoid_collisions = False
                unchecked = self.ops.call(self.ik, request, canceled=canceled)
                contacts = []
                if unchecked.error_code.val == 1:
                    check = self.ops.call(self.validity, GetStateValidity.Request(
                        robot_state=unchecked.solution, group_name='manipulator_e5'),
                        canceled=canceled)
                    contacts = [[c.contact_body_1, c.contact_body_2, c.depth]
                                for c in check.contacts]
                self.ops.event('deposit_ik_rejected', height=height, yaw=yaw, y=y,
                               code=answer.error_code.val, contacts=contacts)
        raise OperationError('no_collision_free_deposit_pose')

    def execute(self, goal):
        """Approach, cut/attach, retreat, carry, release, verify and stow."""
        result = Pickup.Result()
        key = goal.request.target_name
        target = self.targets.get(key)

        def canceled():
            return goal.is_cancel_requested
        attached, cut, released, spawned = False, False, False, False
        original_matrix = None
        spawn_issued, remove_issued = False, False
        try:
            self.lock_base(canceled)
            self.initialize_scene()
            self.stage(goal, 'approach')
            grasp = goal.request.possible_grasps[0]
            self.check_visual_permission(goal.request.target_name, grasp.id, 'approach')
            contact = deepcopy(grasp.grasp_pose.pose)
            if grasp.grasp_pose.header.frame_id != 'map':
                raise OperationError('grasp_frame_must_be_map')
            actual = np.array([contact.position.x, contact.position.y, contact.position.z])
            if self.get_parameter('perception_mode').value == 'vision':
                # Only this simulator boundary associates a visual target with a
                # removable entity. It never replaces the commanded visual pose.
                candidates = sorted((float(np.linalg.norm(actual - item['bag_bottom'])), name)
                                    for name, item in self.targets.items()
                                    if name not in self.collected)
                if not candidates or candidates[0][0] > .08:
                    raise OperationError('visual_entity_association_outside_0.08m')
                if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < .02:
                    raise OperationError('visual_entity_association_ambiguous')
                residual, key = candidates[0]
                target = self.targets[key]
                self.ops.event('visual_entity_associated', visual_id=goal.request.target_name,
                               entity=key, residual_m=residual)
            expected = np.asarray(target['bag_bottom'])
            if np.linalg.norm(actual - expected) > .08:
                raise OperationError('target_pose_mismatch')
            original_matrix = self.allow_target_contact(key)
            frame = 'map'
            if self.get_parameter('perception_mode').value == 'ground_truth':
                robot = self.poses.get('harvester')
                if robot is None or time.monotonic() - robot[1] > 2:
                    raise OperationError('missing_gazebo_robot_pose')
                t, q = robot[0].translation, robot[0].rotation
                inverse = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv()
                position = inverse.apply(actual - np.array([t.x, t.y, t.z]))
                contact.position.x, contact.position.y, contact.position.z = map(float, position)
                rotation = inverse * Rotation.from_quat([
                    contact.orientation.x, contact.orientation.y,
                    contact.orientation.z, contact.orientation.w])
                q = rotation.as_quat()
                contact.orientation.x, contact.orientation.y = float(q[0]), float(q[1])
                contact.orientation.z, contact.orientation.w = float(q[2]), float(q[3])
                self.ops.event('oracle_target_in_base', position=position.tolist(),
                               robot_position=[t.x, t.y, t.z])
                frame = 'platform_base_link'
            axis = Rotation.from_quat([
                contact.orientation.x, contact.orientation.y,
                contact.orientation.z, contact.orientation.w]).apply([0, 0, 1])
            pre = deepcopy(contact)
            pre.position.x -= float(axis[0] * .12)
            pre.position.y -= float(axis[1] * .12)
            pre.position.z -= float(axis[2] * .12)
            self.ops.move(pose=pre, frame=frame, canceled=canceled)
            self.stage(goal, 'insert')
            self.check_visual_permission(goal.request.target_name, grasp.id, 'insert')
            self.ops.move(pose=contact, frame=frame, linear=True, canceled=canceled)
            residual = self.actual_contact_residual(target)
            if residual > .04:
                raise OperationError(f'contact_residual:{residual:.4f}')
            self.stage(goal, 'cut_attach')
            self.check_visual_permission(goal.request.target_name, grasp.id, 'cut')
            request = SpawnEntity.Request()
            request.entity_factory.name = key + '_picked'
            request.entity_factory.sdf = fruit_sdf(target, dynamic=True)
            request.entity_factory.pose.position.x = target['bag_bottom'][0]
            request.entity_factory.pose.position.y = target['bag_bottom'][1]
            request.entity_factory.pose.position.z = target['bag_bottom'][2]
            request.entity_factory.pose.orientation.w = 1.0
            self.states.pop(key, None)
            spawn_issued = True
            if not self.ops.call(self.spawn, request, canceled=canceled).success:
                raise OperationError('fruit_spawn_failed')
            spawned = True
            self.wait_until(lambda: self.states.get(key) == 'attached', 8, canceled)
            attached = True
            delete = DeleteEntity.Request(entity=Entity(name=key, type=Entity.MODEL))
            remove_issued = True
            if not self.ops.call(self.remove, delete, canceled=canceled).success:
                raise OperationError('cut_remove_failed')
            cut = True
            # Attached geometry is expressed in the current wrist frame by MoveIt.
            obj = self.collision_object(key, target['collision_center'],
                                        dimensions=target['collision_size'])
            carry = AttachedCollisionObject(link_name='wrist3_Link', object=obj,
                                            touch_links=['wrist3_Link', 'tcp',
                                                         'tool_body_link', 'sleeve_mouth'])
            remove_world = self.collision_object(key, (), operation=CollisionObject.REMOVE)
            self.apply(objects=[remove_world], attached=[carry])
            self.ops.event('cut_confirmed', target=key, residual_m=residual)
            self.stage(goal, 'retreat')
            self.ops.move(pose=pre, frame=frame, linear=True, canceled=canceled)
            self.stage(goal, 'carry')
            self.ops.move(joints=self.deposit_joints(canceled), canceled=canceled)
            self.stage(goal, 'release')
            released_at = time.monotonic()
            self.detach[key].publish(Empty())
            self.wait_until(lambda: self.states.get(key) == 'detached', 8, canceled)
            attached, released = False, True
            remove_attached = AttachedCollisionObject(link_name='wrist3_Link')
            remove_attached.object.id = key
            remove_attached.object.operation = CollisionObject.REMOVE
            self.apply(attached=[remove_attached])
            self.confirm_settled(key, released_at, canceled)
            self.record_collected_load(key)
            self.ops.event('basket_confirmed', target=key)
            self.stage(goal, 'stow')
            self.ops.move(named='harvest_stow', canceled=canceled)
            self.confirm_settled(key, released_at, canceled)
            self.collected.add(key)
            result.error_code.val = 1
            result.grasp = grasp
            goal.succeed()
            self.ops.event('harvest_confirmed', target=key)
        except Exception as exc:
            self.ops.cancel_all()
            self.recovery_required = spawn_issued or remove_issued or self.ops.recovery_required
            if spawned and not remove_issued:
                try:
                    rollback = DeleteEntity.Request(entity=Entity(
                        name=key + '_picked', type=Entity.MODEL))
                    self.ops.call(self.remove, rollback)
                except OperationError:
                    self.recovery_required = True
            result.error_code.val = -7 if canceled() else -1
            self.ops.event('pickup_failed', target=key, reason=str(exc),
                           cut=cut, attached=attached, released=released)
            if canceled():
                goal.canceled()
            else:
                goal.abort()
        finally:
            if original_matrix is not None and self.context.ok():
                try:
                    self.apply(matrix=original_matrix)
                except OperationError as exc:
                    self.recovery_required = True
                    self.ops.event('contact_matrix_restore_failed', reason=str(exc))
            self.finish_transaction()
            self.busy.release()
        return result


def main(args=None):
    """Run the action server with independent threads for dependency responses."""
    rclpy.init(args=args)
    node = GraspNode()
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
