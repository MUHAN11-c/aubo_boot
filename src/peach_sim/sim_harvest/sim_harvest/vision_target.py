"""Resolve a Gazebo target through the shared RGB-D discovery and reconstruction APIs."""

from copy import deepcopy
import hashlib
import json
import math
import threading
import time

from geometry_msgs.msg import PointStamped, Vector3Stamped
from peach_interfaces.action import BuildTargetModel
from peach_interfaces.msg import BagGraspCandidate, HarvestState, PeachTargetObservation
from peach_interfaces.msg import PeachTargetObservationArray
from peach_interfaces.srv import BeginScene
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point, do_transform_vector3
from tf2_ros import Buffer, TransformException, TransformListener

from .operations import OperationError


def visual_model_identity(message):
    """Use the standard grasp candidate ID to bind the complete existing model identity."""
    run = getattr(message, 'run_id', getattr(message, 'harvest_run_id', ''))
    fields = [run, message.scene_epoch, message.target_id, message.model_revision,
              message.tool_profile_id, message.calibration_revision, message.config_revision,
              message.valid_until.sec, message.valid_until.nanosec]
    return 'sha256:' + hashlib.sha256(json.dumps(fields).encode()).hexdigest()


class VisionTarget:
    """Adapt shared visual evidence to a map-frame pickup target without moving the arm."""

    def __init__(self, node, ops):
        self.node, self.ops = node, ops
        for name, default in [('vision_lock_timeout', 90.0),
                              ('vision_build_timeout', 180.0)]:
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
        self.buffer = Buffer(cache_time=Duration(seconds=120.0))
        self.listener = TransformListener(self.buffer, node)
        self.lock = threading.Lock()
        self.latest = None
        self.received = 0.0
        self.last_diagnostic = 0.0
        self.subscription = node.create_subscription(
            PeachTargetObservationArray, '/peach/perception/target_observations',
            self._observe, 10, callback_group=ops.group)
        self.begin = node.create_client(
            BeginScene, '/peach_scene_perception_node/begin_scene',
            callback_group=ops.group)
        self.build = ActionClient(
            node, BuildTargetModel, '/peach_target_reconstruction_node/build_target_model',
            callback_group=ops.group)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.binding = node.create_publisher(HarvestState, '/peach_supervisor/state', latched)
        self.revision = 0
        self.locked_ids = []

    def _observe(self, message):
        with self.lock:
            self.latest = message
            self.received = time.monotonic()
        if time.monotonic() - self.last_diagnostic > 2:
            self.last_diagnostic = time.monotonic()
            self.ops.event('vision_observations', epoch=message.scene_epoch,
                           locked=message.target_set_locked, run=message.harvest_run_id,
                           collecting=message.collecting_count, pending=message.pending_count,
                           observations=[{
                               'id': item.target_id, 'confirmed': item.confirmed,
                               'tracking': item.tracking_status, 'status': item.candidate.status,
                               'flags': list(item.diagnostic_flags)
                               + list(item.candidate.diagnostic_flags),
                               'frame': item.candidate.header.frame_id,
                               'bottom': [item.candidate.bag_bottom.x, item.candidate.bag_bottom.y,
                                          item.candidate.bag_bottom.z]}
                               for item in message.observations[:10]])

    def _bind(self, run_id, epoch, target_id='', phase=HarvestState.OBSERVING):
        """Publish the shared capability's selected visual identity and scene generation."""
        self.revision += 1
        state = HarvestState(run_id=run_id, target_id=target_id, scene_epoch=epoch,
                             target_phase=phase, batch_state=HarvestState.RUNNING,
                             state_seq=self.revision, revision=self.revision,
                             action_active=True)
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.header.frame_id = 'map'
        self.binding.publish(state)

    def _transform(self, header):
        """Look up only the specified observation/model time; zero stamps are invalid."""
        if not header.frame_id or not (header.stamp.sec or header.stamp.nanosec):
            raise OperationError('vision_geometry_has_no_frame_or_stamp')
        if header.frame_id == 'map':
            return None
        try:
            return self.buffer.lookup_transform(
                'map', header.frame_id, Time.from_msg(header.stamp),
                timeout=Duration(seconds=0.5))
        except TransformException as exc:
            raise OperationError('vision_exact_stamp_tf_unavailable:' + str(exc)) from exc

    @staticmethod
    def _point(point, header, transform):
        if transform is None:
            value = point
        else:
            value = do_transform_point(PointStamped(header=header, point=point), transform).point
        result = [float(value.x), float(value.y), float(value.z)]
        if not all(math.isfinite(item) for item in result):
            raise OperationError('vision_nonfinite_geometry')
        return result

    def _candidates(self, message, completed_ids):
        """Select only visual evidence; entity truth is unavailable at this boundary."""
        candidates = []
        for observation in message.observations:
            candidate = observation.candidate
            flags = set(observation.diagnostic_flags) | set(candidate.diagnostic_flags)
            if (not observation.confirmed or not observation.target_id
                    or observation.target_id in completed_ids
                    or observation.tracking_status != PeachTargetObservation.OBSERVED
                    or candidate.status != BagGraspCandidate.ACCEPT
                    or flags.intersection({'tf_stale', 'tf_unavailable', 'mask_unavailable'})):
                continue
            try:
                transform = self._transform(candidate.header)
                self._point(candidate.bag_bottom, candidate.header, transform)
            except OperationError:
                continue
            candidates.append((observation.priority, observation.target_id))
        return sorted(candidates)

    def _locked_target(self, completed_ids, run_id, epoch, canceled):
        timeout = float(self.node.get_parameter('vision_lock_timeout').value)
        deadline = time.monotonic() + timeout
        next_binding, seen_received = 0.0, 0.0
        while time.monotonic() < deadline:
            if canceled() or not self.node.context.ok():
                raise OperationError('canceled')
            now = time.monotonic()
            if now >= next_binding:
                self._bind(run_id, epoch)
                next_binding = now + 0.5
            with self.lock:
                message, received = self.latest, self.received
            if (message is not None and received > seen_received
                    and message.scene_epoch == epoch and message.target_set_locked
                    and bool(message.harvest_run_id)):
                seen_received = received
                self.locked_ids = [item.target_id for item in message.observations
                                   if item.confirmed and item.target_id]
                candidates = self._candidates(message, completed_ids)
                if candidates:
                    return (*candidates[0], message.harvest_run_id,
                            list(self.locked_ids))
                if message.pending_count == 0:
                    raise OperationError('vision_no_accepted_observed_target')
            time.sleep(0.05)
        raise OperationError('vision_no_accepted_observed_target')

    def resolve(self, canceled, scene_key, completed_ids=()):
        """Discover and reconstruct the observed target, failing if visual evidence is missing."""
        if canceled():
            raise OperationError('canceled')
        run_id = f'vision_{time.time_ns()}'
        self.locked_ids = []
        with self.lock:
            self.latest, self.received = None, 0.0
        self._bind(run_id, 0)
        response = self.ops.call(self.begin, BeginScene.Request(
            request_id=run_id, scene_key=scene_key), canceled=canceled)
        if not response.accepted or response.scene_epoch == 0:
            raise OperationError('vision_begin_scene_rejected:' + response.message)
        epoch = int(response.scene_epoch)
        _, visual_id, source_run, observed_ids = self._locked_target(
            completed_ids, run_id, epoch, canceled)
        self._bind(run_id, epoch, visual_id, HarvestState.FINALIZING)
        self.ops.event('vision_target_bound', source_target_id=visual_id,
                       scene_epoch=epoch, observed_candidates=len(observed_ids))
        result = self.ops.action(self.build, BuildTargetModel.Goal(
            request_id=run_id, target_id=visual_id, scene_epoch=epoch),
            float(self.node.get_parameter('vision_build_timeout').value), canceled)
        model = result.model
        if (not result.success or not model.accepted or model.target_id != visual_id
                or model.scene_epoch != epoch or not model.model_revision or model.view_count == 0
                or model.run_id != source_run):
            raise OperationError('vision_model_rejected:' + result.message)
        now = self.node.get_clock().now().nanoseconds
        capabilities = (model.geometry_capability, model.pregrasp_capability,
                        model.sleeve_capability, model.cut_capability)
        if (any(value != 0 for value in capabilities) or now <= 0
                or Time.from_msg(model.valid_until).nanoseconds <= now
                or not model.calibration_revision or not model.config_revision
                or model.tool_profile_id != self.node.get_parameter('tool_profile').value):
            raise OperationError('vision_model_full_pickup_not_permitted')
        if canceled():
            raise OperationError('canceled')
        # The shared model header is finalize time, not exposure time. Retain
        # its coordinate frame but query TF at the last integrated exposure.
        header = deepcopy(model.header)
        header.stamp = deepcopy(model.source_end)
        transform = self._transform(header)
        bottom = self._point(model.bag_bottom, header, transform)
        neck = self._point(model.bag_neck, header, transform)
        center = self._point(model.shape.center, header, transform)
        vector = Vector3Stamped(header=header, vector=model.bag_axis)
        # Jazzy do_transform_vector3 clears the input transform's translation.
        # Give it a copy so the point transformations and cached data retain theirs.
        axis = (vector.vector if transform is None else
                do_transform_vector3(vector, deepcopy(transform)).vector)
        direction = [float(axis.x), float(axis.y), float(axis.z)]
        length = math.sqrt(sum(value * value for value in direction))
        diameter = float(model.d95_m)
        if (not math.isfinite(length) or length < 1e-6
                or not math.isfinite(diameter) or diameter <= 0.0
                or not all(math.isfinite(value) for value in direction)):
            raise OperationError('vision_model_geometry_invalid')
        resolved = {
            'id': visual_id, 'frame': 'map', 'geometry_source': 'vision',
            'source_target_id': visual_id, 'source_scene_epoch': epoch,
            'source_model_revision': model.model_revision, 'source_run_id': source_run,
            'grasp_id': visual_model_identity(model),
            'bag_bottom': bottom, 'entry': bottom, 'bag_neck': neck, 'fruit_center': center,
            'axis': [value / length for value in direction], 'body_diameter': diameter,
            'bottom_to_neck': math.dist(bottom, neck),
            'source_stamp_ns': Time.from_msg(header.stamp).nanoseconds,
            'observed_ids': observed_ids}
        self._bind(run_id, epoch, visual_id, HarvestState.VALIDATING)
        self.ops.event('vision_model_resolved', source_target_id=visual_id, scene_epoch=epoch,
                       model_revision=model.model_revision, bag_bottom=bottom)
        return resolved
