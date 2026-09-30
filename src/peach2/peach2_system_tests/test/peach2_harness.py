"""
Shared launch_testing harness for the peach2 mock stack (isolated ROS domain).

The harness node lives in the test process and stands in for what camera_enabled:=false does not
launch: a BuildSceneSnapshot stub, a BeginScene stub and a synthetic TargetObservationArray
publisher. It also hosts a spy /aubo_io_controller/set_io server that only counts calls (the mock
IO backend never calls it; any call is a test failure). It never sends goals by itself.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
import tempfile
import threading
import time
from typing import Callable

from ament_index_python.packages import get_package_share_directory
from aubo_msgs.srv import SetIO
from bond.msg import Status as BondStatus
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Point, Pose, Vector3
import launch
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from peach2_interfaces.action import HarvestTarget, RunBatch
from peach2_interfaces.msg import (
    BagLandmark,
    BatchState,
    Enables,
    TargetModelArray,
    TargetObservation,
    TargetObservationArray,
    ToolState,
)
from peach2_interfaces.srv import BeginScene, BuildSceneSnapshot, GetDecision, SetEnables
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros

JOINTS = ('shoulder_joint', 'upperArm_joint', 'foreArm_joint',
          'wrist1_joint', 'wrist2_joint', 'wrist3_joint')
TOOL_ID = 'adaptive_shear_v1'
MOCK_CONTROLLERS = ('joint_state_broadcaster', 'joint_trajectory_controller')
MANAGER = 'peach2_lifecycle_manager'
EXPECTED_EXIT_CODES = (0, -2, -9, -15, 130, 137, 143)
# rviz2 (offscreen) and move_group may die noisily on SIGINT; not part of the peach2 contract.
EXIT_CODE_EXEMPT = ('rviz2', 'move_group')

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
OBSERVATIONS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
BOND = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                  reliability=ReliabilityPolicy.BEST_EFFORT,
                  durability=DurabilityPolicy.VOLATILE)


@dataclass(frozen=True)
class SyntheticBag:
    """Bag geometry in base_link; bottom -> neck along `axis` (unit)."""

    target_id: str
    bottom: tuple[float, float, float]
    axis: tuple[float, float, float]
    length_m: float = 0.07  # adaptive_shear_v1 L_insert = 0.090
    diameter_m: float = 0.06


def _unit(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(sum(c * c for c in v))
    return (v[0] / n, v[1] / n, v[2] / n)


# Known-good reachable geometry of the v1 grid fixture (peach_arm perception_constraint_grid:
# typical_1757 / right_lane_tilt), bottom = entry, axis bottom -> neck.
BAG_A = SyntheticBag('bag_a', (0.30384646154773753, -0.614107279028262, 0.5364744763197402),
                     _unit((0.042490284187384854, 0.13626269543201486, 0.9897611093507753)))
BAG_B = SyntheticBag('bag_b', (0.550477, -0.623546, 0.565096),
                     _unit((0.295079, -0.196719, 0.934265)))


def stack_launch_description(bond_timeout: str | None = None) -> launch.LaunchDescription:
    """Mock peach2_system, camera off; ReadyToTest after 2 s (tests wait on events)."""
    if bond_timeout is None:
        bond_timeout = os.environ.get('PEACH2_LT_BOND_TIMEOUT', '0.0')
    system = os.path.join(get_package_share_directory('peach2_bringup'),
                          'launch', 'peach2_system.launch.py')
    runs = tempfile.mkdtemp(prefix='peach2_lt_runs_')
    return launch.LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'offscreen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system),
            launch_arguments={
                'hardware_mode': 'mock',
                'camera_enabled': 'false',
                'moveit_enabled': 'true',
                'tool_id': TOOL_ID,
                'bond_timeout': bond_timeout,
                'runs_dir': runs,
            }.items()),
        TimerAction(period=2.0, actions=[ReadyToTest()]),
    ])


def wait_until(predicate: Callable[[], bool], timeout_s: float, period_s: float = 0.1) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


def wait_future(future, timeout_s: float):
    """Return the result of a future spun by the harness executor, or None on timeout."""
    if not wait_until(future.done, timeout_s, 0.05):
        return None
    return future.result()


class Harness(Node):
    """Stubs + observers for one launch_testing process (spun by a background executor)."""

    def __init__(self, bags: tuple[SyntheticBag, ...] = (BAG_A,)) -> None:
        super().__init__('peach2_lt_harness')
        self._lock = threading.Lock()
        self._group = ReentrantCallbackGroup()
        self.bags = bags
        self.epoch = 1
        self.begin_scene_calls = 0
        self.snapshot_calls = 0
        self.set_io_calls = 0
        self.publish_observations = True
        self.enables: Enables | None = None
        self.tool_state: ToolState | None = None
        self.batch_state: BatchState | None = None
        self.batch_phases: list[int] = []
        self.models: TargetModelArray | None = None
        self.recovery: bool | None = None
        self.joint_states: JointState | None = None
        self.bond_ids: set[str] = set()
        self._obs_count = 0
        self._state_clients: dict[str, object] = {}
        self._list_controllers = None

        g = self._group
        self.create_subscription(Enables, '/peach/enables', self._on_enables, LATCHED,
                                 callback_group=g)
        self.create_subscription(ToolState, '/peach/end_effector/tool_state',
                                 self._on_tool_state, LATCHED, callback_group=g)
        self.create_subscription(BatchState, '/peach/task/state', self._on_batch_state, LATCHED,
                                 callback_group=g)
        self.create_subscription(TargetModelArray, '/peach/target_model/models',
                                 self._on_models, LATCHED, callback_group=g)
        self.create_subscription(Bool, '/peach/manipulation/recovery_required',
                                 self._on_recovery, LATCHED, callback_group=g)
        self.create_subscription(JointState, '/joint_states', self._on_joints,
                                 qos_profile_sensor_data, callback_group=g)
        self.create_subscription(BondStatus, '/bond', self._on_bond, BOND, callback_group=g)

        self.create_service(BuildSceneSnapshot, '/peach/scene/build_snapshot',
                            self._on_snapshot, callback_group=g)
        self.create_service(BeginScene, '/peach/perception/begin_scene', self._on_begin_scene,
                            callback_group=g)
        self.create_service(SetIO, '/aubo_io_controller/set_io', self._on_set_io,
                            callback_group=g)
        self.obs_pub = self.create_publisher(
            TargetObservationArray, '/peach/perception/observations', OBSERVATIONS)
        self.create_timer(0.2, self._publish_observations, callback_group=g)

        self.run_batch = ActionClient(self, RunBatch, '/peach/task/run_batch', callback_group=g)
        self.harvest = ActionClient(self, HarvestTarget, '/peach/manipulation/harvest_target',
                                    callback_group=g)
        self.set_enables_client = self.create_client(SetEnables, '/peach/task/set_enables',
                                                     callback_group=g)
        self.ack_client = self.create_client(Trigger, '/peach/task/acknowledge_recovery',
                                             callback_group=g)
        self.decision_client = self.create_client(GetDecision, '/peach/target_model/get_decision',
                                                  callback_group=g)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

    # ---------------------------------------------------------------- callbacks
    def _on_enables(self, msg: Enables) -> None:
        self.enables = msg

    def _on_tool_state(self, msg: ToolState) -> None:
        self.tool_state = msg

    def _on_batch_state(self, msg: BatchState) -> None:
        with self._lock:
            self.batch_state = msg
            if not self.batch_phases or self.batch_phases[-1] != msg.phase:
                self.batch_phases.append(msg.phase)

    def _on_models(self, msg: TargetModelArray) -> None:
        self.models = msg

    def _on_recovery(self, msg: Bool) -> None:
        self.recovery = msg.data

    def _on_joints(self, msg: JointState) -> None:
        self.joint_states = msg

    def _on_bond(self, msg: BondStatus) -> None:
        with self._lock:
            self.bond_ids.add(msg.id)

    def _on_snapshot(self, request, response):
        self.snapshot_calls += 1
        response.success = True
        response.n_frames = 1
        response.frame_stamp = self.get_clock().now().to_msg()
        response.message = 'lt_stub'
        return response

    def _on_begin_scene(self, request, response):
        with self._lock:
            if self.begin_scene_calls > 0:
                self.epoch += 1
            self.begin_scene_calls += 1
            response.scene_epoch = self.epoch
        response.accepted = True
        response.message = 'lt_stub'
        return response

    def _on_set_io(self, request, response):
        with self._lock:
            self.set_io_calls += 1
        self.get_logger().error('SetIO called during a mock system test')
        response.success = False
        return response

    # ---------------------------------------------------------------- observations
    def _landmark(self, p: tuple[float, float, float]) -> BagLandmark:
        lm = BagLandmark()
        lm.valid = True
        lm.source = BagLandmark.SOURCE_GEOMETRY
        lm.position = Point(x=p[0], y=p[1], z=p[2])
        lm.covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]
        lm.confidence = 0.95
        return lm

    def observation(self, bag: SyntheticBag, stamp, view: int) -> TargetObservation:
        o = TargetObservation()
        o.header.frame_id = 'base_link'
        o.header.stamp = stamp
        o.target_id = bag.target_id
        o.category = TargetObservation.CATEGORY_BAG
        o.confirmed = True
        b = bag.bottom
        n = tuple(b[i] + bag.length_m * bag.axis[i] for i in range(3))
        o.bottom = self._landmark(b)
        o.neck = self._landmark(n)
        o.tie = BagLandmark()
        o.axis = Vector3(x=bag.axis[0], y=bag.axis[1], z=bag.axis[2])
        o.diameter95_m = bag.diameter_m
        o.camera_distance_m = 0.8
        # Two viewpoints 0.1 m apart (> view_translation_change_m), alternating every second.
        pose = Pose()
        pose.position.x = b[0] + (0.05 if view % 2 else -0.05)
        pose.position.y = b[1] + 0.7
        pose.position.z = b[2] + 0.3
        pose.orientation.w = 1.0
        o.camera_pose = pose
        o.mask_quality = 0.9
        o.depth_coverage = 0.9
        o.edge_touch = False
        o.swing_known = True
        o.swing_amplitude_m = 0.002
        o.swing_period_s = 2.0
        o.roi.x_offset = 300
        o.roi.y_offset = 200
        o.roi.width = 60
        o.roi.height = 120
        return o

    def _publish_observations(self) -> None:
        if not self.publish_observations:
            return
        stamp = self.get_clock().now().to_msg()
        view = self._obs_count // 5
        self._obs_count += 1
        msg = TargetObservationArray()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = stamp
        with self._lock:
            msg.scene_epoch = self.epoch
        msg.target_set_locked = True
        msg.locked_target_ids = [b.target_id for b in self.bags]
        msg.observations = [self.observation(b, stamp, view) for b in self.bags]
        self.obs_pub.publish(msg)

    # ---------------------------------------------------------------- queries
    def lifecycle_state(self, node_name: str, timeout_s: float = 5.0) -> int | None:
        with self._lock:
            client = self._state_clients.get(node_name)
            if client is None:
                client = self.create_client(GetState, f'/{node_name}/get_state',
                                            callback_group=self._group)
                self._state_clients[node_name] = client
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        response = wait_future(client.call_async(GetState.Request()), timeout_s)
        return None if response is None else response.current_state.id

    def wait_active(self, node_name: str, timeout_s: float) -> bool:
        return wait_until(
            lambda: self.lifecycle_state(node_name, 2.0) == LifecycleState.PRIMARY_STATE_ACTIVE,
            timeout_s, 1.0)

    def active_controllers(self) -> set[str]:
        with self._lock:
            if self._list_controllers is None:
                self._list_controllers = self.create_client(
                    ListControllers, '/controller_manager/list_controllers',
                    callback_group=self._group)
        response = self.call(self._list_controllers, ListControllers.Request(), 2.0)
        if response is None:
            return set()
        return {c.name for c in response.controller if c.state == 'active'}

    def wait_controllers(self, names: tuple[str, ...], timeout_s: float) -> bool:
        """Wait until every controller in `names` is active (not implied by lifecycle)."""
        return wait_until(lambda: set(names) <= self.active_controllers(), timeout_s, 0.5)

    def model(self, target_id: str):
        msg = self.models
        if msg is None:
            return None
        return next((m for m in msg.models if m.target_id == target_id), None)

    def wait_converged(self, target_id: str, timeout_s: float) -> bool:
        return wait_until(
            lambda: (m := self.model(target_id)) is not None and m.converged, timeout_s, 0.2)

    def joints(self) -> dict[str, float] | None:
        msg = self.joint_states
        if msg is None:
            return None
        return dict(zip(msg.name, msg.position))

    def tcp_position(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'tcp', Time())
        except tf2_ros.TransformException:
            return None
        v = t.transform.translation
        return (v.x, v.y, v.z)

    def call(self, client, request, timeout_s: float = 10.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            return None
        return wait_future(client.call_async(request), timeout_s)

    def set_enables(self, execution: bool, grasp: bool = False, tool: bool = False):
        request = SetEnables.Request()
        request.execution = execution
        request.grasp = grasp
        request.tool = tool
        return self.call(self.set_enables_client, request)

    def decision(self, target_id: str):
        request = GetDecision.Request()
        request.target_id = target_id
        request.tool_id = TOOL_ID
        return self.call(self.decision_client, request)

    def send_goal(self, client: ActionClient, goal, timeout_s: float = 10.0):
        """Send a goal; return the accepted handle, or None (server missing / rejected)."""
        if not client.wait_for_server(timeout_sec=timeout_s):
            return None
        handle = wait_future(client.send_goal_async(goal), timeout_s)
        if handle is None or not handle.accepted:
            return None
        return handle


def max_joint_delta(a: dict[str, float], b: dict[str, float]) -> float:
    return max(abs(a[j] - b[j]) for j in JOINTS)


def distance(p: tuple[float, float, float], q: tuple[float, float, float]) -> float:
    return math.sqrt(sum((p[i] - q[i]) ** 2 for i in range(3)))


class Session:
    """rclpy context + harness node + background executor, created once per test process."""

    _instance: Session | None = None

    def __init__(self, bags: tuple[SyntheticBag, ...]) -> None:
        rclpy.init()
        self.node = Harness(bags)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    @classmethod
    def get(cls, bags: tuple[SyntheticBag, ...] = (BAG_A,)) -> Harness:
        if cls._instance is None:
            cls._instance = Session(bags)
        return cls._instance.node

    @classmethod
    def shutdown(cls) -> None:
        inst = cls._instance
        if inst is None:
            return
        cls._instance = None
        inst.node.publish_observations = False
        inst.executor.shutdown(timeout_sec=2.0)
        inst.node.destroy_node()
        rclpy.try_shutdown()


def xfail(test_case, reason: str, check: Callable[[], None]) -> None:
    """
    Run `check` as a known upstream failure: failing -> skipped 'xfail: …', passing -> failed.

    unittest.expectedFailure is lost when launch_testing rebinds test methods, so it cannot be
    used here.
    """
    try:
        check()
    except AssertionError as exc:
        test_case.skipTest(f'xfail: {reason} ({exc})')
    test_case.fail(f'XPASS: {reason} no longer reproduces; remove the xfail')


def check_exit_codes(test_case, proc_info) -> None:
    """post_shutdown: every peach2 / driver process exited cleanly (signals count as clean)."""
    bad = []
    for info in proc_info:
        name = info.process_name
        if any(tag in name for tag in EXIT_CODE_EXEMPT):
            continue
        if info.returncode not in EXPECTED_EXIT_CODES:
            bad.append(f'{name}: {info.returncode}')
    test_case.assertFalse(bad, 'unexpected exit codes: ' + ', '.join(bad))
