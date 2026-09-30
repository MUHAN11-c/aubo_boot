"""
peach2_observability — read-only mirror + HTTP GET (default 127.0.0.1:8091).

Subscribes to harvest graph topics and /diagnostics; never creates service/action clients.
"""
from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from peach2_interfaces.msg import (
    BatchState,
    Enables,
    TargetModelArray,
    TargetObservationArray,
    ToolState,
)
from peach2_observability import snapshot as snap
from peach2_observability.http_server import HttpServer
from peach2_observability.ledger import read_ledger
from peach2_observability.params import load_params, resolve_runs_root
from peach2_observability.session_bag import SessionBagRecorder
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
VOLATILE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
DIAG_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


class ObservabilityNode(Node):
    """Regular node (not lifecycle-managed)."""

    def __init__(self) -> None:
        super().__init__('peach2_observability')
        self.declare_parameter('config_file', '')
        self.declare_parameter('use_sim_time', False)
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not config_path:
            raise RuntimeError('config_file parameter is required')
        self._params = load_params(config_path)
        self._runs_root = resolve_runs_root(self._params.runs_root)
        self._store = snap.SnapshotStore()
        self._bag = SessionBagRecorder(
            self._runs_root,
            enabled=self._params.session_bag.enabled,
            sigint_timeout_s=self._params.session_bag.sigint_timeout_s,
            term_timeout_s=self._params.session_bag.term_timeout_s,
            log_warning=lambda msg: self.get_logger().warning(msg),
        )
        self._bag.start()
        self._store.set_session_bag_info(self._bag.info())

        web_root = Path(get_package_share_directory('peach2_observability')) / 'web'
        self._http = HttpServer(
            self._params.host, self._params.port, self, web_root)
        self._http.start()
        self.get_logger().info(
            f'read-only HTTP on http://{self._params.host}:{self._params.port}/')

        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, DIAG_QOS)
        self.create_subscription(BatchState, '/peach/task/state', self._on_task, LATCHED_QOS)
        self.create_subscription(
            ToolState, '/peach/end_effector/tool_state', self._on_tool, LATCHED_QOS)
        self.create_subscription(
            Bool, '/peach/manipulation/recovery_required', self._on_recovery, LATCHED_QOS)
        self.create_subscription(
            TargetModelArray, '/peach/target_model/models', self._on_models, LATCHED_QOS)
        self.create_subscription(
            TargetObservationArray,
            '/peach/perception/observations',
            self._on_observations,
            VOLATILE_QOS,
        )
        self.create_subscription(Enables, '/peach/enables', self._on_enables, LATCHED_QOS)

        self._age_timer = self.create_timer(1.0, self._tick_topic_ages)

    def destroy_node(self) -> bool:
        self._bag.stop()
        self._store.set_session_bag_info(self._bag.info())
        self._http.stop()
        return super().destroy_node()

    # --- HttpServer backend ---

    def snapshot(self) -> dict:
        self._store.set_session_bag_info(self._bag.info())
        return self._store.snapshot()

    def diagnostics(self) -> dict:
        return self._store.diagnostics_payload()

    def ledger(self, request_id: str) -> dict:
        return read_ledger(self._runs_root, request_id)

    def log_debug(self, message: str) -> None:
        self.get_logger().debug(message)

    # --- Callbacks ---

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        statuses = [snap.diagnostic_status_to_dict(s) for s in msg.status]
        self._store.set_diagnostics(statuses)

    def _on_task(self, msg: BatchState) -> None:
        self._store.set_task(snap.batch_state_dict(
            stamp_sec=snap.stamp_to_sec(msg.header.stamp),
            request_id=msg.request_id,
            phase=msg.phase,
            current_target_id=msg.current_target_id,
            blockers=list(msg.blockers),
            attempted=msg.attempted,
            succeeded=msg.succeeded,
            skipped=msg.skipped,
            failed=msg.failed,
            recovery_required=msg.recovery_required,
            message=msg.message,
        ))

    def _on_tool(self, msg: ToolState) -> None:
        self._store.set_tool(snap.tool_state_dict(
            stamp_sec=snap.stamp_to_sec(msg.header.stamp),
            tool_id=msg.tool_id,
            state=msg.state,
            command_closed=msg.command_closed,
            feedback=msg.feedback,
            suspected_loopback=msg.suspected_loopback,
            actuator_current_a=msg.actuator_current_a,
            fault_reason=msg.fault_reason,
        ))

    def _on_recovery(self, msg: Bool) -> None:
        self._store.set_recovery_required(bool(msg.data))

    def _on_enables(self, msg: Enables) -> None:
        self._store.set_enables(snap.enables_dict(
            stamp_sec=snap.stamp_to_sec(msg.header.stamp),
            seq=msg.seq,
            execution=msg.execution,
            grasp=msg.grasp,
            tool=msg.tool,
        ))

    def _on_models(self, msg: TargetModelArray) -> None:
        items = []
        for model in msg.models:
            items.append(snap.model_row(
                target_id=model.target_id,
                model_revision=model.model_revision,
                converged=model.converged,
                n_views=model.n_views,
                d95_m=model.d95_m,
                length_m=model.length_m,
                sigma_lateral95_m=model.sigma_lateral95_m,
                swing_amplitude_m=model.swing_amplitude_m,
            ))
        self._store.set_models({
            'stamp_s': snap.stamp_to_sec(msg.header.stamp),
            'count': len(items),
            'models': items,
        })

    def _on_observations(self, msg: TargetObservationArray) -> None:
        rows = []
        for obs in msg.observations:
            rows.append(snap.observation_row(
                target_id=obs.target_id,
                category=obs.category,
                confirmed=obs.confirmed,
                diameter95_m=obs.diameter95_m,
                swing_known=obs.swing_known,
                swing_amplitude_m=obs.swing_amplitude_m,
                mask_quality=obs.mask_quality,
                depth_coverage=obs.depth_coverage,
                edge_touch=obs.edge_touch,
                flags=list(obs.flags),
            ))
        payload = snap.summarize_observations(
            scene_epoch=msg.scene_epoch,
            target_set_locked=msg.target_set_locked,
            locked_target_ids=list(msg.locked_target_ids),
            observations=rows,
            stamp_sec=snap.stamp_to_sec(msg.header.stamp),
        )
        self._store.set_observations(payload)

    def _tick_topic_ages(self) -> None:
        self._store.advance_topic_ages(1.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObservabilityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
