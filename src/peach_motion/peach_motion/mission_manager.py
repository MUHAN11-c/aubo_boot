"""
任务管理器 — P2 真调度：GLOBAL_SCAN 巡视 → READY 队列逐个 pick → SITE_DONE.

工作线程跑任务循环（不阻塞定时器/订阅）：GLOBAL_SCAN 阶段按 observe_poses
逐个位姿停留 scan_duration_sec（期间订阅回调持续灌注册表）；READY_LOOP 按
confidence×0.6+近距×0.4 排序，逐个 READY 目标经 /pick_executor/pick
action 调 PickExecutor，结果回写注册表（DONE/FAILED+原因）并记 events；
队列清空（全部 DONE/FAILED/LOST）→ SITE_DONE，写 summary 并停 rosbag2。
定时器（2 Hz）只负责发布 ~/registry 快照与 ~/stats 统计。
"""
import json
import threading
import time

from peach_motion.data_store import SessionStore
from peach_motion.target_registry import (
    DONE,
    FAILED,
    LOST,
    ready_order,
    STATUS_NAMES,
    TargetRegistry,
)
from peach_pose_msgs.action import Pick
from peach_pose_msgs.msg import (
    BagGraspCandidateArray,
    TargetInfo,
    TargetRegistry as TargetRegistryMsg,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

# 感知主输出话题（peach_pose_ros2 节点发布）
CANDIDATES_TOPIC = '/peach_pose_node/grasp_candidates'
# pick action（同进程 PickExecutor 提供）
PICK_ACTION = '/pick_executor/pick'

# 参数默认值（与 config/mission.yaml 的 mission_manager 节对齐）
DEFAULTS = {
    'merge_radius': 0.08,           # 空间归并半径 (m)
    'ready_frames': 5,              # READY 所需连续 ACCEPT 帧数
    'ready_pos_drift': 0.02,        # READY 允许的最大帧间漂移 (m)
    'lost_timeout_sec': 2.0,        # LOST 判定超时 (s)
    'scan_duration_sec': 3.0,       # 每个观测位姿的停留时长 (s)
    'execute_insert': False,        # true=真插入；默认仅可达性验证
    'record': True,                 # 是否落盘 peach_records/（含 rosbag2）
    # 巡视位姿：逗号分隔的 7 元组 (x,y,z,qx,qy,qz,qw)，空串=原位扫描一次
    'observe_poses': '',
    # MoveIt（巡视段用；AUBO E5 规划组 manipulator_e5）
    'group_name': 'manipulator_e5',
    'base_frame': 'base_link',
    'tool_frame': 'wrist3_Link',
    'speed_scaling': 0.01,
    'planning_time_sec': 5.0,
    'planning_attempts': 2,
    'position_tolerance_m': 0.01,
    'orientation_tolerance_rad': 0.1,
}


class MissionManager(Node):
    """P2 任务管理节点：注册表灌入 + 工作线程真调度 + 快照/统计发布."""

    def __init__(self):
        """建节点：参数 → 注册表/记录器 → 订阅/发布 → 定时器 + 任务线程."""
        super().__init__('mission_manager')
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)

        def _get(name):
            return self.get_parameter(name).value

        self.registry = TargetRegistry(
            merge_radius=float(_get('merge_radius')),
            ready_frames=int(_get('ready_frames')),
            ready_pos_drift=float(_get('ready_pos_drift')),
            lost_timeout_sec=float(_get('lost_timeout_sec')))
        self._lock = threading.Lock()       # 注册表线程安全（订阅/工作线程并发）
        self.scan_duration_sec = float(_get('scan_duration_sec'))
        self.execute_insert = bool(_get('execute_insert'))
        self.record = bool(_get('record'))
        self._observe_poses = self._parse_poses(str(_get('observe_poses')))
        self._moveit_params = {k: _get(k) for k in DEFAULTS}
        self._moveit = None                 # 巡视段懒建

        self.store = SessionStore(enabled=self.record)
        if self.record:
            self.get_logger().info(f'记录目录: {self.store.start_session()}')
            if self.store.start_bag():
                self.get_logger().info('rosbag2 录制已拉起')

        self.create_subscription(
            BagGraspCandidateArray, CANDIDATES_TOPIC, self._on_candidates, 10)
        self.pub_registry = self.create_publisher(
            TargetRegistryMsg, '~/registry', 10)
        self.pub_stats = self.create_publisher(String, '~/stats', 10)
        self._pick_client = ActionClient(self, Pick, PICK_ACTION)

        self._mission_state = 'GLOBAL_SCAN'
        self._session_start = time.monotonic()
        self.create_timer(0.5, self._tick)
        self._worker = threading.Thread(target=self._mission_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f'mission_manager 就绪（P2 调度）：订阅 {CANDIDATES_TOPIC}，'
            f'observe_poses={len(self._observe_poses)} record={self.record}')

    @staticmethod
    def _parse_poses(text: str):
        """
        解析 observe_poses 字符串为 Pose 列表.

        Args:
            text: 逗号分隔浮点，每 7 个一组 (x,y,z,qx,qy,qz,qw)；空串给 [].

        Returns
        -------
            [geometry_msgs/Pose, ...]；非 7 的倍数截断并忽略尾部.

        """
        from geometry_msgs.msg import Pose
        text = text.strip()
        if not text:
            return []
        values = [float(x) for x in text.split(',')]
        poses = []
        for i in range(0, len(values) - 6, 7):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = values[i:i + 3]
            (pose.orientation.x, pose.orientation.y, pose.orientation.z,
             pose.orientation.w) = values[i + 3:i + 7]
            poses.append(pose)
        return poses

    def _moveit_client(self):
        """懒建巡视段 MoveItClient（moveit_msgs 延迟 import，仅运行时需要）."""
        if self._moveit is None:
            from peach_motion.moveit_client import MoveItClient
            p = self._moveit_params
            self._moveit = MoveItClient(
                self, group_name=p['group_name'], base_frame=p['base_frame'],
                tool_frame=p['tool_frame'],
                velocity_scaling=p['speed_scaling'],
                acceleration_scaling=p['speed_scaling'],
                planning_time_sec=p['planning_time_sec'],
                planning_attempts=p['planning_attempts'],
                position_tolerance_m=p['position_tolerance_m'],
                orientation_tolerance_rad=p['orientation_tolerance_rad'])
        return self._moveit

    def _on_candidates(self, msg):
        """
        感知候选回调：逐候选配准灌入注册表（带置信度），并记事件.

        Args:
            msg: peach_pose_msgs/BagGraspCandidateArray.

        Returns
        -------
            无返回值（None）.

        """
        now = self.get_clock().now().nanoseconds / 1e9
        observations = []
        for c in msg.candidates:
            p = c.entry_pose.position
            # strategy_id 形如 robust_bag_pose:... / robust_fruit_pose:...
            kind = 'fruit' if 'fruit' in (c.strategy_id or '') else 'bag'
            observations.append(((p.x, p.y, p.z), kind, int(c.status),
                                 float(c.confidence), c.target_id))
        with self._lock:
            touched = self.registry.update(observations, now)
        self.store.log_event({
            't': now, 'type': 'candidates',
            'n': len(observations), 'touched': touched,
        })

    # ---- 任务循环（独立工作线程，不阻塞回调）----
    def _mission_loop(self):
        """GLOBAL_SCAN 巡视 → READY_LOOP 逐个 pick → SITE_DONE 收尾."""
        self._global_scan()
        self._set_state('READY_LOOP')
        while rclpy.ok():
            if self._mission_state != 'READY_LOOP':
                time.sleep(0.2)
                continue
            with self._lock:
                records = self.registry.snapshot()
            queue = ready_order(records)
            if not queue:
                if records and all(r.status in (DONE, FAILED, LOST)
                                   for r in records):
                    self._site_done(records)
                    return
                time.sleep(0.5)
                continue
            target = queue[0]
            with self._lock:
                self.registry.mark_executing(target.target_id)
            result = self._send_pick(target.target_id)
            with self._lock:
                if result is not None and result.success:
                    self.registry.mark_done(target.target_id, result.message)
                else:
                    reason = (result.message if result is not None
                              else 'pick 不可达（无结果）')
                    self.registry.mark_failed(target.target_id, reason)
            self.store.log_event({
                't': time.monotonic() - self._session_start,
                'type': 'pick_result', 'target_id': target.target_id,
                'success': bool(result and result.success),
                'message': result.message if result is not None else '',
            })

    def _global_scan(self):
        """GLOBAL_SCAN：按 observe_poses 逐个位姿停留（灌注册表靠订阅回调）."""
        if not self._observe_poses:
            time.sleep(self.scan_duration_sec)
            return
        client = self._moveit_client()
        if not client.server_ready(5.0):
            self.get_logger().warn('MoveIt 不可用，巡视退化为原位停留')
            time.sleep(self.scan_duration_sec)
            return
        for i, pose in enumerate(self._observe_poses):
            ok, cat = client.plan_and_execute(pose)
            self.get_logger().info(
                f'巡视位姿 {i + 1}/{len(self._observe_poses)}: '
                f'{"到达" if ok else f"失败({cat})"}')
            time.sleep(self.scan_duration_sec)

    def _send_pick(self, target_id: str):
        """
        发 pick goal 并阻塞等结果.

        Args:
            target_id: 目标 ID.

        Returns
        -------
            Pick.Result；server 不可达/被拒给 None.

        """
        if not self._pick_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('pick action server 不可达')
            return None
        goal = Pick.Goal()
        goal.target_id = target_id
        goal.execute_insert = self.execute_insert
        send_future = self._pick_client.send_goal_async(goal)
        while not send_future.done():
            time.sleep(0.02)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        while not result_future.done():
            time.sleep(0.02)
        return result_future.result().result

    def _site_done(self, records):
        """SITE_DONE 收尾：写 summary（计数/平均 READY 收敛帧数/总耗时）并停 bag."""
        self._set_state('SITE_DONE')
        stats = self.registry.stats()
        ready_frames = [r.frames_to_ready for r in records
                        if r.frames_to_ready >= 0]
        summary = {
            'total': stats['total'],
            'done': stats['DONE'],
            'failed': stats['FAILED'],
            'lost': stats['LOST'],
            'avg_frames_to_ready': (sum(ready_frames) / len(ready_frames)
                                    if ready_frames else -1),
            'elapsed_sec': time.monotonic() - self._session_start,
        }
        self.store.log_event({'t': summary['elapsed_sec'],
                              'type': 'site_done', **summary})
        self.store.close(summary)
        self.get_logger().info(f'SITE_DONE: {summary}')

    def _set_state(self, state: str):
        """切换任务状态（日志 + 事件）."""
        if state != self._mission_state:
            self.get_logger().info(
                f'{self._mission_state} → {state}')
            self.store.log_event({
                't': time.monotonic() - self._session_start,
                'type': 'state', 'from': self._mission_state, 'to': state,
            })
            self._mission_state = state

    def _tick(self):
        """定时器：发布注册表快照与统计."""
        self._publish_registry()
        self._publish_stats()

    def _publish_registry(self):
        """把注册表快照装成 TargetRegistry 消息发布."""
        msg = TargetRegistryMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        with self._lock:
            records = self.registry.snapshot()
        for rec in records:
            info = TargetInfo()
            info.target_id = rec.target_id
            info.position.x, info.position.y, info.position.z = (
                float(rec.position[0]), float(rec.position[1]),
                float(rec.position[2]))
            info.kind = rec.kind
            info.status = int(rec.status)
            info.attempts = int(rec.attempts)
            info.last_result = rec.last_result
            info.first_seen.sec = int(rec.first_seen)
            info.first_seen.nanosec = int((rec.first_seen % 1.0) * 1e9)
            info.last_seen.sec = int(rec.last_seen)
            info.last_seen.nanosec = int((rec.last_seen % 1.0) * 1e9)
            msg.targets.append(info)
        self.pub_registry.publish(msg)

    def _publish_stats(self):
        """把状态统计装成 JSON 字符串发布（~/stats）."""
        with self._lock:
            stats = self.registry.stats()
            nxt = self.registry.next_target()
        payload = {
            'state': self._mission_state,
            'targets': stats,
            'ready_next': nxt.target_id if nxt is not None else '',
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_stats.publish(msg)


def status_name(code: int) -> str:
    """
    状态码 → 名称字符串（调试/日志用）.

    Args:
        code: 状态机取值（0–6）.

    Returns
    -------
        状态名；未知值给 'UNKNOWN'.

    """
    return STATUS_NAMES.get(code, 'UNKNOWN')
