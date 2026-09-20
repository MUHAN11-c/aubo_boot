"""监控状态缓存、消息→JSON 编解码与性能采样."""
from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from typing import Any

from .job import build_harvest_job

try:
    import psutil
except ImportError:  # pragma: no cover - 部署机保证有 psutil，仅防御
    psutil = None


_STATUS_NAMES = {0: 'ACCEPT', 1: 'REOBSERVE', 2: 'REJECT'}
# 与 PeachTargetObservation.msg 常量一一对应（阶段 D1 追加 4/5）：
# OUT_OF_VIEW=出画（检测框触图像边缘后消失）、DEPTH_VOID=深度空洞
# （掩膜内有效深度占比低于阈值）；沿用既有英文 token 风格，前端徽标
# 配色表（web/app.js trackingChip）按 token 键控。
_TRACKING_NAMES = {
    0: 'OBSERVED', 1: 'OCCLUDED', 2: 'LOST', 3: 'INVALID',
    4: 'OUT_OF_VIEW', 5: 'DEPTH_VOID',
}
_SEVERITY_NAMES = {0: 'INFO', 1: 'WARNING', 2: 'ERROR', 3: 'AUDIT'}

# 与驱动栈 / AGENTS 关节顺序一致
JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)


def parse_json_text(text: str, fallback_key: str = 'text') -> dict:
    """解析 String JSON；普通文本以指定键保留."""
    try:
        value = json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return {fallback_key: str(text)}
    return value if isinstance(value, dict) else {'value': value}


def stamp_seconds(header) -> float:
    """把 ROS Header 时间戳转换为秒."""
    if header is None:
        return 0.0
    stamp = header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def stamp_seconds_dict_time(stamp) -> float:
    """把 builtin_interfaces/Time（如 GraspDecision.valid_until）转秒."""
    if stamp is None:
        return 0.0
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def to_point(point_message) -> list[float]:
    """转换 geometry_msgs Point/Vector3."""
    return [
        float(point_message.x),
        float(point_message.y),
        float(point_message.z),
    ]


def to_candidate(candidate_message) -> dict:
    """转换抓取候选，保留浏览器需要的几何与版本信息."""
    pose = candidate_message.entry_pose
    return {
        'target_id': candidate_message.target_id,
        'entry_position': to_point(pose.position),
        'entry_quaternion_xyzw': [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ],
        'bag_bottom': to_point(candidate_message.bag_bottom),
        'bag_neck': to_point(candidate_message.bag_neck),
        'translation_direction': to_point(
            candidate_message.translation_direction),
        'diameter_m': float(candidate_message.bag_diameter_upper_m),
        'travel_m': float(candidate_message.suggested_travel_m),
        'confidence': float(candidate_message.confidence),
        'status': _STATUS_NAMES.get(
            int(candidate_message.status), str(candidate_message.status)),
        'diagnostic_flags': list(candidate_message.diagnostic_flags),
        'strategy_id': candidate_message.strategy_id,
        'model_version': candidate_message.model_version,
        'calibration_version': candidate_message.calibration_version,
        'tool_version': candidate_message.tool_version,
    }


def to_fitting(fitting_message) -> dict:
    """转换几何拟合质量消息."""
    diameter = float(fitting_message.bag_diameter_upper_m)
    if diameter <= 0.0 and float(fitting_message.fruit_radius_m) > 0.0:
        diameter = 2.0 * float(fitting_message.fruit_radius_m)
    return {
        'target_id': fitting_message.target_id,
        'target_kind': fitting_message.target_kind,
        'status': _STATUS_NAMES.get(
            int(fitting_message.status), str(fitting_message.status)),
        'axis_confidence': float(fitting_message.axis_confidence),
        'valid_depth_ratio': float(fitting_message.valid_depth_ratio),
        'n_points': int(fitting_message.n_points),
        'error_budget_mm': float(fitting_message.error_budget_mm),
        'radial_clearance_mm': float(fitting_message.radial_clearance_mm),
        'diameter_m': diameter,
        'cylinder_rms_m': float(fitting_message.cylinder_rms_m),
        'sphere_rms_m': float(fitting_message.sphere_rms_m),
        'inlier_ratio': max(
            float(fitting_message.cylinder_inlier_ratio),
            float(fitting_message.sphere_inlier_ratio),
        ),
        'diagnostic_flags': list(fitting_message.diagnostic_flags),
    }


def to_target_observations(message) -> dict:
    """转换全局目标快照；故意排除大体积 mask 像素."""
    observations = []
    for item in message.observations:
        observations.append({
            'target_id': item.target_id,
            'priority': int(item.priority),
            'confirmed': bool(item.confirmed),
            'selected': bool(item.selected),
            'harvest_status': item.harvest_status,
            'tracking_status': _TRACKING_NAMES.get(
                int(item.tracking_status), str(item.tracking_status)),
            'camera_distance_m': float(item.camera_distance_m),
            'confidence': float(item.confidence),
            'candidate': to_candidate(item.candidate),
            'fitting': to_fitting(item.fitting),
            'mask': {
                'width': int(item.mask.width),
                'height': int(item.mask.height),
                'stamp': stamp_seconds(item.mask.header),
            },
            'diagnostic_flags': list(item.diagnostic_flags),
        })
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'snapshot_id': int(message.snapshot_id),
        'scene_epoch': int(getattr(message, 'scene_epoch', 0) or 0),
        'harvest_run_id': message.harvest_run_id,
        'target_set_locked': bool(message.target_set_locked),
        'target_count': int(message.target_count),
        'selected_target_id': message.selected_target_id,
        'collecting_count': int(getattr(message, 'collecting_count', 0) or 0),
        'pending_count': int(getattr(message, 'pending_count', 0) or 0),
        'observations': observations,
    }


def to_candidate_array(message) -> dict:
    """转换抓取候选数组."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'candidates': [to_candidate(item) for item in message.candidates],
    }


def to_fitting_array(message) -> dict:
    """转换拟合诊断数组."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'fittings': [to_fitting(item) for item in message.fittings],
    }


def to_vector_stamped(message) -> dict:
    """转换带时间戳向量."""
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'xyz': to_point(message.vector),
    }


def to_harvest_event(message) -> dict:
    """转换调度过程/审计事件（CanonicalEvent）为时间线条目."""
    return {
        'stamp': stamp_seconds(message.header),
        'sequence': int(message.sequence),
        'severity': int(message.severity),
        'severity_name': _SEVERITY_NAMES.get(
            int(message.severity), str(message.severity)),
        'code': message.code,
        'message': message.message,
        'request_id': message.request_id,
        'run_id': message.run_id,
        'cycle_id': getattr(message, 'cycle_id', ''),
        'state_seq': int(getattr(message, 'state_seq', 0) or 0),
        'target_id': message.target_id,
        'details': {
            item.key: item.value for item in message.details},
    }


def to_task_executor_state(message) -> dict:
    """HarvestState（调度类型化状态）→ 浏览器镜像 dict（W10 从节点回调下沉）."""
    return {
        'revision': message.revision,
        'state_seq': int(getattr(message, 'state_seq', 0) or 0),
        'run_id': message.run_id,
        'cycle_id': message.cycle_id,
        'target_id': message.target_id,
        'operation_mode': message.operation_mode,
        'batch_state': message.batch_state,
        'target_phase': message.target_phase,
        'action_active': message.action_active,
        'auto_start_enabled': message.auto_start_enabled,
        'execution_enabled': message.execution_enabled,
        'grasp_enabled': message.grasp_enabled,
        'tool_enabled': message.tool_enabled,
        'recovery_required': message.recovery_required,
        'progress': message.progress,
        'message': message.message,
        'blockers': list(message.blockers),
        'scene_epoch': int(getattr(message, 'scene_epoch', 0) or 0),
    }


def to_robot_status(message) -> dict:
    """转换机械臂状态（aubo_msgs/RobotStatus，简化 industrial 语义）."""
    return {
        'mode': int(message.mode),
        'e_stopped': int(message.e_stopped),
        'drives_powered': int(message.drives_powered),
        'motion_possible': int(message.motion_possible),
        'in_motion': int(message.in_motion),
        'in_error': int(message.in_error),
        'error_code': int(message.error_code),
    }


def _float_at(values, index):
    """数组下标转有限 float；越界或非数 → None."""
    try:
        value = float(values[index])
    except (TypeError, ValueError, IndexError):
        return None
    return value if math.isfinite(value) else None


def to_joint_state(message) -> dict:
    """sensor_msgs/JointState → 按关节名索引的实际角/速度."""
    names = [str(name) for name in (message.name or [])]
    position = list(message.position or [])
    velocity = list(message.velocity or [])
    effort = list(message.effort or [])
    by_name = {}
    for index, name in enumerate(names):
        by_name[name] = {
            'position': _float_at(position, index),
            'velocity': _float_at(velocity, index),
            'effort': _float_at(effort, index),
        }
    return {'stamp': stamp_seconds(message.header), 'by_name': by_name}


def to_joint_status(message) -> dict:
    """aubo_msgs/JointStatus → 6 槽电流/温度/目标/跟随误差（SDK 电流单位原样）."""
    codes = list(message.error_code or [])
    return {
        'current': [_float_at(message.current, i) for i in range(6)],
        'temperature': [_float_at(message.temperature, i) for i in range(6)],
        'tag_pos': [_float_at(message.tag_pos, i) for i in range(6)],
        'tag_vel': [_float_at(message.tag_vel, i) for i in range(6)],
        'following_error': [_float_at(message.following_error, i) for i in range(6)],
        'error_code': [
            int(codes[i]) if i < len(codes) else 0 for i in range(6)],
    }


def merge_joint_hardware(joint_state, joint_status) -> dict:
    """合成网页硬件表：实际角来自 /joint_states，电流等来自 joint_status."""
    by_name = (joint_state or {}).get('by_name') or {}
    status = joint_status or {}
    rows = []
    for index, name in enumerate(JOINT_ORDER):
        actual = by_name.get(name) or {}
        rows.append({
            'name': name,
            'position': actual.get('position'),
            'velocity': actual.get('velocity'),
            'effort': actual.get('effort'),
            'current': (status.get('current') or [None] * 6)[index],
            'temperature': (status.get('temperature') or [None] * 6)[index],
            'tag_pos': (status.get('tag_pos') or [None] * 6)[index],
            'tag_vel': (status.get('tag_vel') or [None] * 6)[index],
            'following_error': (status.get('following_error') or [None] * 6)[index],
            'error_code': (status.get('error_code') or [0] * 6)[index],
        })
    return {
        'rows': rows,
        'stamp': (joint_state or {}).get('stamp'),
    }


def _valid_scalar(value) -> float | None:
    """无效标量约定（-1，见 ReconstructionStatus.msg）→ None；其余转 float."""
    value = float(value)
    return value if value >= 0.0 else None


def to_reconstruction_status(message) -> dict:
    """
    结构化重建诊断（ReconstructionStatus）→ 浏览器镜像 dict.

    键名沿用旧 JSON 契约（state/target_id/captured_views/tf_latency_ms…），
    前端无需改动；无效标量（-1）折回 None 让前端按「无数据」渲染。
    view_coverage 只保留类型化后的摘要键；逐机位明细与 tsdf/registration/
    overlap/refined 由 diagnostics_debug 调试话题在 observability 层合并补充。
    """
    center = [float(v) for v in message.target_center_base]
    depth_ratio = _valid_scalar(message.valid_depth_ratio)
    return {
        'stamp': stamp_seconds(message.header),
        'harvest_run_id': message.harvest_run_id,
        'selected_target_id': message.selected_target_id,
        'state': message.state,
        'target_id': message.target_id,
        'target_center_base': (
            None if center == [-1.0, -1.0, -1.0] else center),
        'captured_views': int(message.captured_views),
        'rejected_views': int(message.rejected_views),
        'tf_failures': int(message.tf_failures),
        'tf_latency_ms': _valid_scalar(message.tf_latency_ms),
        'valid_depth_ratio': depth_ratio,
        'view_coverage': {
            'max_baseline_deg': _valid_scalar(message.max_baseline_deg),
            'mean_nearest_baseline_deg': _valid_scalar(
                message.mean_nearest_baseline_deg),
            'valid_depth_ratio_mean': depth_ratio,
        },
        'view_directions': [to_point(v) for v in message.view_directions],
    }


def to_grasp_hypothesis(message) -> dict:
    """技能侧抓取假设（GraspHypothesis）→ 浏览器镜像；不构成运动指令."""
    pose = message.entry_pose
    return {
        'stamp': stamp_seconds(message.header),
        'frame_id': message.header.frame_id,
        'target_id': message.target_id,
        'entry_position': to_point(pose.position),
        'entry_quaternion_xyzw': [
            float(pose.orientation.x), float(pose.orientation.y),
            float(pose.orientation.z), float(pose.orientation.w),
        ],
        'standoff_m': float(message.standoff_m),
        'travel_m': float(message.travel_m),
        'envelope_clearance_m': float(message.envelope_clearance_m),
        'rank_score': float(message.rank_score),
        'diagnostic_flags': list(message.diagnostic_flags),
    }


def to_grasp_decision(message) -> dict:
    """
    抓取许可（GraspDecision）→ 浏览器镜像 dict.

    融合几何与 allowed 独立：有轴则透出入口/剪切参考供预抓取目视；
    allowed 只表示套入/剪切接触许可。
    """
    value = {
        'stamp': stamp_seconds(message.header),
        'harvest_run_id': message.harvest_run_id,
        'target_id': message.target_id,
        # 工具档案标签（launch tool_profile 注入）：双工具排障时区分当前档案
        'tool_profile_id': message.tool_profile_id,
        'allowed': bool(message.allowed),
        'reason': message.reason,
        # 冻结有效期（秒，心跳不续签）：前端倒计时「许可还剩几秒」
        'valid_until': stamp_seconds_dict_time(message.valid_until),
        'model_revision': message.model_revision,
        'scene_epoch': int(getattr(message, 'scene_epoch', 0) or 0),
        'failure_code': int(getattr(message, 'failure_code', 0) or 0),
    }
    axis = message.axis
    has_geom = (axis.x * axis.x + axis.y * axis.y + axis.z * axis.z) > 0.25
    if message.allowed or has_geom:
        value.update({
            'entry': to_point(message.entry),
            'pregrasp': to_point(message.pregrasp),
            'cut_pose': to_point(message.cut_pose),
            'axis': to_point(message.axis),
            'diameter_m': float(message.diameter_m),
            'rmse_m': float(message.rmse_m),
            'inlier_ratio': float(message.inlier_ratio),
        })
    return value


def finite_or_none(value: Any) -> Any:
    """把非有限浮点递归转换为 JSON null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: finite_or_none(item) for key, item in value.items()}
    if isinstance(value, list):
        return [finite_or_none(item) for item in value]
    return value


class MetricsSampler:
    """周期采集 CPU/内存/load、GPU 与关键进程 CPU/RSS 的后台线程."""

    def __init__(self, period_s: float, patterns: list[str], apply,
                 log_warning=lambda msg: None):
        """保存采样周期、进程 cmdline 匹配关键字与结果回调."""
        self._period = max(0.5, float(period_s))
        self._patterns = [str(item) for item in patterns if str(item)]
        self._apply = apply
        self._log_warning = log_warning
        self._stop = threading.Event()
        self._thread = None
        # 按 pid 缓存 Process 句柄：cpu_percent(None) 依赖上次调用做差分
        self._processes = {}

    def start(self) -> None:
        """启动后台采样线程（幂等）."""
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name='peach-observability-metrics', daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """置停止标志并等待线程退出."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                self._apply(self._collect())
            except Exception as error:  # 采样失败只降级，绝不炸采样线程
                self._log_warning(f'性能采样失败（本轮跳过）: {error}')
            self._stop.wait(self._period)

    def _collect(self) -> dict:
        """采集一帧性能样本；任何子项失败都局部降级."""
        sample = {
            'stamp': time.time(),
            'cpu_percent': None,
            'memory_percent': None,
            'memory_used_mb': None,
            'memory_total_mb': None,
            'load1': None,
            'load5': None,
            'load15': None,
            'gpu': self._collect_gpu(),
            'processes': self._collect_processes(),
        }
        if psutil is not None:
            try:
                sample['cpu_percent'] = float(psutil.cpu_percent(None))
                memory = psutil.virtual_memory()
                sample['memory_percent'] = float(memory.percent)
                sample['memory_used_mb'] = round(memory.used / 1048576.0, 1)
                sample['memory_total_mb'] = round(memory.total / 1048576.0, 1)
                load1, load5, load15 = psutil.getloadavg()
                sample['load1'] = round(float(load1), 2)
                sample['load5'] = round(float(load5), 2)
                sample['load15'] = round(float(load15), 2)
            except (OSError, RuntimeError) as error:
                self._log_warning(f'系统性能采样降级: {error}')
        return sample

    def _collect_gpu(self) -> dict | None:
        """nvidia-smi 查询首块 GPU；不可用/超时/解析失败一律降级 None."""
        try:
            result = subprocess.run(
                ['nvidia-smi',
                 '--query-gpu=utilization.gpu,memory.used,memory.total',
                 '--format=csv,noheader,nounits'],
                capture_output=True, text=True, timeout=2.0, check=False)
            if result.returncode != 0:
                return None
            first = result.stdout.strip().splitlines()[0]
            util, used, total = (float(part.strip()) for part in first.split(','))
            return {
                'utilization_percent': util,
                'memory_used_mb': used,
                'memory_total_mb': total,
            }
        except (OSError, subprocess.SubprocessError, ValueError, IndexError):
            return None

    def _collect_processes(self) -> list[dict]:
        """按 cmdline 关键字匹配关键进程，取每组 RSS 最大者报 CPU/RSS."""
        if psutil is None or not self._patterns:
            return []
        try:
            candidates = list(psutil.process_iter(['pid', 'cmdline']))
        except (OSError, RuntimeError):
            return []
        matched = []
        live_pids = set()
        for pattern in self._patterns:
            best = None
            for info in candidates:
                cmdline = info.info.get('cmdline') or []
                if pattern not in ' '.join(cmdline):
                    continue
                try:
                    process = self._processes.get(info.info['pid'])
                    if process is None:
                        process = psutil.Process(info.info['pid'])
                        self._processes[info.info['pid']] = process
                    live_pids.add(process.pid)
                    rss = process.memory_info().rss
                    if best is None or rss > best[1]:
                        best = (process, rss)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            if best is None:
                continue
            process, rss = best
            try:
                matched.append({
                    'name': pattern,
                    'pid': process.pid,
                    'cpu_percent': float(process.cpu_percent(None)),
                    'rss_mb': round(rss / 1048576.0, 1),
                })
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        # 清掉已退出进程的缓存句柄，防止 pid 复用后误读
        for pid in [pid for pid in self._processes if pid not in live_pids]:
            del self._processes[pid]
        return matched


class ObservabilityState:
    """
    HTTP 与 ROS 回调之间的线程安全最新值缓存（只读监控，无写入口）.

    写侧 copy-on-write、读侧短锁浅拷贝（见模块 docstring）；snapshot()
    返回值只读，叶子与缓存共享引用。作业票（job）按快照代数记忆化：
    build_harvest_job 是全量折叠，而 snapshot 被 Web 轮询与轨迹/路标
    高频调用，每代只算一次、多调用方共享同一只读 dict。
    """

    _revision: int
    """快照代数；每次 update/append 递增."""
    _started: float
    """进程启动 wall time [s]."""
    _values: dict
    """分区缓存：perception / reconstruction / manipulation / …."""
    _updated: dict
    """'section.key' → 最近写入 wall time [s]."""

    def __init__(self):
        """创建空状态."""
        self._lock = threading.Lock()
        self._revision = 0
        self._started = time.time()
        self._job_cache: dict | None = None
        self._job_revision: int = -1
        self._values = {
            'perception': {
                'harvest': {},
                'targets': {},
            },
            'reconstruction': {
                'status': {},
                'diagnostics': {},
                'grasp_decision': {},
            },
            'refined': {
                'pose': {},
                'axis': {},
                'diagnostics': {},
            },
            'manipulation': {
                'status': {},
                'hypothesis': {},
            },
            'task_executor': {
                'state': {},
                # 批次过程/审计事件时间线（按到达顺序追加，超限截断头部）
                'events': [],
            },
            # 全流程阶段时序（服务器侧权威：调度 FSM / 技能周期，见 pipeline.py）
            'pipeline': {
                'fsm': [],
                'arm': [],
            },
            # 批次账本直播（runs/<request_id>/ledger.json 增量归一化行）
            'ledger': {
                'live': {},
            },
            # 机械臂状态（aubo_msgs/RobotStatus）+ latest TF 末端 + 关节硬件表
            'robot': {
                'status': {},
                'tcp': {},
                'joints': {},
            },
            # 系统/GPU/进程性能采样（独立线程写入，单次整体替换）
            'metrics': {
                'sample': {},
            },
            # 监控数据落盘记录器状态（enabled + 当前 run 目录）
            'record': {
                'info': {},
            },
            # 各节点当前参数只读镜像：{节点名: {参数名: 标量值}}
            'params': {},
        }
        self._updated = {}

    def update(self, section: str, key: str, value) -> None:
        """更新一个结构化状态区段."""
        now = time.time()
        with self._lock:
            self._values[section][key] = finite_or_none(value)
            self._updated[f'{section}.{key}'] = now
            self._revision += 1

    def events_snapshot(self) -> list:
        """事件时间线浅拷贝（呈现层选择叠加等只读消费）."""
        with self._lock:
            return list(self._values['task_executor']['events'])

    def topic_ages(self, now: float | None = None) -> dict[str, float]:
        """各镜像键的年龄 [s]（'section.key' → 距最近写入；诊断用）."""
        now = time.time() if now is None else now
        with self._lock:
            return {
                key: round(now - stamp, 3)
                for key, stamp in self._updated.items()}

    def append_event(self, value, limit: int = 100) -> None:
        """追加一条批次事件到环形缓冲，只保留最近 limit 条."""
        now = time.time()
        with self._lock:
            events = self._values['task_executor']['events']
            events.append(finite_or_none(value))
            if len(events) > limit:
                del events[:len(events) - limit]
            self._updated['task_executor.events'] = now
            self._revision += 1

    def update_params(self, node_name: str, values: dict) -> None:
        """整体替换一个节点的参数镜像并刷新其时间戳."""
        now = time.time()
        with self._lock:
            self._values['params'][node_name] = finite_or_none(values)
            self._updated[f'params.{node_name}'] = now
            self._revision += 1

    def snapshot(self) -> dict:
        """返回浏览器状态快照与话题年龄（只读浅拷贝视图，禁止原地改叶子）."""
        now = time.time()
        with self._lock:
            # 短锁内两层浅拷贝：区段 dict 复制一层（顶层键替换不写穿），
            # events 列表复制一份（写侧原地追加/截头）；其余叶子按引用
            # 共享，靠写侧 copy-on-write 保证不被改动。序列化惰性留给
            # HTTP 层，省掉旧的 json 往返深拷贝。
            result = {}
            for section, values in self._values.items():
                copied = dict(values)
                events = copied.get('events')
                if isinstance(events, list):
                    copied['events'] = list(events)
                result[section] = copied
            revision = self._revision
            result['system'] = {
                'revision': revision,
                'server_time': now,
                'uptime_s': now - self._started,
                'topic_age_s': {
                    key: round(now - stamp, 3)
                    for key, stamp in self._updated.items()},
            }
        result['job'] = self._job_for(result, revision)
        return result

    def job(self) -> dict:
        """窄访问器：只取当前作业票（轨迹/路标高频路径不整树浅拷贝）."""
        with self._lock:
            revision = self._revision
        if self._job_cache is not None and self._job_revision == revision:
            return self._job_cache
        return self._job_for(self.snapshot(), revision)

    def _job_for(self, snapshot: dict, revision: int) -> dict:
        """按快照代数缓存作业票；并发重复计算无害（幂等纯折叠）."""
        if self._job_cache is None or self._job_revision != revision:
            self._job_cache = build_harvest_job(snapshot)
            self._job_revision = revision
        return self._job_cache
