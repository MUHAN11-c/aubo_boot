"""
选果、控制面、摘要与账本（批次纯函数）.

ROS msg（builtin_interfaces / peach_interfaces）按调用点延迟 import：
本模块保持零 ROS 环境可导入（纯核单测/回放；对齐 harvest_fsm 惯例），
消息构造只发生在 build_summary / elapsed_msg / dict_to_outcome 等出口。
"""
from __future__ import annotations

import json
from pathlib import Path
import time

from peach_common.paths import runs_root as _runs_root
from peach_common.paths import safe_component

# 与 peach_arm.yaml grasp.minimum/maximum_travel_m 对齐；服务不可用时
# 用同一夹紧估算套入终点半径（FULL 回退窗）。
_SLEEVE_TRAVEL_MIN_M = 0.02
_SLEEVE_TRAVEL_MAX_M = 0.20


def pregrasp_pose_of(item):
    """
    观测候选 → 感知入口 7 元组 (px,py,pz,qx,qy,qz,qw).

    SELECT 把入口送给 CheckReachability；服务端再换成与 MovePregrasp 同一
    停位（沿袋轴后撤 mtc_approach_along_axis_m + alignFrameZ）。本函数仍
    返回入口，供查询与半径回退。几何未绑定 / 坐标系非 base 系返回 None.
    """
    cand = getattr(item, 'candidate', None)
    if cand is None:
        return None
    frame = str(getattr(
        getattr(cand, 'header', None), 'frame_id', '') or '')
    if 'base' not in frame:
        return None
    entry_pose = getattr(cand, 'entry_pose', None)
    entry = getattr(entry_pose, 'position', None)
    if entry is None or not (entry.x or entry.y or entry.z):
        return None
    q = getattr(entry_pose, 'orientation', None)
    if q is None:
        return None
    return (entry.x, entry.y, entry.z, q.x, q.y, q.z, q.w)


def suggested_travel_of(item):
    """候选 suggested_travel_m；缺失/非正则 0（服务端再用最小行程）."""
    cand = getattr(item, 'candidate', None)
    if cand is None:
        return 0.0
    return float(getattr(cand, 'suggested_travel_m', 0.0) or 0.0)


def _quat_z_axis(qx, qy, qz, qw):
    """四元数 → 工具/袋轴 Z（与入口姿态约定一致）."""
    return (
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


def _clamp_sleeve_travel(suggested_m):
    if suggested_m <= 1.0e-6:
        return _SLEEVE_TRAVEL_MIN_M
    return min(
        _SLEEVE_TRAVEL_MAX_M,
        max(_SLEEVE_TRAVEL_MIN_M, float(suggested_m)))


def _sleeve_radius(item):
    """套入终点半径（入口沿袋轴 + clamp(travel)）；几何缺失 None."""
    pose = pregrasp_pose_of(item)
    if pose is None:
        return None
    px, py, pz, qx, qy, qz, qw = pose
    ax, ay, az = _quat_z_axis(qx, qy, qz, qw)
    norm = (ax * ax + ay * ay + az * az) ** 0.5
    if norm < 1.0e-9:
        return None
    travel = _clamp_sleeve_travel(suggested_travel_of(item))
    sx = px + ax / norm * travel
    sy = py + ay / norm * travel
    sz = pz + az / norm * travel
    return (sx * sx + sy * sy + sz * sz) ** 0.5


def _eligible_locked_items(observations, claimed):
    """
    枚举锁定集内可执行候选：(target_id, item) 生成器.

    资格：锁定集内、target_id 非空、未入账、已确认、非裸果
    （unbagged_display_only / fruit 线不进执行候选）、非贴边
    （bbox_edge：P1-A 贴边滑动噪声块不得被选；移入视野内旗标即消；
    tool_clearance_failed：单帧门硬拒，套筒过不去不得被选）。
    reach_queries 与 next_target 共用同一谓词，避免两份过滤条件漂移。
    """
    if observations is None:
        return
    if not bool(getattr(observations, 'target_set_locked', False)):
        return
    claimed = set(claimed)
    for item in observations.observations:
        tid = getattr(item, 'target_id', '')
        if not tid or tid in claimed or not getattr(item, 'confirmed', False):
            continue
        flags = list(getattr(item, 'diagnostic_flags', []) or [])
        cand = getattr(item, 'candidate', None)
        strat = str(getattr(cand, 'strategy_id', '') or '')
        if ('unbagged_display_only' in flags or 'fruit' in strat
                or 'bbox_edge' in flags or 'tool_clearance_failed' in flags):
            continue
        yield str(tid), item


def reach_queries(observations, claimed, preferred=()):
    """
    枚举待检目标：(tid, 入口 pose7, suggested_travel_m)（纯函数，供 IK 预检）.

    只含锁定集内已确认、未入账、非裸果且几何可构造的表项；
    preferred（goal 显式名单）不参与预检——直通不受窗限。
    """
    out = []
    for tid, item in _eligible_locked_items(observations, claimed):
        pose = pregrasp_pose_of(item)
        if pose is not None:
            out.append((tid, pose, suggested_travel_of(item)))
    return out


def _pregrasp_radius(item):
    """估计预抓取点半径（无 IK 时的回退窗基准）；几何缺失 None."""
    pose = pregrasp_pose_of(item)
    if pose is None:
        return None
    return (pose[0] ** 2 + pose[1] ** 2 + pose[2] ** 2) ** 0.5


def next_target(
        observations, claimed, preferred=(),
        depth_range=(0.30, 1.60), ik_results=None,
        fallback_reach_range=(0.15, 0.88),
        check_sleeve=False):
    """
    联合约束选果：有效深度窗 ∩ 可达性，返回 (target_id, filtered).

    可达性判定优先用 **TCP IK 预检结果**（ik_results: tid→(reachable, code)，
    由技能 CheckReachability 把入口换成停位几何后以当前关节为种子求解；
    FULL 时服务端再检套入终点与沿轴笛卡尔，失败码 sleeve_no_ik /
    sleeve_no_cartesian）；
    ik_results 为 None（服务不可用/mock）时回退**估计入口半径窗**
    （fallback_reach_range，现场标定：成功 0.830–0.840 / MTC 0 解 ≥0.917）。
    check_sleeve=True（非 execute_pregrasp_only）时回退窗另卡套入终点半径。
    goal.target_ids 显式名单直通（不受窗限）。深度距离窗用
    camera_distance_m（质量随距离退化；采集门 min_mask_depth_ratio 仍逐帧
    把关）。超窗目标记入 filtered（tid→reason）并由调用方发
    targets_filtered 事件——「为什么没人被选」必须可归因。
    """
    claimed = set(claimed)
    # S6：preferred（goal 显式名单）同样过资格谓词——未确认/裸果/贴边
    # 不得被点名直通；过滤后无候选走常规路径（不受窗限语义保留给
    # 合格的 preferred）。
    eligible = dict(
        _eligible_locked_items(observations, claimed))
    for tid in preferred:
        if tid and str(tid) in eligible and tid not in claimed:
            return str(tid), []

    def _window_reasons(item, tid):
        reasons = []
        depth = float(getattr(item, 'camera_distance_m', 0.0) or 0.0)
        if depth > 0 and not depth_range[0] <= depth <= depth_range[1]:
            reasons.append(f'out_of_depth_window:{depth:.2f}m')
        if ik_results is not None and tid in ik_results:
            reachable, code = ik_results[tid]
            if not reachable:
                reasons.append(f'ik_no_solution:{code or "no_ik"}')
        else:
            radius = _pregrasp_radius(item)
            if radius is not None and not (
                    fallback_reach_range[0] <= radius
                    <= fallback_reach_range[1]):
                reasons.append(f'out_of_reach_window:{radius:.2f}m')
            if check_sleeve:
                sleeve_r = _sleeve_radius(item)
                if sleeve_r is not None and not (
                        fallback_reach_range[0] <= sleeve_r
                        <= fallback_reach_range[1]):
                    reasons.append(
                        f'out_of_sleeve_reach_window:{sleeve_r:.2f}m')
        return reasons

    filtered = {}
    # 次序策略：感知 priority 主序（越小越先），同级按**检测框面积降序**
    # ——近距双检（同一颗袋大框+遮挡残片小框）先做大框；小框多为叶片
    # 遮挡残片或误检（09-01 现场定夺）。窗过滤按此次序进行，首个合格即选。
    candidates = []
    for tid, item in _eligible_locked_items(observations, claimed):
        box = getattr(item, 'candidate_2d', None)
        area = float(box.bbox_w) * float(box.bbox_h) if (
            box is not None and box.bbox_w > 0 and box.bbox_h > 0) else 0.0
        candidates.append((int(getattr(item, 'priority', 0) or 0), -area, tid, item))
    candidates.sort(key=lambda row: (row[0], row[1]))
    for _prio, _neg_area, tid, item in candidates:
        reasons = _window_reasons(item, tid)
        if reasons:
            filtered[tid] = ';'.join(reasons)
            continue
        return tid, filtered
    return '', filtered


def apply_control(
        state_seq: int, expected: int, command: int, paused: bool,
        allowed=None):
    """
    纯函数控制面.

    expected==0 表示不校验。
    PAUSE=0 RESUME=1 ENTER_MAINTENANCE=2 EXIT_MAINTENANCE=3
    CANCEL_NOW=4 SKIP_TARGET=5 ACKNOWLEDGE_RECOVERY=6.
    allowed 为 permissions_for 结果；None 表示不按状态机过滤。
    返回 (accepted, new_seq, paused, cancel, skip).
    """
    if expected != 0 and expected != state_seq:
        return False, state_seq, paused, False, False
    if allowed is not None and command not in allowed:
        return False, state_seq, paused, False, False
    if command in (0, 2):
        return True, state_seq + 1, True, False, False
    if command in (1, 3):
        return True, state_seq + 1, False, False, False
    if command == 4:
        return True, state_seq + 1, paused, True, False
    if command == 5:
        return True, state_seq + 1, paused, False, True
    if command == 6:
        return True, state_seq + 1, paused, False, False
    return False, state_seq, paused, False, False


def elapsed_msg(seconds: float):
    """单调时钟秒数 → Duration（小数进位到 sec，nanosec 恒 < 1s）."""
    from builtin_interfaces.msg import Duration
    msg = Duration()
    safe = max(0.0, float(seconds))
    total_ns = int(round(safe * 1e9))
    msg.sec = total_ns // 1_000_000_000
    msg.nanosec = total_ns % 1_000_000_000
    return msg


def build_summary(run_id: str, outcomes, discovered: int,
                  elapsed_s: float):
    """填充 HarvestSummary 计数与账本."""
    from peach_interfaces.msg import HarvestSummary, TargetOutcome
    summary = HarvestSummary()
    summary.run_id = run_id
    summary.outcomes = list(outcomes)
    summary.discovered = int(max(discovered, len(summary.outcomes)))
    summary.attempted = len(summary.outcomes)
    summary.elapsed = elapsed_msg(elapsed_s)
    for item in summary.outcomes:
        code = int(item.outcome)
        if code == TargetOutcome.SUCCEEDED:
            summary.succeeded += 1
        elif code == TargetOutcome.SKIPPED_QUALITY:
            summary.skipped_quality += 1
        elif code == TargetOutcome.SKIPPED_UNREACHABLE:
            summary.skipped_unreachable += 1
        elif code == TargetOutcome.FAILED:
            summary.failed += 1
        elif code == TargetOutcome.CANCELED:
            summary.canceled += 1
    return summary


def default_runs_root() -> Path:
    """过程数据根目录（W6-B 单源委托 peach_common.paths.runs_root）."""
    return _runs_root()


def resolve_runs_root(configured: str = '') -> Path:
    """参数给出绝对路径则用之，否则回 ``default_runs_root()``."""
    return _runs_root(configured)


def default_ledger_root() -> Path:
    """账本根目录，与观测/session 同为工作区 ``runs/``."""
    return default_runs_root()


def _safe_run_component(request_id: str, fallback: str) -> str:
    """
    目录名段净化（W1 起单源委托 peach_common.paths.safe_component）.

    语义收紧：在旧「拒路径分隔符与上跳」之上补拒 NUL 与纯点段；非法
    回退 fallback（折叠隔离目录，不拒绝整批）。
    """
    return safe_component(request_id, fallback)


def ledger_file(root: Path, request_id: str) -> Path:
    """单个批次的 ledger.json 路径（request_id 过滤路径穿越）."""
    safe = _safe_run_component(request_id, 'harvest')
    return Path(root) / safe / 'ledger.json'


def elapsed_s(outcome) -> float | None:
    """TargetOutcome.elapsed → 秒；零/缺省给 None."""
    elapsed = getattr(outcome, 'elapsed', None)
    if elapsed is None:
        return None
    value = (float(getattr(elapsed, 'sec', 0) or 0)
             + float(getattr(elapsed, 'nanosec', 0) or 0) * 1e-9)
    return round(value, 3) if value > 0.0 else None


def set_elapsed(outcome, seconds: float) -> None:
    """把墙钟秒写入 TargetOutcome.elapsed（小数进位到 sec）."""
    value = max(float(seconds or 0.0), 0.0)
    sec = int(value)
    nanosec = int(round((value - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    outcome.elapsed.sec = sec
    outcome.elapsed.nanosec = nanosec


# ledger 遥测附加键白名单（W6-B/S4）：_cmd_full/_record_skip 写入的
# extra 键全集，对齐 observability pipeline._ROW_PASSTHROUGH 期望。
# cut/retreat/harvest_confirmed 三 bool 键不补——W7 IDL 收敛将随
# ExecuteTarget.Result 顶层删除。
_EXTRA_KEYS = (
    'failure_code', 'failure_code_n', 'completion_level', 'stage_names',
    'stage_durations', 'build_view_count', 'build_status', 'build_duration_s',
    'timeout_source')


def outcome_to_dict(outcome, extra: dict | None = None) -> dict:
    """Serialize TargetOutcome plus optional telemetry extra."""
    data = {
        'target_id': str(outcome.target_id),
        'outcome': int(outcome.outcome),
        'reason': str(outcome.reason),
        'quality_score': float(getattr(outcome, 'quality_score', 0.0) or 0.0),
        'elapsed_s': elapsed_s(outcome),
    }
    extra = extra or {}
    for key in _EXTRA_KEYS:
        if extra.get(key) is not None:
            data[key] = extra[key]
    return data


def dict_to_outcome(data: dict):
    """Build TargetOutcome from a ledger dict."""
    from peach_interfaces.msg import TargetOutcome
    item = TargetOutcome()
    item.target_id = str(data.get('target_id', ''))
    item.outcome = int(data.get('outcome', TargetOutcome.FAILED))
    item.reason = str(data.get('reason', ''))
    item.quality_score = float(data.get('quality_score', 0.0) or 0.0)
    if data.get('elapsed_s') is not None:
        set_elapsed(item, float(data['elapsed_s']))
    return item


def load_ledger(path: Path) -> tuple:
    """
    Return claimed IDs, outcomes and per-outcome extras; missing file yields empty.

    W6-B/S7：extras（白名单键）随账本回读，供断点恢复回填
    ``_outcome_details``——恢复批的遥测细节不再归零成空 dict。
    """
    if not path.is_file():
        return set(), [], []
    raw = json.loads(path.read_text(encoding='utf-8'))
    claimed = {str(tid) for tid in raw.get('claimed', []) if tid}
    outcomes = []
    details = []
    for item in raw.get('outcomes', []):
        outcome = dict_to_outcome(item)
        outcomes.append(outcome)
        details.append({
            key: item[key] for key in _EXTRA_KEYS
            if item.get(key) is not None})
        if outcome.target_id:
            claimed.add(outcome.target_id)
    return claimed, outcomes, details


def save_ledger(path: Path, claimed, outcomes, details=None) -> None:
    """原子写 ledger.json（details 与 outcomes 等长的遥测附加字段）."""
    path.parent.mkdir(parents=True, exist_ok=True)
    extras = list(details or [])
    rows = []
    for index, item in enumerate(outcomes):
        extra = extras[index] if index < len(extras) else {}
        rows.append(outcome_to_dict(item, extra))
    document = {
        'claimed': sorted(str(tid) for tid in claimed if tid),
        'outcomes': rows,
    }
    tmp = path.with_suffix('.json.tmp')
    tmp.write_text(
        json.dumps(document, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8')
    tmp.replace(path)


# ---- 结果入账纯函数（W6-A 自 executor_node 下沉；零 ROS）----

def stages_from_execute(executed) -> dict:
    """将 ExecuteTarget 结果中的阶段耗时转换为 ledger extra."""
    if executed is None:
        return {}
    names = [str(n) for n in list(getattr(executed, 'stage_names', []) or [])]
    raw = list(getattr(executed, 'stage_durations', []) or [])
    durations = []
    for item in raw:
        durations.append(
            round(float(getattr(item, 'sec', 0) or 0)
                  + float(getattr(item, 'nanosec', 0) or 0) * 1e-9, 3))
    if not names:
        return {}
    return {'stage_names': names, 'stage_durations': durations}


def full_failure_code(outcome, operator_skip: bool) -> str:
    """按 TargetOutcome 分级 FULL 失败码，质量/不可达不记 full_failed."""
    from peach_interfaces.msg import TargetOutcome
    if operator_skip or int(outcome.outcome) == int(TargetOutcome.CANCELED):
        return 'canceled'
    code = int(outcome.outcome)
    if code == int(TargetOutcome.SKIPPED_QUALITY):
        return 'skipped_quality'
    if code == int(TargetOutcome.SKIPPED_UNREACHABLE):
        return 'skipped_unreachable'
    return 'full_failed'


def merge_cycle_extra(observe_extra, full_extra) -> dict:
    """合并 OBSERVE_ONLY 与 FULL 的阶段耗时；丢掉 FULL 里为零的观察段."""
    merged = dict(observe_extra or {})
    full_extra = dict(full_extra or {})
    obs_names = [str(n) for n in list(merged.get('stage_names') or [])]
    obs_durs = list(merged.get('stage_durations') or [])
    while len(obs_durs) < len(obs_names):
        obs_durs.append(0.0)
    obs_durs = obs_durs[:len(obs_names)]
    skip = {'prepare', 'observe', 'finalize'}
    keep_names = []
    keep_durs = []
    full_names = [str(n) for n in list(full_extra.get('stage_names') or [])]
    full_durs = list(full_extra.get('stage_durations') or [])
    for index, name in enumerate(full_names):
        if name in skip:
            continue
        keep_names.append(name)
        keep_durs.append(
            float(full_durs[index]) if index < len(full_durs) else 0.0)
    if obs_names or keep_names:
        merged['stage_names'] = obs_names + keep_names
        merged['stage_durations'] = [
            round(float(d), 3) for d in obs_durs + keep_durs]
    for key, value in full_extra.items():
        if key in ('stage_names', 'stage_durations'):
            continue
        merged[key] = value
    return merged


def build_details(build_feedback: dict, dispatch_t0: float, built) -> dict:
    """Build 反馈与耗时摘要（build_feedback 为节点侧反馈缓存快照）."""
    started = float(build_feedback.get('started_s') or dispatch_t0)
    details = {
        'build_view_count': int(
            build_feedback.get('view_count') or 0),
        'build_status': str(build_feedback.get('status') or ''),
        'build_duration_s': round(time.monotonic() - started, 3),
    }
    if built is not None:
        model = getattr(built, 'model', None)
        if model is not None and getattr(model, 'view_count', None) is not None:
            details['build_view_count'] = int(model.view_count)
        status = str(getattr(built, 'message', '') or '')
        if status:
            details['build_status'] = status
        # P2 反馈补齐：BuildTargetModel.Result.failure_code 数值码透传进账本
        code = int(getattr(built, 'failure_code', 0) or 0)
        if code:
            details['build_failure_code'] = code
    return details


def classify_build_failure(built, message: str) -> tuple:
    """区分执行器等待超时 / 重建内部 timeout / finalize 失败."""
    text = str(message or '')
    if built is None or text == 'build_timeout:executor_wait':
        return 'build_timeout:executor_wait', 'build_timeout:executor_wait'
    if text == 'timeout':
        return 'build_timeout:reconstruction', 'build_timeout:reconstruction'
    if text in ('canceled', 'cancelled'):
        return 'canceled', 'build_canceled'
    return 'build_finalize_failed', 'build_finalize_failed: ' + text
