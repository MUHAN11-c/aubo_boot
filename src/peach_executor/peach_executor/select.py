from __future__ import annotations
"""选果、控制面、摘要与账本（批次纯函数）。"""


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


def _eligible_locked_items(observations, claimed):
    """
    枚举锁定集内可执行候选：(target_id, item) 生成器.

    资格：锁定集内、target_id 非空、未入账、已确认、非裸果
    （unbagged_display_only / fruit 线不进执行候选）。reach_queries
    与 next_target 共用同一谓词，避免两份过滤条件漂移。
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
        if 'unbagged_display_only' in flags or 'fruit' in strat:
            continue
        yield str(tid), item


def reach_queries(observations, claimed, preferred=()):
    """
    枚举待检目标与估计预抓取位姿：[(tid, pose7)]（纯函数，供 IK 预检）.

    只含锁定集内已确认、未入账、非裸果且几何可构造的表项；
    preferred（goal 显式名单）不参与预检——直通不受窗限。
    """
    out = []
    for tid, item in _eligible_locked_items(observations, claimed):
        pose = pregrasp_pose_of(item)
        if pose is not None:
            out.append((tid, pose))
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
        fallback_reach_range=(0.15, 0.88)):
    """
    联合约束选果：有效深度窗 ∩ 可达性，返回 (target_id, filtered).

    可达性判定优先用 **TCP IK 预检结果**（ik_results: tid→(reachable, code)，
    由技能 CheckReachability 把入口换成停位几何后以当前关节为种子求解）；
    ik_results 为 None（服务不可用/mock）时回退**估计预抓取点半径窗**
    （fallback_reach_range，现场标定：成功 0.830–0.840 / MTC 0 解 ≥0.917）。
    goal.target_ids 显式名单直通（不受窗限）。深度距离窗用
    camera_distance_m（质量随距离退化；采集门 min_mask_depth_ratio 仍逐帧
    把关）。超窗目标记入 filtered（tid→reason）并由调用方发
    targets_filtered 事件——「为什么没人被选」必须可归因。
    """
    claimed = set(claimed)
    for tid in preferred:
        if tid and tid not in claimed:
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
