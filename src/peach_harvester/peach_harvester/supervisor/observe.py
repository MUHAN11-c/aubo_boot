"""
Fast 档观察纯核（W6-A 自 executor_node 抽取；零 ROS import）.

观测缓存 → 视点质量信号 / 目标锚点，TF 相机位解析（查询闭包注入），
以及 fast 档补视循环驱动。节点侧（executor_node）只保留薄接线：闭包
注入取观测、TF 查询与 MoveTo 发送；等待/事件顺序与拆分前逐字一致。
"""
from __future__ import annotations

import time

from peach_harvester.cycle_core.view_planner import (
    basis_to_quat,
    generate,
    look_at_optical,
    ViewContext,
    ViewPlannerConfig,
)
from peach_harvester.cycle_core.view_policy import (
    decide_fast,
    FastViewConfig,
    ViewDecision,
    ViewPolicyState,
    ViewSignals,
)

# latest TF 回退陈旧上限：补视在 MoveTo 刚结束时调用，臂静止时 TF 年龄
# 应远低于此；超限说明链路异常，latest 位姿不可当补视几何用
TF_FALLBACK_STALE_S = 1.0


def _observation_item(observations, target_id: str):
    if observations is None:
        return None
    for item in getattr(observations, 'observations', []):
        if str(getattr(item, 'target_id', '')) == str(target_id):
            return item
    return None


def view_signals(observations, target_id: str) -> ViewSignals:
    """
    当前机位质量信号（观测缓存 → ViewSignals）.

    TF 门读观测 diagnostic_flags：tf_stale / tf_unavailable 时本帧
    信号不作数（ViewSignals.tf_ok=False → 不给 ENOUGH，防静止图像
    被当好单视收口）。
    """
    item = _observation_item(observations, target_id)
    if item is None:
        return ViewSignals(tf_ok=False, bbox_valid=False)
    flags = set(getattr(item, 'diagnostic_flags', []) or [])
    tf_ok = not ({'tf_stale', 'tf_unavailable'} & flags)
    bbox = getattr(item, 'candidate_2d', None)
    mask = getattr(item, 'mask', None)
    fitting = getattr(item, 'fitting', None)
    width = int(getattr(mask, 'width', 0) or 0) or 640
    height = int(getattr(mask, 'height', 0) or 0) or 480
    bbox_valid = bool(
        getattr(bbox, 'bbox_w', 0) and getattr(bbox, 'bbox_h', 0))
    area_ratio = 0.0
    if bbox_valid:
        area_ratio = (
            float(bbox.bbox_w) * float(bbox.bbox_h)) / float(
            max(1, width) * max(1, height))
    return ViewSignals(
        bbox_area_ratio=area_ratio,
        mask_foreground_ratio=float(
            getattr(fitting, 'foreground_ratio', -1.0)),
        tf_ok=tf_ok,
        bbox_valid=bbox_valid)


def target_anchor(observations, target_id: str):
    """观测候选 bag_bottom → base 系锚点 [x,y,z]；缺观测/几何 None."""
    item = _observation_item(observations, target_id)
    if item is None:
        return None
    bottom = getattr(getattr(item, 'candidate', None), 'bag_bottom', None)
    if bottom is None:
        return None
    return [float(bottom.x), float(bottom.y), float(bottom.z)]


def camera_position(lookup_exact, lookup_latest, now_s: float, warn,
                    stale_limit_s: float = TF_FALLBACK_STALE_S):
    """
    相机位（base 系）：精确时刻优先；latest 回退须过陈旧门.

    lookup_exact(target, source, timeout_s) / lookup_latest(target, source)
    由节点注入（tf2 Buffer 查询）；本函数只做回退次序、陈旧判定与平移
    提取，返回 [x, y, z]；链路缺失/回退陈旧返回 None。
    """
    target, source = 'base_link', 'camera_depth_optical_frame'
    try:
        # 精确时刻查询（运动中正确）；给 0.5s 缓冲等链路就绪，避免
        # 首帧/瞬时未就绪即抛异常导致补视被跳过（09-17 E2E 实测）
        tf = lookup_exact(target, source, 0.5)
    except Exception:  # noqa: BLE001 精确时刻不可得；latest 须新鲜
        try:
            tf = lookup_latest(target, source)
            stamp_s = (
                tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
            if now_s - stamp_s > stale_limit_s:
                warn(f'latest TF 回退已陈旧（{now_s - stamp_s:.2f}s > '
                     f'{stale_limit_s}s），fast 补视按几何缺失收口')
                return None
        except Exception as exc:  # noqa: BLE001 链路缺失
            warn(f'相机 TF 链路缺失（{target}←{source}），'
                 f'fast 补视按几何缺失收口: {exc}')
            return None
    tr = tf.transform.translation
    return [float(tr.x), float(tr.y), float(tr.z)]


def fast_observe_loop(
        target_id: str, *, signals, camera_pos, anchor, move,
        canceled, skip_requested, warn, action_timeout_s: float,
        min_views: int) -> tuple:
    """
    Fast 档观察循环：单视决策→低置信补视（封顶 3 视）→交 Build 收口.

    依赖全部注入（零 ROS）：signals() → ViewSignals，camera_pos() /
    anchor() → [x,y,z]|None，move(position, quat, timeout_s) 执行补视
    移动并返回 MoveTo 结果（None=失败/未到），canceled()/skip_requested()
    为谓词，warn(str) 记日志。返回 (observe_ok, details)；取消/跳过/
    几何缺失 → not ok。
    """
    # W2/S2：单步移动上限压到仓规 ≤18s（旧式 min(timeout,30)=30s）。
    move_timeout = min(float(action_timeout_s), 18.0)
    state = ViewPolicyState(used_views=1)
    cfg = FastViewConfig()
    moves_used = 0
    t0 = time.monotonic()
    while not canceled() and not skip_requested():
        decision = decide_fast(
            signals(), state, cfg, min_views=int(min_views))
        if decision is not ViewDecision.SUPPLEMENT:
            return True, {
                'view_policy': 'fast',
                'view_decision': decision.value,
                'view_moves': moves_used,
                'observe_elapsed_s': round(time.monotonic() - t0, 3)}
        camera = camera_pos()
        target_xyz = anchor()
        if camera is None or target_xyz is None:
            warn(f'fast 补视缺几何（camera={camera is not None} '
                 f'anchor={target_xyz is not None}），按封顶收口')
            return True, {
                'view_policy': 'fast', 'view_decision': 'geometry_missing',
                'view_moves': moves_used,
                'observe_elapsed_s': round(time.monotonic() - t0, 3)}
        context = ViewContext(
            target=target_xyz, current_camera_position=camera,
            observed_directions=[[
                camera[0] - target_xyz[0],
                camera[1] - target_xyz[1],
                camera[2] - target_xyz[2]]])
        candidates = generate(context, ViewPlannerConfig())
        if not candidates:
            return True, {
                'view_policy': 'fast', 'view_decision': 'no_candidate',
                'view_moves': moves_used,
                'observe_elapsed_s': round(time.monotonic() - t0, 3)}
        top = candidates[0]
        basis = look_at_optical(top.position, target_xyz)
        quat = basis_to_quat(basis)
        moved = move(top.position, quat, move_timeout)
        moves_used += 1
        state = ViewPolicyState(used_views=state.used_views + 1)
        if moved is None:
            warn(f'fast 补视 MoveTo 失败（{target_id}，已移 {moves_used}）')
            return (not canceled()), {
                'view_policy': 'fast', 'view_decision': 'move_failed',
                'view_moves': moves_used,
                'observe_elapsed_s': round(time.monotonic() - t0, 3)}
    return (not canceled()), {
        'view_policy': 'fast', 'view_decision': 'canceled',
        'view_moves': moves_used,
        'observe_elapsed_s': round(time.monotonic() - t0, 3)}
