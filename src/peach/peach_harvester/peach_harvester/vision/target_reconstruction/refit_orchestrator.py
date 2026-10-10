"""
refit 编排纯核：TSDF refit → 袋关键点融合 → merge（W4 自节点下沉）.

节点 ``_run_refit`` 只保留缓存成对写入与产物版本记账；本模块承接
``_run_refit`` 的计算与日志本体（数学与分支判据零改动）。调用方须持
``_state_lock``（与旧节点方法同锁域）；``_refined``/``_bag_model`` 的
成对写入权在节点侧，本模块不触碰任何缓存。
"""
from __future__ import annotations

import time
from typing import Callable, Optional, Tuple

import numpy as np
from peach_harvester.vision.common.tool_budget import ToolBudgetParams
from peach_harvester.vision.target_reconstruction.refine import (
    BagModel,
    collect_bag_views,
    fuse_bag_views,
    merge_fused_bag_model,
    RefitConfig,
    RefitResult,
    select_refitter,
    STATUS_ACCEPT,
)


class RefitOrchestrator:
    """
    refit 管线编排器（refitters/refit_config/timing 注入；零 ROS）.

    ctor 另注入单调时钟 now（协议 I3）和宿主 logger。

    G3（2026-09-20）：本对象是 revision 单调计数的宿主——进程级
    ``_finalize_counter`` 每次 ``run``（每轮 refit/融合，含 finalize 与
    COLLECTING 期 live refit）递增，节点 reset_reconstruction 不清零
    （本类无 reset 口，节点也不重建实例），保证同目标重 Build 且机位
    数相同时 ``model_revision`` 仍变化。``run`` 按模块契约在调用方
    ``_state_lock`` 内执行，计数读写随之线程安全（现有锁纪律）。
    """

    def __init__(self, refitters: dict, refit_config: RefitConfig,
                 timing, logger, now: Callable[[], float] = time.perf_counter):
        """
        保存注入组件（协议 I3：now 注入单调时钟）.

        节点注入 RclpyClockAdapter.now；纯核自包含缺省回退
        time.perf_counter（与 LocalTsdf 同模式）。

        Args:
            refitters: {'cylinder': …, 'sphere': …}（session.refitters）.
            refit_config: RefitConfig 门控.
            timing: capture.TimingStats（refit 耗时落账）.
            logger: 宿主 logger（节点 get_logger()，行为与旧节点内打日志一致）.
            now: 单调时钟（秒）.

        Returns
        -------
            无返回值（None）.

        """
        self._refitters = refitters
        self._refit_config = refit_config
        self._timing = timing
        self._logger = logger
        self._now = now
        # G3：进程级单调 finalize/refit 轮次计数（见类 docstring）
        self._finalize_counter = 0

    def run(
            self, *,
            tsdf_xyz: Optional[np.ndarray],
            frames: list,
            target_center,
            kind: str,
            kind_defaulted: bool,
            bound_axis_hint,
            target_id: str,
            entry_standoff_m: float,
            pregrasp_standoff_m: float,
            budget_params: ToolBudgetParams,
            mark_final: bool = False,
            previous: Optional[RefitResult] = None,
            on_view: Optional[Callable] = None) -> Tuple[RefitResult, BagModel]:
        """
        Fuse bag landmarks; TSDF cylinder/sphere is visualization only.

        自节点 ``_run_refit`` 本体迁入（docstring 语义原样）：Contact
        authority is the fused bag model plus dynamic budget. TSDF cloud
        supplies envelope-axis consistency only; squat volumes skip the 12°
        veto. Fusion still runs from cloud_base when TSDF is
        enabled-but-empty (tsdf_xyz=None 即该路径). keep_last_good keeps
        the last bag model that already has a budget（调用方把 previous=
        上一帧 _refined 传入即启用；本方法只按 previous 打「保留」日志，
        缓存写入分支由节点按同一判据执行）. Finalize marks the result
        final.

        Args:
            tsdf_xyz: TSDF 提取云 (N,3) [m]；None/空=无 TSDF 云路径.
            frames: 已采帧快照（锁内 list(collector.frames)）.
            target_center: 绑定目标中心（collect_bag_views 机位聚类用）.
            kind: 'bag'/'fruit'（节点 _resolve_target_kind）.
            kind_defaulted: kind 为缺省值（追加 target_kind_defaulted 旗标）.
            bound_axis_hint: 绑定轴先验（base 系）或 None.
            target_id: 绑定目标 ID（model_revision 组成）.
            entry_standoff_m: 入口后撤量 [m]（yaml refit.entry_standoff_m）.
            pregrasp_standoff_m: 预抓取后撤量 [m].
            budget_params: ToolBudgetParams（tool_profile 注入 d_inner）.
            mark_final: finalize 定稿标记.
            previous: keep_last_good 时的上一帧 _refined（None=无保留对象）.
            on_view: 逐视角回调（geometry.jsonl 视角行追加）.

        Returns
        -------
            (result, fused)：节点按成对写入约定落缓存；result.ok and
            result.budget 为「更新」分支判据（与旧节点逐字一致）。

        """
        result: Optional[RefitResult] = None
        cloud_xyz = None
        if tsdf_xyz is not None and np.asarray(tsdf_xyz).size:
            xyz = np.asarray(tsdf_xyz)
            cloud_xyz = xyz
            self._logger.info(
                f'REFINING：几何二次拟合开始（kind={kind}，{xyz.shape[0]} 点）')
            t_refit0 = self._now()
            try:
                result = select_refitter(self._refitters, kind).refit(
                    xyz, kind, self._refit_config, bound_axis_hint)
                self._timing.record_refit(
                    (self._now() - t_refit0) * 1000.0)
            except Exception as exc:  # noqa: BLE001
                self._timing.record_refit(
                    (self._now() - t_refit0) * 1000.0)
                self._logger.warning(f'refit 异常: {exc}')
                result = RefitResult(
                    ok=False, reason=f'exception:{exc}', kind=kind,
                    n_points=int(xyz.shape[0]))
            if kind_defaulted and result is not None:
                result.flags.append('target_kind_defaulted')
        else:
            result = RefitResult(
                ok=False, reason='no_tsdf_cloud', kind=kind,
                flags=['no_tsdf_cloud'])
        if result is None:
            result = RefitResult(ok=False, reason='refit_missing', kind=kind)
        result.final = bool(mark_final)
        views = collect_bag_views(frames, target_center, on_view=on_view)
        fused = fuse_bag_views(
            views,
            cloud_xyz=cloud_xyz,
            detection_axis=bound_axis_hint,
            entry_standoff_m=float(entry_standoff_m),
            pregrasp_standoff_m=float(pregrasp_standoff_m),
            # 许可数学内径随当前工具档案（tool_profile launch 注入）
            params=budget_params)
        # G3：先取号再 merge——本轮 revision 尾段即该单调计数
        self._finalize_counter += 1
        result = merge_fused_bag_model(
            result, fused, len(views), bound_axis_hint, target_id,
            finalize_counter=self._finalize_counter)
        # 分支日志（缓存写入分支由节点 _run_refit 按同一判据执行）
        if result.ok and result.budget:
            status_text = (
                'ACCEPT' if result.status == STATUS_ACCEPT else 'REOBSERVE')
            self._logger.info(
                f'REFINING 完成：{result.kind} status={status_text} '
                f'final={result.final} '
                f'axis={np.round(result.axis, 4).tolist()} '
                f'diameter={float(result.diameter or 0) * 1000.0:.1f}mm')
        elif previous is not None and previous.ok and previous.budget:
            self._logger.warning(
                f'refit/融合未收敛，保留上一帧袋模型：{result.reason}')
        else:
            self._logger.warning(
                f'REFINING：无接触权威几何（{result.reason}）')
        return result, fused
