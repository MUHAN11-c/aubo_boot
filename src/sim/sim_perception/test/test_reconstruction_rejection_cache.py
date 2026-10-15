"""新模型明确错轴时，不得继续发布上一次模型的接触许可."""
from types import SimpleNamespace

import numpy as np
import pytest
from sim_perception.vision.common.tool_budget import ToolBudgetParams
from sim_perception.vision.target_reconstruction.refine import (
    BagModel, RefitResult, STATUS_REOBSERVE,
)
from sim_perception.vision.target_reconstruction.target_reconstruction_node import (
    TargetReconstructionNode,
)


@pytest.mark.parametrize('reason,retain', [
    ('fused_axis_conflict', False), ('no_landmark_views', True),
])
def test_hard_axis_rejection_invalidates_pair_but_transient_failure_retains(reason, retain):
    node = TargetReconstructionNode.__new__(TargetReconstructionNode)
    previous = RefitResult(ok=True, budget={'allowed': True}, model_revision='t:1:1')
    previous_bag = BagModel(ok=True, allowed=True)
    rejected = RefitResult(ok=False, reason=reason, status=STATUS_REOBSERVE,
                           flags=[reason], model_revision='t:2:2')
    rejected_bag = BagModel(ok=False, reason=reason)
    node._refined, node._bag_model = previous, previous_bag
    node._resolve_target_kind = lambda: ('bag', False)
    node._tsdf_cloud_cache = None
    node.collector = SimpleNamespace(frames=[], target_center=np.zeros(3), target_id='t')
    node._bound_axis_hint = None
    node.params = SimpleNamespace(
        refit=SimpleNamespace(entry_standoff_m=0.0, pregrasp_standoff_m=0.0),
        tool=SimpleNamespace(budget=ToolBudgetParams()))
    node._refit_orchestrator = SimpleNamespace(run=lambda **kwargs: (rejected, rejected_bag))
    node._on_view_geometry = lambda *args: None
    versions = []
    node._bump_products_version = lambda: versions.append(1)
    node._run_refit(keep_last_good=True)
    if retain:
        assert node._refined is previous
        assert node._bag_model is previous_bag
        assert not versions
    else:
        assert node._refined is rejected
        assert node._bag_model is rejected_bag
        assert node._refined.budget == {}
        assert versions == [1]


@pytest.mark.parametrize('state', ['READY', 'COLLECTING'])
def test_rejected_axis_reason_reaches_the_public_decision(state):
    node = SimpleNamespace(
        _harvest_run_id='run', collector=SimpleNamespace(state=state, target_id='t'),
        _refined=RefitResult(ok=False, reason='fused_axis_conflict'))
    decision = TargetReconstructionNode._grasp_decision(node)
    assert decision['reason'] == 'fused_axis_conflict'
    assert decision['allowed'] is False
    assert decision['geometry_valid'] is False
