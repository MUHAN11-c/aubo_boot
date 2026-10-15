"""模型发布身份取真实场景世代，调度心跳不得回写旧世代."""
from pathlib import Path
import threading
from types import SimpleNamespace

from peach_harvester.vision.target_reconstruction.target_reconstruction_node import (
    TargetReconstructionNode,
)
from peach_interfaces.action import BuildTargetModel
from peach_interfaces.msg import HarvestState
import pytest


class _BindingComplete(Exception):
    """在采集启动边界结束无ROS夹具."""


def test_build_binds_goal_epoch_before_any_collection_or_model_publication():
    """Build目标的世代必须在自动采集/发布前成为模型身份."""
    node = SimpleNamespace(
        _scene_epoch=0, _state_lock=threading.RLock(),
        _on_reset=lambda *_args: None,
    )

    def start():
        assert node._scene_epoch == 7
        raise _BindingComplete()

    node._auto_start = start
    handle = SimpleNamespace(request=BuildTargetModel.Goal(
        target_id='t1', scene_epoch=7))
    with pytest.raises(_BindingComplete):
        TargetReconstructionNode._build_target_model_body(node, handle)


def test_executor_heartbeat_does_not_overwrite_canonical_scene_epoch():
    """晚到的调度状态不能将已绑定的感知/Build世代回写成零."""
    node = SimpleNamespace(
        _scene_epoch=7, _state_lock=threading.RLock(),
        _harvest_data=SimpleNamespace(base_dir=Path('/tmp')),
        _recorder=SimpleNamespace(bind_executor_run_id=lambda *_args: None),
    )
    TargetReconstructionNode._on_executor_state(
        node, HarvestState(target_id='t1', run_id='run1', scene_epoch=0))
    assert node._scene_epoch == 7


@pytest.mark.parametrize('old_run, old_epoch, expected', [
    ('run1', 0, 7), ('run1', 9, 9), ('old_run', 9, 7),
])
def test_observation_epoch_is_canonical_and_same_run_never_regresses(
        old_run, old_epoch, expected):
    """新批次接纳真实世代，同批次旧帧不得让场景身份倒退."""
    from peach_interfaces.msg import PeachTargetObservationArray

    node = SimpleNamespace(
        _scene_epoch=old_epoch, _harvest_run_id=old_run,
        _state_lock=threading.RLock(), _executor_state_seen=False,
        _preferred_target_id='', collector=SimpleNamespace(state='IDLE'),
        _harvest_data=SimpleNamespace(attach=lambda *_args: None),
        _reset_capture_session=lambda: None, _target_masks={},
    )

    def arbitrate(*_args, **_kwargs):
        assert node._scene_epoch == expected
        assert node._harvest_run_id == 'run1'
        raise _BindingComplete()

    node._switch_holdoff = SimpleNamespace(arbitrate=arbitrate)
    with pytest.raises(_BindingComplete):
        TargetReconstructionNode._on_target_observations(
            node, PeachTargetObservationArray(harvest_run_id='run1', scene_epoch=7))


@pytest.mark.parametrize('new_run, new_epoch', [('new_run', 1), ('old_run', 4)])
def test_scene_identity_change_revokes_same_target_old_model(new_run, new_epoch):
    """同目标ID在换批次/换世代后不能把旧READY模型重贴新身份."""
    from peach_interfaces.msg import PeachTargetObservationArray
    from peach_harvester.vision.target_reconstruction.refine import RefitResult

    node = TargetReconstructionNode.__new__(TargetReconstructionNode)
    node._state_lock = threading.RLock()
    node._harvest_run_id, node._scene_epoch = 'old_run', 3
    node._executor_state_seen, node._executor_target_id = True, 't1'
    node._preferred_target_id = 't1'
    node.collector = SimpleNamespace(state='READY', target_id='t1', frames=[object()])

    def reset():
        node.collector.state = 'IDLE'
        node.collector.target_id = ''
        node.collector.frames.clear()

    node.collector.reset = reset
    node._refined = RefitResult(ok=True, model_revision='old_revision')
    node._bag_model = object()
    node._view_progress = threading.Event()
    node._target_kind_memory = SimpleNamespace(reset=lambda: None)
    node._clear_frame_ring = lambda: None
    node._icp_target_cache = SimpleNamespace(invalidate=lambda: None)
    node._bump_products_version = lambda **_kwargs: None
    node._target_masks = {'old': object()}
    node._harvest_data = SimpleNamespace(attach=lambda *_args: None,
                                         append_event=lambda *_args: None)
    node._switch_holdoff = SimpleNamespace(arbitrate=lambda *_args, **_kwargs: 'FOLLOW')
    node.params = SimpleNamespace(frames=SimpleNamespace(base_frame='base_link'))
    published = []
    node._publish_all = lambda: published.append(node._grasp_decision())
    message = PeachTargetObservationArray(harvest_run_id=new_run, scene_epoch=new_epoch)
    message.header.frame_id = 'other_frame'
    node._on_target_observations(message)
    assert node._refined is None
    assert node._bag_model is None
    assert node.collector.frames == []
    assert published and published[-1]['allowed'] is False
    assert not published[-1].get('model_revision')
