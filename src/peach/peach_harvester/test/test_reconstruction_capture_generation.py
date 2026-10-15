"""锁外TF/构云准备不能把旧会话帧写进同ID的新采集会话."""
import threading
from types import SimpleNamespace

from peach_harvester.vision.target_reconstruction.capture import GATE_ALLOW
from peach_harvester.vision.target_reconstruction.reconstruction_core import ReconstructionCore
import pytest


@pytest.mark.parametrize('reset_during_prepare', [False, True])
def test_prepared_frame_must_keep_its_capture_generation(reset_during_prepare):
    """模拟准备期间reset后同ID重新COLLECTING，旧payload必须丢弃."""
    committed = []
    host = SimpleNamespace(
        _state_lock=threading.RLock(), _capture_generation=1,
        _last_captured_stamp_sec=-1., _algo_clock=SimpleNamespace(now=lambda: 1.),
        collector=SimpleNamespace(state='COLLECTING', target_id='t1',
                                  auto_capture_decision=lambda *_args: ('capture', '')),
        _logger=SimpleNamespace(debug=lambda *_args: None),
        _publish_all=lambda: None,
    )

    def prepare(*_args):
        if reset_during_prepare:
            host._capture_generation += 1
        return 'prepared', ''

    host._prepare_frame = prepare
    host._commit_prepared_frame = lambda *_args: (committed.append('frame') or True, '')
    context = (None, None, None, 1., None, None, None)
    ReconstructionCore._auto_capture_commit(
        host, SimpleNamespace(action=GATE_ALLOW), context, 't1')
    assert committed == ([] if reset_during_prepare else ['frame'])


def test_tf_query_cannot_cross_capture_generation():
    """TF查询期间会话改变时，不再用原始图像上下文执行finish/commit."""
    finished = []
    host = SimpleNamespace(
        _state_lock=threading.RLock(), _capture_generation=1,
        collector=SimpleNamespace(state='COLLECTING', target_id='t1', frames=[],
                                  should_auto_finalize=lambda: False),
        params=SimpleNamespace(capture=SimpleNamespace(max_views=10)),
        _gated_capture_begin=lambda **_kwargs: (None, 'tf_request'),
    )

    def query(_request):
        host._capture_generation += 1
        return 'old_tf'

    host._gated_capture_query_tf = query
    host._gated_capture_finish = lambda **_kwargs: (finished.append('finish'), None)
    host._auto_capture_commit = lambda *_args: None
    ReconstructionCore._auto_drive(host)
    assert finished == []


def test_inactive_lifecycle_cannot_start_another_capture_after_stop():
    """停止后已排队的帧不能取得新token绕过在途帧失效门."""
    host = SimpleNamespace(_state_lock=threading.RLock(), _lifecycle_active=False)
    ReconstructionCore._auto_drive(host)
