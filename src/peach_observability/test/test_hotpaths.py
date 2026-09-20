"""
W10 热路径与生命周期修复的单测（零 ROS）.

覆盖：作业票按快照代数记忆化、bag 写队列有界丢弃（含 task_done 销账）、
CatchAllRecorder.stop 真正销毁订阅、调度状态转换器下沉。
"""
from __future__ import annotations

import time
from types import SimpleNamespace

from peach_observability.catch_all_recorder import CatchAllRecorder
from peach_observability.recorder import Recorder
import peach_observability.state as state_mod
from peach_observability.state import ObservabilityState, to_task_executor_state


def test_job_memoized_by_revision(monkeypatch):
    """同一快照代数多次 snapshot 只折叠一次作业票；update 后才重算."""
    calls = []
    real = state_mod.build_harvest_job

    def counting(snapshot):
        calls.append(snapshot.get('system', {}).get('revision'))
        return real(snapshot)

    monkeypatch.setattr(state_mod, 'build_harvest_job', counting)
    state = ObservabilityState()
    first = state.snapshot()
    second = state.snapshot()
    assert len(calls) == 1
    assert first['job'] is second['job']
    state.update('perception', 'harvest', {'x': 1})
    third = state.snapshot()
    assert len(calls) == 2
    # 窄访问器命中同一份缓存，不触发重算
    assert state.job() is third['job']
    assert len(calls) == 2
    state.update('manipulation', 'status', {'state': 'IDLE'})
    assert state.job() is not third['job']
    assert len(calls) == 3


def test_recorder_bounded_queue_drops_oldest(tmp_path):
    """写队列满时丢最旧保最新并计数；被丢条目销账后 close 不悬挂."""
    recorder = Recorder(root_dir=str(tmp_path), enabled=False, queue_depth=4)
    try:
        for index in range(10):
            recorder._enqueue(('msg', f'/t{index}', index, index))
        assert recorder._queue.qsize() == 4
        assert recorder._drops == 6
        kept = []
        for _ in range(4):
            kept.append(recorder._queue.get_nowait())
            recorder._queue.task_done()
        assert [item[1] for item in kept] == ['/t6', '/t7', '/t8', '/t9']
        info = recorder.info()
        assert info['drops'] == 6
        assert info['queue_depth'] == 4
        assert info['queue_size'] == 0
    finally:
        recorder.close()  # enabled=False：跳过 join 路径，只收线程


class _StubNode:
    """CatchAllRecorder 依赖的最小节点面（create/destroy 记账）."""

    def __init__(self):
        self.destroyed_subs = []
        self.destroyed_timers = []

    def create_timer(self, period, callback):
        del period, callback
        return object()

    def destroy_timer(self, timer):
        self.destroyed_timers.append(timer)

    def destroy_subscription(self, sub):
        self.destroyed_subs.append(sub)


def test_catch_all_stop_destroys_subscriptions():
    """停止发现并销毁全部通配订阅/timer，清发现态且幂等."""
    node = _StubNode()
    recorder = CatchAllRecorder(node, recorder=None)
    timer = object()
    subs = [object(), object()]
    recorder._timer = timer
    recorder._subs = list(subs)
    recorder._recorded = {'/a'}
    recorder.stop()
    assert node.destroyed_timers == [timer]
    assert set(node.destroyed_subs) == set(subs)
    assert recorder._subs == []
    assert recorder._recorded == set()
    assert recorder._timer is None
    recorder.stop()  # 幂等
    assert len(node.destroyed_subs) == 2


def test_to_task_executor_state_converter():
    """调度类型化状态 → 浏览器镜像 dict（缺 state_seq 属性走默认 0）."""
    message = SimpleNamespace(
        revision=7,
        run_id='rid',
        cycle_id='cid',
        target_id='tid',
        operation_mode='AUTO',
        batch_state=2,
        target_phase=5,
        action_active=True,
        auto_start_enabled=False,
        execution_enabled=True,
        grasp_enabled=True,
        tool_enabled=False,
        recovery_required=False,
        progress=0.5,
        message='ok',
        blockers=['b1'],
        scene_epoch=3,
    )
    value = to_task_executor_state(message)
    assert value['revision'] == 7
    assert value['state_seq'] == 0  # 未带 state_seq → getattr 默认
    assert value['scene_epoch'] == 3
    assert value['blockers'] == ['b1']
    value['blockers'].append('b2')
    assert list(message.blockers) == ['b1']  # 拷贝不回写


def test_topic_ages_reports_update_recency():
    """topic_ages 给诊断用的镜像键年龄；未更新的键不在表内."""
    state = ObservabilityState()
    assert state.topic_ages() == {}
    state.update('perception', 'harvest', {'x': 1})
    ages = state.topic_ages()
    assert set(ages) == {'perception.harvest'}
    assert 0.0 <= ages['perception.harvest'] < 1.0
    # 显式 now 注入：年龄按给定时刻计算
    later = state.topic_ages(now=time.time() + 100.0)
    assert 100.0 <= later['perception.harvest'] < 101.0
