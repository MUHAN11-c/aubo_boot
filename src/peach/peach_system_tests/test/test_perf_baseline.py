"""perf_baseline.json 结构与量纲守卫（W0）。

基线数字来自仓内实测（见文件 _provenance），是 W3-W5 性能优化前后对拍
的锚点。本测试只锁 schema 与量纲健全性，不锁具体数值——优化后应更新
基线并附新来源；回放塔的时延断言在 W3/W4 接线时引用本文件。
"""
import json
from pathlib import Path

BASELINE = Path(__file__).parent / 'perf_baseline.json'


def _load():
    data = json.loads(BASELINE.read_text(encoding='utf-8'))
    assert data['_provenance'], '基线必须附来源'
    return data


def test_schema_sections_present():
    data = _load()
    for key in (
            'perception_frame_ms', 'inference_ms', 'reconstruction_ms_peak',
            'delivery', 'tf_absent_perception_fps'):
        assert key in data, key


def test_ranges_well_formed():
    data = _load()
    for key, span in data['perception_frame_ms'].items():
        assert len(span) == 2 and 0.0 < span[0] <= span[1], key
    assert 0.0 < data['stereo_ab_perception_total_ms']
    assert 0.0 < data['tf_absent_perception_fps'] < 5.0
    lo, hi = data['delivery']['color_raw_reliable_hz']
    assert 0.0 < lo <= hi
    # 实测结论：RELIABLE raw 塌陷区间必须低于 compressed 稳态（PF-1 依据）
    assert hi < data['delivery']['color_compressed_hz']


def test_reconstruction_peak_consistency():
    peak = _load()['reconstruction_ms_peak']
    assert peak['frames_timed'] > 0
    # 峰值口径：frame_total 应覆盖 tsdf_integrate（同一帧链路）
    assert peak['frame_total_ema'] >= peak['tsdf_integrate_ema']
