"""离线记录解析：只读选定目录，拒绝跑出 runs."""
from pathlib import Path

from peach_observability.record_analysis import (
    analyze_record,
    list_records,
    read_record_image,
)
import pytest


def test_analyze_ledger_geometry_and_image(tmp_path: Path):
    record = tmp_path / 'round_a'
    record.mkdir()
    (record / 'ledger.json').write_text(
        '{"outcomes":[{"target_id":"target_1","outcome":1,"reason":"质量跳过",'
        '"elapsed_s":1.2,"completion_level":0}]}',
        encoding='utf-8')
    (record / 'geometry.jsonl').write_text(
        '{"target_id":"target_1","fused":true,"allowed":false,'
        '"reason":"bag_mouth_geometry","d95_m":0.064,'
        '"radial_margin_m":-0.019,"bag_bottom":[0.1,0.2,0.3],'
        '"bag_neck":[0.1,0.2,0.4],"cut_pose":[0.2,0.2,0.4]}\n',
        encoding='utf-8')
    (record / 'view.jpg').write_bytes(b'\xff\xd8\xff')
    report = analyze_record(tmp_path, 'round_a')
    assert report['outcomes'][0]['reason'] == '质量跳过'
    assert report['geometry']['d95_mm']['mean'] == 64.0
    assert report['geometry']['targets'][0]['points'][0]['name'] == '袋底'
    assert report['images'][0]['name'] == 'view.jpg'
    assert read_record_image(tmp_path, 'round_a', 'view.jpg').startswith(b'\xff\xd8')
    assert 'round_a' in {item['path'] for item in list_records(tmp_path)}


def test_reject_path_outside_runs(tmp_path: Path):
    with pytest.raises(ValueError):
        analyze_record(tmp_path, '../outside')
