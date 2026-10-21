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
        '{"outcomes":['
        '{"target_id":"target_1","outcome":1,"reason":"质量跳过",'
        '"elapsed_s":1.2,"completion_level":0},'
        '{"target_id":"target_2","outcome":0,"reason":"","elapsed_s":2,'
        '"completion_level":6,"stage_names":["reconfirm","retreat"],'
        '"stage_durations":[0.4,1.1]}]}',
        encoding='utf-8')
    (record / 'geometry.jsonl').write_text(
        '{"target_id":"target_1","fused":true,"allowed":false,'
        '"reason":"bag_mouth_geometry","d95_m":0.064,'
        '"radial_margin_m":-0.019,"bag_bottom":[0.1,0.2,0.3],'
        '"bag_neck":[0.1,0.2,0.4],"cut_pose":[0.2,0.2,0.4]}\n',
        encoding='utf-8')
    (record / 'view.jpg').write_bytes(b'\xff\xd8\xff')
    (record / 'colors').mkdir()
    (record / 'masks').mkdir()
    (record / 'colors' / '100_target_1.png').write_bytes(b'\x89PNG\r\n')
    (record / 'masks' / '100_target_1.png').write_bytes(b'\x89PNG\r\n')
    report = analyze_record(tmp_path, 'round_a')
    assert report['outcomes'][0]['reason'] == '质量跳过'
    assert report['rates']['fruits'] == 2
    assert report['rates']['quality_skipped'] == 1
    assert report['rates']['motion_completed'] == 1
    assert report['rates']['harvest_confirmed'] == 0
    assert 'cut_confirmed' not in report['rates']
    assert [item['target_id'] for item in report['fruits']] == ['target_1', 'target_2']
    assert [item['batch'] for item in report['fruits']] == ['round_a', 'round_a']
    assert 'color' not in report['fruits'][0]
    assert report['fruits'][0]['mask']['name'] == 'masks/100_target_1.png'
    assert report['fruits'][0]['center_m'] == [0.1, 0.2]
    assert report['fruits'][0]['d95_mm'] == 64.0
    assert report['fruits'][0]['radial_margin_mm'] == -19.0
    assert [point['name'] for point in report['fruits'][0]['points']] == [
        '袋底', '袋口', '剪切参考']
    assert report['fruits'][1]['points'] == []
    assert report['fruits'][1]['mask'] is None
    assert report['fruits'][0]['stages'] == [{'name': 'finalize', 's': 1.2}]
    assert report['fruits'][1]['stages'][0] == {'name': 'finalize', 's': 0.5}
    assert report['fruits'][1]['stages'][1] == {'name': 'reconfirm', 's': 0.4}
    assert report['geometry']['d95_mm']['mean'] == 64.0
    assert report['geometry']['targets'][0]['points'][0]['name'] == '袋底'
    assert read_record_image(tmp_path, 'round_a', 'view.jpg') is None
    assert read_record_image(tmp_path, 'round_a', 'colors/100_target_1.png') is None
    assert read_record_image(
        tmp_path, 'round_a', 'masks/100_target_1.png').startswith(b'\x89PNG')
    assert 'round_a' in {item['path'] for item in list_records(tmp_path)}


def test_harvest_confirmed_is_level_7(tmp_path: Path):
    record = tmp_path / 'levels'
    record.mkdir()
    (record / 'ledger.json').write_text(
        '{"outcomes":['
        '{"target_id":"retreat","outcome":0,"elapsed_s":1,"completion_level":6},'
        '{"target_id":"cut","outcome":0,"elapsed_s":1,"completion_level":5},'
        '{"target_id":"harvest","outcome":0,"elapsed_s":1,"completion_level":7}]}',
        encoding='utf-8')
    report = analyze_record(tmp_path, 'levels')
    assert report['rates']['harvest_confirmed'] == 1
    assert report['rates']['motion_completed'] == 2


def test_reject_path_outside_runs(tmp_path: Path):
    with pytest.raises(ValueError):
        analyze_record(tmp_path, '../outside')
