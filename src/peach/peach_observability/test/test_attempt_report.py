"""离线记录解析：只读选定目录，拒绝跑出 runs."""
from datetime import datetime
import json
import os
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


def test_success_counts_each_grasp_once(tmp_path: Path):
    from peach_observability.record_analysis import success_summary
    session = tmp_path / 'session_20261022_150000'
    (session / 'cycles').mkdir(parents=True)
    (session / 'cycles' / 'one.json').write_text(json.dumps({
        'request_id': 'web_one',
        'target_id': 'target_0',
        'outcome': 0,
        'completion_level': 6,
        'elapsed_s': 20,
        'start_unix': 1_792_653_000.0,
    }), encoding='utf-8')
    batch = tmp_path / 'web_one'
    batch.mkdir()
    (batch / 'ledger.json').write_text(json.dumps({'outcomes': [{
        'target_id': 'target_0', 'outcome': 0, 'completion_level': 6,
        'elapsed_s': 20,
    }]}), encoding='utf-8')
    other = tmp_path / 'web_fail'
    other.mkdir()
    (other / 'ledger.json').write_text(json.dumps({'outcomes': [{
        'target_id': 'target_1', 'outcome': 3, 'completion_level': 2,
        'elapsed_s': 4,
    }]}), encoding='utf-8')
    report = success_summary(tmp_path)
    assert report['all']['attempts'] == 2
    assert report['all']['sleeve_ok'] == 1
    assert report['all']['sleeve_rate'] == 0.5
    assert report['all']['harvest_ok'] == 0


def test_reject_path_outside_runs(tmp_path: Path):
    with pytest.raises(ValueError):
        analyze_record(tmp_path, '../outside')


def test_force_episode_sits_on_its_grasp(tmp_path: Path):
    session = tmp_path / 'session_20261022_150712'
    (session / 'force').mkdir(parents=True)
    batch = tmp_path / 'web_one'
    batch.mkdir()
    elapsed = 22.0
    ledger = batch / 'ledger.json'
    ledger.write_text(
        '{"outcomes":[{"target_id":"target_0","outcome":0,'
        '"elapsed_s":22,"completion_level":6,'
        '"stage_names":["finalize","reconfirm","approach_insert","tool","retreat"],'
        '"stage_durations":[0.01,0.93,11.0,0.0,10.06]}]}',
        encoding='utf-8')
    end = 1_792_653_000.0
    os.utime(ledger, (end, end))
    wall = datetime.fromtimestamp(end - elapsed + 10).replace(microsecond=0)
    (session / 'force' / 'episode_001.json').write_text(json.dumps({
        'reason': 'returned',
        'wall_start': wall.isoformat(timespec='seconds'),
        'duration_s': 4.2,
        'contact_s': 3.8,
        'peak_n': [0.1, 0.2, 0.3, 0.4, 2.1],
    }), encoding='utf-8')
    (session / 'force' / 'episode_002.json').write_text(json.dumps({
        'reason': 'returned',
        'wall_start': datetime.fromtimestamp(end + 5000).isoformat(timespec='seconds'),
        'duration_s': 2,
        'contact_s': 1,
        'peak_n': [1, 0, 0, 0, 0],
    }), encoding='utf-8')
    report = analyze_record(tmp_path, 'session_20261022_150712')
    assert len(report['force_cycles']) == 1
    cycle = report['force_cycles'][0]
    assert cycle['target_id'] == 'target_0'
    assert len(cycle['force']) == 1
    force = cycle['force'][0]
    assert force['start_s'] == pytest.approx(wall.timestamp() - (end - elapsed), abs=0.02)
    assert force['stages'] == ['approach_insert', 'retreat']
    assert force['peak_channel'] == 'f7'
    own = analyze_record(tmp_path, 'web_one')
    assert own['force_cycles'][0]['force'][0]['episode'] == 'episode_001'


def test_rise_and_fall_follow_samples(tmp_path: Path):
    session = tmp_path / 'session_sample'
    (session / 'force').mkdir(parents=True)
    (session / 'startup.json').write_text('{}', encoding='utf-8')
    start = 1_792_654_000.0
    elapsed = 22.0
    (session / 'cycles').mkdir()
    (session / 'cycles' / 'one.json').write_text(json.dumps({
        'request_id': 'web_one',
        'target_id': 'target_0',
        'outcome': 0,
        'elapsed_s': elapsed,
        'completion_level': 6,
        'stage_names': ['approach_insert', 'retreat'],
        'stage_durations': [11.919, 10.081],
        'start_unix': start,
        'end_unix': start + elapsed,
    }), encoding='utf-8')
    # 整秒 wall_start 是 10，写完时间对应真正起点 10.4；归零尾在 3.6 s。
    wall = datetime.fromtimestamp(start + 10).replace(microsecond=0)
    csv_path = session / 'force' / 'episode_001.csv'
    csv_path.write_text(
        't_s,f1,f2,f3,f5,f7\n'
        '0,0,0,0.2,0,0\n'
        '2.405,0,0,5,0,0\n'
        '3.900,0,0,0.1,0,0\n'
        '4.260,0,0,0,0,0\n',
        encoding='utf-8')
    os.utime(csv_path, (start + 10.4 + 4.260, start + 10.4 + 4.260))
    (session / 'force' / 'episode_001.json').write_text(json.dumps({
        'wall_start': wall.isoformat(timespec='seconds'),
        'duration_s': 4.26,
        'contact_s': 4.26,
        'peak_n': [0, 0, 5, 0, 0],
        'reason': 'returned',
    }), encoding='utf-8')
    report = analyze_record(tmp_path, 'session_sample')
    force = report['force_cycles'][0]['force'][0]
    assert force['start_s'] == pytest.approx(10.4, abs=0.002)
    assert force['parts'] == [
        {'kind': 'rise', 'start_s': 10.4, 'end_s': 12.805},
        {'kind': 'fall', 'start_s': 12.805, 'end_s': 14.3},
    ]
    assert force['stages'] == ['approach_insert', 'retreat']


def test_archived_cycle_splits_rise_and_fall(tmp_path: Path):
    session = tmp_path / 'session_20261022_160000'
    (session / 'force').mkdir(parents=True)
    (session / 'startup.json').write_text('{}', encoding='utf-8')
    start = 1_792_654_000.0
    elapsed = 20.0
    (session / 'cycles').mkdir()
    (session / 'cycles' / '20261022_160020_target_0.json').write_text(
        json.dumps({
            'request_id': 'grasp_20261022_160000_001',
            'target_id': 'target_0',
            'outcome': 0,
            'elapsed_s': elapsed,
            'completion_level': 6,
            'stage_names': ['approach_insert', 'retreat'],
            'stage_durations': [12.0, 8.0],
            'start_unix': start,
            'end_unix': start + elapsed,
        }),
        encoding='utf-8')
    batch = tmp_path / 'grasp_20261022_160000_001'
    batch.mkdir()
    ledger = batch / 'ledger.json'
    ledger.write_text('{"outcomes":[]}', encoding='utf-8')
    os.utime(ledger, (start + elapsed, start + elapsed))
    wall = datetime.fromtimestamp(start + 10).replace(microsecond=0)
    (session / 'force' / 'episode_001.json').write_text(json.dumps({
        'wall_start': wall.isoformat(timespec='seconds'),
        'duration_s': 4,
        'contact_s': 3,
        'peak_n': [0, 0, 1, 0, 0],
    }), encoding='utf-8')
    (session / 'force' / 'episode_001.csv').write_text(
        't_s,f1,f2,f3,f5,f7\n'
        '0,0,0,0.2,0,0\n'
        '1.5,0,0,2.0,0,0\n'
        '3.5,0,0,0.1,0,0\n',
        encoding='utf-8')
    report = analyze_record(tmp_path, 'session_20261022_160000')
    assert len(report['force_cycles']) == 1
    force = report['force_cycles'][0]['force'][0]
    assert [part['kind'] for part in force['parts']] == ['rise', 'fall']
    assert force['parts'][0]['end_s'] == force['parts'][1]['start_s']
    assert force['parts'][0]['start_s'] == pytest.approx(
        wall.timestamp() - start, abs=0.02)
    assert 'approach_insert' in force['stages']
    names = {item['path'] for item in list_records(tmp_path)}
    assert 'session_20261022_160000' in names
    assert 'grasp_20261022_160000_001' not in names
