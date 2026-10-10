"""baseline_inventory.json schema 守卫：只锁结构不锁数值（批次0）。"""
import json
from pathlib import Path

INVENTORY = Path(__file__).resolve().parent / 'baseline_inventory.json'


def test_inventory_schema():
    data = json.loads(INVENTORY.read_text(encoding='utf-8'))
    assert data['schema_version'] == 1
    assert set(data['counts']) == {'parameters', 'interfaces', 'functions'}
    assert all(isinstance(v, int) and v > 0 for v in data['counts'].values())
    assert data['counts']['parameters'] == len(data['parameters'])
    assert data['counts']['interfaces'] == len(data['interfaces'])
    assert data['counts']['functions'] == len(data['functions'])
    for p in data['parameters'][:20]:
        assert set(p) == {'file', 'key', 'value'}, p
        assert p['file'].startswith('src/')
    for f in data['functions'][:20]:
        assert set(f) == {'file', 'line', 'name'}, f
        assert f['line'] > 0
    # 感知/臂关键符号必须在索引里（防误删模块导致静默缩表）；
    # C++ 为全限定名，按后缀匹配。
    names = {f['name'] for f in data['functions']}
    for must in ('evaluate_sleeve_cut', 'react',
                 'executeCycle', 'tryStagingTransit', 'confirmFeedback'):
        hit = any(n == must or n.endswith('::' + must) for n in names)
        assert hit, f'函数索引缺 {must}'


def test_interfaces_contain_core_contracts():
    data = json.loads(INVENTORY.read_text(encoding='utf-8'))
    names = set(data['interfaces'])
    for must in ('/peach/perception/target_observations',
                 '/peach_arm/execute_target', '/peach_supervisor/run_harvest',
                 '/peach/reconstruction/grasp_decision'):
        assert must in names, f'接口清单缺 {must}'
