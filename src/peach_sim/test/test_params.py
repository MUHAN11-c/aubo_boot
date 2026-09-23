"""场景参数 schema 与跨键约束测试（零 ROS）."""

from copy import deepcopy
from pathlib import Path
import re

from peach_sim.params import check_params, params_from_dict
import yaml

PKG_DIR = Path(__file__).resolve().parents[1]
CONFIG = PKG_DIR / 'config' / 'orchard.yaml'
XACRO = PKG_DIR / 'urdf' / 'harvester_robot.urdf.xacro'


def _raw() -> dict:
    """原样加载部署 yaml."""
    return yaml.safe_load(CONFIG.read_text(encoding='utf-8'))


def _params(raw: dict):
    """构造参数并返回 (params, 全部问题)."""
    params, problems = params_from_dict(raw)
    return params, problems


def _tool_profile_path() -> Path | None:
    """定位 aubo_description 工具档案（安装树外的源码树布局）."""
    for parent in PKG_DIR.parents:
        candidate = parent / 'src' / 'aubo_description' / 'config'
        for path in sorted(candidate.glob('*.yaml')) if candidate.is_dir() else []:
            if 'adaptive' in path.name:
                return path
    return None


def test_default_config_is_valid():
    """默认 config/orchard.yaml 逐键合法且跨键约束全过."""
    raw = _raw()
    params, problems = _params(raw)
    assert problems == []
    assert check_params(params) == []


def test_unknown_key_is_rejected():
    """未知键必须报错，不能静默吞掉."""
    raw = _raw()
    raw['bag']['body_diametr'] = 0.09
    _params0, problems = _params(raw)
    assert any('body_diametr' in problem for problem in problems)


def test_range_must_be_ordered():
    """区间上下限倒置要报错."""
    raw = _raw()
    raw['bag']['bottom_to_neck'] = [0.12, 0.09]
    _params0, problems = _params(raw)
    assert any('bottom_to_neck' in problem for problem in problems)


def test_bag_must_pass_tool_bore():
    """袋体 + 双侧余量不得超出工具内径."""
    raw = _raw()
    raw['bag']['body_diameter'] = [0.115, 0.115]
    params, problems = _params(raw)
    assert problems == []
    assert any('无法通过工具' in problem for problem in check_params(params))


def test_bag_length_within_insertion():
    """袋底→袋颈不得超出最大插入深度."""
    raw = _raw()
    raw['bag']['bottom_to_neck'] = [0.09, 0.25]
    params, problems = _params(raw)
    assert problems == []
    assert any('插不进筒底' in problem for problem in check_params(params))


def test_row_spacing_blocks_canopy_overlap():
    """行距过窄（相邻行冠层穿模）要报错."""
    raw = _raw()
    raw['rows']['spacing'] = 1.0
    params, problems = _params(raw)
    assert problems == []
    assert any('冠层穿模' in problem for problem in check_params(params))


def test_platform_row_gap_clearance():
    """车体到树行的横向距离要留安全间隙."""
    raw = _raw()
    raw['platform']['row_gap'] = 0.3
    params, problems = _params(raw)
    assert problems == []
    assert any('安全间隙' in problem for problem in check_params(params))


def test_mount_height_above_deck():
    """臂座必须高于车体顶面（立柱座有空间）."""
    raw = _raw()
    raw['platform']['arm_mount_height'] = 0.2
    params, problems = _params(raw)
    assert problems == []
    assert any('臂座' in problem for problem in check_params(params))


def test_xacro_defaults_match_config():
    """装配 xacro 的默认几何必须与 config/orchard.yaml 的 platform.* 一致."""
    raw = _raw()
    platform = raw['platform']
    expected = {
        'arm_mount_height': platform['arm_mount_height'],
        'arm_mount_yaw_deg': platform['arm_mount_yaw_deg'],
        'body_x': platform['body_size'][0],
        'body_y': platform['body_size'][1],
        'body_z': platform['body_size'][2],
        'body_bottom': platform['body_bottom'],
        'track_length': platform['track_length'],
        'track_width': platform['track_width'],
        'track_height': platform['track_height'],
        'track_separation': platform['track_separation'],
    }
    text = XACRO.read_text(encoding='utf-8')
    defaults = dict(re.findall(
        r'<xacro:arg name="(\w+)" default="([^"]+)"/>', text))
    for key, value in expected.items():
        assert key in defaults, f'xacro 缺 arg {key}'
        assert float(defaults[key]) == float(value), (
            f'xacro {key} 默认 {defaults[key]} != orchard.yaml {value}')


def test_tool_mirror_matches_description():
    """工具镜像必须与 aubo_description 的 config/<profile>.yaml 一致."""
    profile = _tool_profile_path()
    if profile is None:
        return
    tool = yaml.safe_load(profile.read_text(encoding='utf-8'))
    raw = _raw()['tool']
    geometry = tool['geometry_m']
    assert raw['profile_id'] == tool['profile_id']
    assert raw['d_inner'] == geometry['D_inner']
    assert raw['l_insert'] == geometry['L_insert']
    assert raw['clearance_min'] == geometry['clearance_min']


def test_copy_isolated_from_source():
    """深拷贝后改副本不影响原参数（不可变容器语义）."""
    raw = _raw()
    clone = deepcopy(raw)
    clone['bag']['body_diameter'] = [0.0, 0.0]
    assert raw['bag']['body_diameter'] != [0.0, 0.0]
