"""safe_component 目录段净化（消息 id 拼路径的统一守卫）."""
from peach_common.paths import safe_component


def test_normal_ids_pass_through():
    assert safe_component('harvest_2026', 'x') == 'harvest_2026'
    assert safe_component('run:42', 'x') == 'run:42'


def test_traversal_and_separators_fall_back():
    for bad in ('..', '../etc', 'a/b', 'a\\b', '', '   ', '.', 'nul\x00'):
        assert safe_component(bad, 'harvest') == 'harvest', bad


def test_fallback_itself_invalid_degrades_to_unknown():
    assert safe_component('..', '../evil') == 'unknown'
    assert safe_component('..', '') == 'unknown'


def test_whitespace_only_stripped_then_fallback():
    assert safe_component('  \t ', 'run') == 'run'
