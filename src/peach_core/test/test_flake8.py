from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest

# 显式传包内路径：colcon test 的 cwd 是包根（默认 '.' 即可），但按
# AGENTS.md 第 5 节从工作区根裸跑 pytest 时 cwd 是整个工作区，默认 '.'
# 会扫描全树（含 build/ 与 venv），既慢又会误报他包问题
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_LINT_PATHS = [
    str(_PACKAGE_ROOT / 'peach_core'),
    str(_PACKAGE_ROOT / 'test'),
    str(_PACKAGE_ROOT / 'setup.py'),
]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=_LINT_PATHS)
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
