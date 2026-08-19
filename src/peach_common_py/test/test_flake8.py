from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest

# 显式传包内路径，避免 cwd 不是包根时扫到整个工作区
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_LINT_PATHS = [
    str(_PACKAGE_ROOT / 'peach_common_py'),
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
