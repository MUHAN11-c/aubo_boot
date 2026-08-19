from pathlib import Path

from ament_pep257.main import main
import pytest

# 显式传包内路径（cwd 未必是包根）
_PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[
        str(_PACKAGE_ROOT / 'peach_common_py'),
        str(_PACKAGE_ROOT / 'test'),
        str(_PACKAGE_ROOT / 'setup.py'),
    ])
    assert rc == 0, 'Found code style errors / warnings'
