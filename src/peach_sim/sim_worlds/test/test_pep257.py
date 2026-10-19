from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # 同 test_flake8：只查运行时代码，reconstruction/blender 离线管线不在门内
    rc = main(argv=['sim_worlds', 'test', 'setup.py'])
    assert rc == 0, 'Found code style errors / warnings'
