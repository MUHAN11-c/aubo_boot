from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # 范围 = 运行时代码（sim_worlds 模块 + test + setup.py）。包根的
    # reconstruction/ 与 blender/ 是离线资产重建管线（不入 colcon 测试、
    # 不随包安装），不进 lint 门；手动检查：python3 -m flake8 reconstruction/
    rc, errors = main_with_errors(argv=['sim_worlds', 'test', 'setup.py'])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
