from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # Match the installed surface checked by the flake8 test.
    rc = main(argv=[
        'sim_perception/__init__.py', 'sim_perception/scene_node.py',
        'sim_perception/reconstruction_node.py', 'setup.py', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
