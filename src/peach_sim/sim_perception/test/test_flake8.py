from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # Only the installed adapter is maintained here; vision/ is an uninstalled
    # historical snapshot. Its runtime replacement is linted in the Peach packages.
    rc, errors = main_with_errors(argv=[
        'sim_perception/__init__.py', 'sim_perception/scene_node.py',
        'sim_perception/reconstruction_node.py', 'setup.py', 'test'])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
