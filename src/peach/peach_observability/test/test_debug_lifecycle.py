"""Lifecycle debug uses the actual Nav2 service contract."""
from types import SimpleNamespace

from nav2_msgs.srv import ManageLifecycleNodes
from peach_observability.debug_actions import (
    _goal_for,
    _summarize_service,
    DebugBridge,
)
import pytest


@pytest.mark.parametrize('command', [
    'STARTUP', 'PAUSE', 'RESUME', 'RESET', 'SHUTDOWN'])
def test_lifecycle_request_matches_nav2(command):
    """The wire type and command must match the active service server."""
    request = _goal_for('manage_nodes_service', {'command': command})
    assert isinstance(request, ManageLifecycleNodes.Request)
    assert request.command == getattr(ManageLifecycleNodes.Request, command)


@pytest.mark.parametrize('success', [True, False])
def test_nav2_response_without_message(success):
    """Nav2 responses contain only success; do not invent a message field."""
    response = ManageLifecycleNodes.Response(success=success)
    assert _summarize_service(response) == {'success': success}


def test_unavailable_lifecycle_service_is_503():
    """An unavailable service is rejected before any request is sent."""
    bridge = DebugBridge(None, {}, 1.0, False, lambda _message: None)
    bridge._clients['manage_nodes_service'] = SimpleNamespace(
        wait_for_service=lambda timeout_sec: False)
    status, body = bridge.command(
        'manage_nodes_service', {'command': 'STARTUP'})
    assert status == 503
    assert body['accepted'] is False
