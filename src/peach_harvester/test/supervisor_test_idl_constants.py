"""Zero-ROS: kernel enums must match peach_interfaces numeric constants."""
from pathlib import Path
import re

from peach_harvester.supervisor.domain import lifecycle
import peach_harvester.supervisor.harvest_fsm as fsm

SRC = Path(__file__).resolve().parents[2]
_UINT8 = re.compile(r'^uint8\s+([A-Z][A-Z0-9_]*)=(\d+)')


def _uint8_constants(relative: str) -> dict:
    text = (SRC / relative).read_text(encoding='utf-8')
    found = {}
    for line in text.splitlines():
        match = _UINT8.match(line.strip())
        if match:
            found[match.group(1)] = int(match.group(2))
    return found


def test_harvest_state_constants_match_fsm():
    idl = _uint8_constants('peach_interfaces/msg/HarvestState.msg')
    names = (
        'WAITING_READY', 'DISCOVERY', 'RUNNING', 'PAUSE_PENDING', 'PAUSED',
        'MAINTENANCE', 'COMPLETED', 'RECOVERY_REQUIRED', 'INTERRUPTED',
        'TARGET_IDLE', 'SELECTING', 'OBSERVING', 'FINALIZING', 'VALIDATING',
        'APPROACHING', 'TOOL_ACTION', 'RETREATING', 'COMPLETING',
        'TARGET_SUCCEEDED', 'TARGET_SKIPPED', 'TARGET_FAILED',
        'MODE_AUTO', 'MODE_PAUSED', 'MODE_MAINTENANCE',
    )
    for name in names:
        assert getattr(fsm, name) == idl[name], name


def test_navigating_is_reserved_and_unwired():
    idl = _uint8_constants('peach_interfaces/msg/HarvestState.msg')
    assert idl['NAVIGATING'] == 9
    assert not hasattr(fsm, 'NAVIGATING')


def test_control_task_commands_match_fsm():
    idl = _uint8_constants('peach_interfaces/srv/ControlTask.srv')
    assert fsm.CMD_PAUSE == idl['PAUSE']
    assert fsm.CMD_RESUME == idl['RESUME']
    assert fsm.CMD_ENTER_MAINTENANCE == idl['ENTER_MAINTENANCE']
    assert fsm.CMD_EXIT_MAINTENANCE == idl['EXIT_MAINTENANCE']
    assert fsm.CMD_CANCEL_NOW == idl['CANCEL_NOW']
    assert fsm.CMD_SKIP_TARGET == idl['SKIP_TARGET']
    assert fsm.CMD_ACKNOWLEDGE_RECOVERY == idl['ACKNOWLEDGE_RECOVERY']


def test_manage_lifecycle_commands_match_domain():
    idl = _uint8_constants('peach_interfaces/srv/ManageLifecycleNodes.srv')
    assert lifecycle.CMD_STARTUP == idl['STARTUP']
    assert lifecycle.CMD_PAUSE == idl['PAUSE']
    assert lifecycle.CMD_RESUME == idl['RESUME']
    assert lifecycle.CMD_RESET == idl['RESET']
    assert lifecycle.CMD_SHUTDOWN == idl['SHUTDOWN']
