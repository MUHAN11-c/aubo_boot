"""ControlTask 幂等：expected_state_seq 与当前序一致才接受."""


def apply_control(
        state_seq: int, expected: int, command: int, paused: bool,
        allowed=None):
    """
    纯函数控制面.

    expected==0 表示不校验。
    PAUSE=0 RESUME=1 ENTER_MAINTENANCE=2 EXIT_MAINTENANCE=3
    CANCEL_NOW=4 SKIP_TARGET=5 ACKNOWLEDGE_RECOVERY=6.
    allowed 为 permissions_for 结果；None 表示不按状态机过滤。
    返回 (accepted, new_seq, paused, cancel, skip).
    """
    if expected != 0 and expected != state_seq:
        return False, state_seq, paused, False, False
    if allowed is not None and command not in allowed:
        return False, state_seq, paused, False, False
    if command in (0, 2):
        return True, state_seq + 1, True, False, False
    if command in (1, 3):
        return True, state_seq + 1, False, False, False
    if command == 4:
        return True, state_seq + 1, paused, True, False
    if command == 5:
        return True, state_seq + 1, paused, False, True
    if command == 6:
        return True, state_seq + 1, paused, False, False
    return False, state_seq, paused, False, False
