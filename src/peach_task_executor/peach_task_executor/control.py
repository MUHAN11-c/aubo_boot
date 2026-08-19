"""ControlTask 幂等：expected_state_seq 与当前序一致才接受."""


def apply_control(state_seq: int, expected: int, command: int, paused: bool):
    """
    纯函数控制面.

    expected==0 表示不校验。PAUSE=0 RESUME=1 CANCEL_NOW=4.
    返回 (accepted, new_seq, paused, cancel).
    """
    if expected != 0 and expected != state_seq:
        return False, state_seq, paused, False
    if command == 0:
        return True, state_seq + 1, True, False
    if command == 1:
        return True, state_seq + 1, False, False
    if command == 4:
        return True, state_seq + 1, paused, True
    return False, state_seq, paused, False
