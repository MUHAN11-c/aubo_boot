"""web_goal 纯核单测: 档位参数校验与空串回退."""

from aubo_hand_eye_calibration.web_goal import (
    goal_params_from_body,
    normalize_method,
    normalize_pose_source,
    normalize_solve_target,
)

import pytest


def test_empty_values_fall_back_to_server_defaults():
    assert normalize_method('') == ''
    assert normalize_method(None) == ''
    assert normalize_pose_source('  ') == ''
    assert normalize_solve_target(None) == ''


def test_valid_values_pass_through_stripped():
    assert normalize_method('tsai') == 'tsai'
    assert normalize_method(' auto ') == 'auto'
    assert normalize_pose_source('poses') == 'poses'
    assert normalize_pose_source('auto') == 'auto'
    assert normalize_solve_target('hand_eye') == 'hand_eye'
    assert normalize_solve_target('joint') == 'joint'


@pytest.mark.parametrize('value', ['tsaai', 'AUTO', 'auto/joint', '0'])
def test_invalid_method_raises(value):
    with pytest.raises(ValueError, match='求解方法'):
        normalize_method(value)


@pytest.mark.parametrize('value', ['automatic', 'POSES', 'random'])
def test_invalid_pose_source_raises(value):
    with pytest.raises(ValueError, match='位姿来源'):
        normalize_pose_source(value)


@pytest.mark.parametrize('value', ['joints', 'intrinsics', 'both'])
def test_invalid_solve_target_raises(value):
    with pytest.raises(ValueError, match='求解目标'):
        normalize_solve_target(value)


def test_goal_params_from_body_extracts_and_validates():
    params = goal_params_from_body({
        'method': 'park', 'pose_source': 'auto', 'solve_target': 'joint'})
    assert params == {
        'method': 'park', 'pose_source': 'auto', 'solve_target': 'joint'}


def test_goal_params_from_body_defaults_on_missing_keys():
    expected = {'method': '', 'pose_source': '', 'solve_target': ''}
    assert goal_params_from_body({}) == expected
    assert goal_params_from_body(None) == expected


def test_goal_params_from_body_rejects_invalid_value():
    with pytest.raises(ValueError, match='位姿来源'):
        goal_params_from_body({'pose_source': 'magic'})
