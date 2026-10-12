"""Catch replacement hands, missing contact dynamics and mismatched controllers."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ivg_sim.arm_description import build_arm_urdf
import pytest
import yaml


def test_gazebo_uses_original_four_prismatic_fingers():
    robot = ET.fromstring(build_arm_urdf())
    fingers = [j for j in robot.findall('joint')
               if j.get('type') == 'prismatic']
    assert len(fingers) == 4
    assert all(j.find('parent').get('link') == 'gripper2_Link'
               for j in fingers)
    assert robot.find("link[@name='hand_root']") is None
    assert robot.find("link[@name='tool_tcp']") is not None
    for joint in fingers:
        child = robot.find(f"link[@name='{joint.find('child').get('link')}']")
        assert float(child.find('inertial/mass').get('value')) > 0
        assert child.find('collision') is not None
        assert float(joint.find('limit').get('upper')) > 0.04


def test_fingers_use_force_commands_not_penetrating_position_commands():
    robot = ET.fromstring(build_arm_urdf())
    fingers = {j.get('name') for j in robot.findall('joint')
               if j.get('type') == 'prismatic'}
    controlled = robot.findall('ros2_control/joint')
    interfaces = {j.get('name'): j.find('command_interface').get('name')
                  for j in controlled if j.get('name') in fingers}
    assert len(interfaces) == 4
    assert set(interfaces.values()) == {'effort'}
    config = yaml.safe_load((Path(__file__).parents[1] /
                             'config/gz_controllers.yaml').read_text())
    assert config['gripper_effort_controller']['ros__parameters']['joints'] == list(interfaces)


@pytest.mark.parametrize('gripper', ['none', 'gripper0', 'gripper1', 'gripper2',
                                     'gripper1coffeecup', 'gripper1milkcup'])
def test_ivg_model_choices_are_srdf_compatible(gripper):
    import xacro
    root = Path(__file__).parents[2]
    description = root / 'ivg_description/urdf/aubo_e5.urdf.xacro'
    assert description.exists(), 'IVG must have its own parameterized description'
    robot = ET.fromstring(xacro.process_file(
        str(description), mappings={'gripper': gripper}).toxml())
    links = {link.get('name') for link in robot.findall('link')}
    assert {'tool_tcp', 'camera_Link', 'kuaihuan_Link'} <= links
    assert 'tool_body_link' not in links
    srdf_path = root / 'ivg_moveit_config/config/aubo_e5.srdf'
    assert srdf_path.exists()
    srdf = ET.parse(srdf_path).getroot()
    for element in srdf.iter():
        for key in ('base_link', 'tip_link', 'link1', 'link2'):
            if key in element.attrib:
                assert element.get(key) in links


def test_unknown_gripper_is_rejected():
    import xacro
    description = Path(__file__).parents[2] / 'ivg_description/urdf/aubo_e5.urdf.xacro'
    assert description.exists()
    with pytest.raises(xacro.XacroException):
        xacro.process_file(str(description), mappings={'gripper': 'typo'})


def test_grasp_height_uses_collision_center_not_model_bottom():
    from ivg_sim import arm_description
    assert hasattr(arm_description, 'collision_center_z')
    # This model's root is on its bottom; gripping at root+1cm hits the table.
    assert arm_description.collision_center_z('potted_meat_can') == pytest.approx(0.041)


@pytest.mark.parametrize('gripper', ['none', 'gripper0', 'gripper1', 'gripper2',
                                     'gripper1coffeecup', 'gripper1milkcup'])
def test_fixed_tool_home_is_not_in_permanent_self_collision(gripper, tmp_path):
    from ivg_moveit_config.configuration import build_moveit_config
    from moveit.core.collision_detection import CollisionRequest, CollisionResult
    from moveit.core.planning_scene import PlanningScene
    from moveit.core.robot_model import RobotModel
    from moveit.core.robot_state import RobotState

    config = build_moveit_config(gripper, 'none')
    urdf = tmp_path / 'model.urdf'
    srdf = tmp_path / 'model.srdf'
    urdf.write_text(config.robot_description['robot_description'])
    srdf.write_text(config.robot_description_semantic['robot_description_semantic'])
    model = RobotModel(str(urdf), str(srdf))
    state = RobotState(model)
    state.set_to_default_values('manipulator_e5', 'home')
    state.update()
    result = CollisionResult()
    PlanningScene(model).check_self_collision(CollisionRequest(), result, state)
    assert not result.collision, f'{gripper} cannot plan from its home state'
