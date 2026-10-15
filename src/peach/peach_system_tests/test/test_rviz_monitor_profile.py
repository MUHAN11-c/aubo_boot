"""Monitor RViz uses one robot renderer while retaining execution evidence."""
from pathlib import Path

import yaml

CONFIG = Path(__file__).parents[1] / 'config/live_camera_mock.rviz'


def displays():
    config = yaml.safe_load(CONFIG.read_text())
    output = []

    def walk(value):
        if isinstance(value, dict):
            if value.get('Class'):
                output.append(value)
            for child in value.values():
                walk(child)
        elif isinstance(value, list):
            for child in value:
                walk(child)

    walk(config['Visualization Manager']['Displays'])
    return output


def test_monitor_has_only_one_robot_renderer_and_no_planning_panel():
    items = displays()
    assert not any(item['Class'] == 'moveit_rviz_plugin/MotionPlanning' for item in items)
    robots = [item for item in items if item['Class'] == 'rviz_default_plugins/RobotModel']
    assert len(robots) == 1 and robots[0]['Enabled'] and robots[0]['Visual Enabled']
    assert not robots[0]['Collision Enabled']


def test_live_tcp_tool_frames_and_keypoint_trajectory_markers_remain_visible():
    enabled = [item for item in displays() if item.get('Enabled')]
    topics = {item.get('Topic', {}).get('Value') for item in enabled}
    assert '/peach/observability/markers' in topics
    assert '/peach/perception/markers' in topics
    assert '/peach_arm/planned_views' in topics
    transform = next(item for item in enabled if item['Class'] == 'rviz_default_plugins/TF')
    assert transform['Show Axes']
    for frame in ('tcp', 'tool_axis', 'sleeve_mouth', 'cutting_plane'):
        assert transform['Frames'][frame]['Value']


def test_monitor_has_unique_cloud_subscriptions_and_keeps_camera_images():
    enabled = [item for item in displays() if item.get('Enabled')]
    clouds = [item for item in enabled if item['Class'] == 'rviz_default_plugins/PointCloud2']
    topics = [item['Topic']['Value'] for item in clouds]
    assert len(topics) == len(set(topics)) == 2
    assert set(topics) == {'/peach/perception/single_cloud', '/peach/reconstruction/preview_cloud'}
    for item in clouds:
        assert item['Style'] == 'Points' and item['Decay Time'] == 0
        assert item['Topic']['Depth'] == 1
        assert item['Topic']['Reliability Policy'] == 'Best Effort'
    images = [item for item in enabled if item['Class'] == 'rviz_default_plugins/Image']
    assert {item['Topic']['Value'] for item in images} == {
        '/camera/color/image_raw', '/peach/perception/debug_image'}
