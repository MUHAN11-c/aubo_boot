# -*- coding: utf-8 -*-
"""零 ROS 纯核测试：generate_table_world（SDF 结构/确定性/manifest 联动）."""

from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET

import pytest

from ivg_sim.generate_table_world import generate, generate_sdf
from ivg_sim.layout import load_layout


@pytest.fixture()
def layout():
    return load_layout()


def test_sdf_wellformed_and_complete(layout, tmp_path):
    world_path = generate(worlds_dir=tmp_path)
    tree = ET.parse(world_path)
    root = tree.getroot()
    assert root.tag == 'sdf' and root.get('version') == '1.11'
    world = root.find('world')
    plugins = {p.get('name') for p in world.findall('plugin')}
    assert 'gz::sim::systems::Physics' in plugins
    assert 'gz::sim::systems::Sensors' in plugins
    models = {m.get('name') for m in world.findall('model')}
    assert {'ground_plane', 'table', 'camera_rig'} <= models
    includes = world.findall('include')
    assert len(includes) == len(layout.placements)
    uris = {i.find('uri').text for i in includes}
    assert 'model://mustard_bottle' in uris
    # 相机传感器
    sensor = world.find(".//model[@name='camera_rig']//sensor")
    assert sensor.get('type') == 'rgbd_camera'
    assert sensor.find('topic').text == 'camera/depth'


def test_generation_deterministic(layout, tmp_path):
    a = generate_sdf(layout)
    b = generate_sdf(load_layout())
    assert a == b
    world_path = generate(worlds_dir=tmp_path)
    digest = hashlib.sha256(world_path.read_bytes()).hexdigest()
    generate(worlds_dir=tmp_path)
    assert hashlib.sha256(world_path.read_bytes()).hexdigest() == digest


def test_jitter_changes_world_and_manifest(tmp_path):
    world_path = generate(worlds_dir=tmp_path)
    base_text = world_path.read_text(encoding='utf-8')
    jittered_path = generate(jitter=True, seed=42, worlds_dir=tmp_path)
    assert jittered_path == world_path  # 同一产物路径，内容应不同
    assert world_path.read_text(encoding='utf-8') != base_text
    manifest_path = tmp_path / 'ivg_table.manifest.yaml'
    text = manifest_path.read_text(encoding='utf-8')
    assert 'seed: 42' in text
    # 同 seed 再生成 → manifest 一致
    before = text
    generate(jitter=True, seed=42, worlds_dir=tmp_path)
    assert manifest_path.read_text(encoding='utf-8') == before


def test_jitter_requires_seed(tmp_path):
    with pytest.raises(SystemExit):
        generate(jitter=True, seed=None, worlds_dir=tmp_path)
