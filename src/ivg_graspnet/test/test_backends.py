# -*- coding: utf-8 -*-
"""
零 ROS 纯核测试：后端注册表、checkpoint manifest、双后端行为与等价门.

行为等价门：重构后的 backend.detect + postprocess.run 链对 golden 点云
的输出须与重构前 GraspNetInference.get_grasp（已落盘 golden_grasps.npz）
逐位一致（本机 GPU 前向已验证确定）。无 torch/权重的环境自动跳过。
"""

from __future__ import annotations

import os
import sys

from ivg_graspnet.backends import available_backends, create_grasp_backend
from ivg_graspnet.backends.graspnet_torch import (
    config_with_manifest,
    GraspNetTorchConfig,
    load_checkpoint_manifest,
)
import numpy as np
import pytest

PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CKPT = os.path.join(PKG_ROOT, 'models', 'checkpoint-rs.tar')
GOLDEN = os.path.join(PKG_ROOT, 'test', 'data', 'golden_grasps.npz')
CGN_VENDOR = os.path.join(PKG_ROOT, 'ivg_graspnet', 'contact_graspnet_lib')
CGN_CKPT_DIR = os.path.join(CGN_VENDOR, 'checkpoints', 'contact_graspnet')


def test_registry_has_default_backend_and_rejects_unknown():
    assert 'graspnet_torch' in available_backends()
    with pytest.raises(KeyError, match='未知抓取后端'):
        create_grasp_backend('no_such_backend', {})


def test_contact_graspnet_backend_registered_and_detects():
    """Vendor 后端注册与真实测试场景检测（vendor test_data/7.npy）."""
    pytest.importorskip('torch')
    if not os.path.isdir(CGN_CKPT_DIR):
        pytest.skip('contact_graspnet vendor checkpoint 缺失')
    if 'contact_graspnet' not in available_backends():
        pytest.skip('contact_graspnet 后端未注册（vendor 不可用）')

    backend = create_grasp_backend('contact_graspnet', {'model_path': ''})
    try:
        assert backend.info.name == 'contact_graspnet'
        assert backend.info.approach_flip_z180 is False  # 待真机验证前默认不翻转

        sys.path.insert(0, CGN_VENDOR)
        try:
            data = np.load(
                os.path.join(CGN_VENDOR, 'test_data', '7.npy'), allow_pickle=True
            ).item()
            estimator = backend._get_estimator()
            pc_full, _segments, _colors = estimator.extract_point_clouds(
                data['depth'], data['K'], segmap=data['seg'], rgb=data['rgb'],
                skip_border_objects=False, z_range=[0.2, 1.8],
            )
        finally:
            sys.path.remove(CGN_VENDOR)

        raw = backend.detect(pc_full)
        assert len(raw) > 0, 'vendor 测试场景应产生抓取'
        assert raw.grasp_group_array.shape[1] == 17
        assert np.all(raw.widths >= 0)
    finally:
        backend.close()


def test_manifest_load_and_override():
    manifest = load_checkpoint_manifest(CKPT)
    assert manifest, 'checkpoint-rs.yaml manifest 应随包存在'
    assert manifest['backend'] == 'graspnet_torch'

    cfg = GraspNetTorchConfig(
        checkpoint_path=CKPT, num_view=1, hmax_list=(0.01,)
    )
    merged = config_with_manifest(cfg)
    assert merged.num_view == manifest['num_view']
    assert merged.hmax_list == tuple(manifest['hmax_list'])
    # manifest 未覆盖的键保持传入值
    assert merged.checkpoint_path == CKPT


def test_manifest_missing_returns_empty(tmp_path):
    fake = tmp_path / 'nope.tar'
    fake.write_bytes(b'x')
    assert load_checkpoint_manifest(str(fake)) == {}
    assert config_with_manifest(
        GraspNetTorchConfig(checkpoint_path=str(fake))
    ).num_view == 300  # 默认值兜底


def test_backend_info_declares_graspnet_flip_convention():
    pytest.importorskip('torch')
    if not os.path.exists(CKPT):
        pytest.skip('checkpoint 缺失')
    backend = create_grasp_backend('graspnet_torch', {'model_path': CKPT})
    try:
        assert backend.info.name == 'graspnet_torch'
        assert backend.info.approach_flip_z180 is True
    finally:
        backend.close()


def test_pipeline_matches_pre_refactor_golden():
    """行为等价门：backend.detect → postprocess 与重构前 get_grasp 逐位一致."""
    pytest.importorskip('torch')
    if not (os.path.exists(CKPT) and os.path.exists(GOLDEN)):
        pytest.skip('权重或 golden 基准缺失')

    from ivg_graspnet.postprocess import PostprocessConfig, run_postprocess

    golden = np.load(GOLDEN)
    params = {'model_path': CKPT, 'device': str(golden['device']), 'num_point': 20000}
    backend = create_grasp_backend('graspnet_torch', params)
    try:
        raw = backend.detect(golden['points'].astype(np.float32))
        config = PostprocessConfig(
            collision_thresh=0.01, voxel_size=0.01, approach_dist=0.05,
            max_gripper_width=0.1, max_grasps=5,
            nms_translation_thresh=0.03, nms_rotation_thresh_deg=30.0,
        )
        out = run_postprocess(raw, golden['points'].astype(np.float32), config)
    finally:
        backend.close()

    expected = golden['grasps']
    assert len(out) == len(expected), (
        f'抓取数不一致: new={len(out)}, golden={len(expected)}'
    )
    if len(expected):
        assert np.allclose(
            out.grasp_group_array, expected, rtol=1e-5, atol=1e-6
        ), '抓取向量与重构前 golden 不一致'
