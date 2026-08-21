"""skip_codes 纯核：中文门禁原因 → 稳定短码."""

from peach_target_reconstruction.skip_codes import classify_skip_reason


def test_classify_known_reasons():
    """已知中文原因映射到计划里的短码."""
    cases = {
        '缺少所选 target_id 的同时间戳掩膜': 'missing_mask',
        '目标掩膜仅 12 像素 < 300': 'mask_pixels',
        '掩膜内有效深度占比 0.10 < 0.35': 'mask_depth_ratio',
        '目标漂移 50.0 mm > 40.0 mm': 'target_drift',
        '邻近锁定目标锚点间距 80.0 mm < 150.0 mm（防串扰拒帧，I6）':
            'neighbor_gap',
        '缓存帧龄期 2.50 s > max_frame_age_s=2.0（陈帧拒采）': 'stale_frame',
        '缓存帧未更新（与上次采帧同帧），请等下一帧': 'same_stamp',
        'TF base_link←camera_link 查询失败（已计 tf_failures）': 'tf_failure',
        '近重复视角不积分：平移 0.5 mm / 旋转 0.10 deg': 'near_duplicate',
        '连续运动超上限：平移 90.0 mm / 旋转 30.00 deg': 'motion_jump',
        '尚无同步 RGB-D 帧（确认相机/回放在线）': 'no_frame',
        '已达 max_views=24，请 finalize 或 remove_last': 'max_views',
        '机器人未静止：最大关节速度 0.1000 rad/s > 0.01': 'robot_not_static',
        '深度图 header.frame_id 为空，无法查 TF': 'empty_frame_id',
        'TF 查询期间缓存帧已更新，请等下一帧重试': 'frame_changed',
        '配准拒帧：low_fitness，fitness=0.100': 'icp_reject',
    }
    for reason, code in cases.items():
        assert classify_skip_reason(reason) == code, reason


def test_classify_empty_and_other():
    """空串与未知原因有确定出口."""
    assert classify_skip_reason('') == 'empty'
    assert classify_skip_reason('未知的奇怪原因') == 'other'
