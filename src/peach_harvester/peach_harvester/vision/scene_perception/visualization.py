"""
感知可视化拆分壳（W3）：消息组装在 msg_builders，像素绘制在 debug_draw.

pipeline / 节点已改走新模块公开名；本壳仅 re-export 保持旧 import
路径可用（向后兼容），不再自带实现。原 ``_pack_rgb_bgr`` 死别名已删
（位打包单源 common.geometry.pack_rgb_bgr）。
"""
from __future__ import annotations

from peach_harvester.vision.scene_perception.debug_draw import (  # noqa: F401
    draw_debug,
)
from peach_harvester.vision.scene_perception.msg_builders import (  # noqa: F401
    bbox_cloud_xyzrgb,
    best_axis_direction,
    quat_to_msg,
    to_candidate,
    to_candidate_2d,
    to_detection2d,
    to_fitting,
    to_markers,
    TRACKING_STATUS_TO_MSG,
    xyzrgb_to_cloud_msg,
)
