"""
latte_imitation — 拉花轨迹模仿学习包 (MoveIt2 标准管线) 喵~

公共 API:
  from latte_imitation import CartesianTrajectory     # npz I/O + ROS2 导出
  from latte_imitation import apply_start_pose        # 6-DOF 刚性变换
  from latte_imitation import get_current_ee_pose     # TF 查询 (ROS 节点模式)
  from latte_imitation import get_ee_pose_from_tf     # TF 查询 (独立脚本模式)
  from latte_imitation import TfQueryNode             # TF 查询 (GUI 持久模式)
"""

from .trajectory import CartesianTrajectory
from .trajectory_transform import (
    apply_start_pose,
    quat_to_rot,
    rot_to_quat,
    quat_multiply,
    euler_deg_to_quat,
    is_default_position,
    is_default_orientation,
    is_default_pose,
)
from .tf_utils import get_current_ee_pose, get_ee_pose_from_tf, TfQueryNode

__all__ = [
    "CartesianTrajectory",
    "apply_start_pose",
    "quat_to_rot",
    "rot_to_quat",
    "quat_multiply",
    "euler_deg_to_quat",
    "is_default_position",
    "is_default_orientation",
    "is_default_pose",
    "get_current_ee_pose",
    "get_ee_pose_from_tf",
    "TfQueryNode",
]
