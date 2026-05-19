"""
latte_imitation — 拉花轨迹模仿学习包 (MoveIt2 标准管线) 喵~

公共 API:
  # 数据结构
  from latte_imitation import CartesianTrajectory

  # SE(3) 变换
  from latte_imitation import retarget_trajectory
  from latte_imitation import quat_to_rot, rot_to_quat, quat_multiply
  from latte_imitation import euler_deg_to_quat, quat_to_euler_deg
  from latte_imitation import is_default_position, is_default_orientation

  # 配置加载
  from latte_imitation import load_tool_offset, load_workspace_safety
  from latte_imitation import ToolOffsetConfig, WorkspaceSafetyConfig

  # TF 查询
  from latte_imitation import get_current_ee_pose, get_ee_pose_from_tf

理论依据:
  SPOT (arXiv:2411.00965): Object-centric SE(3) trajectory
  Isaac Teleop: Se3RelRetargeter delta 语义
  SO(3) Action Repr. (Savva 2025): Hamilton 四元数约定
  SVRC: Object-relative Cartesian → Very High generalization
"""

from .trajectory import CartesianTrajectory
from .trajectory_transform import (
    retarget_trajectory,
    quat_to_rot,
    rot_to_quat,
    quat_multiply,
    euler_deg_to_quat,
    quat_to_euler_deg,
    is_default_position,
    is_default_orientation,
    is_default_pose,
)
from .config_loader import (
    load_tool_offset,
    load_workspace_safety,
    ToolOffsetConfig,
    WorkspaceSafetyConfig,
)
from .tf_utils import get_current_ee_pose, get_ee_pose_from_tf, TfQueryNode

__all__ = [
    "CartesianTrajectory",
    "retarget_trajectory",
    "quat_to_rot",
    "rot_to_quat",
    "quat_multiply",
    "euler_deg_to_quat",
    "quat_to_euler_deg",
    "is_default_position",
    "is_default_orientation",
    "is_default_pose",
    "load_tool_offset",
    "load_workspace_safety",
    "ToolOffsetConfig",
    "WorkspaceSafetyConfig",
    "get_current_ee_pose",
    "get_ee_pose_from_tf",
    "TfQueryNode",
]
