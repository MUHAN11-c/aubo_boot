"""latte_art — 拉花图案参数化轨迹生成器 (纯 Python, 无 ROS 依赖) 喵~

来源: latte_art_robot_research/轨迹生成方案/latte_art_trajectory.py
理论: Sunergos Milk Training Video + latteartguide.com 多源交叉验证

用法:
    from latte_imitation.latte_art import LatteArtTrajectory, CupConfig, PourConfig

    cup = CupConfig(center_x=-0.28, center_y=-0.03, surface_z=0.22, radius=0.035)
    pour = PourConfig()
    gen = LatteArtTrajectory(cup, pour)
    xyz = gen.heart()           # → (200, 3) numpy array
    xyz = gen.rosetta()         # → (350, 3)
    xyz = gen.tulip(layers=3)   # → (350, 3)
    xyz = gen.swan()            # → (400, 3)

    # 后处理
    from latte_imitation.latte_art import compose_full_trajectory, apply_anti_sloshing
    full = compose_full_trajectory(xyz, cup, pour)
    smooth = apply_anti_sloshing(full, pour)

    # 桥接到 CartesianTrajectory
    from latte_imitation.latte_art import parametric_to_cartesian
    cart = parametric_to_cartesian(smooth)
"""

from .orientation_profile import (
    compute_roll_profile,
    assemble_cartesian_with_orientation,
)

from .config import CupConfig, PourConfig
from .generator import LatteArtTrajectory
from .anti_sloshing import (
    apply_anti_sloshing,
    compose_full_trajectory,
    to_moveit_waypoints,
)
from .bridge import parametric_to_cartesian, euler_deg_to_quat

__all__ = [
    "CupConfig",
    "PourConfig",
    "LatteArtTrajectory",
    "apply_anti_sloshing",
    "compose_full_trajectory",
    "to_moveit_waypoints",
    "parametric_to_cartesian",
    "euler_deg_to_quat",
]
