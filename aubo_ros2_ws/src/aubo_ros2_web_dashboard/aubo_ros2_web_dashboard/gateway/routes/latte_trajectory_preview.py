"""拉花轨迹预览 BFF 端点 — POST /api/v1/latte/trajectory/preview 喵~

支持两种轨迹源:
  ① 录制回放 (pattern_type="") — 加载 npz 文件, 现有逻辑
  ② 参数化生成 (pattern_type="heart"|"rosetta"|"tulip"|"swan") — latte_art 共享库生成

BFF 纯 HTTP 设计: 不查 TF/不初始化 rclpy, start_pose 由前端传入喵~

⚠ 迁移计划 (2026-05-20):
  目标: BFF 不再直接 import latte_imitation, 改为通过 rosbridge 调 ROS 服务
  当前: BFF 直接 import latte_imitation 做轨迹生成+retargeting
  阻塞: ReplayLatteTrajectory.srv 响应缺少 tcp_path/spout_path/cup_pose/workspace_bounds 字段
  方案: 增补 .srv 字段 → 重建 ivg_interfaces → ROS mode="preview" 返回完整轨迹 JSON
  迁移后: 删除本文件, 前端通过 rosbridge call_service 获取轨迹数据
"""
from __future__ import annotations
from __future__ import annotations

import logging
from typing import Any, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

logger = logging.getLogger("gateway.latte_preview")

router = APIRouter(prefix="/api/v1/latte", tags=["latte-trajectory-preview"])


# ── Request / Response models ──────────────────────────────────────

class LatteStartPose(BaseModel):
    """前端传入的起点位姿 (EE 在 base_link 坐标系中) 喵~"""
    x: float = 0.0; y: float = 0.0; z: float = 0.0
    qx: float = 0.0; qy: float = 0.0; qz: float = 0.0; qw: float = 1.0


class LattePreviewRequest(BaseModel):
    # ── 轨迹源 ──
    episode_idx: int = 0
    arm: str = "right"
    pattern_type: str = ""            # ""=录制回放 / "heart"|"rosetta"|"tulip"|"swan"|"custom"
    pattern_image_path: str = ""      # custom 模式图片路径
    tulip_layers: int = 3             # 郁金香层数

    # ── 杯子参数 ──
    cup_center_x: float = 0.0
    cup_center_y: float = 0.0
    cup_surface_z: float = 0.15
    cup_radius: float = 0.04

    # ── 倾倒参数 ──
    pour_mix_height_offset: float = 0.076
    pour_draw_height_offset: float = 0.006
    pour_finish_height_offset: float = 0.076
    pour_wiggle_amplitude: float = 0.006
    pour_wiggle_frequency: float = 5.0
    pour_max_velocity: float = 0.05
    pour_max_acceleration: float = 0.1
    pour_max_jerk: float = 0.5
    enable_anti_sloshing: bool = True

    # ── 通用参数 ──
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    speed_scale: float = 1.0
    tool_offset_id: str = "default"
    start_pose: Optional[LatteStartPose] = None  # 前端传入, BFF 不查 TF 喵~


class LatteWaypoint(BaseModel):
    x: float; y: float; z: float
    qx: float; qy: float; qz: float; qw: float


class LatteCupPose(BaseModel):
    x: float; y: float; z: float
    qx: float; qy: float; qz: float; qw: float


class LatteWorkspaceBounds(BaseModel):
    x_min: float; x_max: float
    y_min: float; y_max: float
    z_min: float; z_max: float


class LattePreviewResponse(BaseModel):
    success: bool
    num_frames: int
    path_length: float
    tcp_path: list[LatteWaypoint]
    spout_path: list[LatteWaypoint]
    cup_pose: LatteCupPose
    workspace_bounds: LatteWorkspaceBounds
    pattern_type: str = ""
    is_generated: bool = False
    message: str


# ── Lazy imports ───────────────────────────────────────────────────

_latte_available: bool | None = None


def _check_latte_available() -> bool:
    global _latte_available
    if _latte_available is None:
        try:
            import latte_imitation  # noqa: F401
            _latte_available = True
        except ImportError:
            logger.warning("latte_imitation 不可用, 预览端点将返回 503")
            _latte_available = False
    return _latte_available


def _get_cartesian_resource_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    import os
    share = get_package_share_directory("latte_imitation")
    return os.path.join(share, "resource", "cartesian")


def _build_start_pose(req: LattePreviewRequest) -> "Pose":
    """构建起点位姿: 前端必须传入有效位姿, 否则拒绝请求 喵~

    BFF 零 ROS 依赖 — 不查 TF, 不 import rclpy 喵~
    """
    from geometry_msgs.msg import Pose, Point, Quaternion

    if req.start_pose is None:
        raise HTTPException(status_code=400, detail="缺少 start_pose (机械臂当前末端位姿)")

    sp = req.start_pose
    is_zero = (abs(sp.x) < 1e-9 and abs(sp.y) < 1e-9 and abs(sp.z) < 1e-9
               and abs(sp.qx) < 1e-9 and abs(sp.qy) < 1e-9
               and abs(sp.qz) < 1e-9 and abs(sp.qw - 1.0) < 1e-9)
    if is_zero:
        raise HTTPException(status_code=400, detail="start_pose 为零点位姿, 请确认机械臂位姿数据已就绪")

    logger.info(f"使用前端传入起点: ({sp.x:.3f},{sp.y:.3f},{sp.z:.3f})")
    return Pose(
        position=Point(x=sp.x, y=sp.y, z=sp.z),
        orientation=Quaternion(x=sp.qx, y=sp.qy, z=sp.qz, w=sp.qw),
    )


def _pose_to_dict(pose: "Pose") -> dict[str, float]:
    return {
        "x": float(pose.position.x), "y": float(pose.position.y), "z": float(pose.position.z),
        "qx": float(pose.orientation.x), "qy": float(pose.orientation.y),
        "qz": float(pose.orientation.z), "qw": float(pose.orientation.w),
    }


def _np_to_waypoints(positions: "np.ndarray", orientations: "np.ndarray | None",
                     frame_step: int = 1) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    T = len(positions)
    for i in range(0, T, frame_step):
        p = positions[i]
        wp: dict[str, float] = {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])}
        if orientations is not None and i < len(orientations):
            q = orientations[i]
            wp.update(qx=float(q[0]), qy=float(q[1]), qz=float(q[2]), qw=float(q[3]))
        else:
            wp.update(qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        points.append(wp)
    return points


def _compute_spout_path(positions: "np.ndarray", orientations: "np.ndarray | None",
                        tool_offset: tuple[float, float, float],
                        frame_step: int = 1) -> list[dict[str, float]]:
    import numpy as np
    from latte_imitation.trajectory_transform import quat_to_rot

    ox, oy, oz = tool_offset
    offset = np.array([ox, oy, oz])
    points: list[dict[str, float]] = []
    T = len(positions)
    for i in range(0, T, frame_step):
        p_tcp = positions[i]
        if orientations is not None and i < len(orientations):
            R = quat_to_rot(orientations[i])
        else:
            R = np.eye(3)
        p_spout = p_tcp + R @ offset
        points.append({"x": float(p_spout[0]), "y": float(p_spout[1]), "z": float(p_spout[2]),
                       "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0})
    return points


# ── Route ─────────────────────────────────────────────────────────

@router.post("/trajectory/preview", response_model=LattePreviewResponse)
async def latte_trajectory_preview(req: LattePreviewRequest):
    """拉花轨迹预览 — 录制回放 或 参数化生成, 返回 JSON 数据 喵~

    调用链:
      pattern_type!="" → latte_art 生成 → compose → anti_slosh → retarget → JSON
      pattern_type==""  → npz 加载 → retarget → JSON (现有逻辑)
    """
    # 参数校验
    if req.pattern_type and req.pattern_type not in ("heart", "rosetta", "tulip", "swan", "custom"):
        raise HTTPException(status_code=422,
                           detail="pattern_type 必须为 ''/'heart'/'rosetta'/'tulip'/'swan'/'custom'")
    if req.arm not in ("left", "right"):
        raise HTTPException(status_code=422, detail="arm 必须为 'left' 或 'right'")
    # preview 模式只算几何, speed_scale 会在 ROS 执行时生效 喵~
    if not (0.01 <= req.speed_scale <= 10.0):
        raise HTTPException(status_code=422, detail="speed_scale 必须在 [0.01, 10.0]")

    # episode_idx 仅在录制回放模式 (pattern_type="") 下需要校验 喵~
    if not req.pattern_type and not (0 <= req.episode_idx <= 39):
        raise HTTPException(status_code=422, detail="episode_idx 必须在 [0, 39]")

    if not _check_latte_available():
        raise HTTPException(status_code=503, detail="latte_imitation 包不可用")

    try:
        from latte_imitation.trajectory import CartesianTrajectory
        from latte_imitation.trajectory_transform import retarget_trajectory
        from latte_imitation.config_loader import load_tool_offset, load_workspace_safety
        import numpy as np
    except ImportError as e:
        raise HTTPException(status_code=503, detail=f"latte_imitation 导入失败: {e}")

    is_generated = False
    rpy_user = (req.roll_deg, req.pitch_deg, req.yaw_deg)

    # ── 轨迹源分支 ──────────────────────────────────────────
    if req.pattern_type:
        # 模式 B: 参数化生成
        try:
            from latte_imitation.latte_art import (
                LatteArtTrajectory, CupConfig, PourConfig,
                compose_full_trajectory, apply_anti_sloshing,
                parametric_to_cartesian,
            )
        except ImportError as e:
            raise HTTPException(status_code=503, detail=f"latte_art 模块不可用: {e}")

        cup = CupConfig(
            center_x=req.cup_center_x,
            center_y=req.cup_center_y,
            surface_z=req.cup_surface_z,
            radius=req.cup_radius,
        )
        pour = PourConfig(
            mix_height_offset=req.pour_mix_height_offset,
            draw_height_offset=req.pour_draw_height_offset,
            finish_height_offset=req.pour_finish_height_offset,
            wiggle_amplitude=req.pour_wiggle_amplitude,
            wiggle_frequency=req.pour_wiggle_frequency,
            max_velocity=req.pour_max_velocity,
            max_acceleration=req.pour_max_acceleration,
            max_jerk=req.pour_max_jerk,
        )

        gen = LatteArtTrajectory(cup, pour)
        try:
            if req.pattern_type == "tulip":
                xyz = gen.tulip(layers=max(1, req.tulip_layers))
            elif req.pattern_type == "custom":
                if not req.pattern_image_path:
                    raise HTTPException(status_code=422, detail="custom 模式需要 pattern_image_path")
                xyz = gen.from_image(req.pattern_image_path)
            else:
                xyz = getattr(gen, req.pattern_type)()
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"轨迹生成失败: {e}")

        # 后处理: compose + anti_slosh
        xyz = compose_full_trajectory(xyz, cup, pour)
        if req.enable_anti_sloshing:
            xyz = apply_anti_sloshing(xyz, pour)

        # 桥接 → CartesianTrajectory
        cart = parametric_to_cartesian(xyz)
        is_generated = True

        logger.info(f"[latte_preview] 参数化生成 pattern={req.pattern_type} "
                    f"frames={cart.num_frames}")
    else:
        # 模式 A: 录制回放 (现有 npz 逻辑)
        resource_dir = _get_cartesian_resource_dir()
        try:
            all_traj = CartesianTrajectory.load_all(resource_dir, req.arm)
        except FileNotFoundError:
            raise HTTPException(status_code=404, detail=f"轨迹目录不存在: {resource_dir}/{req.arm}/")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"加载轨迹失败: {e}")

        if req.episode_idx not in all_traj:
            available = sorted(all_traj.keys())
            raise HTTPException(status_code=404,
                               detail=f"Episode {req.episode_idx} 不存在, 可用: {available}")
        cart = all_traj[req.episode_idx]

    # ── 公共处理 (两种模式都走) ──────────────────────────
    start_pose = _build_start_pose(req)

    try:
        transformed = retarget_trajectory(cart, start_pose, rpy_user=rpy_user)
        # 叠加杯子 XY 偏移: 轨迹在原点生成, retarget 后移到杯位置 喵~
        if req.cup_center_x or req.cup_center_y:
            transformed.positions[:, 0] += req.cup_center_x
            transformed.positions[:, 1] += req.cup_center_y
    except Exception as e:
        logger.exception("retarget_trajectory 失败")
        raise HTTPException(status_code=500, detail=f"轨迹变换失败: {e}")

    tool_cfg = load_tool_offset(req.tool_offset_id)
    ws_cfg = load_workspace_safety()
    is_safe, safety_msg, _ = transformed.check_workspace_bounds(
        (ws_cfg.x_min, ws_cfg.x_max),
        (ws_cfg.y_min, ws_cfg.y_max),
        (ws_cfg.z_min, ws_cfg.z_max),
    )

    key_indices = transformed.adaptive_sample(displacement_threshold=0.005)
    sampled = transformed._slice(key_indices)
    tcp_waypoints = _np_to_waypoints(sampled.positions, sampled.orientations)
    spout_waypoints = _compute_spout_path(
        transformed.positions, transformed.orientations, tool_cfg.pos
    )
    cup_pose_dict = _pose_to_dict(start_pose)

    message = "OK (generated)" if is_generated else "OK"
    if not is_safe:
        message = f"工作空间警告: {safety_msg} (策略: {ws_cfg.safety_policy})"
        logger.warning(f"[latte_preview] {message}")

    logger.info(
        f"[latte_preview] source={'gen:'+req.pattern_type if req.pattern_type else 'ep:'+str(req.episode_idx)} "
        f"frames={transformed.num_frames} path={transformed.path_length():.2f}m safe={is_safe}"
    )

    # 注: success 在此指 "轨迹在工作空间内" (is_safe)
    # 与 ROS 端 ReplayLatteTrajectory.success ("无错误") 语义不同 喵~
    return LattePreviewResponse(
        success=is_safe,
        num_frames=transformed.num_frames,
        path_length=round(transformed.path_length(), 4),
        tcp_path=[LatteWaypoint(**w) for w in tcp_waypoints],
        spout_path=[LatteWaypoint(**w) for w in spout_waypoints],
        cup_pose=LatteCupPose(**cup_pose_dict),
        workspace_bounds=LatteWorkspaceBounds(
            x_min=ws_cfg.x_min, x_max=ws_cfg.x_max,
            y_min=ws_cfg.y_min, y_max=ws_cfg.y_max,
            z_min=ws_cfg.z_min, z_max=ws_cfg.z_max,
        ),
        pattern_type=req.pattern_type,
        is_generated=is_generated,
        message=message,
    )
