"""
YAML 配置加载器 — 工具偏移 + 工作空间安全边界喵~

=== 设计 ===

  所有配置用 dataclass 定义类型，用 yaml.safe_load 加载。
  提供默认值和验证逻辑，调用方无需处理文件不存在的情况。

  理论依据:
    tools.yaml (src/tool_changer/config/tools.yaml) — 工具配置格式参考
    workspace_limits.yaml (aubo_moveit_config/scripts/) — 工作空间边界格式参考
"""

import os
from dataclasses import dataclass

import yaml
from ament_index_python.packages import get_package_share_directory


# ═══════════════════════════════════════════════════════════════
# Dataclasses
# ═══════════════════════════════════════════════════════════════

@dataclass
class ToolOffsetConfig:
    """拉花壶工具偏移配置 (tool_tcp → pitcher_spout) 喵~

    AUBO E5 末端链 (aubo_e5_10.urdf L340-398):
      wrist3_Link --(+Z0.020)--> camera_Link --(+Z0.0215,Rz180°)--> kuaihuan_Link
      kuaihuan_Link --(+Z0.033)--> tool_tcp --(offset)--> pitcher_spout
    """
    tool_id: str
    name: str
    description: str
    pos: tuple[float, float, float]   # (x, y, z) in tool_tcp frame (m)
    rpy: tuple[float, float, float]   # (roll, pitch, yaw) in degrees


@dataclass
class WorkspaceSafetyConfig:
    """AUBO E5 工作空间安全边界 喵~

    来源: aubo_moveit_config/scripts/workspace_limits.yaml
    坐标系: base_link = world (URDF identity fixed joint)

    默认值:
      X[-0.7, 0.7] Y[-0.35, 0.35] Z[0.0, 0.7]
    """
    x_min: float; x_max: float
    y_min: float; y_max: float
    z_min: float; z_max: float
    safety_policy: str = "warn_and_block"


# ═══════════════════════════════════════════════════════════════
# 加载函数
# ═══════════════════════════════════════════════════════════════

def _config_dir() -> str:
    """获取 config/ 目录路径 (优先 ament_index, 回退源码目录) 喵~"""
    try:
        share = get_package_share_directory("latte_imitation")
        path = os.path.join(share, "config")
        if os.path.isdir(path):
            return path
    except Exception:
        pass
    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(pkg, "config")


def load_tool_offset(tool_id: str = "default") -> ToolOffsetConfig:
    """从 config/tool_offset.yaml 加载工具偏移配置喵~

    Args:
        tool_id: 工具 ID (对应 YAML 中的 key), 默认 "default"

    Returns:
        ToolOffsetConfig — 未找到时返回默认值 (pos=(0,0,-0.15), rpy=(0,0,0))
    """
    config_path = os.path.join(_config_dir(), "tool_offset.yaml")
    try:
        with open(config_path, "r") as f:
            data = yaml.safe_load(f)
    except Exception:
        return _default_tool_offset(tool_id)

    tools = data.get("tools", {})
    tool = tools.get(tool_id)
    if tool is None:
        if tool_id == "default":
            return _default_tool_offset(tool_id)
        # 非 default 且未找到 → 警告并回退 default
        import logging
        logging.getLogger("latte_imitation").warning(
            f"工具 '{tool_id}' 未在 tool_offset.yaml 中找到, 回退到 'default'"
        )
        return load_tool_offset("default")

    pos = tool.get("position", {"x": 0.0, "y": 0.0, "z": -0.15})
    ori = tool.get("orientation", {"roll": 0.0, "pitch": 0.0, "yaw": 0.0})
    return ToolOffsetConfig(
        tool_id=tool_id,
        name=tool.get("name", tool_id),
        description=tool.get("description", ""),
        pos=(float(pos["x"]), float(pos["y"]), float(pos["z"])),
        rpy=(float(ori["roll"]), float(ori["pitch"]), float(ori["yaw"])),
    )


def load_workspace_safety() -> WorkspaceSafetyConfig:
    """从 config/workspace_safety.yaml 加载工作空间边界喵~

    Returns:
        WorkspaceSafetyConfig — 未找到时返回默认值
    """
    config_path = os.path.join(_config_dir(), "workspace_safety.yaml")
    try:
        with open(config_path, "r") as f:
            data = yaml.safe_load(f)
    except Exception:
        return _default_workspace_safety()

    return WorkspaceSafetyConfig(
        x_min=float(data.get("x_min", -0.87)),
        x_max=float(data.get("x_max", 0.87)),
        y_min=float(data.get("y_min", -0.87)),
        y_max=float(data.get("y_max", 0.87)),
        z_min=float(data.get("z_min", -0.85)),
        z_max=float(data.get("z_max", 1.10)),
        safety_policy=str(data.get("safety_policy", "warn_and_block")),
    )


def _default_tool_offset(tool_id: str) -> ToolOffsetConfig:
    return ToolOffsetConfig(
        tool_id=tool_id,
        name="默认拉花壶",
        description="Standard 350ml pitcher, spout ~15cm below TCP",
        pos=(0.0, 0.0, -0.15),
        rpy=(0.0, 0.0, 0.0),
    )


def _default_workspace_safety() -> WorkspaceSafetyConfig:
    return WorkspaceSafetyConfig(
        x_min=-0.87, x_max=0.87,
        y_min=-0.87, y_max=0.87,
        z_min=-0.85, z_max=1.10,
        safety_policy="warn_and_block",
    )
