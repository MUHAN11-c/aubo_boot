"""ros_web_link 配置中心 — 仅网关段（不含 UI 面板配置）.

链路: YAML → _cfg → 类型安全访问器。UI 面板的 defaults.yaml 段（vision_panel 等）
由 aubo_ros2_web_dashboard 的 BFF 子路由自行管理。
"""
from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Any

try:
    import yaml as _yaml
    _HAS_YAML = True
except ImportError:
    _yaml = None
    _HAS_YAML = False
    logging.getLogger("gateway.config").warning(
        "PyYAML 不可用，网关配置回退硬编码默认值（pip install -r requirements.txt）")

_SRC_ROOT = Path(__file__).resolve().parent.parent
_YAML_PATH = _SRC_ROOT / "config" / "defaults.yaml"

if not _YAML_PATH.is_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        _p = Path(get_package_share_directory("ros_web_link")) / "config" / "defaults.yaml"
        if _p.is_file():
            _YAML_PATH = _p
    except Exception:
        pass


def _load_yaml() -> dict[str, Any]:
    if not _YAML_PATH.is_file():
        return {}
    raw = _YAML_PATH.read_text(encoding="utf-8")
    if not _HAS_YAML or _yaml is None:
        return {}
    d = _yaml.safe_load(raw)
    return d if isinstance(d, dict) else {}


_cfg: dict[str, Any] = _load_yaml()

_CLI_MAP: dict[str, str] = {
    "foxglove_bridge_host": "foxglove_bridge.host",
    "foxglove_bridge_port": "foxglove_bridge.port",
    "foxglove_bridge_ws_path": "foxglove_bridge.ws_path",
    "foxglove_bridge_compression": "foxglove_bridge.compression",
    "foxglove_bridge_max_msg_bytes": "foxglove_bridge.max_message_bytes",
    "foxglove_bridge_ping_interval": "foxglove_bridge.ping_interval",
    "foxglove_bridge_ping_timeout": "foxglove_bridge.ping_timeout",
    "foxglove_bridge_close_timeout": "foxglove_bridge.close_timeout",
    "gateway_ws_ping_interval": "gateway.ws_ping_interval",
    "gateway_ws_ping_timeout": "gateway.ws_ping_timeout",
    "gateway_ws_per_message_deflate": "gateway.ws_per_message_deflate",
    "proxy_upstream_connect_attempts": "proxy.upstream_connect_attempts",
    "proxy_upstream_connect_retry_ms": "proxy.upstream_connect_retry_ms",
    "web_video_host": "web_video.host",
    "web_video_port": "web_video.port",
    "proxy_video_connect_timeout": "proxy.video_connect_timeout",
    "proxy_video_pool_timeout": "proxy.video_pool_timeout",
    "proxy_video_chunk_bytes": "proxy.video_chunk_bytes",
    "port": "gateway.port",
    "bind": "gateway.bind",
}


def init(cli_args: dict[str, Any] | None = None) -> None:
    if cli_args:
        _apply_cli(cli_args)


def _apply_cli(args: dict[str, Any]) -> None:
    for key, val in args.items():
        if val is None:
            continue
        path = _CLI_MAP.get(key)
        if not path:
            continue
        parts = path.split(".")
        if len(parts) == 2:
            section, field = parts
            if section in _cfg and isinstance(_cfg[section], dict):
                _cfg[section][field] = val


def _g(path: str, default=None):
    c = _cfg
    for k in path.split("."):
        if isinstance(c, dict):
            c = c.get(k)
        else:
            return default
    return c if c is not None else default


# ---- 网关 ----
def gateway_bind() -> str:
    return str(_g("gateway.bind", "0.0.0.0"))


def gateway_port() -> int:
    return int(_g("gateway.port", 8095))


def gateway_gzip_min_size() -> int:
    return int(_g("gateway.gzip_min_size", 512))


def gateway_ws_ping_interval() -> float | None:
    """客户端腿 WS 心跳间隔（秒）。None=关闭（旧行为，半开连接不可检测，不建议）."""
    v = _g("gateway.ws_ping_interval", 20)
    return None if v is None else float(v)


def gateway_ws_ping_timeout() -> float | None:
    v = _g("gateway.ws_ping_timeout", 20)
    return None if v is None else float(v)


def gateway_ws_per_message_deflate() -> bool:
    """客户端腿 permessage-deflate。CDR 二进制压缩收益差 CPU 代价高，默认关
    （对齐 foxglove_bridge 官方 use_compression=false 建议）。"""
    return bool(_g("gateway.ws_per_message_deflate", False))


# ---- foxglove_bridge ----
def foxglove_bridge_host() -> str:
    return str(_g("foxglove_bridge.host", "127.0.0.1"))


def foxglove_bridge_port() -> int:
    return int(_g("foxglove_bridge.port", 8765))


def foxglove_bridge_ws_path() -> str:
    """上游桥 WebSocket 路径。vendored ivg_foxglove_bridge 服务于根路径 "/"."""
    return str(_g("foxglove_bridge.ws_path", "/"))


def foxglove_bridge_compression() -> bool:
    """上游腿 permessage-deflate（默认关，理由同 gateway_ws_per_message_deflate）."""
    return bool(_g("foxglove_bridge.compression", False))


def foxglove_bridge_max_message_bytes() -> int:
    return int(_g("foxglove_bridge.max_message_bytes", 134217728))


def foxglove_bridge_ping_interval() -> int:
    return int(_g("foxglove_bridge.ping_interval", 20))


def foxglove_bridge_ping_timeout() -> int:
    return int(_g("foxglove_bridge.ping_timeout", 60))


def foxglove_bridge_close_timeout() -> int:
    return int(_g("foxglove_bridge.close_timeout", 10))


# ---- web_video ----
def web_video_host() -> str:
    return str(_g("web_video.host", "127.0.0.1"))


def web_video_port() -> int:
    return int(_g("web_video.port", 8089))


# ---- proxy.video ----
def proxy_video_connect_timeout() -> float:
    return float(_g("proxy.video_connect_timeout", 10.0))


def proxy_video_read_timeout() -> float:
    return float(_g("proxy.video_read_timeout", 30.0))


def proxy_video_pool_timeout() -> float:
    return float(_g("proxy.video_pool_timeout", 10.0))


def proxy_video_chunk_bytes() -> int:
    return int(_g("proxy.video_chunk_bytes", 65536))


# ---- proxy.upstream（WS 上游连接重试）----
def proxy_upstream_connect_attempts() -> int:
    """上游连接总尝试次数（含首次；1=不重试）."""
    return max(1, int(_g("proxy.upstream_connect_attempts", 3)))


def proxy_upstream_connect_retry_ms() -> float:
    """重试基础间隔（毫秒），实际间隔加 ±50% 抖动."""
    return float(_g("proxy.upstream_connect_retry_ms", 250))
