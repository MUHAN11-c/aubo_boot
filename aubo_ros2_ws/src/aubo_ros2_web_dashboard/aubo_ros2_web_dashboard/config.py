"""IVG 配置中心 — 加载 config/defaults.yaml，提供类型安全的访问器。

链路: YAML 文件 → _load_yaml() → _cfg 字典 → 类型访问器函数
覆盖: CLI 参数通过 init() → _apply_cli() → 写入 _cfg 对应字段

整个进程只有一份 _cfg，所有模块通过访问器函数读取，不直接访问字典。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

# ── YAML 加载 ────────────────────────────────────────────────────────────────

try:
    import yaml as _yaml
    _HAS_YAML = True
except ImportError:
    import json as _json  # 回退：YAML 是 JSON 的超集，纯 JSON 也能读
    _HAS_YAML = False

# 源树路径: <pkg>/config/defaults.yaml
_SRC_ROOT = Path(__file__).resolve().parent.parent
_YAML_PATH = _SRC_ROOT / "config" / "defaults.yaml"

# 安装后路径: share/<pkg>/config/defaults.yaml
if not _YAML_PATH.is_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        _p = Path(get_package_share_directory("aubo_ros2_web_dashboard")) / "config" / "defaults.yaml"
        if _p.is_file():
            _YAML_PATH = _p
    except Exception:
        pass


def _load_yaml() -> dict[str, Any]:
    """读取 YAML 文件，返回配置字典。文件不存在时返回空字典。"""
    if not _YAML_PATH.is_file():
        return {}
    raw = _YAML_PATH.read_text(encoding="utf-8")
    d = _yaml.safe_load(raw) if _HAS_YAML else _json.loads(raw)
    return d if isinstance(d, dict) else {}


# 模块导入时立即加载，确保 argparse 默认值能读到 YAML 值
_cfg: dict[str, Any] = _load_yaml()


# ── CLI 覆盖映射 ─────────────────────────────────────────────────────────────

# CLI 参数名 → YAML 路径（点号分隔），init() 时用于合并覆盖
_CLI_MAP: dict[str, str] = {
    "rosbridge_host":              "rosbridge.host",
    "rosbridge_port":              "rosbridge.port",
    "rosbridge_max_msg_bytes":     "rosbridge.max_message_bytes",
    "rosbridge_ping_interval":     "rosbridge.ping_interval",
    "rosbridge_ping_timeout":      "rosbridge.ping_timeout",
    "rosbridge_close_timeout":     "rosbridge.close_timeout",
    "web_video_host":              "web_video.host",
    "web_video_port":              "web_video.port",
    "proxy_video_connect_timeout": "proxy.video_connect_timeout",
    "proxy_video_pool_timeout":    "proxy.video_pool_timeout",
    "proxy_video_chunk_bytes":     "proxy.video_chunk_bytes",
    "port":                        "gateway.port",
    "bind":                        "gateway.bind",
}


def init(cli_args: dict[str, Any] | None = None) -> None:
    """将 CLI 参数合并到配置中。在 main() 解析完命令行后调用一次。"""
    if cli_args:
        _apply_cli(cli_args)


def _apply_cli(args: dict[str, Any]) -> None:
    """根据 _CLI_MAP 将 CLI 参数值写入 _cfg 对应嵌套字段。"""
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


# ── 通用取值 ─────────────────────────────────────────────────────────────────

def _g(path: str, default=None):
    """按点号路径从 _cfg 中取值，如 'rosbridge.port' → 9090。不存在则返回 default。"""
    c = _cfg
    for k in path.split("."):
        if isinstance(c, dict):
            c = c.get(k)
        else:
            return default
    return c if c is not None else default


# ── RobotWebTools 资产路径 ───────────────────────────────────────────────────

def robotwebtools_search_dirs() -> list[str]:
    """返回 candidate 目录列表，launch 文件按顺序查找。"""
    dirs = _cfg.get("robotwebtools", {}).get("search_dirs")
    return [str(d) for d in dirs] if isinstance(dirs, list) else []


# ── 类型访问器（按模块分组）──────────────────────────────────────────────────

# ---- 网关 ----
def gateway_bind() -> str:              return str(_g("gateway.bind", "0.0.0.0"))
def gateway_port() -> int:              return int(_g("gateway.port", 8090))
def gateway_gzip_min_size() -> int:     return int(_g("gateway.gzip_min_size", 512))

# ---- rosbridge ----
def rosbridge_host() -> str:            return str(_g("rosbridge.host", "127.0.0.1"))
def rosbridge_port() -> int:            return int(_g("rosbridge.port", 9090))
def rosbridge_ws_path() -> str:         return str(_g("rosbridge.ws_path", "/ws/rosbridge"))
def rosbridge_max_message_bytes() -> int: return int(_g("rosbridge.max_message_bytes", 67108864))
def rosbridge_ping_interval() -> int:   return int(_g("rosbridge.ping_interval", 20))
def rosbridge_ping_timeout() -> int:    return int(_g("rosbridge.ping_timeout", 60))
def rosbridge_close_timeout() -> int:   return int(_g("rosbridge.close_timeout", 10))

# ---- web_video ----
def web_video_host() -> str:            return str(_g("web_video.host", "127.0.0.1"))
def web_video_port() -> int:            return int(_g("web_video.port", 8089))
def web_video_listen_address() -> str:  return str(_g("web_video.listen_address", "0.0.0.0"))
def web_video_server_threads() -> int:  return int(_g("web_video.server_threads", 4))
def web_video_ros_threads() -> int:     return int(_g("web_video.ros_threads", 2))
def web_video_proxy_prefix() -> str:    return str(_g("web_video.proxy_path_prefix", "/api/ivg/proxy/web-video"))

# ---- 代理调优 ----
def proxy_video_connect_timeout() -> float: return float(_g("proxy.video_connect_timeout", 15))
def proxy_video_pool_timeout() -> float:    return float(_g("proxy.video_pool_timeout", 10))
def proxy_video_read_timeout() -> float | None:
    v = _g("proxy.video_read_timeout")
    return float(v) if v is not None and float(v) > 0 else None
def proxy_video_chunk_bytes() -> int:
    v = int(_g("proxy.video_chunk_bytes", 65536))
    return v if v >= 4096 else 65536

# ---- 包版本 ----
def package_version_fallback() -> str:  return str(_g("package.version_fallback", "0.4.0"))

# ---- 前端设置页面话题/服务定义 ----
def vision_panel_config() -> dict[str, Any]:
    """返回原始嵌套结构（common/vision/latte 分类）。"""
    return _cfg.get("vision_panel", {})

def _flatten_categories(vp: dict) -> dict:
    """将分类结构扁平化为旧格式 {topics, tf_topics, services, fixed_service_types}。"""
    c = vp.get("common", {})
    v = vp.get("vision", {})
    return {
        "topics": c.get("topics", []) + v.get("topics", []),
        "tf_topics": c.get("tf_topics", []),
        "services": v.get("services", []),
        "fixed_service_types": v.get("fixed_service_types", {}),
    }


# ── 前端运行时接口 (BFF) ─────────────────────────────────────────────────────
# GET /api/v1/runtime 返回的数据，浏览器用它发现 rosbridge/视频代理/话题定义

def runtime_config_dict(static_root: str) -> dict[str, Any]:
    from importlib.metadata import version as _version
    try:
        pkg_ver = _version("aubo_ros2_web_dashboard")
    except Exception:
        pkg_ver = package_version_fallback()
    vp = vision_panel_config()
    return {
        "package": "aubo_ros2_web_dashboard",
        "version": pkg_ver,
        "rosbridge_port": rosbridge_port(),
        "web_video_port": web_video_port(),
        "static_root": static_root,
        "unified_proxy": True,
        "rosbridge_ws_path": rosbridge_ws_path(),
        "web_video_proxy_prefix": web_video_proxy_prefix(),
        "vision_panel": _flatten_categories(vp),         # 旧格式（视觉抓取面板用）
        "settings_categories": vp,                        # 新格式（设置页用，含 label）
    }


# ── 设置持久化 ───────────────────────────────────────────────────────────

def save_settings_to_yaml(settings: dict[str, Any]) -> bool:
    """将前端提交的设置写入 config/defaults.yaml 的 vision_panel 段。

    只更新 topics/tf_topics/services 中各项的 default 字段，
    保留 label/msg_type/srv_type 等元数据不变。
    同时更新内存 _cfg 使修改即时生效。
    """
    if not _YAML_PATH.is_file():
        return False

    full = _load_yaml()
    vp = full.get("vision_panel", {})

    # 遍历所有分类和子列表，更新 default 值
    for _cat_name, cat_val in vp.items():
        if not isinstance(cat_val, dict):
            continue
        for _list_key in ("topics", "tf_topics", "services"):
            items = cat_val.get(_list_key)
            if not isinstance(items, list):
                continue
            for item in items:
                if isinstance(item, dict) and item.get("id") in settings:
                    item["default"] = settings[item["id"]]

    # 写回文件
    try:
        raw = _YAML_PATH.read_text(encoding="utf-8")
        if _HAS_YAML:
            _yaml.safe_dump(full, _YAML_PATH.open("w", encoding="utf-8"),
                           allow_unicode=True, default_flow_style=False, sort_keys=False)
        else:
            _YAML_PATH.write_text(_json.dumps(full, indent=2, ensure_ascii=False), encoding="utf-8")
    except Exception:
        return False

    # 更新内存
    global _cfg
    _cfg = full
    return True
