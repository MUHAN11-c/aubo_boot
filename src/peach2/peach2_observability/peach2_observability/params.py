"""config/observability.yaml loader with explicit validation (no ROS)."""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

import yaml


@dataclass(frozen=True)
class SessionBagParams:
    enabled: bool
    sigint_timeout_s: float
    term_timeout_s: float


@dataclass(frozen=True)
class ObservabilityParams:
    host: str
    port: int
    runs_root: str
    session_bag: SessionBagParams


_RULES: dict[str, tuple] = {
    'host': str,
    'port': int,
    'runs_root': str,
}


def _validate_session_bag(raw: dict) -> SessionBagParams:
    if not isinstance(raw, dict):
        raise ValueError('session_bag must be a mapping')
    for key in ('enabled', 'sigint_timeout_s', 'term_timeout_s'):
        if key not in raw:
            raise ValueError(f'session_bag missing key {key!r}')
    sigint = float(raw['sigint_timeout_s'])
    term = float(raw['term_timeout_s'])
    if sigint <= 0.0 or term <= 0.0:
        raise ValueError('session_bag timeouts must be positive')
    return SessionBagParams(
        enabled=bool(raw['enabled']),
        sigint_timeout_s=sigint,
        term_timeout_s=term,
    )


def params_from_dict(raw: dict) -> ObservabilityParams:
    """Validate a flat config mapping (under peach2_observability or root)."""
    if not isinstance(raw, dict):
        raise ValueError('config root must be a mapping')
    bag = _validate_session_bag(raw.get('session_bag') or {})
    for key, typ in _RULES.items():
        if key not in raw:
            raise ValueError(f'missing key {key!r}')
        if not isinstance(raw[key], typ):
            raise ValueError(f'{key!r} must be {typ.__name__}')
    port = int(raw['port'])
    if port < 1 or port > 65535:
        raise ValueError('port out of range')
    host = str(raw['host']).strip()
    if not host:
        raise ValueError('host must be non-empty')
    return ObservabilityParams(
        host=host,
        port=port,
        runs_root=str(raw['runs_root']),
        session_bag=bag,
    )


def load_params(path: str | Path) -> ObservabilityParams:
    with open(path, 'r', encoding='utf-8') as handle:
        document = yaml.safe_load(handle) or {}
    if 'peach2_observability' in document:
        document = document['peach2_observability']
    return params_from_dict(document)


def resolve_runs_root(explicit: str, *, cwd: Path | None = None) -> Path:
    """Resolve the runs directory for ledgers and session bags."""
    text = str(explicit or '').strip()
    if text:
        return Path(text).expanduser().resolve()
    env = os.environ.get('PEACH_RUNS_ROOT', '').strip()
    if env:
        return Path(env).expanduser().resolve()
    return (cwd or Path.cwd()) / 'runs'
