"""
Load a ROS node yaml and attach it as nested attributes on an rclpy node.

``config/<node>.yaml`` is the fact source. ``attach(node, path)`` declares every
leaf so ``ros2 param set`` works, and returns a SimpleNamespace tree that
mutates in place on a successful set. Nested yaml groups and dotted keys
(``leaf.exg_min``) both become ``params.leaf.exg_min``.

Single source since W1 (``peach_common``); leaf packages keep their old
import paths as shims that re-export from here.
"""
from __future__ import annotations

import copy
from pathlib import Path
import re
from types import SimpleNamespace
from typing import Any, Callable, Iterable, Optional

import yaml

_SHARE = re.compile(r'\$\(find-pkg-share\s+([^)]+)\)')

ValidateFn = Callable[[str, Any], Optional[str]]
PreviewFn = Callable[[SimpleNamespace], None]
CommitFn = Callable[[SimpleNamespace], None]


def flatten_leaves(obj: Any, prefix: str = '') -> Iterable[tuple[str, Any]]:
    """Yield (dotted_key, leaf) pairs from a nested mapping."""
    if isinstance(obj, dict):
        for name, child in obj.items():
            key = f'{prefix}.{name}' if prefix else str(name)
            if isinstance(child, dict):
                yield from flatten_leaves(child, key)
            else:
                yield key, child
        return
    if prefix:
        yield prefix, obj


def load_ros_parameters(path: Path | str, node_name: str) -> dict:
    """Return the ``ros__parameters`` mapping for ``node_name`` from yaml."""
    path = Path(path)
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(document, dict):
        raise ValueError(f'{path}: expected a mapping')
    block = document.get(node_name)
    if not isinstance(block, dict) or 'ros__parameters' not in block:
        hits = [
            value['ros__parameters']
            for value in document.values()
            if isinstance(value, dict) and 'ros__parameters' in value]
        if block is None and len(hits) == 1:
            return hits[0]
        raise KeyError(f'{path}: missing {node_name}.ros__parameters')
    params = block['ros__parameters']
    if not isinstance(params, dict):
        raise ValueError(f'{path}: ros__parameters must be a mapping')
    return params


def leaf_keys(path: Path | str, node_name: str) -> set[str]:
    """Dotted leaf keys in a deployment yaml (no substitution)."""
    return {key for key, _ in flatten_leaves(load_ros_parameters(path, node_name))}


def set_dotted(root: Any, dotted: str, value: Any) -> None:
    """Write ``a.b.c`` onto nested SimpleNamespace objects."""
    current = root
    parts = dotted.split('.')
    for part in parts[:-1]:
        child = getattr(current, part, None)
        if child is None:
            child = SimpleNamespace()
            setattr(current, part, child)
        current = child
    setattr(current, parts[-1], value)


def dict_to_ns(spec: dict) -> SimpleNamespace:
    """Build a nested SimpleNamespace from a ros__parameters mapping."""
    root = SimpleNamespace()
    for key, value in flatten_leaves(spec):
        set_dotted(root, key, value)
    return root


def expand_share(value: Any) -> Any:
    """Replace ``$(find-pkg-share pkg)`` in strings; recurse into lists/dicts."""
    if isinstance(value, str):
        def _replace(match: re.Match) -> str:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory(match.group(1).strip())
        return _SHARE.sub(_replace, value)
    if isinstance(value, list):
        return [expand_share(item) for item in value]
    if isinstance(value, dict):
        return {key: expand_share(child) for key, child in value.items()}
    return value


def package_yaml(package: str, filename: str) -> Path:
    """``share/<package>/config/<filename>``."""
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package)) / 'config' / filename


def attach(
        node,
        yaml_path: Path | str,
        *,
        node_name: str | None = None,
        validate: ValidateFn | None = None,
        preview: PreviewFn | None = None,
        on_commit: CommitFn | None = None) -> SimpleNamespace:
    """
    Declare yaml leaves on ``node`` and return a live nested namespace.

    Launch ``ParameterFile`` overlays win over yaml defaults. ``validate``
    returns a rejection string or None. ``preview(trial)`` may raise to reject
    a whole set-batch (cross-field checks). ``on_commit`` runs after a
    successful runtime set, not at startup.
    """
    from rcl_interfaces.msg import SetParametersResult
    from rclpy.exceptions import InvalidParameterValueException

    node_name = node_name or node.get_name()
    spec = expand_share(load_ros_parameters(yaml_path, node_name))
    leaves = list(flatten_leaves(spec))
    keyset = {key for key, _ in leaves}
    ns = SimpleNamespace()
    for key, default in leaves:
        if not node.has_parameter(key):
            node.declare_parameter(key, default)
        value = node.get_parameter(key).value
        why = validate(key, value) if validate else None
        if why:
            raise InvalidParameterValueException(key, value, why)
        set_dotted(ns, key, value)
    if preview is not None:
        preview(ns)

    def _on_set(parameters):
        ours = [item for item in parameters if item.name in keyset]
        if not ours:
            return SetParametersResult(successful=True)
        for item in ours:
            if validate is not None:
                why = validate(item.name, item.value)
                if why:
                    return SetParametersResult(successful=False, reason=why)
        trial = copy.deepcopy(ns)
        for item in ours:
            set_dotted(trial, item.name, item.value)
        if preview is not None:
            try:
                preview(trial)
            except (TypeError, ValueError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        for item in ours:
            set_dotted(ns, item.name, item.value)
        if on_commit is not None:
            on_commit(ns)
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(_on_set)
    return ns
