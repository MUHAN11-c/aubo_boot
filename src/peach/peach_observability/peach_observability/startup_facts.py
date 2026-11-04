"""
启动事实（startup facts）规范化纯核：launch 采集 → 会话工件/自检.

harvest_system.launch 在展开期把解析后的启动参数 + git 版本 + 域 ID 汇成
JSON 注入 observability；本模块负责规范化（键白名单、标量化、派生
require_robot_status），供 startup.json 落盘与参数一致性检查共用。
"""
from __future__ import annotations

import json
from typing import Any, Mapping

# 键白名单：进 startup.json 的启动事实（顺序即工件展示序）
FACT_KEYS = (
    'launched_at', 'hardware_mode', 'robot_ip', 'tool_profile',
    'moveit_enabled', 'move_group_shutdown_workaround',
    'rosbridge_enabled', 'rosbridge_port',
    'camera_enabled', 'camera_frontend', 'camera_ip',
    'extrinsics_enabled', 'force_enabled', 'bond_timeout', 'use_sim_time',
    'reconstruction_enabled', 'skip_reconstruction', 'autostart', 'require_robot_status',
    'expected_tcp_norm_m', 'ros_domain_id', 'git',
)


def parse_facts_json(raw: str) -> dict:
    """Decode the optional launch facts supplied to observability."""
    try:
        facts = json.loads(raw or '{}')
    except ValueError:
        return {}
    return facts if isinstance(facts, dict) else {}


def flag_enabled(value) -> bool:
    """Interpret the boolean spellings accepted by Jazzy IfCondition."""
    return str(value).strip().lower() in ('true', '1')


def normalize_facts(raw: Mapping | None) -> dict[str, Any]:
    """键白名单过滤 + 标量规范化；派生 require_robot_status（缺省时）."""
    raw = raw or {}
    facts: dict[str, Any] = {}
    for key in FACT_KEYS:
        if key in raw and raw[key] is not None:
            value = raw[key]
            facts[key] = value if isinstance(value, (dict, list, int, float)) \
                else str(value)
    if 'require_robot_status' not in facts and 'hardware_mode' in facts:
        facts['require_robot_status'] = (
            'false' if facts['hardware_mode'] == 'mock' else 'true')
    return facts


def facts_valid(facts: Mapping) -> bool:
    """工件落盘前的最低完整性：hardware_mode 必须在场且取值合法."""
    return facts.get('hardware_mode') in ('mock', 'real')


def selfcheck_overrides_from_facts(facts: Mapping) -> dict[str, Any]:
    """
    从启动事实推导自检开关覆盖（launch 只经 facts 一条通道注入）.

    覆盖项与 ObservabilityParams 字段同名；facts 为空（独立起栈）时
    不覆盖，维持 yaml 基线。
    """
    overrides: dict[str, Any] = {}
    if not facts:
        return overrides
    if 'camera_enabled' in facts:
        overrides['selfcheck_camera_probe_enabled'] = flag_enabled(facts['camera_enabled'])
    if 'require_robot_status' in facts:
        overrides['robot_status_probe_enabled'] = flag_enabled(facts['require_robot_status'])
    if 'moveit_enabled' in facts:
        overrides['selfcheck_moveit_expected'] = flag_enabled(facts['moveit_enabled'])
    if 'force_enabled' in facts:
        overrides['selfcheck_force_expected'] = flag_enabled(facts['force_enabled'])
    tcp = facts.get('expected_tcp_norm_m')
    if isinstance(tcp, (int, float)) and not isinstance(tcp, bool) \
            and float(tcp) > 0.0:
        overrides['selfcheck_expected_tcp_norm_m'] = float(tcp)
    return overrides
