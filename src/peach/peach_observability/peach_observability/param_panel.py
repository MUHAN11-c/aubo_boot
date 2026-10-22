"""调试台参数面板：白名单节点的 ROS 参数读写（只转发 rcl_interfaces 服务）.

设计约束：
  - 只允许显式列出的节点/参数；不暴露任意参数、不做新控制路径.
  - 运动类参数（臂速度/加速度/行程/位姿门）须 `debug.motion_enabled` 放行.
  - 静态参数（启动时读取）标注 `static=true`，设置仍落参但提示需重启.
  - 每次写（含被拒）由 DebugAudit 追加一行；读取不写审计.
  - 观测节点自身参数走本地 Parameter API；其余节点走 /<node>/{list,get,set}_parameters.
"""
from __future__ import annotations

import threading
import time
from typing import Callable

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.parameter import Parameter as RclpyParameter, parameter_value_to_python

PARAM_GROUPS = [
    {
        'node': 'peach_supervisor', 'label': '调度 supervisor', 'params': [
            {'name': 'execute_pregrasp_only', 'type': 'bool', 'label': '只到预抓取'},
            {'name': 'skip_reconstruction', 'type': 'bool', 'label': '跳过重建'},
            {'name': 'reconstruct_in_trajectory', 'type': 'bool', 'label': '轨迹内重建'},
            {'name': 'build_start_timeout_s', 'type': 'double', 'label': 'Build 超时', 'min': 1, 'max': 120},
            {'name': 'selection_depth_min_m', 'type': 'double', 'label': '选果深度下限', 'min': 0, 'max': 3},
            {'name': 'selection_depth_max_m', 'type': 'double', 'label': '选果深度上限', 'min': 0.1, 'max': 5},
            {'name': 'selection_priority', 'type': 'int', 'label': '只抓顺序号（0=不限）'},
        ],
    },
    {
        'node': 'peach_arm', 'label': '臂 peach_arm', 'params': [
            {'name': 'moveit.velocity_scaling', 'type': 'double', 'label': '接触速度缩放', 'min': 0.02, 'max': 1.0, 'motion': True},
            {'name': 'moveit.acceleration_scaling', 'type': 'double', 'label': '接触加速度缩放', 'min': 0.02, 'max': 1.0, 'motion': True},
            {'name': 'moveit.transit_velocity_scaling', 'type': 'double', 'label': '转移速度缩放', 'min': 0.02, 'max': 1.0, 'motion': True},
            {'name': 'moveit.transit_acceleration_scaling', 'type': 'double', 'label': '转移加速度缩放', 'min': 0.02, 'max': 1.0, 'motion': True},
            {'name': 'moveit.execute_timeout_s', 'type': 'double', 'label': '执行超时', 'min': 1, 'max': 120},
            {'name': 'timeouts.refined_s', 'type': 'double', 'label': '精化等待', 'min': 1, 'max': 120},
            {'name': 'quality.allow_unrefined_geometry', 'type': 'bool', 'label': '允许未精化几何'},
            {'name': 'quality.minimum_baseline_deg', 'type': 'double', 'label': '覆盖基线角', 'min': 0, 'max': 90},
            {'name': 'grasp.maximum_travel_m', 'type': 'double', 'label': '最大行程', 'min': 0.01, 'max': 0.30, 'motion': True},
            {'name': 'grasp.minimum_travel_m', 'type': 'double', 'label': '最小行程', 'min': 0.01, 'max': 0.20, 'motion': True},
            {'name': 'grasp.neck_margin_m', 'type': 'double', 'label': '袋颈余量', 'min': 0, 'max': 0.10, 'motion': True},
            {'name': 'moveit.mtc_cartesian_min_fraction', 'type': 'double', 'label': '插入完成比', 'min': 0.5, 'max': 1.0},
            {'name': 'moveit.mtc_approach_along_axis_m', 'type': 'double', 'label': '预抓取后撤', 'min': 0, 'max': 0.20},
            {'name': 'moveit.mtc_approach_max_chord_deviation_m', 'type': 'double', 'label': '接近弦偏离', 'min': 0, 'max': 0.50},
            {'name': 'moveit.mtc_approach_max_detour_ratio', 'type': 'double', 'label': '接近绕行比', 'min': 0, 'max': 5.0},
        ],
    },
    {
        'node': 'peach_scene_perception_node', 'label': '场景感知', 'params': [
            {'name': 'tool.L_insert', 'type': 'double', 'label': 'L_insert', 'min': 0.001, 'max': 0.30},
            {'name': 'tool.L_blade', 'type': 'double', 'label': 'L_blade', 'min': 0, 'max': 0.20},
            {'name': 'tool.margin_neck', 'type': 'double', 'label': '袋颈余量', 'min': 0, 'max': 0.10},
            {'name': 'pipeline.min_depth_m', 'type': 'double', 'label': '最小深度', 'min': 0.1, 'max': 3},
            {'name': 'pipeline.max_depth_m', 'type': 'double', 'label': '最大深度', 'min': 0.1, 'max': 5},
            {'name': 'identity.match_radius_m', 'type': 'double', 'label': '匹配半径', 'min': 0.01, 'max': 0.30},
            {'name': 'identity.recovery_scale', 'type': 'double', 'label': '恢复尺度', 'min': 1.0, 'max': 5.0},
        ],
    },
    {
        'node': 'peach_target_reconstruction_node', 'label': '重建', 'params': [
            {'name': 'capture.auto_mode', 'type': 'bool', 'label': '自动采帧'},
            {'name': 'tool.budget.cut_reference_offset_m', 'type': 'double', 'label': '切点工艺偏置', 'min': -0.03, 'max': 0.03},
        ],
    },
    {
        'node': 'peach_observability', 'label': '观测/调试台', 'params': [
            {'name': 'debug.enabled', 'type': 'bool', 'label': '调试 POST 总开关'},
            {'name': 'debug.motion_enabled', 'type': 'bool', 'label': '运动类放行'},
            {'name': 'record.enabled', 'type': 'bool', 'label': '会话 bag 录制', 'static': True},
            {'name': 'record.level', 'type': 'string', 'label': '录制档', 'choices': ['std', 'all', 'core'], 'static': True},
        ],
    },
]

_TYPE = {
    'bool': ParameterType.PARAMETER_BOOL,
    'int': ParameterType.PARAMETER_INTEGER,
    'double': ParameterType.PARAMETER_DOUBLE,
    'string': ParameterType.PARAMETER_STRING,
}


def _spec(node: str, name: str) -> dict | None:
    for group in PARAM_GROUPS:
        if group['node'] != node:
            continue
        for item in group['params']:
            if item['name'] == name:
                return item
    return None


def _value(spec: dict, raw) -> ParameterValue:
    kind = spec['type']
    if kind == 'bool':
        if not isinstance(raw, bool):
            raise ValueError('需要 bool')
        return ParameterValue(type=_TYPE[kind], bool_value=raw)
    if kind == 'int':
        if isinstance(raw, bool) or not isinstance(raw, int):
            raise ValueError('需要 int')
        return ParameterValue(type=_TYPE[kind], integer_value=raw)
    if kind == 'double':
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError('需要 number')
        number = float(raw)
        if 'min' in spec and number < spec['min']:
            raise ValueError(f"低于下限 {spec['min']}")
        if 'max' in spec and number > spec['max']:
            raise ValueError(f"高于上限 {spec['max']}")
        return ParameterValue(type=_TYPE[kind], double_value=number)
    if kind == 'string':
        if not isinstance(raw, str):
            raise ValueError('需要 string')
        if spec.get('choices') and raw not in spec['choices']:
            raise ValueError(f"只能是 {spec['choices']}")
        return ParameterValue(type=_TYPE[kind], string_value=raw)
    raise ValueError('未知类型')


class ParamBridge:
    """白名单参数读写；客户端在 ROS 线程建立，HTTP 线程只 call_async + 等待."""

    def __init__(self, node, audit, motion_enabled: bool,
                 on_change: Callable[[str, object], None],
                 log_warning: Callable[[str], None]):
        self._node = node
        self._audit = audit
        self._motion_enabled = bool(motion_enabled)
        self._on_change = on_change
        self._log = log_warning
        self._clients: dict = {}
        self._lock = threading.Lock()

    def open_all(self) -> None:
        for group in PARAM_GROUPS:
            node = group['node']
            if node == 'peach_observability':
                continue
            prefix = f'/{node}'
            self._clients[node] = {
                'list': self._node.create_client(ListParameters, prefix + '/list_parameters'),
                'get': self._node.create_client(GetParameters, prefix + '/get_parameters'),
                'set': self._node.create_client(SetParameters, prefix + '/set_parameters'),
            }

    def close(self) -> None:
        for clients in self._clients.values():
            for client in clients.values():
                self._node.destroy_client(client)
        self._clients.clear()

    @staticmethod
    def _wait(future, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.1, float(timeout_s))
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return True

    def _node_params(self, node: str, names: list) -> tuple:
        """读取节点当前参数；返回 (running, values, error)."""
        if node == 'peach_observability':
            values = {p.name: p.value
                      for p in self._node.get_parameters(names)}
            return True, values, ''
        clients = self._clients.get(node)
        if clients is None:
            return False, {}, '未配置该节点'
        try:
            for client in clients.values():
                if not client.wait_for_service(timeout_sec=0.2):
                    return False, {}, '节点未运行/服务未就绪'
            listed = clients['list'].call_async(ListParameters.Request(
                prefixes=[], depth=ListParameters.Request.DEPTH_RECURSIVE))
            if not self._wait(listed, 2.0) or listed.result() is None:
                return False, {}, 'list_parameters 超时'
            present = set(listed.result().result.names)
            wanted = [name for name in names if name in present]
            if not wanted:
                return True, {}, '参数名不在节点声明中'
            got = clients['get'].call_async(GetParameters.Request(names=wanted))
            if not self._wait(got, 2.0) or got.result() is None:
                return False, {}, 'get_parameters 超时'
            values = {name: parameter_value_to_python(value)
                      for name, value in zip(wanted, got.result().values)}
            missing = [name for name in names if name not in values]
            return True, values, ('未声明: ' + ', '.join(missing)) if missing else ''
        except (ValueError, RuntimeError, OSError) as exc:
            return False, {}, f'读取失败: {exc}'

    def report(self) -> dict:
        groups = []
        for group in PARAM_GROUPS:
            node = group['node']
            names = [item['name'] for item in group['params']]
            running, values, error = self._node_params(node, names)
            params = []
            for item in group['params']:
                params.append({
                    **item,
                    'value': values.get(item['name']),
                    'present': item['name'] in values,
                    'node_running': running,
                })
            groups.append({
                'node': node, 'label': group['label'],
                'running': running, 'error': error, 'params': params,
            })
        return {
            'groups': groups,
            'motion_enabled': self._motion_enabled,
            'debug_enabled': bool(self._node.get_parameter('debug.enabled').value),
        }

    def set(self, changes) -> tuple:
        if not isinstance(changes, list) or not changes:
            return 400, {'accepted': False, 'message': 'changes 必须是非空数组'}
        results = []
        accepted = 0
        for change in changes:
            if not isinstance(change, dict):
                results.append({'node': '', 'name': '', 'accepted': False,
                                'message': 'change 须为对象'})
                continue
            node = str(change.get('node') or '')
            name = str(change.get('name') or '')
            spec = _spec(node, name)
            row = {'node': node, 'name': name, 'accepted': False, 'message': ''}
            if spec is None:
                row['message'] = '不在白名单'
                results.append(row)
                continue
            if spec.get('motion') and not self._motion_enabled:
                row['message'] = '运动类参数需先开启 debug.motion_enabled'
                results.append(row)
                continue
            try:
                pv = _value(spec, change.get('value'))
            except (TypeError, ValueError) as exc:
                row['message'] = f'值非法: {exc}'
                results.append(row)
                continue
            if node == 'peach_observability':
                response = self._node.set_parameters(
                    [RclpyParameter(name, value=change.get('value'))])
                ok = bool(response) and all(item.successful for item in response)
                row['accepted'] = ok
                row['message'] = '已设置' if ok else '节点拒绝'
            else:
                clients = self._clients.get(node)
                if clients is None:
                    row['message'] = '未配置该节点'
                    results.append(row)
                    continue
                if not clients['set'].wait_for_service(timeout_sec=0.5):
                    row['message'] = '节点未运行/服务未就绪'
                    results.append(row)
                    continue
                future = clients['set'].call_async(SetParameters.Request(
                    parameters=[Parameter(name=name, value=pv)]))
                if not self._wait(future, 3.0) or future.result() is None:
                    row['message'] = 'set_parameters 超时'
                    results.append(row)
                    continue
                response = future.result().results
                ok = bool(response) and all(item.successful for item in response)
                row['accepted'] = ok
                row['message'] = '已设置' if ok else '节点拒绝'
            if row['accepted']:
                accepted += 1
                if name == 'debug.motion_enabled':
                    self._motion_enabled = bool(change.get('value'))
                self._on_change(name, change.get('value'))
                if spec.get('static'):
                    row['message'] += '（静态参数，可能需重启生效）'
            results.append(row)
        status = 200 if accepted == len(results) else (207 if accepted else 400)
        return status, {
            'accepted': accepted == len(results),
            'accepted_count': accepted,
            'results': results,
            'motion_enabled': self._motion_enabled,
        }
