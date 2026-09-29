"""
启动自检纯核（零 ROS）：检查函数与汇总裁决.

runner（selfcheck/runner.py）负责 ROS 接线（订阅探针/服务查询/工件落盘），
本模块只做数据 → CheckResult 的判定，全部可 pytest。

状态词表：pass / warn / fail / skip。汇总规则见 evaluate().
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping, Sequence

PASS = 'pass'
WARN = 'warn'
FAIL = 'fail'
SKIP = 'skip'

# MUST 关节序（AGENTS 第 1 章冻结；配置可覆盖但默认即权威序）
DEFAULT_JOINT_ORDER = (
    'shoulder_joint', 'upperArm_joint', 'foreArm_joint',
    'wrist1_joint', 'wrist2_joint', 'wrist3_joint',
)

# 各硬件模式期望 active 的控制器（controller_manager 语义）
EXPECTED_CONTROLLERS = {
    'mock': ('joint_state_broadcaster', 'joint_trajectory_controller'),
    'real': (
        'joint_state_broadcaster', 'aubo_io_controller',
        'aubo_passthrough_trajectory_controller'),
}


@dataclass
class CheckResult:
    """单项检查结论；value 携带机器可比对的关键数值."""

    name: str
    status: str
    detail: str = ''
    value: object = None

    def to_dict(self) -> dict:
        """序列化为 JSON 友好形态（工件/8090 共用）."""
        return {
            'name': self.name,
            'status': self.status,
            'detail': self.detail,
            'value': self.value,
        }


def check_joint_order(names: Sequence[str] | None,
                      expected: Sequence[str] = DEFAULT_JOINT_ORDER) -> CheckResult:
    """
    /joint_states 六关节名集合须与 MUST 冻结集一致.

    名单序不判：JSB 发布按字母序、命令序在 controllers.yaml（与 CI
    test_mock_launch 同语义——缺/多关节才是 FAIL，透传拧腕风险在驱动侧序）.
    """
    if not names:
        return CheckResult('joint_order', FAIL, '未收到 joint_states', None)
    names = list(names)
    missing = [j for j in expected if j not in names]
    extra = [j for j in names if j not in expected]
    if missing or extra:
        detail = f'缺 {missing} 多 {extra}: {names}'
        return CheckResult('joint_order', FAIL, detail, names)
    return CheckResult('joint_order', PASS, '六关节名集合一致', names)


def check_min_rate(count: int, window_s: float, min_hz: float,
                   name: str, skip_reason: str = '') -> CheckResult:
    """窗口内计数折算帧率 ≥ min_hz；count is None 表示该项不适用（SKIP）."""
    if count is None:
        return CheckResult(name, SKIP, skip_reason or '未启用', None)
    rate = count / window_s if window_s > 0 else 0.0
    if rate + 1e-9 < min_hz:
        return CheckResult(
            name, FAIL, f'帧率 {rate:.2f}Hz < {min_hz}Hz', round(rate, 3))
    return CheckResult(name, PASS, f'帧率 {rate:.2f}Hz', round(rate, 3))


def check_fresh(age_s: float | None, max_age_s: float, name: str,
                skip_reason: str = '') -> CheckResult:
    """最近一条消息年龄 ≤ max_age_s；age_s None 且有 skip_reason → SKIP."""
    if age_s is None:
        if skip_reason:
            return CheckResult(name, SKIP, skip_reason, None)
        return CheckResult(name, FAIL, '从未收到消息', None)
    if age_s > max_age_s:
        return CheckResult(
            name, FAIL, f'数据陈旧 {age_s:.1f}s > {max_age_s}s',
            round(age_s, 2))
    return CheckResult(name, PASS, f'新鲜 {age_s:.1f}s', round(age_s, 2))


def check_controllers(states: Mapping[str, str] | None,
                      expected: Iterable[str]) -> CheckResult:
    """期望控制器全部 active；states None=查询未返回（WARN 不 FAIL）."""
    expected = list(expected)
    if states is None:
        return CheckResult(
            'controllers', WARN, 'controller_manager 查询未返回', None)
    missing = [
        name for name in expected
        if states.get(name) != 'active']
    if missing:
        detail = ', '.join(
            f'{name}={states.get(name, "absent")}' for name in missing)
        return CheckResult('controllers', FAIL, detail, dict(states))
    return CheckResult('controllers', PASS, '期望控制器均 active', dict(states))


def check_disk_free(free_bytes: int | None, min_bytes: int) -> CheckResult:
    """校验 runs 根所在盘剩余空间须容纳 bag 预算."""
    if free_bytes is None:
        return CheckResult('disk_free', SKIP, '无法读取磁盘用量', None)
    if free_bytes < min_bytes:
        return CheckResult(
            'disk_free', FAIL,
            f'剩余 {free_bytes / (1 << 30):.1f}GB < {min_bytes / (1 << 30):.1f}GB',
            free_bytes)
    return CheckResult(
        'disk_free', PASS, f'剩余 {free_bytes / (1 << 30):.1f}GB', free_bytes)


def check_param_consistency(facts: Mapping) -> CheckResult:
    """启动事实互斥校验：mock↔require_robot_status、sim_time 等组合告警."""
    problems = []
    mode = str(facts.get('hardware_mode', ''))
    require_rs = str(facts.get('require_robot_status', ''))
    if mode == 'mock' and require_rs == 'true':
        problems.append('mock 模式却要求 robot_status（Survey 必挂）')
    if mode == 'real' and require_rs == 'false':
        problems.append('真机模式未要求 robot_status（安全门被旁路）')
    if str(facts.get('use_sim_time', 'false')) == 'true' \
            and mode == 'real':
        problems.append('真机禁止 use_sim_time=true')
    if str(facts.get('camera_enabled', 'false')) == 'true' and not any(
            str(facts.get(key, '')) for key in ('camera_ip',)):
        pass  # 前端 percipio 无 camera_ip 参数，不告警
    if problems:
        return CheckResult(
            'param_consistency', FAIL, '；'.join(problems), problems)
    return CheckResult('param_consistency', PASS, '启动参数组合自洽', None)


def check_tcp_norm(norm_m: float | None, expected_m: float,
                   tol_m: float = 0.002) -> CheckResult:
    """wrist3_Link→tcp 平移模长对工具档案（±2mm；-1=跳过）."""
    if expected_m <= 0.0:
        return CheckResult('tcp_frame', SKIP, '未提供档案期望值', None)
    if norm_m is None:
        return CheckResult('tcp_frame', FAIL, 'TF wrist3_Link→tcp 不可用', None)
    if abs(norm_m - expected_m) > tol_m:
        return CheckResult(
            'tcp_frame', FAIL,
            f'|tcp|={norm_m * 1000:.1f}mm ≠ 档案 {expected_m * 1000:.1f}mm',
            round(norm_m, 5))
    return CheckResult(
        'tcp_frame', PASS, f'|tcp|={norm_m * 1000:.1f}mm 对档案',
        round(norm_m, 5))


def check_present(flag: bool | None, name: str, ok_detail: str,
                  missing_detail: str, skip_reason: str = '') -> CheckResult:
    """布尔存在性检查；None=不适用（SKIP）."""
    if flag is None:
        return CheckResult(name, SKIP, skip_reason or '未启用', None)
    if flag:
        return CheckResult(name, PASS, ok_detail, True)
    return CheckResult(name, FAIL, missing_detail, False)


def evaluate(results: Sequence[CheckResult]) -> dict:
    """
    汇总裁决：任一 FAIL→fail；否则任一 WARN→warn；否则 pass.

    passed 语义（autostart 硬等门）：无 FAIL 即 True（WARN 不拦自动批，
    例如首轮 controller 查询未返回）。
    """
    failed = [r.name for r in results if r.status == FAIL]
    warned = [r.name for r in results if r.status == WARN]
    skipped = [r.name for r in results if r.status == SKIP]
    passed_count = sum(1 for r in results if r.status == PASS)
    if failed:
        status = 'fail'
    elif warned:
        status = 'warn'
    else:
        status = 'pass'
    summary = (
        f'{passed_count}/{len(results)} 通过'
        + (f'，失败: {", ".join(failed)}' if failed else '')
        + (f'，警告: {", ".join(warned)}' if warned else '')
        + (f'，跳过: {", ".join(skipped)}' if skipped else ''))
    return {
        'status': status,
        'passed': not failed,
        'failed': failed,
        'warned': warned,
        'skipped': skipped,
        'summary': summary,
    }


@dataclass
class RateProbe:
    """滑动窗计数探针的纯核侧（runner 喂时间戳，检查读窗口计数）."""

    window_s: float = 5.0
    timestamps: list = field(default_factory=list)

    def push(self, now: float) -> None:
        """记录一次到达（monotonic 秒）."""
        self.timestamps.append(now)
        self._trim(now)

    def _trim(self, now: float) -> None:
        cutoff = now - max(self.window_s, 0.1) * 4.0
        self.timestamps = [t for t in self.timestamps if t >= cutoff]

    def count(self, now: float) -> int:
        """当前窗口内到达次数."""
        cutoff = now - self.window_s
        self._trim(now)
        return sum(1 for t in self.timestamps if t >= cutoff)

    def age(self, now: float) -> float | None:
        """最近一次到达的年龄；从未到达为 None."""
        return (now - self.timestamps[-1]) if self.timestamps else None
