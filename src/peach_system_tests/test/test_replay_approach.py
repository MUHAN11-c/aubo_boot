#!/usr/bin/env python3
"""接近/护栏解析回放塔：确定性语料对照冻结基线。

三层语料（零 ROS、不规划关节、不执臂）：
1. field——现场真袋案册（peach_arm/config/field_pregrasp_cases.yaml
   的 targets_20260909，14 例）；
2. stratified——分层合成 200 例（seed 20260911，09-11 mock typical 同源）；
3. random——随机 100 例（seed 20260910，09-10 接近重写同 seed）。

analytic_ok 是全链路成功率的下界（joint_travel / PTP 绕行 / IK 自碰不在此
层观测）；lin_chord_fail（拍照→staging 直连弦穿囊）是 LIN 对照传感器，
降=护栏变松、升=变紧。改接近/护栏/包络任一处必跑本门。
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import pytest  # noqa: E402

import replay_oracle as ro  # noqa: E402

BASELINES = json.loads(
    (Path(__file__).resolve().parent / 'replay_baselines.json').read_text())


def _stats(cases, book):
    rows = ro.evaluate_all(cases, book)
    typical = [r for r in rows.values() if r['typical']]
    return {
        'n': len(rows),
        'ok': sum(1 for r in rows.values() if r['analytic_ok']),
        'lin_chord_fail': sum(
            1 for r in rows.values() if not r['staging_chord_keepout_ok']),
        'lin_fallback_ok': sum(
            1 for r in rows.values() if r['lin_fallback_ok']),
        'typical_n': len(typical),
        'typical_ok': sum(1 for r in typical if r['analytic_ok']),
    }


def _assert_tier(tag, got, want):
    # 分母精确：采样器或案册漂移必须显式重封基线，不许静默混过。
    assert got['n'] == want['n'], (
        f'{tag}: 语料规模 {got["n"]} != 基线 {want["n"]}（采样器/案册漂移，'
        '重跑生成器并显式重封 replay_baselines.json）')
    assert got['typical_n'] == want['typical_n'], (
        f'{tag}: typical 子集 {got["typical_n"]} != {want["typical_n"]}')
    # analytic 只许升不许降。
    assert got['ok'] >= want['ok'], (
        f'{tag}: analytic_ok {got["ok"]}/{got["n"]} 低于基线 '
        f'{want["ok"]}/{want["n"]}')
    assert got['typical_ok'] >= want['typical_ok'], (
        f'{tag}: typical_ok {got["typical_ok"]} 低于基线 {want["typical_ok"]}')
    # LIN 对照传感器：双侧容差 1（降=护栏变松，升=变紧）。
    assert abs(got['lin_chord_fail'] - want['lin_chord_fail']) <= 1, (
        f'{tag}: lin_chord_fail {got["lin_chord_fail"]} 偏离基线 '
        f'{want["lin_chord_fail"]} 超容差 1（护栏数学/常数变动，须重封基线）')
    assert abs(got['lin_fallback_ok'] - want['lin_fallback_ok']) <= 2, (
        f'{tag}: lin_fallback_ok {got["lin_fallback_ok"]} 偏离基线 '
        f'{want["lin_fallback_ok"]} 超容差 2')


@pytest.fixture(scope='module')
def book():
    return ro.load_casebook()


def test_field_casebook(book):
    templates = book['targets_20260909']
    _assert_tier('field', _stats(ro.field_cases(templates), book),
                 BASELINES['field'])


def test_stratified_200_seed20260911(book):
    templates = book['targets_20260909']
    photo = ro._photo_tcp(book)
    box = ro._workspace(templates, photo)
    cases = ro.sample_stratified(200, 20260911, templates, photo, box)
    got = _stats(cases, book)
    _assert_tier('stratified_200', got, BASELINES['stratified_200_seed20260911'])
    want = BASELINES['stratified_200_seed20260911']['strata']
    by_stratum = {}
    for row in ro.evaluate_all(cases, book).values():
        s = by_stratum.setdefault(row['stratum'], {'n': 0, 'ok': 0})
        s['n'] += 1
        s['ok'] += 1 if row['analytic_ok'] else 0
    for name, w in want.items():
        g = by_stratum.get(name, {'n': 0, 'ok': 0})
        assert g['n'] == w['n'], (
            f'stratified_200/{name}: 分层规模 {g["n"]} != {w["n"]}')
        assert g['ok'] >= w['ok'], (
            f'stratified_200/{name}: ok {g["ok"]}/{g["n"]} 低于基线 '
            f'{w["ok"]}/{w["n"]}')


def test_random_100_seed20260910(book):
    templates = book['targets_20260909']
    photo = ro._photo_tcp(book)
    cases = ro.sample_random_cases(templates, 100, 20260910, photo)
    _assert_tier('random_100', _stats(cases, book),
                 BASELINES['random_100_seed20260910'])
