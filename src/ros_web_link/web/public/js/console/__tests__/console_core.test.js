// console/__tests__/console_core.test.js — 深读移植轮纯核单测
// 覆盖：命令总线（bus）、DLS IK 伺服/包络/航点（ikcore）、绘图计算字段（plot 纯核）

import { describe, it, expect, beforeEach } from 'vitest';

import { emitCommand, emitJointBatch, onCommand, _resetBus } from '../bus.js';
import {
  buildTrajectoryPoints,
  clampToWorkspaceEnvelope,
  createIKSolver,
  createTeachRecorder,
  estimateWorkspaceEnvelope,
  solve3x3,
} from '../ikcore.js';
import { extract, parseComputed, parseFields } from '../plot.js';
import {
  covarianceEllipse,
  createJointMotion,
  createOdomTrail,
  createPresetStore,
  easeInOutCubic,
  easeInOutQuad,
  replaySegmentDuration,
  sampleSegment,
} from '../viz_core.js';

// ── 命令总线 ───────────────────────────────────────────────────────────────

describe('bus', () => {
  beforeEach(() => _resetBus());

  it('浅拷贝分发 + source/stamp 附着', () => {
    const seen = [];
    onCommand((cmd) => seen.push(cmd));
    emitCommand({ type: 'joint', name: 'j0', value: 0.5, source: 'slider' });
    expect(seen).toHaveLength(1);
    expect(seen[0].source).toBe('slider');
    expect(typeof seen[0].stamp).toBe('number');
    // 浅拷贝：修改收到的信封不影响后续
    seen[0].extra = true;
    emitCommand({ type: 'joint', source: 'drag' });
    expect(seen[1].extra).toBeUndefined();
  });

  it('单个坏监听者不破坏循环', () => {
    const ok = [];
    onCommand(() => { throw new Error('boom'); });
    onCommand((cmd) => ok.push(cmd.type));
    emitCommand({ type: 'x', source: 'user' });
    expect(ok).toEqual(['x']);
  });

  it('emitJointBatch 过滤非数值 + 计数', () => {
    const seen = [];
    onCommand((cmd) => seen.push(cmd));
    emitJointBatch({ j0: 0.1, j1: NaN, j2: 'x', j3: 0.2 }, ['j0', 'j1', 'j2', 'j3'], 'trajectory-target');
    expect(seen).toHaveLength(1);
    expect(Object.keys(seen[0].joints).sort()).toEqual(['j0', 'j3']);
  });

  it('退订后不再收到', () => {
    const seen = [];
    const off = onCommand((cmd) => seen.push(cmd));
    emitCommand({ type: 'a', source: 'user' });
    off();
    emitCommand({ type: 'b', source: 'user' });
    expect(seen).toHaveLength(1);
  });
});

// ── 3×3 解算 ──────────────────────────────────────────────────────────────

describe('solve3x3', () => {
  it('单位阵解出 b 本身', () => {
    const y = solve3x3([1, 0, 0, 0, 1, 0, 0, 0, 1], [1, 2, 3]);
    expect(y[0]).toBeCloseTo(1); expect(y[1]).toBeCloseTo(2); expect(y[2]).toBeCloseTo(3);
  });
  it('奇异阵返回 null', () => {
    expect(solve3x3([1, 2, 3, 2, 4, 6, 1, 1, 1], [1, 2, 3])).toBeNull();
  });
});

// ── DLS IK 伺服（2 关节平面臂收敛测试） ────────────────────────────────

describe('IKSolver', () => {
  // 2 关节平面臂：fk = l1+l2 前向（简单已知解场景）
  const fk2 = (a) => {
    const l1 = 1, l2 = 1;
    return {
      x: l1 * Math.cos(a.j0) + l2 * Math.cos(a.j0 + a.j1),
      y: l1 * Math.sin(a.j0) + l2 * Math.sin(a.j0 + a.j1),
      z: 0,
    };
  };
  const joints2 = [
    { name: 'j0', lower: -Math.PI, upper: Math.PI },
    { name: 'j1', lower: -Math.PI, upper: Math.PI },
  ];

  it('solveTarget 收敛到可达目标', () => {
    const solver = createIKSolver({ fk: fk2, joints: joints2 });
    // 目标：(1.2, 0.5) 在 2 关节臂可达范围内
    const r = solver.solveTarget({ x: 1.2, y: 0.5, z: 0 }, { j0: 0.3, j1: 0.3 });
    expect(r.ok).toBe(true);
    const p = fk2(r.angles);
    expect(Math.hypot(p.x - 1.2, p.y - 0.5)).toBeLessThan(0.01);
  });

  it('限位钳制：目标超范围关节角不越界', () => {
    const limited = [
      { name: 'j0', lower: -1, upper: 1 },
      { name: 'j1', lower: -0.5, upper: 0.5 },
    ];
    const solver = createIKSolver({ fk: fk2, joints: limited });
    const r = solver.solveTarget({ x: 1.8, y: 0, z: 0 }, { j0: 0, j1: 0 });
    expect(r.angles.j0).toBeLessThanOrEqual(1 + 1e-9);
    expect(r.angles.j1).toBeLessThanOrEqual(0.5 + 1e-9);
    expect(r.angles.j0).toBeGreaterThanOrEqual(-1 - 1e-9);
    expect(r.angles.j1).toBeGreaterThanOrEqual(-0.5 - 1e-9);
  });
});

// ── 工作空间包络 ─────────────────────────────────────────────────────────

describe('workspace envelope', () => {
  it('estimate + clamp 半球语义', () => {
    const fk = (a) => ({
      x: Math.cos(a.j0) * (1 + Math.cos(a.j1)),
      y: Math.sin(a.j0) * (1 + Math.cos(a.j1)),
      z: Math.sin(a.j1) + 1,  // 基座高度 1
    });
    const env = estimateWorkspaceEnvelope({
      fk, joints: [
        { name: 'j0', lower: -Math.PI, upper: Math.PI },
        { name: 'j1', lower: -Math.PI / 2, upper: Math.PI / 2 },
      ], nominalReach: 0.65,
    });
    expect(env.radius).toBeGreaterThan(0.3);
    expect(env.heightLimit).toBeGreaterThan(0.5);

    // 地面高度夹紧
    const r1 = clampToWorkspaceEnvelope({ x: 0.5, y: 0, z: -1 }, env);
    expect(r1.point.z).toBe(0);
    expect(r1.clamped).toBe(true);

    // 水平半径随高度收缩
    const top = clampToWorkspaceEnvelope({ x: 99, y: 0, z: env.heightLimit }, env);
    expect(Math.hypot(top.point.x, top.point.y)).toBeLessThanOrEqual(0.04); // planarLimit≈0.03 附近

    // 低处不夹紧
    const ok = clampToWorkspaceEnvelope({ x: 0.1, y: 0.1, z: 0.05 }, env);
    expect(ok.clamped).toBe(false);
  });
});

// ── 航点录制闸 ───────────────────────────────────────────────────────────

describe('teach recorder', () => {
  it('90ms 时间闸：间隔内丢弃', () => {
    const rec = createTeachRecorder();
    const wp = { joints: { j0: 0 }, tcp: { x: 0, y: 0, z: 0 } };
    expect(rec.record(wp)).toBe(true);   // 首条（自动 force=false 但 lastTcp=null → 通过）
    expect(rec.record(wp)).toBe(false);  // 同位置 + 间隔 < 90ms → 拒
  });

  it('4mm 距离闸：位移足够即时过', async () => {
    const rec = createTeachRecorder();
    rec.record({ joints: { j0: 0 }, tcp: { x: 0, y: 0, z: 0 } });
    // 模拟时间过后 + 位移 < 4mm → 拒
    await new Promise((r) => setTimeout(r, 95));
    expect(rec.record({ joints: { j0: 0.001 }, tcp: { x: 0.001, y: 0, z: 0 } })).toBe(false);
    // 位移 > 4mm → 过
    await new Promise((r) => setTimeout(r, 95));
    expect(rec.record({ joints: { j0: 0.1 }, tcp: { x: 0.01, y: 0, z: 0 } })).toBe(true);
  });

  it('force 起终点强制记录', () => {
    const rec = createTeachRecorder();
    const wp = { joints: {}, tcp: { x: 0, y: 0, z: 0 } };
    rec.record(wp);
    expect(rec.record(wp, true)).toBe(true);  // force 绕过双闸
  });

  it('buildTrajectoryPoints：首点当前位姿@50ms + 比例重排 + ≥0.2s 地板', () => {
    const rec = createTeachRecorder();
    rec.record({ joints: { j0: 0 }, tcp: { x: 0, y: 0, z: 0 } }, true);
    rec.record({ joints: { j0: 0.5 }, tcp: { x: 0.5, y: 0, z: 0 } }, true);
    const pts = buildTrajectoryPoints(rec.waypoints, { j0: 0.2 }, ['j0'], 4);
    expect(pts[0].seconds).toBe(0.05);
    expect(pts[0].positions[0]).toBeCloseTo(0.2);
    expect(pts[pts.length - 1].seconds).toBe(4);
    expect(pts.length).toBe(3);
  });
});

// ── 绘图计算字段 ─────────────────────────────────────────────────────────

describe('plot computed fields', () => {
  it('parseComputed 后缀解析', () => {
    expect(parseComputed('ranges_computed_min')).toEqual({ arrayPath: 'ranges', agg: 'min' });
    expect(parseComputed('pose.pose.position_computed_avg')).toEqual(
      { arrayPath: 'pose.pose.position', agg: 'avg' });
    expect(parseComputed('plain.field')).toBeNull();
    expect(parseComputed('_computed_min')).toBeNull();  // 空数组路径
  });

  it('extract 数值/嵌套/索引', () => {
    const data = { a: 1, b: { c: 2.5 }, arr: [10, 20, 30] };
    expect(extract(data, 'a')).toBe(1);
    expect(extract(data, 'b.c')).toBe(2.5);
    expect(extract(data, 'arr[1]')).toBe(20);
  });

  it('extract 计算字段 min/max/avg', () => {
    const data = { ranges: [1, 2, 3, 4, 5] };
    expect(extract(data, 'ranges_computed_min')).toBe(1);
    expect(extract(data, 'ranges_computed_max')).toBe(5);
    expect(extract(data, 'ranges_computed_avg')).toBe(3);
  });

  it('parseFields depth=1：数值 + 数组→[0]+计算 + 嵌套展开', () => {
    const fields = parseFields({
      x: 1,
      arr: [1, 2],
      nested: { y: 2 },
    });
    const paths = fields.map((f) => f.path);
    expect(paths).toContain('x');
    expect(paths).toContain('arr[0]');
    expect(paths).toContain('arr_computed_min');
    expect(paths).toContain('nested.y');
    // depth=1：nested 内的数组不递归
    expect(paths.filter((p) => p.startsWith('nested.')).length).toBe(1);
  });
});

// ── viz_core：缓动 / 运动动画器 / 协方差椭圆 / odom 轨迹 / 回放段 / 预设 ──

describe('viz_core easing', () => {
  it('边界与中点', () => {
    expect(easeInOutCubic(0)).toBe(0);
    expect(easeInOutCubic(1)).toBeCloseTo(1);
    expect(easeInOutCubic(0.5)).toBeCloseTo(0.5);
    expect(easeInOutQuad(0)).toBe(0);
    expect(easeInOutQuad(1)).toBeCloseTo(1);
    expect(easeInOutQuad(0.5)).toBeCloseTo(0.5);
  });
  it('先加速后减速（中点前斜率单调）', () => {
    // cubic 中点前导数 12t² 单调增
    const d = (t) => easeInOutCubic(t + 1e-4) - easeInOutCubic(t);
    expect(d(0.2)).toBeGreaterThan(d(0.1));
    expect(d(0.9)).toBeLessThan(d(0.8));
  });
});

describe('viz_core createJointMotion（moveToAngles 语义）', () => {
  it('起止与中间插值', () => {
    const m = createJointMotion();
    expect(m.sample(0)).toBeNull();           // 未启动
    m.start({ j0: 0, j1: 1 }, { j0: 1, j1: 1 }, 1000);
    const s0 = m.sample(0);
    expect(s0.angles.j0).toBeCloseTo(0);
    expect(s0.done).toBe(false);
    const half = m.sample(500);
    expect(half.angles.j0).toBeCloseTo(0.5);   // cubic 中点=线性中点
    expect(half.angles.j1).toBeCloseTo(1);     // 不动的关节保持
    const done = m.sample(1000);
    expect(done.angles.j0).toBeCloseTo(1);
    expect(done.done).toBe(true);
    expect(m.sample(9999).angles.j0).toBeCloseTo(1);  // 超时不越界
  });
  it('cancel 后 sample 返回 null', () => {
    const m = createJointMotion();
    m.start({ j0: 0 }, { j0: 1 }, 100);
    m.cancel();
    expect(m.sample(50)).toBeNull();
  });
  it('duration=0/缺省回退 850，负值钳 1ms', () => {
    const m = createJointMotion();
    m.start({ j0: 0 }, { j0: 1 }, 0);
    expect(m.duration).toBe(850);             // 0 视为缺省（参考 moveToAngles 同语义）
    m.start({ j0: 0 }, { j0: 1 }, -5);
    expect(m.duration).toBe(1);
  });
});

describe('viz_core covarianceEllipse（95% 置信）', () => {
  it('各向同性协方差 → 圆', () => {
    const cov = new Array(36).fill(0);
    cov[0] = cov[7] = 0.01;                    // σ=0.1 → 2σ=0.2
    const e = covarianceEllipse(cov);
    expect(e.a).toBeCloseTo(0.2, 3);
    expect(e.b).toBeCloseTo(0.2, 3);
    expect(e.angle).toBeCloseTo(0);
  });
  it('非对角协方差 → 主轴旋转', () => {
    const cov = new Array(36).fill(0);
    cov[0] = 0.02; cov[7] = 0.005; cov[1] = 0.008;
    const e = covarianceEllipse(cov);
    expect(e.a).toBeGreaterThan(e.b);
    // 校验：把椭圆参数代回，主轴方差应等于 (a/2)²
    expect((e.a / 2) ** 2).toBeCloseTo((cov[0] + cov[7]) / 2
      + Math.hypot((cov[0] - cov[7]) / 2, cov[1]), 4);
    expect(Math.abs(e.angle)).toBeGreaterThan(0.1);
  });
  it('零协方差 → 退化椭圆不抛', () => {
    const e = covarianceEllipse(new Array(36).fill(0));
    expect(e.a).toBe(0);
    expect(e.b).toBe(0);
  });
});

describe('viz_core createOdomTrail（0.1m 阈值 + 环形上限）', () => {
  it('小位移去抖，大位移追加', () => {
    const t = createOdomTrail(100, 0.1);
    expect(t.push(0, 0, 0)).toBe(true);
    expect(t.push(0.05, 0, 0)).toBe(false);    // < 0.1m 拒
    expect(t.push(0.12, 0, 0)).toBe(true);
    expect(t.points).toHaveLength(2);
  });
  it('超上限环形淘汰最旧点', () => {
    const t = createOdomTrail(10, 0.01);
    for (let i = 0; i < 30; i++) t.push(i * 0.1, 0, 0);
    expect(t.points).toHaveLength(10);
    expect(t.points[0][0]).toBeCloseTo(20 * 0.1);   // 最旧的 0..1.9 被淘汰
  });
  it('clear 复位', () => {
    const t = createOdomTrail(100, 0.1);
    t.push(0, 0, 0);
    t.clear();
    expect(t.points).toHaveLength(0);
    expect(t.push(0, 0, 0)).toBe(true);        // last 已清 → 同点也过
  });
});

describe('viz_core 回放段（updateTeachingPlayback 语义）', () => {
  it('段时长 80~900ms 钳制', () => {
    expect(replaySegmentDuration(0, 10)).toBe(80);
    expect(replaySegmentDuration(0, 500)).toBe(500);
    expect(replaySegmentDuration(0, 5000)).toBe(900);
  });
  it('sampleSegment 端点与 easeInOutQuad 中点', () => {
    const a = sampleSegment({ j0: 0 }, { j0: 1 }, 0);
    expect(a.j0).toBeCloseTo(0);
    const b = sampleSegment({ j0: 0 }, { j0: 1 }, 1);
    expect(b.j0).toBeCloseTo(1);
    const mid = sampleSegment({ j0: 0 }, { j0: 1 }, 0.5);
    expect(mid.j0).toBeCloseTo(0.5);
    expect(sampleSegment({ j0: 0 }, { j0: 1 }, -1).j0).toBeCloseTo(0);   // 越界钳制
    expect(sampleSegment({ j0: 0 }, { j0: 1 }, 2).j0).toBeCloseTo(1);
  });
});

describe('viz_core createPresetStore（localStorage 存取）', () => {
  const mkStorage = () => {
    const m = new Map();
    return { getItem: (k) => m.get(k) ?? null, setItem: (k, v) => m.set(k, v) };
  };
  it('save/list 往返', () => {
    const st = createPresetStore(mkStorage());
    expect(st.list()).toEqual([]);
    st.save('home', { j0: 0.1, j1: -0.2 });
    expect(st.list()).toEqual([{ name: 'home', joints: { j0: 0.1, j1: -0.2 } }]);
  });
  it('remove 删除 + 缺名拒存', () => {
    const st = createPresetStore(mkStorage());
    st.save('a', { j0: 1 });
    expect(st.remove('a')).toBe(true);
    expect(st.remove('a')).toBe(false);
    expect(st.save('', { j0: 1 })).toBe(false);
  });
  it('坏 JSON 容错（返回空表）', () => {
    const st = createPresetStore({ getItem: () => '{oops', setItem: () => {} });
    expect(st.list()).toEqual([]);
    st.save('x', { j0: 0 });
  });
});
