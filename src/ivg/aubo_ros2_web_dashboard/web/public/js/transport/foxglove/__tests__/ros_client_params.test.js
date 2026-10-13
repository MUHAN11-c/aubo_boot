// transport/foxglove/__tests__/ros_client_params.test.js — 参数助手纯核单测
// valueFromParameterValue / parameterValueFrom 与 rcl_interfaces/msg/ParameterValue
// 的 type 码对齐（rosidl 定义 0-9）。
import { describe, it, expect, vi } from 'vitest';

// ros_client.js 顶部 import 了 ivg_runtime（浏览器依赖）——参数助手是纯函数，
// 通过模块内联复制的方式不可取；改用 vi.mock 不引入 vitest 内部件。这里直接
// 动态 import 并 mock 依赖链：
vi.mock('../../../ivg_runtime.js', () => ({
  foxgloveWebSocketUrl: () => 'ws://mock',
}));

import { valueFromParameterValue, parameterValueFrom, PARAM_TYPE } from '../ros_client.js';

describe('parameterValueFrom', () => {
  it('bool → PARAMETER_BOOL', () => {
    const pv = parameterValueFrom(true);
    expect(pv.type).toBe(PARAM_TYPE.BOOL);
    expect(pv.bool_value).toBe(true);
  });
  it('整数 → INTEGER', () => {
    const pv = parameterValueFrom(42);
    expect(pv.type).toBe(PARAM_TYPE.INTEGER);
    expect(pv.integer_value).toBe(42);
  });
  it('浮点 → DOUBLE', () => {
    const pv = parameterValueFrom(0.15);
    expect(pv.type).toBe(PARAM_TYPE.DOUBLE);
    expect(pv.double_value).toBeCloseTo(0.15, 8);
  });
  it('字符串 → STRING', () => {
    const pv = parameterValueFrom('heart');
    expect(pv.type).toBe(PARAM_TYPE.STRING);
    expect(pv.string_value).toBe('heart');
  });
  it('数值数组 → 数组类型（整数组/浮点组）', () => {
    expect(parameterValueFrom([1, 2, 3]).type).toBe(PARAM_TYPE.INTEGER_ARRAY);
    expect(parameterValueFrom([0.1, 0.2]).type).toBe(PARAM_TYPE.DOUBLE_ARRAY);
    expect(parameterValueFrom(['a', 'b']).type).toBe(PARAM_TYPE.STRING_ARRAY);
    expect(parameterValueFrom([true, false]).type).toBe(PARAM_TYPE.BOOL_ARRAY);
  });
  it('空数组/未知 → NOT_SET', () => {
    expect(parameterValueFrom([]).type).toBe(PARAM_TYPE.NOT_SET);
  });
});

describe('valueFromParameterValue（roundtrip）', () => {
  const cases = [true, 42, 0.125, 'latte', [1, 2], [0.5, 0.25], ['x', 'y'], [true, false]];
  for (const c of cases) {
    it(String(JSON.stringify(c)), () => {
      const pv = parameterValueFrom(c);
      const back = valueFromParameterValue(pv);
      expect(back).toEqual(c);
    });
  }
  it('未设置类型返回 undefined', () => {
    expect(valueFromParameterValue({ type: PARAM_TYPE.NOT_SET })).toBeUndefined();
  });
});
