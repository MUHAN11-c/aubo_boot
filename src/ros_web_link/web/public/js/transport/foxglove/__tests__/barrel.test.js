// barrel.test.js — 稳定公开入口（/js/ros_web_link.js）导出契约
import { describe, expect, it } from 'vitest';
import * as api from '../../../ros_web_link.js';

describe('web/public/js/ros_web_link.js barrel', () => {
  it('导出门面/action/参数转换 API', () => {
    expect(typeof api.foxgloveRosClient).toBe('function');
    expect(typeof api.FoxgloveRosClient).toBe('function');
    expect(typeof api.createActionClient).toBe('function');
    expect(api.GOAL_STATUS.SUCCEEDED).toBe(4);
    expect(typeof api.valueFromParameterValue).toBe('function');
    expect(typeof api.parameterValueFrom).toBe('function');
  });

  it('单例可重复获取（不因 import 次序产生第二实例）', () => {
    expect(api.foxgloveRosClient()).toBe(api.foxgloveRosClient());
  });
});
