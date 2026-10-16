// 升级轮纯核用例：costmap/map 色带、teleop 命令混合器、cfg v4 迁移
import { describe, it, expect } from 'vitest';
import {
  mapGridPixel, costmapPixel, mixTeleopCommand,
  migrateCfg, defaultCfg, DISPLAY_TYPES,
} from '../viz_core.js';

describe('OccupancyGrid 色板', () => {
  it('map 语义：未知/自由/占据/中间值', () => {
    expect(mapGridPixel(-1)).toEqual([127, 127, 127, 120]);
    expect(mapGridPixel(0)).toEqual([255, 255, 255, 40]);
    expect(mapGridPixel(100)).toEqual([10, 10, 10, 255]);
    const mid = mapGridPixel(50);
    expect(mid).toHaveLength(4);
    expect(mid[3]).toBe(200);
  });

  it('costmap 语义：未知/自由/代价渐变（代价越高越红且越不透明）', () => {
    expect(costmapPixel(-1)[3]).toBeLessThan(100);
    expect(costmapPixel(0)[3]).toBeLessThan(30);
    const low = costmapPixel(10);
    const high = costmapPixel(90);
    expect(high[0]).toBeGreaterThan(low[0]);   // 越红
    expect(high[2]).toBeLessThan(low[2]);      // 蓝越少
    expect(high[3]).toBeGreaterThan(low[3]);   // 越不透明
  });
});

describe('teleop 命令混合器', () => {
  it('W 前进 = +maxLin', () => {
    const { active, twist } = mixTeleopCommand(new Set(['w']), 0.2, 1.0);
    expect(active).toBe(true);
    expect(twist.linear.x).toBeCloseTo(0.2);
    expect(twist.angular.z).toBe(0);
  });

  it('A/D 差速转向方向相反', () => {
    const a = mixTeleopCommand(new Set(['a']), 0.2, 1.0);
    const d = mixTeleopCommand(new Set(['d']), 0.2, 1.0);
    expect(a.twist.angular.z).toBeCloseTo(1.0);
    expect(d.twist.angular.z).toBeCloseTo(-1.0);
  });

  it('W+A 合成前进行走', () => {
    const { twist } = mixTeleopCommand(new Set(['w', 'a']), 0.2, 1.0);
    expect(twist.linear.x).toBeCloseTo(0.2);
    expect(twist.angular.z).toBeCloseTo(1.0);
  });

  it('S 后退为 0.6 倍限速', () => {
    const { twist } = mixTeleopCommand(new Set(['s']), 0.2, 1.0);
    expect(twist.linear.x).toBeCloseTo(-0.12);
  });

  it('松键 = 零速急停', () => {
    const { active, twist } = mixTeleopCommand(new Set(), 0.2, 1.0);
    expect(active).toBe(false);
    expect(twist.linear.x).toBe(0);
    expect(twist.angular.z).toBe(0);
  });
});

describe('cfg v4 迁移', () => {
  it('默认配置含官方代价地图叠层三实例', () => {
    const cfg = defaultCfg();
    const types = cfg.instances.map((i) => i.type);
    expect(types.filter((t) => t === 'costmap')).toHaveLength(2);
    const map1 = cfg.instances.find((i) => i.id === 'map1');
    expect(map1.topic).toBe('/map');
    expect(map1.opacity).toBeCloseTo(0.85);
  });

  it('默认相机走官方压缩流（gz 无头相机不节流的火流防护）', () => {
    const cfg = defaultCfg();
    const img = cfg.instances.find((i) => i.type === 'image');
    const imgc = cfg.instances.find((i) => i.type === 'imagec');
    expect(img.on).toBe(false);          // 原始流默认关（可手动开启）
    expect(imgc.on).toBe(true);
    expect(imgc.topic).toBe('/camera/image_raw/compressed');
  });

  it('v3 固定键 → v4 实例：保留自定义 topic 与开关', () => {
    const v3 = {
      grid: { on: true },
      robot: { on: true, topic: '/robot_description' },
      map: { on: false, topic: '/my_map' },
      cloud: { on: true, topic: '/my_cloud', size: 0.05 },
      odom: { on: true, topic: '/odom', trail: true },
      image: { on: true, topic: '/cam' },
    };
    const cfg = migrateCfg(v3);
    const map1 = cfg.instances.find((i) => i.id === 'map1');
    expect(map1.on).toBe(false);
    expect(map1.topic).toBe('/my_map');
    const cloud = cfg.instances.find((i) => i.type === 'cloud');
    expect(cloud.topic).toBe('/my_cloud');
    expect(cloud.size).toBeCloseTo(0.05);
    // v4 新增类型不受 v3 影响：代价地图默认仍在
    expect(cfg.instances.filter((i) => i.type === 'costmap')).toHaveLength(2);
    expect(cfg.global.fixedFrame).toBe('');
  });

  it('v4 配置原样保留', () => {
    const cfg = defaultCfg();
    cfg.instances[3].topic = '/x';
    const out = migrateCfg(cfg);
    expect(out.instances[3].topic).toBe('/x');
    expect(out.instances).toHaveLength(cfg.instances.length);
  });

  it('全部类型在注册表中（add-display 下拉来源）', () => {
    for (const t of ['grid', 'robot', 'tf', 'map', 'costmap', 'cloud', 'scan3d',
      'marker', 'path', 'poses', 'odom', 'range', 'wrench', 'posecov',
      'image', 'imagec']) {
      expect(DISPLAY_TYPES[t]).toBeTruthy();
    }
  });
});
