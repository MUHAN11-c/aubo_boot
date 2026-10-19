// 升级轮纯核用例：costmap/map 色带、teleop 命令混合器、cfg v5 迁移
import { describe, it, expect } from 'vitest';
import {
  mapGridPixel, costmapPixel, mixTeleopCommand,
  migrateCfg, defaultCfg, DISPLAY_TYPES,
  adaptInstancesToDataPlane, teleopMessage,
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

describe('cfg v5 迁移（点大小改像素）', () => {
  it('默认配置含官方代价地图叠层三实例', () => {
    const cfg = defaultCfg();
    expect(cfg.v).toBe(5);
    const types = cfg.instances.map((i) => i.type);
    expect(types.filter((t) => t === 'costmap')).toHaveLength(2);
    const map1 = cfg.instances.find((i) => i.id === 'map1');
    expect(map1.topic).toBe('/map');
    expect(map1.opacity).toBeCloseTo(0.85);
    const cloud = cfg.instances.find((i) => i.type === 'cloud');
    expect(cloud.size).toBe(3);          // RViz2 Size (Pixels): 3
    expect(cloud.colorBy).toBe('axis');  // AxisColor + Use rainbow
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
    expect(cloud.size).toBe(4);          // 0.05 m → 4 px（×80）
    expect(cloud.colorBy).toBe('axis');
    // v4 新增类型不受 v3 影响：代价地图默认仍在
    expect(cfg.instances.filter((i) => i.type === 'costmap')).toHaveLength(2);
    expect(cfg.global.fixedFrame).toBe('');
  });

  it('v4 旧默认 0.035m 迁到 3px；自定义 0.05m 迁到 4px', () => {
    const v4 = defaultCfg();
    const cloud = v4.instances.find((i) => i.type === 'cloud');
    cloud.size = 0.035;
    delete cloud.colorBy;
    let out = migrateCfg(v4);
    let c = out.instances.find((i) => i.type === 'cloud');
    expect(c.size).toBe(3);
    expect(c.colorBy).toBe('axis');

    cloud.size = 0.05;
    out = migrateCfg(v4);
    c = out.instances.find((i) => i.type === 'cloud');
    expect(c.size).toBe(4);
  });

  it('v5 自定义 topic/属性保留', () => {
    const cfg = defaultCfg();
    cfg.instances[3].topic = '/x';
    cfg.instances.find((i) => i.type === 'cloud').size = 6;
    const out = migrateCfg(cfg);
    expect(out.instances[3].topic).toBe('/x');
    expect(out.instances.find((i) => i.type === 'cloud').size).toBe(6);
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

describe('数据面自动适配（独立监控模式）', () => {
  const fullSimTopics = [
    '/robot_description', '/joint_states', '/tf', '/tf_static', '/scan', '/odom',
    '/map', '/plan', '/particle_cloud', '/amcl_pose', '/cmd_vel',
    '/global_costmap/costmap', '/local_costmap/costmap',
    '/camera/color/image_raw', '/camera/color/camera_info',
    '/camera/depth_registered/points',
    '/peach/perception/markers', '/peach/reconstruction/markers',
  ];

  it('果园仿真：原始相机 / 腕载点云 / 感知标记', () => {
    const out = adaptInstancesToDataPlane(defaultCfg().instances, fullSimTopics);
    const by = (type) => out.filter((i) => i.type === type);
    expect(by('image')[0].on).toBe(true);
    expect(by('image')[0].topic).toBe('/camera/color/image_raw');
    expect(by('imagec')[0].on).toBe(false);
    expect(by('cloud')[0].topic).toBe('/camera/depth_registered/points');
    expect(by('cloud')[0].on).toBe(true);
    expect(by('marker').map((i) => i.topic)).toEqual(
      ['/peach/perception/markers', '/peach/reconstruction/markers']);
    // 原有官方默认项仍在（只是重定向）
    expect(by('map')[0].topic).toBe('/map');
    expect(by('path')[0].topic).toBe('/plan');
  });

  it('官方 TB3：压缩图 / test_cloud 保持不变', () => {
    const topics = ['/camera/image_raw', '/camera/image_raw/compressed',
      '/test_cloud', '/map', '/plan', '/particle_cloud', '/amcl_pose'];
    const out = adaptInstancesToDataPlane(defaultCfg().instances, topics);
    const by = (type) => out.filter((i) => i.type === type);
    expect(by('imagec')[0].on).toBe(true);
    expect(by('imagec')[0].topic).toBe('/camera/image_raw/compressed');
    expect(by('image')[0].on).toBe(false);
    expect(by('cloud')[0].topic).toBe('/test_cloud');
    expect(by('marker').length).toBe(0);
  });

  it('空图不改变默认配置（不猜测）', () => {
    const before = defaultCfg().instances;
    const out = adaptInstancesToDataPlane(before, []);
    expect(out).toEqual(before);
    expect(out).not.toBe(before);
  });
});

describe('Teleop 消息方言', () => {
  it('Twist：直接返回 linear/angular', () => {
    const twist = { linear: { x: 0.2, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 1 } };
    const msg = teleopMessage('Twist', twist);
    expect(msg).toEqual(twist);
    expect(msg.header).toBeUndefined();
  });

  it('TwistStamped：包 header 与 twist', () => {
    const twist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
    const msg = teleopMessage('TwistStamped', twist, 'base_footprint');
    expect(msg.header.frame_id).toBe('base_footprint');
    expect(msg.twist).toBe(twist);
  });
});
