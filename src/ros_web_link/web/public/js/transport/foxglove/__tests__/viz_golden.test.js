// transport/foxglove/__tests__/viz_golden.test.js — CDR 黄金字节交叉验证
//
// 真值来源：rclpy 官方序列化（test/generate_viz_golden.py 产出的
// viz_golden.json，schema 文本与 vendored 桥 message_definition_cache 同构）。
// 本文件用浏览器同款 vendored codec（parse + MessageReader）离线解码并逐字段
// 比对——无需活栈即可证明 JS 解码与 ROS 序列化字节级一致。
// 覆盖 RViz/Foxglove 常用可视化消息的 CDR 难点：blob、定长 float64[36]、
// 变长并行数组、嵌套数组、inf、int8 标量。

import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { parse, MessageReader } from '../vendor/foxglove_cdr.js';

const golden = JSON.parse(readFileSync(
  join(dirname(fileURLToPath(import.meta.url)), 'viz_golden.json'), 'utf8'));

function decode(caseEntry) {
  const bytes = new Uint8Array(
    caseEntry.hex.match(/../g).map((h) => parseInt(h, 16)));
  const definitions = parse(caseEntry.schema, { ros2: true });
  return new MessageReader(definitions).readMessage(bytes);
}

function arr(v) { return Array.from(v ?? []); }
function close(actual, expected, digits = 5) {
  expect(actual).toBeCloseTo(expected, digits);
}

const byName = Object.fromEntries(golden.cases.map((c) => [c.name, c]));

describe('golden: blob 类（Image/CompressedImage/PointCloud2）', () => {
  it('sensor_msgs/Image mono8 棋盘', () => {
    const m = decode(byName.image);
    const d = arr(m.data);
    expect(m.width).toBe(4);
    expect(m.height).toBe(2);
    expect(m.step).toBe(4);
    expect(m.encoding).toBe('mono8');
    expect(m.is_bigendian).toBe(0);
    expect(d.length).toBe(8);
    expect(d.slice(0, 4)).toEqual([10, 20, 30, 40]);
    expect(m.header.frame_id).toBe('rwl_gold');
    expect(m.header.stamp.sec).toBe(1791900000);
    expect(Number(m.header.stamp.nanosec)).toBe(123456789);
  });

  it('sensor_msgs/CompressedImage jpeg 字节串', () => {
    const m = decode(byName.compressed_image);
    const d = arr(m.data);
    expect(m.format).toBe('jpeg');
    expect(d.length).toBe(7);
    expect(d.slice(0, 5)).toEqual([255, 216, 1, 2, 3]);
    expect(d.slice(-2)).toEqual([255, 217]);
  });

  it('sensor_msgs/PointCloud2 XYZ 字段与点数据', () => {
    const m = decode(byName.pointcloud2);
    expect(m.width).toBe(3);
    expect(m.height).toBe(1);
    expect(m.point_step).toBe(12);
    expect(m.fields.map((f) => ({
      name: f.name, offset: Number(f.offset),
      datatype: Number(f.datatype), count: Number(f.count),
    }))).toEqual(byName.pointcloud2.expected.fields);
    const d = m.data instanceof Uint8Array ? m.data : new Uint8Array(m.data);
    expect(d.length).toBe(36);
    const xyz = (i) => {
      const v = new DataView(d.buffer, d.byteOffset);
      return [v.getFloat32(i * 12, true), v.getFloat32(i * 12 + 4, true),
        v.getFloat32(i * 12 + 8, true)];
    };
    byName.pointcloud2.expected.first_point_xyz.forEach(
      (e, k) => close(xyz(0)[k], e));
    byName.pointcloud2.expected.third_point_xyz.forEach(
      (e, k) => close(xyz(2)[k], e));
  });
});

describe('golden: 定长数组（CameraInfo/Odometry 协方差）', () => {
  it('sensor_msgs/CameraInfo 畸变与内参矩阵', () => {
    const m = decode(byName.camera_info);
    expect(m.height).toBe(48);
    expect(m.width).toBe(64);
    expect(m.distortion_model).toBe('plumb_bob');
    arr(m.d).forEach((v, i) => close(v, byName.camera_info.expected.d[i]));
    arr(m.k).forEach((v, i) => close(v, byName.camera_info.expected.k[i]));
    // 定长 9 元素（非变长）
    expect(arr(m.k).length).toBe(9);
  });

  it('nav_msgs/Odometry float64[36] 协方差 ×2', () => {
    const m = decode(byName.odometry);
    const e = byName.odometry.expected;
    expect(m.child_frame_id).toBe('base');
    close(m.pose.pose.position.x, e.position[0]);
    close(m.pose.pose.position.y, e.position[1]);
    close(m.pose.pose.position.z, e.position[2]);
    const pc = arr(m.pose.covariance);
    expect(pc.length).toBe(36);
    close(pc[0], e.pose_cov0);
    close(pc[7], e.pose_cov7);
    close(pc[35], e.pose_cov35);
    close(m.twist.twist.linear.x, e.twist[0]);
    close(m.twist.twist.angular.z, e.twist[5]);
    close(arr(m.twist.covariance)[35], e.twist_cov35);
  });
});

describe('golden: 变长数组 / 特殊浮点（LaserScan/JointState/Range）', () => {
  it('sensor_msgs/LaserScan inf 语义', () => {
    const m = decode(byName.laserscan);
    const ranges = arr(m.ranges);
    expect(ranges.length).toBe(3);
    expect(ranges[0]).toBe(Infinity); // CDR float32 inf → JS Infinity
    close(ranges[1], 1.5);
    close(ranges[2], 2.5);
    arr(m.intensities).forEach(
      (v, i) => close(v, byName.laserscan.expected.intensities[i]));
    close(m.range_min, 0.1);
  });

  it('sensor_msgs/JointState 变长并行数组', () => {
    const m = decode(byName.joint_state);
    expect(m.name).toEqual(['a', 'b']);
    arr(m.position).forEach((v, i) => close(v, [0.5, -0.5][i]));
    arr(m.velocity).forEach((v, i) => close(v, [1.0, 2.0][i]));
    arr(m.effort).forEach((v, i) => close(v, [10.0, 20.0][i]));
  });

  it('sensor_msgs/Range 标量', () => {
    const m = decode(byName.range);
    expect(Number(m.radiation_type)).toBe(1);
    close(m.field_of_view, 0.25);
    close(m.range, 2.75);
  });
});

describe('golden: 嵌套数组结构（Path/PoseArray/Wrench/Marker*/Diagnostics/TF）', () => {
  it('nav_msgs/Path PoseStamped[]', () => {
    const m = decode(byName.path);
    expect(m.poses.length).toBe(2);
    close(m.poses[0].pose.position.x, 0.0);
    close(m.poses[1].pose.position.x, 1.0);
    expect(m.poses[0].header.frame_id).toBe('rwl_gold');
  });

  it('geometry_msgs/PoseArray Pose[]', () => {
    const m = decode(byName.pose_array);
    expect(m.poses.length).toBe(2);
    close(m.poses[0].position.z, 1.0);
    close(m.poses[1].position.z, 2.0);
    close(m.poses[0].orientation.w, 1.0);
  });

  it('geometry_msgs/PoseStamped / WrenchStamped', () => {
    const p = decode(byName.pose_stamped);
    close(p.pose.position.x, 0.3);
    close(p.pose.position.y, 0.4);
    close(p.pose.position.z, 0.5);
    const w = decode(byName.wrench);
    close(w.wrench.force.x, 1.5);
    close(w.wrench.torque.z, -0.25);
  });

  it('geometry_msgs/PoseWithCovarianceStamped float64[36]', () => {
    const m = decode(byName.pose_cov);
    const e = byName.pose_cov.expected;
    close(m.pose.pose.position.x, e.position[0]);
    close(m.pose.pose.position.y, e.position[1]);
    close(m.pose.pose.position.z, e.position[2]);
    const cov = arr(m.pose.covariance);
    expect(cov.length).toBe(36);
    close(cov[0], e.cov0);
    close(cov[1], e.cov1);
    close(cov[7], e.cov7);
    close(cov[35], e.cov35);
  });

  it('visualization_msgs/Marker SPHERE_LIST（points+colors 嵌套）', () => {
    const m = decode(byName.marker);
    const e = byName.marker.expected;
    expect(m.ns).toBe(e.ns);
    expect(Number(m.id)).toBe(e.id);
    expect(Number(m.type)).toBe(e.type);
    expect(Number(m.action)).toBe(e.action);
    m.points.forEach((pt, i) => {
      close(pt.x, e.points[i][0]);
      close(pt.y, e.points[i][1]);
      close(pt.z, e.points[i][2]);
    });
    close(m.colors[0].r, e.colors[0].r);
    close(m.colors[0].a, e.colors[0].a);
  });

  it('visualization_msgs/MarkerArray 文本（UTF-8 往返）', () => {
    const m = decode(byName.marker_array);
    expect(m.markers.length).toBe(1);
    expect(m.markers[0].ns).toBe('g1');
    expect(Number(m.markers[0].type)).toBe(1);
    expect(m.markers[0].text).toBe('你好 gold');
  });

  it('diagnostic_msgs/DiagnosticArray 嵌套 KeyValue（int8 标量→number）', () => {
    const m = decode(byName.diagnostics);
    const e = byName.diagnostics.expected;
    expect(m.status.length).toBe(1);
    expect(m.status[0].name).toBe(e.name);
    // rclpy 侧 int8 是 bytes(b'\x00')；JS codec 解码为 number
    expect(Number(m.status[0].level)).toBe(e.level);
    expect(m.status[0].message).toBe(e.message);
    expect(m.status[0].values[0].key).toBe('k');
    expect(m.status[0].values[0].value).toBe('v');
  });

  it('tf2_msgs/TFMessage TransformStamped[]', () => {
    const m = decode(byName.tf_message);
    expect(m.transforms.length).toBe(1);
    const t = m.transforms[0];
    expect(t.child_frame_id).toBe('odom');
    expect(t.header.frame_id).toBe('map');
    close(t.transform.translation.x, 1.5);
    close(t.transform.rotation.w, 1.0);
  });
});
