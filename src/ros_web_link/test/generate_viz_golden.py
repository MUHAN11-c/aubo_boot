"""生成 CDR 黄金字节 fixtures — rclpy 序列化 + 桥同构 schema 文本 → JSON.

产出 web/public/js/transport/foxglove/__tests__/viz_golden.json，供 vitest 用
JS codec 离线解码并逐字段比对（浏览器解码侧的真值交叉验证，无需活栈）。

schema 文本与 vendored ivg_foxglove_bridge 的 message_definition_cache 同构：
根 .msg 全文 + DFS 依赖（首见序）+ "====…\\nMSG: pkg/msg/Type\\n" 分隔递归拼接。

运行（aubo_py3.12 venv 或已 source Jazzy 的 python）：
  python src/ros_web_link/test/generate_viz_golden.py
"""
from __future__ import annotations

import json
import re
import struct
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from rclpy.serialization import serialize_message

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (Point, Pose, PoseArray, PoseStamped,
                               PoseWithCovarianceStamped, TransformStamped,
                               WrenchStamped)
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath  # 避免覆盖 pathlib.Path
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image, JointState,
                             LaserScan, PointCloud2, PointField, Range)
from std_msgs.msg import ColorRGBA, Header
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

PKG_ROOT = Path(__file__).resolve().parent.parent
OUT = PKG_ROOT / 'web' / 'public' / 'js' / 'transport' / 'foxglove' / '__tests__' / 'viz_golden.json'

BUILTIN = {
    'bool', 'byte', 'char', 'int8', 'uint8', 'int16', 'uint16', 'int32',
    'uint32', 'int64', 'uint64', 'float32', 'float64', 'string', 'wstring',
    'builtin_interfaces/Time', 'builtin_interfaces/Duration',
}

_DELIM_EQ = '=' * 80


def msg_text(res_name: str) -> str:
    """读包 share 下的 .msg 全文（res_name 形如 'sensor_msgs/msg/Image'）."""
    pkg, _, name = res_name.partition('/')
    path = Path(get_package_share_directory(pkg)) / 'msg' / f'{name.split("/")[-1]}.msg'
    return path.read_text(encoding='utf-8')


def field_types(text: str, root_pkg: str) -> list[str]:
    """按字段序提取依赖类型（与桥 load_message_spec 的发现序一致）.

    .msg 文件约定：跨包依赖写作 pkg/Type，同包依赖与 Header 写裸名——
    裸名按同包解析，Header 固定 std_msgs/Header。
    """
    deps: list[str] = []
    for line in text.splitlines():
        line = line.split('#')[0].strip()
        if not line or line.startswith('MSG:') or set(line) == {'-'}:
            continue
        m = re.match(r'^([A-Za-z_][\w/]*)\s*[\[=]?\s*\S', line)
        if not m:
            continue
        t = m.group(1)
        if t in BUILTIN:
            continue
        if t == 'Header':
            deps.append('std_msgs/Header')
        elif '/' in t:
            deps.append(t)
        else:
            deps.append(f'{root_pkg}/{t}')
    return deps


def full_text(res_name: str) -> str:
    """桥同构 schema 全文：根文本 + DFS 首见序依赖拼接."""
    seen: set[str] = set()

    def rec(name: str) -> str:
        text = msg_text(name)
        out = text
        for dep in field_types(text, name.split("/")[0]):
            dep_norm = dep if '/' in dep else f'unknown/{dep}'
            if dep_norm in seen:
                continue
            seen.add(dep_norm)
            out += '\n' + _DELIM_EQ + '\nMSG: ' + dep_norm + '\n' + rec(dep_norm)
        return out

    seen.add(res_name)
    return rec(res_name)


def _hdr(frame: str = 'rwl_gold') -> Header:
    h = Header()
    h.stamp.sec, h.stamp.nanosec = 1791900000, 123456789
    h.frame_id = frame
    return h


def build_cases() -> list[dict]:
    cases = []

    def add(name: str, res: str, msg, expected: dict) -> None:
        raw = bytes(serialize_message(msg))
        cases.append({
            'name': name,
            'type': res,
            'schema': full_text(res),
            'hex': raw.hex(),
            'expected': expected,
        })

    img = Image()
    img.header = _hdr()
    img.height, img.width, img.encoding = 2, 4, 'mono8'
    img.is_bigendian, img.step = 0, 4
    img.data = bytes([10, 20, 30, 40, 50, 60, 70, 80])
    add('image', 'sensor_msgs/msg/Image', img, {
        'width': 4, 'height': 2, 'step': 4, 'encoding': 'mono8',
        'data_len': 8, 'data_head': [10, 20, 30, 40],
        'frame_id': 'rwl_gold',
    })

    cimg = CompressedImage()
    cimg.header = _hdr()
    cimg.format = 'jpeg'
    cimg.data = bytes([0xFF, 0xD8, 1, 2, 3, 0xFF, 0xD9])
    add('compressed_image', 'sensor_msgs/msg/CompressedImage', cimg, {
        'format': 'jpeg', 'data_len': 7, 'data_head': [255, 216, 1, 2, 3],
    })

    ci = CameraInfo()
    ci.header = _hdr()
    ci.height, ci.width = 48, 64
    ci.distortion_model = 'plumb_bob'
    ci.d = [0.5, -0.25]
    ci.k = [1.0, 0.0, 2.0, 0.0, 3.0, 4.0, 0.0, 0.0, 1.0]
    add('camera_info', 'sensor_msgs/msg/CameraInfo', ci, {
        'height': 48, 'width': 64, 'distortion_model': 'plumb_bob',
        'd': [0.5, -0.25], 'k': [1.0, 0.0, 2.0, 0.0, 3.0, 4.0, 0.0, 0.0, 1.0],
    })

    pc = PointCloud2()
    pc.header = _hdr()
    pc.height, pc.width = 1, 3
    pc.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    pc.is_bigendian, pc.point_step, pc.row_step = False, 12, 36
    pc.data = b''.join(struct.pack('<fff', float(i), float(i) * 2.0, 0.5) for i in range(3))
    add('pointcloud2', 'sensor_msgs/msg/PointCloud2', pc, {
        'width': 3, 'height': 1, 'point_step': 12,
        'fields': [
            {'name': 'x', 'offset': 0, 'datatype': 7, 'count': 1},
            {'name': 'y', 'offset': 4, 'datatype': 7, 'count': 1},
            {'name': 'z', 'offset': 8, 'datatype': 7, 'count': 1},
        ],
        'data_len': 36,
        'first_point_xyz': [0.0, 0.0, 0.5],
        'third_point_xyz': [2.0, 4.0, 0.5],
    })

    ls = LaserScan()
    ls.header = _hdr()
    ls.angle_min, ls.angle_max = -1.0, 1.0
    ls.angle_increment, ls.range_min, ls.range_max = 0.5, 0.1, 10.0
    ls.ranges = [float('inf'), 1.5, 2.5]
    ls.intensities = [0.0, 10.0, 20.0]
    add('laserscan', 'sensor_msgs/msg/LaserScan', ls, {
        'range_min': 0.1, 'ranges_len': 3,
        'ranges1': 1.5, 'ranges2': 2.5, 'ranges0_inf': True,
        'intensities': [0.0, 10.0, 20.0],
    })

    rg = Range()
    rg.header = _hdr()
    rg.radiation_type, rg.field_of_view = 1, 0.25
    rg.min_range, rg.max_range, rg.range = 0.1, 5.0, 2.75
    add('range', 'sensor_msgs/msg/Range', rg, {
        'radiation_type': 1, 'field_of_view': 0.25, 'range': 2.75,
    })

    od = Odometry()
    od.header = _hdr()
    od.child_frame_id = 'base'
    od.pose.pose.position.x, od.pose.pose.position.y = 1.25, -3.5
    od.pose.pose.orientation.w = 1.0
    od.pose.covariance = [0.0] * 36
    od.pose.covariance[0], od.pose.covariance[7] = 0.01, 0.02
    od.twist.twist.linear.x = 0.5
    od.twist.twist.angular.z = -0.1
    od.twist.covariance = [0.0] * 36
    od.twist.covariance[35] = 0.03
    add('odometry', 'nav_msgs/msg/Odometry', od, {
        'child_frame_id': 'base',
        'position': [1.25, -3.5, 0.0],
        'pose_cov0': 0.01, 'pose_cov7': 0.02, 'pose_cov35': 0.0,
        'twist': [0.5, 0.0, 0.0, 0.0, 0.0, -0.1],
        'twist_cov35': 0.03,
    })

    pa = NavPath()
    pa.header = _hdr()
    for i in range(2):
        ps = PoseStamped()
        ps.header = _hdr()
        ps.pose.position.x = float(i)
        ps.pose.orientation.w = 1.0
        pa.poses.append(ps)
    add('path', 'nav_msgs/msg/Path', pa, {
        'poses_len': 2, 'pose0_x': 0.0, 'pose1_x': 1.0,
    })

    pr = PoseArray()
    pr.header = _hdr()
    for i in range(2):
        p = Pose()
        p.position.z = float(i + 1)
        p.orientation.w = 1.0
        pr.poses.append(p)
    add('pose_array', 'geometry_msgs/msg/PoseArray', pr, {
        'poses_len': 2, 'pose0_z': 1.0, 'pose1_z': 2.0,
    })

    ps2 = PoseStamped()
    ps2.header = _hdr()
    ps2.pose.position.x, ps2.pose.position.y, ps2.pose.position.z = 0.3, 0.4, 0.5
    ps2.pose.orientation.w = 1.0
    add('pose_stamped', 'geometry_msgs/msg/PoseStamped', ps2, {
        'position': [0.3, 0.4, 0.5],
    })

    pc2 = PoseWithCovarianceStamped()
    pc2.header = _hdr()
    pc2.pose.pose.position.x = -0.6
    pc2.pose.pose.orientation.w = 1.0
    cov36 = [0.0] * 36
    cov36[0], cov36[1], cov36[7] = 0.04, 0.015, 0.01
    pc2.pose.covariance = cov36
    add('pose_cov', 'geometry_msgs/msg/PoseWithCovarianceStamped', pc2, {
        'position': [-0.6, 0.0, 0.0],
        'cov0': 0.04, 'cov1': 0.015, 'cov7': 0.01, 'cov35': 0.0,
    })

    ws = WrenchStamped()
    ws.header = _hdr()
    ws.wrench.force.x, ws.wrench.torque.z = 1.5, -0.25
    add('wrench', 'geometry_msgs/msg/WrenchStamped', ws, {
        'force': [1.5, 0.0, 0.0], 'torque': [0.0, 0.0, -0.25],
    })

    mk = Marker()
    mk.header = _hdr()
    mk.ns, mk.id, mk.type, mk.action = 'gold', 7, Marker.SPHERE_LIST, Marker.ADD
    mk.scale.x = mk.scale.y = mk.scale.z = 0.1
    mk.pose.orientation.w = 1.0
    p0, p1 = Point(), Point()
    p0.x, p1.x = 1.0, 2.0
    mk.points = [p0, p1]
    c0 = ColorRGBA()
    c0.r, c0.a = 1.0, 0.5
    mk.colors = [c0]
    add('marker', 'visualization_msgs/msg/Marker', mk, {
        'ns': 'gold', 'id': 7, 'type': 7, 'action': 0,
        'points': [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        'colors': [{'r': 1.0, 'a': 0.5}],
    })

    ma = MarkerArray()
    m1 = Marker()
    m1.header = _hdr()
    m1.ns, m1.id, m1.type = 'g1', 1, Marker.CUBE
    m1.text = '你好 gold'
    m1.pose.orientation.w = 1.0
    ma.markers.append(m1)
    add('marker_array', 'visualization_msgs/msg/MarkerArray', ma, {
        'markers_len': 1, 'm0_ns': 'g1', 'm0_type': 1, 'm0_text': '你好 gold',
    })

    da = DiagnosticArray()
    da.header = _hdr()
    st = DiagnosticStatus()
    st.name, st.level, st.message = 'g/status', b'\x00', 'ok'  # int8 标量须 bytes
    kv = KeyValue()
    kv.key, kv.value = 'k', 'v'
    st.values.append(kv)
    da.status.append(st)
    add('diagnostics', 'diagnostic_msgs/msg/DiagnosticArray', da, {
        'status_len': 1, 'name': 'g/status', 'level': 0, 'message': 'ok',
        'values': [{'key': 'k', 'value': 'v'}],
    })

    js = JointState()
    js.header = _hdr()
    js.name = ['a', 'b']
    js.position = [0.5, -0.5]
    js.velocity = [1.0, 2.0]
    js.effort = [10.0, 20.0]
    add('joint_state', 'sensor_msgs/msg/JointState', js, {
        'name': ['a', 'b'], 'position': [0.5, -0.5],
        'velocity': [1.0, 2.0], 'effort': [10.0, 20.0],
    })

    tf = TFMessage()
    t = TransformStamped()
    t.header = _hdr('map')
    t.child_frame_id = 'odom'
    t.transform.translation.x = 1.5
    t.transform.rotation.w = 1.0
    tf.transforms.append(t)
    add('tf_message', 'tf2_msgs/msg/TFMessage', tf, {
        'transforms_len': 1, 'child': 'odom',
        'translation': [1.5, 0.0, 0.0],
    })

    return cases


def main() -> None:
    cases = build_cases()
    OUT.write_text(
        json.dumps({'cases': cases}, ensure_ascii=False, indent=1),
        encoding='utf-8')
    print(f'写入 {len(cases)} 个用例 → {OUT}')
    for c in cases:
        print(f"  {c['name']:18s} {c['type']:38s} {len(c['hex']) // 2} B schema={len(c['schema'])}ch")


if __name__ == '__main__':
    main()
