"""只读预览的真实 JPEG 边界、最新槽和生命周期."""
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from peach_observability.image_preview import ImagePreview
from sensor_msgs.msg import Image


def image(width=1200, height=600, stamp=10):
    """构造实际 ROS 彩色帧，保留可验证的原始时间戳."""
    msg = CvBridge().cv2_to_imgmsg(
        np.zeros((height, width, 3), dtype=np.uint8), encoding='bgr8')
    msg.header.stamp.sec = stamp
    msg.header.stamp.nanosec = 123
    msg.header.frame_id = 'camera_optical'
    return msg


def wait_result(cache, stream='perception', predicate=lambda row: row['stamp_sec']):
    """有限等待后台结果；不依赖测试设备或 ROS executor."""
    end = time.monotonic() + 3
    while time.monotonic() < end:
        row = cache.metadata(10.0)[stream]
        if predicate(row):
            return row
        time.sleep(0.01)
    raise AssertionError('预览编码线程未产出结果')


def test_no_frame_and_shutdown():
    """未收帧原因明确；关闭线程并拒收后续帧."""
    cache = ImagePreview()
    assert cache.jpeg('raw', 10)[0] is None
    assert cache.metadata(10)['raw']['error'] == 'no_frame'
    cache.close()
    cache.submit('raw', image())
    assert not cache.worker_alive
    assert cache.jpeg('raw', 10)[0] is None


def test_jpeg_dimensions_stamp_and_stale():
    """JPEG 最长边 960，头对应原帧且过旧不标可用."""
    with ImagePreview() as cache:
        cache.submit('perception', image())
        row = wait_result(cache)
        data, metadata = cache.jpeg('perception', 10.1)
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert decoded.shape[:2] == (480, 960)
        assert row['width'] == 960 and row['height'] == 480
        assert metadata['stamp_ns'] == '10000000123'
        assert metadata['frame_id'] == 'camera_optical'
        old = cache.metadata(20)['perception']
        assert old['available'] is False and old['error'] == 'stale'
        assert old['age_s'] > 9


def test_bad_frame_invalidates_previous_jpeg():
    """坏帧不能把旧 JPEG 与新鲜 stamp 配对."""
    with ImagePreview() as cache:
        cache.submit('perception', image())
        wait_result(cache)
        bad = Image(width=5, height=5, encoding='invalid')
        bad.header.stamp.sec = 11
        cache.submit('perception', bad)
        row = wait_result(cache, predicate=lambda row: row['stamp_sec'] == 11)
        assert row['available'] is False
        assert row['error'].startswith('encode_failed:')
        assert cache.jpeg('perception', 11)[0] is None


def test_latest_slots_bounded_single_worker_and_rate():
    """并发输入只保留每流一个最新槽，编码串行且不超过 5Hz."""
    active = []
    starts = []
    gate = threading.Event()

    def encode(msg):
        active.append(msg.header.stamp.sec)
        assert len(active) == 1
        starts.append(time.monotonic())
        gate.wait(1)
        active.pop()
        return b'jpeg', 1, 1

    with ImagePreview(encoder=encode) as cache:
        cache.submit('raw', image(2, 2, 1))
        end = time.monotonic() + 1
        while not active and time.monotonic() < end:
            time.sleep(0.005)
        for stamp in range(2, 20):
            cache.submit('raw', image(2, 2, stamp))
        assert cache.pending_count <= 2
        gate.set()
        row = wait_result(cache, 'raw', lambda row: int(row['stamp_sec'] or 0) == 19)
        assert int(row['stamp_sec'] or 0) == 19
        assert len(starts) == 2
        assert starts[1] - starts[0] >= 0.19


def test_ros_clock_backwards_invalidates_freshness():
    """ROS 时钟回退后，未来帧不能被零年龄伪装成新鲜."""
    with ImagePreview() as cache:
        cache.submit('perception', image())
        wait_result(cache)
        row = cache.metadata(9.5)['perception']
        assert not row['available'] and row['error'] == 'clock_mismatch'
        assert row['age_s'] < -0.49


def test_two_streams_share_global_five_hz_encoding_limit():
    """两流连续输入也必须共享全局限频，并公平处理最新槽."""
    starts = []

    def encode(msg):
        starts.append((msg.header.frame_id, time.monotonic()))
        return b'jpeg', 1, 1

    with ImagePreview(encoder=encode) as cache:
        for stream in ('raw', 'perception'):
            msg = image(2, 2)
            msg.header.frame_id = stream
            cache.submit(stream, msg)
        wait_result(cache, 'raw')
        wait_result(cache, 'perception')
        assert len(starts) == 2
        assert {stream for stream, _stamp in starts} == {'raw', 'perception'}
        assert starts[1][1] - starts[0][1] >= 0.2
