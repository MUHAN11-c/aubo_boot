"""有界只读图像预览：每流最新槽，单线程限频编码，HTTP 只取缓存."""
from __future__ import annotations

import threading
import time

import cv2
from cv_bridge import CvBridge

STREAMS = ('perception', 'raw')


def encode_jpeg(message):
    """用 Jazzy CvBridge 转 BGR8，缩到长边 960 后 JPEG80 编码."""
    if not (0 < message.width <= 16384 and 0 < message.height <= 16384):
        raise ValueError('invalid_dimensions')
    if message.width * message.height > 16_777_216:
        raise ValueError('image_too_large')
    frame = CvBridge().imgmsg_to_cv2(message, desired_encoding='bgr8')
    height, width = frame.shape[:2]
    ratio = min(1.0, 960 / max(height, width))
    if ratio < 1:
        frame = cv2.resize(frame, (max(1, round(width * ratio)),
                                   max(1, round(height * ratio))),
                           interpolation=cv2.INTER_AREA)
    encoded, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not encoded:
        raise ValueError('jpeg_encoder_failed')
    return jpeg.tobytes(), frame.shape[1], frame.shape[0]


class ImagePreview:
    """有限内存图像缓存；每流最多一个待编码帧，所有流共用一个编码任务."""

    def __init__(self, encoder=encode_jpeg):
        """启动工作线程；编码速率用单调时钟，帧年龄由调用者 ROS 时钟计算."""
        self._encoder = encoder
        self._condition = threading.Condition()
        self._pending = {}
        self._records = {}
        self._jpeg = {}
        self._last_encoded = dict.fromkeys(STREAMS, 0.0)
        self._next_encode = 0.0
        self._closed = False
        self._worker = threading.Thread(target=self._run,
                                        name='peach-image-preview', daemon=True)
        self._worker.start()

    @property
    def worker_alive(self):
        """工作线程是否仍存活（收尾验证用）."""
        return self._worker.is_alive()

    @property
    def pending_count(self):
        """返回有界待处理槽数量."""
        with self._condition:
            return len(self._pending)

    def submit(self, stream, message):
        """替换该流最新待处理帧，订阅回调不编码、不等待工作线程."""
        if stream not in STREAMS:
            raise ValueError('unknown_stream')
        with self._condition:
            if self._closed:
                return
            self._pending[stream] = message
            self._condition.notify()

    def _run(self):
        while True:
            with self._condition:
                while not self._closed:
                    now = time.monotonic()
                    if self._pending and self._next_encode <= now:
                        stream = min(self._pending,
                                     key=lambda key: self._last_encoded[key])
                        message = self._pending.pop(stream)
                        self._last_encoded[stream] = now
                        break
                    delay = self._next_encode - now if self._pending else None
                    self._condition.wait(delay)
                if self._closed:
                    return
            stamp = message.header.stamp
            ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            record = {'available': False, 'stamp_sec': ns * 1e-9,
                      'stamp_ns': str(ns), 'frame_id': message.header.frame_id,
                      'width': 0, 'height': 0, 'error': '', 'age_s': None}
            try:
                data, width, height = self._encoder(message)
                record.update(available=True, width=width, height=height)
            except Exception as exc:  # noqa: BLE001 native encoder boundary
                data = None
                record['error'] = f'encode_failed:{exc}'
            with self._condition:
                if self._closed:
                    return
                # 编码完成后留足 200ms，全局所有调用之间至少 200ms.
                self._next_encode = time.monotonic() + 0.2
                self._records[stream] = record
                self._jpeg[stream] = data

    def _metadata_locked(self, stream, now_sec):
        row = dict(self._records.get(stream, {
            'available': False, 'stamp_sec': None, 'stamp_ns': None,
            'frame_id': '', 'width': 0, 'height': 0, 'error': 'no_frame',
            'age_s': None}))
        if row['stamp_sec'] is not None:
            age = now_sec - row['stamp_sec']
            row['age_s'] = age if age <= -0.1 else max(0.0, age)
            if row['available'] and age <= -0.1:
                row.update(available=False, error='clock_mismatch')
            elif row['available'] and row['age_s'] > 1.0:
                row.update(available=False, error='stale')
        return row

    def metadata(self, now_sec):
        """用同一 ROS 时间域计算陈旧状态；不在请求线程编码."""
        with self._condition:
            return {key: self._metadata_locked(key, now_sec) for key in STREAMS}

    def jpeg(self, stream, now_sec):
        """原子取 JPEG 与对应帧元数据，陈帧保留字节并标 stale."""
        with self._condition:
            return self._jpeg.get(stream), self._metadata_locked(stream, now_sec)

    def close(self):
        """停收帧、清槽并等待唯一编码任务结束，释放全部图像缓存."""
        with self._condition:
            self._closed = True
            self._pending.clear()
            self._condition.notify_all()
        self._worker.join()
        with self._condition:
            self._records.clear()
            self._jpeg.clear()

    def __enter__(self):
        """允许纯核测试与非 ROS 调用者显式管理生命周期."""
        return self

    def __exit__(self, *_args):
        """退出上下文时关闭工作线程."""
        self.close()
