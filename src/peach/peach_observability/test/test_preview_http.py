"""预览 HTTP 必须只取缓存并保留固定静态白名单与安全响应头."""
import json
from urllib.error import HTTPError
from urllib.request import urlopen

from peach_observability.http_server import start_http
import pytest


class Backend:
    """真实 HTTP 测试的只读字节后端."""

    def image(self, stream):
        """原始流无帧，perception 固定缓存 JPEG."""
        if stream == 'raw':
            return None, {'error': 'no_frame', 'available': False}
        return b'\xff\xd8\xff\xd9', {'stamp_ns': '1234567891234567890',
                                     'frame_id': 'camera_optical', 'age_s': 0.25}


@pytest.fixture
def endpoint(tmp_path):
    """启动临时 HTTP 并保证测试后关闭服务."""
    (tmp_path / 'vendor').mkdir()
    for path in ['vendor/peach-three.js', 'viz3d.js', 'visualization.js',
                 'visualization.css', 'workbench.js', 'workbench.css']:
        (tmp_path / path).write_text('asset')
    (tmp_path / 'secret.txt').write_text('secret')
    server = start_http('127.0.0.1', 0, tmp_path, Backend(), lambda _msg: None)
    try:
        yield f'http://127.0.0.1:{server.server_address[1]}'
    finally:
        server.shutdown()
        server.server_close()


def test_jpeg_binary_headers_and_missing_reason(endpoint):
    """二进制不走 JSON；原始 ns 不丢精度；无帧返回有原因的404."""
    with urlopen(endpoint + '/api/images/perception.jpg') as response:
        assert response.read() == b'\xff\xd8\xff\xd9'
        assert response.headers['Content-Type'] == 'image/jpeg'
        assert response.headers['X-Frame-Stamp-Ns'] == '1234567891234567890'
        assert response.headers['X-Frame-Id'] == 'camera_optical'
        assert response.headers['X-Frame-Age-S'] == '0.25'
        assert response.headers['Cache-Control'] == 'no-store'
        assert "default-src 'self'" in response.headers['Content-Security-Policy']
        assert "img-src 'self' blob:" in response.headers['Content-Security-Policy']
    with pytest.raises(HTTPError) as exc:
        urlopen(endpoint + '/api/images/raw.jpg')
    assert exc.value.code == 404
    assert json.loads(exc.value.read())['error'] == 'no_frame'


@pytest.mark.parametrize('path', ['vendor/peach-three.js', 'viz3d.js',
                                  'visualization.js', 'visualization.css',
                                  'workbench.js', 'workbench.css'])
def test_fixed_static_assets(endpoint, path):
    """显式白名单资源可访问，同源 CSP 允许本地模块."""
    with urlopen(endpoint + '/' + path) as response:
        assert response.read() == b'asset'
        assert 'unsafe-inline' not in response.headers['Content-Security-Policy']
    with pytest.raises(HTTPError):
        urlopen(endpoint + '/secret.txt')


def test_clock_mismatch_cached_jpeg_returns_404(endpoint, monkeypatch):
    """即使有缓存 JPEG，ROS 时间域不匹配也返回 JSON404."""
    monkeypatch.setattr(Backend, 'image', lambda self, stream: (
        b'cached', {'error': 'clock_mismatch', 'available': False,
                    'stamp_ns': '20', 'frame_id': 'camera_optical', 'age_s': -0.25}))
    with pytest.raises(HTTPError) as exc:
        urlopen(endpoint + '/api/images/raw.jpg')
    assert exc.value.code == 404
    assert json.loads(exc.value.read())['error'] == 'clock_mismatch'


def test_stale_cached_jpeg_still_returns_binary(endpoint, monkeypatch):
    """陈帧允许保留最近图像，与时间域不匹配区别处理."""
    monkeypatch.setattr(Backend, 'image', lambda self, stream: (
        b'cached', {'error': 'stale', 'available': False,
                    'stamp_ns': '20', 'frame_id': 'camera_optical', 'age_s': 5.0}))
    with urlopen(endpoint + '/api/images/raw.jpg') as response:
        assert response.status == 200
        assert response.read() == b'cached'
        assert response.headers['X-Frame-Age-S'] == '5.0'
