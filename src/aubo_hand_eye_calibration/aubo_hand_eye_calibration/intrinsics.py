"""
彩色相机内参标定产物的解析、校验与落盘 (纯核, 无 ROS 依赖).

配合 vendored camera_calibration (image_pipeline jazzy 分支) 使用:

    ros2 launch aubo_hand_eye_calibration intrinsics_calibration.launch.py
    # GUI 里 CALIBRATE -> SAVE, 产物在 /tmp/calibrationdata.tar.gz
    ros2 run aubo_hand_eye_calibration apply_intrinsics /tmp/calibrationdata.tar.gz

落盘目标 src/percipio_camera/config/color_camera_info.yaml 是全项目
彩色内参唯一事实源 (percipio 与 peach_stereo 两前端共用, 决策 7a)。
"""

import argparse
from pathlib import Path
import tarfile

import yaml

from .storage import atomic_write_yaml

# cameracalibrator SAVE 的 tarball 内固定成员名 (calibrator.do_tarfile_save)
TARBALL_MEMBER = 'ost.yaml'

# 与两前端的 640x480 彩色流一致; 换分辨率重标定时显式传 --width/--height
DEFAULT_EXPECTED_WIDTH = 640
DEFAULT_EXPECTED_HEIGHT = 480

_MATRIX_SHAPES = {
    'camera_matrix': (3, 3),
    'rectification_matrix': (3, 3),
    'projection_matrix': (3, 4),
}
_DISTORTION_MODEL_COEFFICIENTS = {
    'plumb_bob': 5,
    'rational_polynomial': 8,
}


class IntrinsicsError(ValueError):
    """标定产物结构或数值不合法."""


def load_calibration_document(source):
    """
    读取标定产物, 返回 (字典, 来源路径).

    source 可以是 cameracalibrator SAVE 的 tarball (取 ost.yaml 成员),
    ost.yaml / camera_calibration_parsers 格式的 yaml, 或本包
    candidate/active.yaml (取顶层 intrinsics 增量节)。
    """
    source = Path(source)
    if not source.is_file():
        raise IntrinsicsError(f'标定产物不存在: {source}')
    if tarfile.is_tarfile(source):
        with tarfile.open(source, 'r:*') as archive:
            try:
                member = archive.getmember(TARBALL_MEMBER)
            except KeyError as error:
                raise IntrinsicsError(
                    f'tarball 内无 {TARBALL_MEMBER} 成员: {source}') from error
            text = archive.extractfile(member).read().decode('utf-8')
        return yaml.safe_load(text), source
    document = yaml.safe_load(source.read_text(encoding='utf-8'))
    if (isinstance(document, dict) and 'intrinsics' in document
            and 'camera_matrix' not in document):
        # 本包 candidate/active.yaml: 联合标定产物存在 hand_eye/ 目录,
        # intrinsics 节即等价 OST 文档
        nested = document['intrinsics']
        if not isinstance(nested, dict):
            raise IntrinsicsError('intrinsics 节不是键值字典')
        return nested, source
    return document, source


def _matrix_values(document, key):
    entry = document.get(key)
    rows, cols = _MATRIX_SHAPES[key]
    if not isinstance(entry, dict):
        raise IntrinsicsError(f'缺少 {key} 矩阵')
    if entry.get('rows') != rows or entry.get('cols') != cols:
        raise IntrinsicsError(f'{key} 形状应为 {rows}x{cols}')
    data = entry.get('data')
    if not isinstance(data, list) or len(data) != rows * cols:
        raise IntrinsicsError(f'{key}.data 应为 {rows * cols} 个元素')
    try:
        return [float(value) for value in data]
    except (TypeError, ValueError) as error:
        raise IntrinsicsError(f'{key}.data 含非数值') from error


def validate_intrinsics(
    document,
    expected_width=DEFAULT_EXPECTED_WIDTH,
    expected_height=DEFAULT_EXPECTED_HEIGHT,
    allow_size_mismatch=False,
):
    """校验 OST 标定字典; 非法即抛 IntrinsicsError, 通过则原样返回."""
    if not isinstance(document, dict):
        raise IntrinsicsError('标定产物不是键值字典')
    if not document.get('camera_name'):
        raise IntrinsicsError('缺少 camera_name')

    width = document.get('image_width')
    height = document.get('image_height')
    if (
        not isinstance(width, int) or not isinstance(height, int)
        or width <= 0 or height <= 0
    ):
        raise IntrinsicsError(f'分辨率非法: {width}x{height}')
    if (width, height) != (expected_width, expected_height):
        if not allow_size_mismatch:
            raise IntrinsicsError(
                f'分辨率 {width}x{height} 与预期 '
                f'{expected_width}x{expected_height} 不符 '
                '(确认标定时相机流分辨率后可用 --allow-size-mismatch 覆盖)')

    model = document.get('distortion_model')
    if model not in _DISTORTION_MODEL_COEFFICIENTS:
        raise IntrinsicsError(f'畸变模型不支持: {model}')
    distortion = document.get('distortion_coefficients')
    if not isinstance(distortion, dict):
        raise IntrinsicsError('缺少 distortion_coefficients')
    expected_coefficients = _DISTORTION_MODEL_COEFFICIENTS[model]
    distortion_data = distortion.get('data')
    if (
        not isinstance(distortion_data, list)
        or len(distortion_data) != expected_coefficients
    ):
        raise IntrinsicsError(
            f'{model} 畸变系数应为 {expected_coefficients} 个')
    try:
        [float(value) for value in distortion_data]
    except (TypeError, ValueError) as error:
        raise IntrinsicsError('distortion_coefficients.data 含非数值') from error

    camera_matrix = _matrix_values(document, 'camera_matrix')
    _matrix_values(document, 'rectification_matrix')
    _matrix_values(document, 'projection_matrix')
    fx, skew, cx, _, fy, cy = camera_matrix[:6]
    if skew != 0.0:
        raise IntrinsicsError(f'K 矩阵含非零偏斜项: {skew}')
    if fx <= 0.0 or fy <= 0.0:
        raise IntrinsicsError(f'焦距非正: fx={fx} fy={fy}')
    if not (0.0 <= cx <= width and 0.0 <= cy <= height):
        raise IntrinsicsError(
            f'主点越界: cx={cx} cy={cy} (图像 {width}x{height})')
    return document


def default_output_path():
    """定位 src/percipio_camera/config/color_camera_info.yaml (源码树向上查找)."""
    for parent in Path(__file__).resolve().parents:
        package = parent / 'src' / 'percipio_camera'
        if package.is_dir():
            return package / 'config' / 'color_camera_info.yaml'
    raise IntrinsicsError(
        '未找到 src/percipio_camera 源码树; 请用 --output 显式指定落盘路径')


def summarize(document):
    """单行摘要, 供 CLI 与日志核对."""
    k = document['camera_matrix']['data']
    distortion = document['distortion_coefficients']['data']
    return (
        f"{document['image_width']}x{document['image_height']} "
        f"{document['distortion_model']} "
        f'fx={k[0]:.3f} fy={k[4]:.3f} cx={k[2]:.3f} cy={k[5]:.3f} '
        f'D={distortion}')


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='apply_intrinsics',
        description='校验并原子落盘彩色相机内参标定产物')
    parser.add_argument(
        'source',
        help='cameracalibrator SAVE 产物 (calibrationdata.tar.gz)、'
             'ost.yaml 或本包 hand_eye/active.yaml (取 intrinsics 节)')
    parser.add_argument(
        '-o', '--output', default=None,
        help='落盘路径 (默认: src/percipio_camera/config/color_camera_info.yaml)')
    parser.add_argument(
        '--width', type=int, default=DEFAULT_EXPECTED_WIDTH,
        help=f'预期图像宽 (默认 {DEFAULT_EXPECTED_WIDTH})')
    parser.add_argument(
        '--height', type=int, default=DEFAULT_EXPECTED_HEIGHT,
        help=f'预期图像高 (默认 {DEFAULT_EXPECTED_HEIGHT})')
    parser.add_argument(
        '--allow-size-mismatch', action='store_true',
        help='允许标定分辨率与预期不一致 (需人工确认前端流也换了分辨率)')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='只校验与显示, 不写入')
    args = parser.parse_args(argv)

    try:
        document, source = load_calibration_document(args.source)
        validate_intrinsics(
            document, args.width, args.height, args.allow_size_mismatch)
        output = Path(args.output) if args.output else default_output_path()
    except (IntrinsicsError, OSError, yaml.YAMLError) as error:
        parser.exit(2, f'apply_intrinsics: 错误: {error}\n')

    print(f'内参来源: {source}')
    print(f'内参摘要: {summarize(document)}')
    if args.dry_run:
        print(f'[dry-run] 未写入: {output}')
        return 0
    atomic_write_yaml(output, document)
    print(f'已原子写入: {output}')
    print('后续步骤:')
    print('  1. colcon build --packages-select percipio_camera '
          '(launch 经 FindPackageShare 读 install 副本, 重建后新内参才进前端)')
    print('  2. 重启相机前端 (percipio 或 stereo) 生效')
    print('  3. 验证: ros2 topic echo /camera/color/camera_info --once 核对 K')
    print('  4. 手动更新 src/peach_harvester/config/scene_perception.yaml '
          '的 calibration_version 标签')
    return 0
