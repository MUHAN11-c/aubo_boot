"""Zero-ROS tests for the intrinsics parser/validator/CLI."""

import io
import tarfile

from aubo_hand_eye_calibration import intrinsics
from aubo_hand_eye_calibration.intrinsics import (
    IntrinsicsError,
    load_calibration_document,
    validate_intrinsics,
)
import pytest
import yaml


def ost_document(width=640, height=480):
    return {
        'image_width': width,
        'image_height': height,
        'camera_name': 'camera_color',
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': [466.17, 0.0, 326.07, 0.0, 465.56, 244.79,
                     0.0, 0.0, 1.0],
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 5, 'cols': 1,
            'data': [-0.33, 0.12, -0.001, -0.0004, 0.0],
        },
        'rectification_matrix': {
            'rows': 3, 'cols': 3,
            'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        },
        'projection_matrix': {
            'rows': 3, 'cols': 4,
            'data': [466.17, 0.0, 326.07, 0.0,
                     0.0, 465.56, 244.79, 0.0,
                     0.0, 0.0, 1.0, 0.0],
        },
    }


def write_tarball(path, document, include_ost=True):
    stream = io.BytesIO(yaml.safe_dump(document).encode('utf-8'))
    with tarfile.open(path, 'w:gz') as archive:
        image = io.BytesIO(b'not-a-real-png')
        member = tarfile.TarInfo('left-0000.png')
        member.size = len(image.getvalue())
        archive.addfile(tarinfo=member, fileobj=image)
        if include_ost:
            member = tarfile.TarInfo('ost.yaml')
            member.size = len(stream.getvalue())
            archive.addfile(tarinfo=member, fileobj=stream)


def test_validate_accepts_ost_document():
    document = ost_document()
    assert validate_intrinsics(document) is document


def test_load_from_tarball(tmp_path):
    tarball = tmp_path / 'calibrationdata.tar.gz'
    write_tarball(tarball, ost_document())
    document, source = load_calibration_document(tarball)
    assert source == tarball
    assert document['camera_matrix']['data'][0] == pytest.approx(466.17)


def test_load_from_yaml_file(tmp_path):
    path = tmp_path / 'ost.yaml'
    path.write_text(yaml.safe_dump(ost_document()), encoding='utf-8')
    document, source = load_calibration_document(path)
    assert source == path
    assert document['camera_name'] == 'camera_color'


def test_load_from_package_candidate_yaml(tmp_path):
    """本包 candidate/active.yaml: 取顶层 intrinsics 增量节 (joint 档产物)."""
    package_document = {
        'schema_version': 1,
        'candidate_id': '20260917T000000_000000Z',
        'transforms': {'wrist_from_camera_optical': {'xyz_m': [0, 0, 0]}},
        'intrinsics': ost_document(),
    }
    path = tmp_path / 'active.yaml'
    path.write_text(yaml.safe_dump(package_document), encoding='utf-8')
    document, source = load_calibration_document(path)
    assert source == path
    assert document['camera_name'] == 'camera_color'
    assert document['camera_matrix']['data'][0] == pytest.approx(466.17)


def test_load_missing_source_raises(tmp_path):
    with pytest.raises(IntrinsicsError, match='不存在'):
        load_calibration_document(tmp_path / 'nowhere.tar.gz')


def test_load_tarball_without_ost_raises(tmp_path):
    tarball = tmp_path / 'bad.tar.gz'
    write_tarball(tarball, ost_document(), include_ost=False)
    with pytest.raises(IntrinsicsError, match='ost.yaml'):
        load_calibration_document(tarball)


@pytest.mark.parametrize('mutate,match', [
    (lambda d: d.pop('camera_name'), 'camera_name'),
    (lambda d: d.update(image_width=320, image_height=240), '分辨率'),
    (lambda d: d.update(distortion_model='fisheye'), '畸变模型'),
    (lambda d: d['distortion_coefficients'].update(data=[0.1, 0.2]),
     '畸变系数'),
    (lambda d: d['camera_matrix'].update(rows=2), '形状'),
    (lambda d: d['camera_matrix']['data'].__setitem__(0, -1.0), '焦距'),
    (lambda d: d['camera_matrix']['data'].__setitem__(1, 0.5), '偏斜'),
    (lambda d: d['camera_matrix']['data'].__setitem__(2, 900.0), '主点'),
])
def test_validate_rejects_bad_documents(mutate, match):
    document = ost_document()
    mutate(document)
    with pytest.raises(IntrinsicsError, match=match):
        validate_intrinsics(document)


def test_validate_rejects_non_dict():
    with pytest.raises(IntrinsicsError, match='不是键值字典'):
        validate_intrinsics([1, 2, 3])


def test_validate_allows_size_mismatch_when_requested():
    document = ost_document(width=1280, height=720)
    with pytest.raises(IntrinsicsError):
        validate_intrinsics(document, 640, 480)
    validate_intrinsics(document, 640, 480, allow_size_mismatch=True)


def test_main_dry_run_does_not_write(tmp_path, capsys):
    tarball = tmp_path / 'calibrationdata.tar.gz'
    write_tarball(tarball, ost_document())
    output = tmp_path / 'out.yaml'
    code = intrinsics.main([str(tarball), '--output', str(output), '--dry-run'])
    assert code == 0
    assert not output.exists()
    assert 'dry-run' in capsys.readouterr().out


def test_main_writes_output_atomically(tmp_path, capsys):
    tarball = tmp_path / 'calibrationdata.tar.gz'
    write_tarball(tarball, ost_document())
    output = tmp_path / 'out.yaml'
    output.write_text('stale: true\n', encoding='utf-8')
    code = intrinsics.main([str(tarball), '--output', str(output)])
    assert code == 0
    document = yaml.safe_load(output.read_text(encoding='utf-8'))
    assert document['camera_matrix']['data'][0] == pytest.approx(466.17)
    assert '已原子写入' in capsys.readouterr().out
    assert list(tmp_path.glob('.*.tmp')) == []


def test_main_exits_on_size_mismatch(tmp_path):
    tarball = tmp_path / 'calibrationdata.tar.gz'
    write_tarball(tarball, ost_document(width=320, height=240))
    with pytest.raises(SystemExit) as excinfo:
        intrinsics.main([str(tarball), '--output',
                         str(tmp_path / 'out.yaml')])
    assert excinfo.value.code == 2
    assert not (tmp_path / 'out.yaml').exists()
