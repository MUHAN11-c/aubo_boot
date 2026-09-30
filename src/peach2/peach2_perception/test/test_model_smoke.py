"""
Optional smoke test of the real weights (best.pt + mobile_sam.pt) on CPU.

Runs in a subprocess under the workspace venv interpreter (torch / ultralytics are not in the
system python); skipped when the venv or the weights are absent.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest

_PKG = Path(__file__).resolve().parents[1]
_WS = _PKG.parents[2]
_VENV_PY = _WS / 'aubo_py3.12' / 'bin' / 'python'
_MODELS = _WS / 'src' / 'peach_harvester' / 'model'
_YOLO = _MODELS / 'best.pt'
_SAM = _MODELS / 'mobile_sam.pt'

_SCRIPT = textwrap.dedent("""
    import json, sys
    import numpy as np
    from peach2_perception.detector import Detector, UltralyticsYoloBackend
    from peach2_perception.segmenter import MobileSamBackend, RefineParams, Segmenter
    yolo, sam = sys.argv[1], sys.argv[2]
    det = Detector(UltralyticsYoloBackend(yolo, '', 'cpu', True, 640, False), 0.35, 0.5,
                   ('peach_bag', 'peach_nobag'), 0.6, 0.2, 0.5)
    det.check_class_names()
    img = np.full((480, 640, 3), 90, np.uint8)
    img[150:380, 280:360] = (40, 180, 220)
    dets = det.detect(img)
    seg = Segmenter(MobileSamBackend(sam, 'cpu', True, 1024), 16, 0.1, 8,
                    RefineParams(0.03, 0.01, 0.34, 5, 100))
    depth = np.full((480, 640), 0.6, np.float32)
    out, truncated = seg.segment(img, [(280, 150, 360, 380), (100, 100, 150, 200)], depth,
                                 depth > 0)
    print(json.dumps({
        'detections': len(dets), 'device': det.device,
        'masks': [None if r.mask is None else int(r.mask.sum()) for r in out],
        'shapes': [None if r.mask is None else list(r.mask.shape) for r in out],
        'truncated': truncated}))
""")


@pytest.mark.skipif(not (_VENV_PY.exists() and _YOLO.exists() and _SAM.exists()),
                    reason='aubo_py3.12 venv or model weights not available')
def test_real_models_cpu(tmp_path):
    env = dict(os.environ)
    core = _PKG.parent / 'peach2_core'
    env['PYTHONPATH'] = os.pathsep.join([str(_PKG), str(core), env.get('PYTHONPATH', '')])
    env['YOLO_CONFIG_DIR'] = str(tmp_path)
    env['CUDA_VISIBLE_DEVICES'] = ''
    probe = subprocess.run([str(_VENV_PY), '-c', 'import torch, ultralytics'], env=env,
                           capture_output=True, timeout=120)
    if probe.returncode != 0:
        pytest.skip('torch / ultralytics not importable in the venv')
    proc = subprocess.run([str(_VENV_PY), '-c', _SCRIPT, str(_YOLO), str(_SAM)], env=env,
                          capture_output=True, text=True, timeout=300)
    assert proc.returncode == 0, proc.stderr[-4000:]
    result = json.loads(proc.stdout.strip().splitlines()[-1])
    assert result['device'] == 'cpu'
    assert result['truncated'] == 0
    assert result['shapes'][0] == [480, 640]
    assert result['masks'][0] is not None and result['masks'][0] > 1000
    print(result, file=sys.stderr)
