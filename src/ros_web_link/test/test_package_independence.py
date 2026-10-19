"""独立包门：无工作区硬编码、无 dashboard 运行时依赖、元数据/资产完整."""
from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parent.parent


def _runtime_files():
    roots = [_PKG_ROOT / 'ros_web_link', _PKG_ROOT / 'launch',
             _PKG_ROOT / 'config', _PKG_ROOT / 'web' / 'public' / 'js']
    for root in roots:
        if not root.exists():
            continue
        for f in root.rglob('*'):
            if not f.is_file():
                continue
            if {'node_modules', 'vendor', '__pycache__', '.mimosa',
                '.v2c', 'robotwebtools'} & set(f.parts):
                continue
            if f.suffix in ('.py', '.js', '.xml', '.yaml', '.html', '.css'):
                yield f


def test_no_dashboard_runtime_import():
    pat = re.compile(r'^\s*(from|import)\s+aubo_ros2_web_dashboard\b',
                     re.MULTILINE)
    bad = [str(f.relative_to(_PKG_ROOT)) for f in _runtime_files()
           if f.suffix == '.py' and pat.search(f.read_text(errors='ignore'))]
    assert not bad, bad


def test_no_workspace_hardcode_in_runtime_or_scripts():
    bad = []
    for f in list(_runtime_files()) + sorted((_PKG_ROOT / 'scripts').glob('*.sh')):
        if f.suffix not in ('.py', '.js', '.sh', '.xml', '.yaml', '.html'):
            continue
        text = f.read_text(errors='ignore')
        if '/home/mu' in text:
            bad.append(str(f.relative_to(_PKG_ROOT)))
    assert not bad, bad


def test_package_metadata_core_deps():
    root = ET.parse(_PKG_ROOT / 'package.xml').getroot()
    assert root.findtext('version').strip() == '1.1.0'
    deps = {e.text.strip() for e in root.findall('exec_depend')}
    assert {'rcl_interfaces', 'ivg_foxglove_bridge', 'rclpy'} <= deps
    setup_py = (_PKG_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert "version='1.1.0'" in setup_py
    assert "_data_tree('examples'" in setup_py
    assert "_data_tree('doc'" in setup_py


def test_examples_and_barrels_are_present():
    for rel in ('examples/README.md',
                'examples/python/01_subscribe.py',
                'examples/python/05_raw_protocol.py',
                'examples/http/README.md',
                'examples/launch/README.md',
                'web/public/js/ros_web_link.js',
                'web/public/js/transport/foxglove/index.js',
                'web/public/examples/minimal_client.html'):
        assert (_PKG_ROOT / rel).is_file(), rel


def test_monitor_entry_is_core_and_has_external_bridge_mode():
    text = (_PKG_ROOT / 'launch' / 'monitor.launch.py').read_text(encoding='utf-8')
    assert "start_bridge" in text
    assert "bridge_port" in text and "gateway_port" in text
    assert "get_package_prefix" in text
