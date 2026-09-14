"""
Zero-ROS tests for contact_monitor.hpp (synthetic current sequences).

Compiles a tiny probe against the header and runs it. Does not import rclpy
or construct a DDS graph.
"""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
HEADER = PACKAGE / 'include' / 'peach_manipulation' / 'contact_monitor.hpp'
PROBE = Path(__file__).resolve().parent / 'contact_monitor_probe.cpp'


@pytest.mark.skipif(shutil.which('g++') is None, reason='g++ not on PATH')
def test_contact_monitor_synthetic_sequences(tmp_path):
    """Compile contact_monitor.hpp probe; five synthetic current sequences."""
    assert HEADER.is_file(), HEADER
    binary = tmp_path / 'contact_monitor_probe'
    compile_cmd = [
        'g++', '-std=c++17', '-O0',
        f'-I{PACKAGE / "include"}',
        str(PROBE), '-o', str(binary),
    ]
    built = subprocess.run(
        compile_cmd, check=False, capture_output=True, text=True)
    if built.returncode != 0:
        pytest.fail(
            'g++ failed:\n' + built.stdout + built.stderr)
    ran = subprocess.run(
        [str(binary)], check=False, capture_output=True, text=True,
        env={**os.environ})
    assert ran.returncode == 0, ran.stdout + ran.stderr
    assert 'PASS' in ran.stdout


if __name__ == '__main__':
    sys.exit(pytest.main([__file__]))
