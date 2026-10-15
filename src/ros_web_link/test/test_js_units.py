"""JS 纯核单测包装 — 调 vitest 跑 web/public/js 下通信层测试.

依赖复用 aubo_ros2_web_dashboard 的 node_modules（本包不重复安装）：
首次运行自动建 node_modules 符号链接（与 launch_gateway 的 JS 合并同款模式）。
链接/vitest 不在时 skip（需要 node 环境的机器上先在 dashboard 执行过 npm install）。
"""
from __future__ import annotations

import asyncio
import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parent.parent
_DASH_NM = _PKG_ROOT.parent.parent / 'src' / 'ivg' / 'aubo_ros2_web_dashboard' / 'node_modules'
_ROOT_S = str(_PKG_ROOT)


def _ensure_node_modules() -> bool:
    """本包 node_modules 链到 dashboard 的（ESM 解析不认 sibling 目录）."""
    nm = _PKG_ROOT / 'node_modules'
    if nm.is_symlink() or nm.is_dir():
        return True
    if not _DASH_NM.is_dir():
        return False
    try:
        nm.symlink_to(_DASH_NM, target_is_directory=True)
        return True
    except OSError:
        return False


def test_js_units():
    vitest_bin = _PKG_ROOT / 'node_modules' / '.bin' / 'vitest'
    if not vitest_bin.is_file() and not _ensure_node_modules():
        sys.stdout.write('SKIP: node_modules 不在（先在 aubo_ros2_web_dashboard npm install）\n')
        return
    if not vitest_bin.is_file():
        sys.stdout.write('SKIP: vitest 不在 node_modules\n')
        return

    async def run_vitest() -> tuple[int, str]:
        proc = await asyncio.create_subprocess_exec(
            str(vitest_bin), 'run', '--root', _ROOT_S,
            cwd=_ROOT_S,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        out = await proc.communicate()
        text = (out[0] or b'').decode('utf-8', errors='replace')
        return proc.returncode, text

    code, text = asyncio.run(run_vitest())
    tail = '\n'.join(text.splitlines()[-12:])
    assert code == 0, f'vitest 退出码 {code}:\n{tail}'
    sys.stdout.write(tail + '\n')
