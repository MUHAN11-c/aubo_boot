"""文档相对链接门：README/doc/examples 中的链接必须可达（http/# 跳过）."""
from __future__ import annotations

import re
from pathlib import Path

import pytest

_PKG_ROOT = Path(__file__).resolve().parent.parent
_LINK = re.compile(r'\[[^\]]*\]\(([^)\s]+)\)')
_SKIP_PREFIX = ('http://', 'https://', 'mailto:', '#', 'data:')


def _markdown_files():
    files = [_PKG_ROOT / 'README.md']
    files += sorted((_PKG_ROOT / 'doc').rglob('*.md'))
    files += sorted((_PKG_ROOT / 'examples').rglob('*.md'))
    return [f for f in files if f.is_file()]


def test_docs_relative_links_exist():
    missing = []
    for md in _markdown_files():
        text = md.read_text(encoding='utf-8')
        for raw in _LINK.findall(text):
            link = raw.strip().strip('<>')
            if not link or link.startswith(_SKIP_PREFIX) or link.startswith('/'):
                continue
            target = link.split('#', 1)[0]
            if not target or '{' in target or '<' in target:
                continue
            if not (md.parent / target).exists():
                missing.append(f'{md.relative_to(_PKG_ROOT)} -> {link}')
    assert not missing, 'broken links:\n' + '\n'.join(missing)


def test_api_index_has_core_docs():
    text = (_PKG_ROOT / 'doc' / 'api' / 'README.md').read_text(encoding='utf-8')
    for name in ('python.md', 'js.md', 'http-ws.md', 'launch.md', 'protocol.md'):
        assert name in text
    assert (_PKG_ROOT / 'doc' / 'install.md').is_file()
