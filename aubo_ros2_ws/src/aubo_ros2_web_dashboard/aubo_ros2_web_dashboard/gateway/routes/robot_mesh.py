"""机器人 3D 模型文件服务 — GET /api/ivg/robot-mesh/{pkg}/{path}。

根据 ROS 包名定位 share 目录，在其中查找 STL/DAE/OBJ 网格文件并返回。
支持大小写不敏感的文件名匹配。
"""
from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse

router = APIRouter(prefix="/api/ivg/robot-mesh", tags=["ivg-robot-mesh"])


def _media_type(path: Path) -> str:
    """根据文件扩展名返回 MIME 类型。"""
    s = path.suffix.lower()
    if s == ".stl": return "model/stl"
    if s == ".dae": return "model/vnd.collada+xml"
    if s == ".obj": return "model/obj"
    return "application/octet-stream"


def _is_under(candidate: Path, parent: Path) -> bool:
    """检查 candidate 是否在 parent 目录树下（防路径穿越）。"""
    try:
        candidate.absolute().relative_to(parent.absolute())
        return True
    except ValueError:
        return False


def _resolve_mesh(share: Path, rel_path: str) -> Path | None:
    """在 share 目录中解析网格文件路径，大小写不敏感。"""
    share = share.resolve()

    # 直接路径
    direct = share / rel_path
    if direct.is_file() and _is_under(direct, share):
        return direct

    parts = [p for p in Path(rel_path).parts if p not in ("", ".")]
    if not parts:
        return None

    # 逐级解析目录（大小写不敏感）
    current = share
    for part in parts[:-1]:
        nxt = current / part
        if nxt.is_dir():
            current = nxt
            continue
        # 大小写不敏感匹配
        found = None
        try:
            for c in current.iterdir():
                if c.is_dir() and c.name.lower() == part.lower():
                    found = c
                    break
        except OSError:
            return None
        if found is None:
            return None
        current = found

    # 匹配最终文件名
    last = parts[-1]
    cand = current / last
    if cand.is_file() and _is_under(cand, share):
        return cand
    try:
        for c in current.iterdir():
            if c.is_file() and c.name.lower() == last.lower() and _is_under(c, share):
                return c
    except OSError:
        return None
    return None


@router.get("/{remainder:path}")
def serve_robot_mesh(remainder: str) -> FileResponse:
    """获取机器人网格文件: /api/ivg/robot-mesh/{ROS包名}/{相对路径}"""
    # 安全检查
    if not remainder or remainder.startswith("/") or ".." in remainder.split("/"):
        raise HTTPException(status_code=400, detail="非法路径")

    parts = remainder.split("/", 1)
    if len(parts) != 2 or not parts[0] or not parts[1]:
        raise HTTPException(status_code=404, detail="需要 包名/相对路径 格式")

    pkg, rel_path = parts[0], parts[1]
    if ".." in rel_path.split("/"):
        raise HTTPException(status_code=400, detail="非法路径")

    # 获取包 share 目录
    try:
        share = Path(get_package_share_directory(pkg)).resolve()
    except PackageNotFoundError as exc:
        raise HTTPException(status_code=404, detail=f"包不存在: {pkg}") from exc

    # 查找文件
    target = _resolve_mesh(share, rel_path)
    if target is None or not target.is_file():
        raise HTTPException(status_code=404, detail="文件未找到")

    return FileResponse(
        target,
        media_type=_media_type(target),
        filename=target.name,
        headers={"Cache-Control": "public, max-age=86400"},
    )
