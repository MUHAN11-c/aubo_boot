"""
同源 HTTP 提供 ``package://<pkg>/...`` 下的网格文件，供浏览器 ros3d ``UrdfClient`` 拉取。

路径格式：``/api/ivg/robot-mesh/<pkg>/<relative_path_under_share>``
例：``aubo_description/meshes/aubo_e5_10/visual/link0.DAE``

依赖运行环境中的 ``AMENT_PREFIX_PATH``（与 ``ros2 launch`` 启动网关时一致）。
"""
from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse

router = APIRouter(prefix="/api/ivg/robot-mesh", tags=["ivg-robot-mesh"])


def _suffix_media_type(path: Path) -> str:
	s = path.suffix.lower()
	if s == ".stl":
		return "model/stl"
	if s == ".dae":
		return "model/vnd.collada+xml"
	if s == ".obj":
		return "model/obj"
	return "application/octet-stream"


def _is_under_share(candidate: Path, share: Path) -> bool:
	try:
		candidate.resolve().relative_to(share.resolve())
	except ValueError:
		return False
	return True


def _resolve_mesh_under_share(share: Path, rel_path: str) -> Path | None:
	"""
	在包 share 目录下解析相对路径；若精确路径不存在，则按段做不区分大小写匹配。

	浏览器侧常将 ``.DAE/.STL`` 规范为小写，而仓库内文件多为大写扩展名；
	Linux 默认区分大小写，会导致 ``link0.dae`` 无法打开 ``link0.DAE``。
	"""
	share = share.resolve()
	direct = (share / rel_path).resolve()
	if direct.is_file() and _is_under_share(direct, share):
		return direct
	parts = [p for p in Path(rel_path).parts if p not in ("", ".")]
	if not parts:
		return None
	current = share
	for part in parts[:-1]:
		nxt = current / part
		if nxt.is_dir():
			current = nxt.resolve()
			continue
		found: Path | None = None
		try:
			for c in current.iterdir():
				if c.is_dir() and c.name.lower() == part.lower():
					found = c.resolve()
					break
		except OSError:
			return None
		if found is None:
			return None
		current = found
	last = parts[-1]
	cand = (current / last).resolve()
	if cand.is_file() and _is_under_share(cand, share):
		return cand
	try:
		for c in current.iterdir():
			if not c.is_file():
				continue
			if c.name.lower() != last.lower():
				continue
			t = c.resolve()
			if _is_under_share(t, share):
				return t
	except OSError:
		return None
	return None


@router.get("/{remainder:path}")
def serve_robot_mesh(remainder: str) -> FileResponse:
	if not remainder or remainder.startswith("/") or ".." in remainder.split("/"):
		raise HTTPException(status_code=400, detail="invalid path")
	parts = remainder.split("/", 1)
	if len(parts) != 2 or not parts[0] or not parts[1]:
		raise HTTPException(status_code=404, detail="expected package/relative/path")
	pkg, rel_path = parts[0], parts[1]
	if ".." in rel_path.split("/"):
		raise HTTPException(status_code=400, detail="invalid path")
	try:
		share = Path(get_package_share_directory(pkg)).resolve()
	except PackageNotFoundError as exc:
		raise HTTPException(status_code=404, detail=f"package not found: {pkg}") from exc
	target = _resolve_mesh_under_share(share, rel_path)
	if target is None or not target.is_file():
		raise HTTPException(status_code=404, detail="file not found")
	# 网格不变时可缓存，减轻 DAE/STL 反复解析（机械臂多 link 时明显）
	return FileResponse(
		target,
		media_type=_suffix_media_type(target),
		filename=target.name,
		headers={"Cache-Control": "public, max-age=86400"},
	)
