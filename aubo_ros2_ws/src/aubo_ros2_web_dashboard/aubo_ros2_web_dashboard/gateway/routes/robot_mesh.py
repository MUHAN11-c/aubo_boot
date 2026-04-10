"""
同源 HTTP 提供 ``package://<pkg>/...`` 下的网格文件，供浏览器 ros3d ``UrdfClient`` 拉取。

路径格式：``/api/ivg/robot-mesh/<pkg>/<relative_path_under_share>``
例：``aubo_description/meshes/aubo_e5_10/visual/link0.DAE``

依赖运行环境中的 ``AMENT_PREFIX_PATH``（与 ``ros2 launch`` 启动网关时一致）。
"""
from __future__ import annotations

from pathlib import Path

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
		from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
	except ImportError as exc:
		raise HTTPException(status_code=503, detail="ament_index_python unavailable") from exc
	try:
		share = Path(get_package_share_directory(pkg)).resolve()
	except PackageNotFoundError as exc:
		raise HTTPException(status_code=404, detail=f"package not found: {pkg}") from exc
	target = (share / rel_path).resolve()
	try:
		target.relative_to(share)
	except ValueError as exc:
		raise HTTPException(status_code=403, detail="path outside package share") from exc
	if not target.is_file():
		raise HTTPException(status_code=404, detail="file not found")
	# 网格不变时可缓存，减轻 DAE/STL 反复解析（机械臂多 link 时明显）
	return FileResponse(
		target,
		media_type=_suffix_media_type(target),
		filename=target.name,
		headers={"Cache-Control": "public, max-age=86400"},
	)
