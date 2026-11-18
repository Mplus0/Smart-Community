"""Camera and detection routes (single pipeline, see camera_core).

* ``GET  /api/camera/sources``     — ROS image topics + USB cameras.
* ``GET  /api/camera/detectors``   — detectors.yaml names + trash pipeline.
* ``GET  /api/camera/config``      — current selection and live status.
* ``POST /api/camera/config``      — switch source / detector / thresholds.
* ``GET  /api/camera/stream.mjpeg`` — annotated MJPEG stream.
* ``POST /api/camera/snapshot``    — save the current annotated frame.
* ``GET  /api/camera/snaps``       — recent snapshots;
  ``GET /api/camera/snaps/{name}`` downloads one.
"""

from __future__ import annotations

import logging
import re
from typing import Any, Dict, Optional, Union

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel

from .. import config
from ..camera_core import get_camera, list_sources
from ..deps import check_writable
from ..detector_core import NO_DETECTOR, list_detectors

_LOG = logging.getLogger("infra.routes.camera")

router = APIRouter(prefix="/api/camera", tags=["camera"])

_SNAP_RE = re.compile(r"^snap_[0-9_]+\.jpg$")


class CameraConfig(BaseModel):
    source: Optional[str] = None
    detector: Optional[str] = None
    conf: Optional[Union[float, str]] = None   # number or "auto"
    iou: Optional[Union[float, str]] = None
    fps: Optional[float] = None


@router.get("/sources")
def get_sources() -> Dict[str, Any]:
    return {"sources": list_sources()}


@router.get("/detectors")
def get_detectors() -> Dict[str, Any]:
    try:
        names = list_detectors()
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"读取 detectors.yaml 失败：{exc}") from exc
    return {"detectors": [NO_DETECTOR] + names}


@router.get("/config")
def get_config() -> Dict[str, Any]:
    camera = get_camera()
    return {"settings": camera.settings, "status": camera.status}


@router.post("/config", dependencies=[Depends(check_writable)])
def set_config(req: CameraConfig) -> Dict[str, Any]:
    if req.source is not None and not re.match(r"^(ros:/\S+|usb:\w+)$", req.source):
        raise HTTPException(status_code=400, detail=f"无效相机来源 {req.source!r}")
    if req.detector is not None and req.detector != NO_DETECTOR and req.detector not in list_detectors():
        raise HTTPException(status_code=400, detail=f"未知检测器 {req.detector!r}")
    for key in ("conf", "iou"):
        value = getattr(req, key)
        if isinstance(value, str) and value != "auto":
            raise HTTPException(status_code=400, detail=f"{key} 只能是 0~1 的数值或 auto")
        if isinstance(value, float) and not 0.0 < value < 1.0:
            raise HTTPException(status_code=400, detail=f"{key} 必须在 0~1 之间")
    if req.fps is not None and not 1 <= req.fps <= 30:
        raise HTTPException(status_code=400, detail="fps 必须在 1~30 之间")
    try:
        settings = get_camera().configure(**req.dict())
    except RuntimeError as exc:
        raise HTTPException(409, str(exc)) from exc
    return {"ok": True, "settings": settings}


@router.get("/stream.mjpeg")
def stream_mjpeg() -> StreamingResponse:
    return StreamingResponse(
        get_camera().stream(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store"},
    )


@router.post("/snapshot", dependencies=[Depends(check_writable)])
def capture_snapshot() -> Dict[str, Any]:
    try:
        path = get_camera().snapshot()
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return {"ok": True, "name": path.name, "path": str(path), "url": f"/api/camera/snaps/{path.name}"}


class CaptureOptions(BaseModel):
    directory: str = ""
    name: str = ""


@router.post("/collect/{kind}", dependencies=[Depends(check_writable)])
def collect(kind: str, req: CaptureOptions) -> Dict[str, Any]:
    if kind not in ("photo", "video"):
        raise HTTPException(404, "未知采集类型")
    try:
        path = get_camera().collect(req.directory.strip(), req.name, kind == "video")
    except (ValueError, OSError) as exc:
        raise HTTPException(400, str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(409, str(exc)) from exc
    return {"path": str(path)}


@router.post("/record/stop", dependencies=[Depends(check_writable)])
def stop_recording() -> Dict[str, Any]:
    return {"path": get_camera().stop_recording()}


@router.get("/snaps")
def list_snapshots(limit: int = 20) -> Dict[str, Any]:
    if not config.SNAPSHOT_DIR.is_dir():
        return {"snaps": []}
    files = sorted(config.SNAPSHOT_DIR.glob("snap_*.jpg"), reverse=True)[:max(1, limit)]
    return {"snaps": [{"name": f.name, "url": f"/api/camera/snaps/{f.name}"} for f in files]}


@router.get("/snaps/{filename}")
def download_snapshot(filename: str) -> FileResponse:
    path = config.SNAPSHOT_DIR / filename
    if not _SNAP_RE.match(filename) or not path.is_file():
        raise HTTPException(status_code=404, detail=f"快照不存在：{filename}")
    return FileResponse(path, media_type="image/jpeg", filename=filename)
