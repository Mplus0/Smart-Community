"""Launch control routes (batch 1 scope only).

Endpoints:

* ``GET  /api/launches``                    — list known launches + status.
* ``POST /api/launch/{name}/start``         — spawn roslaunch.
* ``POST /api/launch/{name}/stop``          — SIGINT escalation.
* ``GET  /api/logs/{name}``                 — tail last 400 lines.
* ``GET  /api/logs/{name}/download``        — streamed log download.
* ``WS   /api/logs``                        — live log fan-out.

Teleop / mission / nav writers listed in WORKBENCH.md §3 are stubbed out
in :mod:`infra.routes.control` (telop) etc. so that frontend code
can bind URLs today and receive 501 until the matching batch lands.
"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict, List

from fastapi import APIRouter, Body, Depends, HTTPException, WebSocket, WebSocketDisconnect, status
from fastapi.responses import FileResponse

from .. import config
from ..deps import check_writable
from ..launch_manager import (
    AlreadyRunning,
    LaunchError,
    NotRunning,
    build_roslaunch_cmd,
    get_manager,
)
from ..state_bus import get_bus

_LOG = logging.getLogger(__name__)

router = APIRouter(tags=["control"])


# ---------------------------------------------------------------------------
# Launch registry views
# ---------------------------------------------------------------------------


def _merge_registry_with_status() -> List[Dict[str, Any]]:
    """Return one entry per registered launch with latest status folded in."""
    manager = get_manager()
    rows: List[Dict[str, Any]] = []
    for name, spec in config.LAUNCHES.items():
        record = manager.status(name)
        rows.append(
            {
                "name": name,
                "description": spec.description,
                "kind": spec.kind,
                "package": spec.package,
                "launch_file": spec.launch_file,
                "args": dict(spec.args),
                "status": record.to_dict() if record is not None else {"status": "idle"},
            }
        )
    return rows


@router.get("/api/launches")
async def list_launches() -> Dict[str, Any]:
    return {"launches": _merge_registry_with_status()}


# ---------------------------------------------------------------------------
# Start / stop
# ---------------------------------------------------------------------------


@router.post("/api/launch/{name}/start", dependencies=[Depends(check_writable)])
async def start_launch(name: str, body: Dict[str, str] = Body(default_factory=dict)) -> Dict[str, Any]:
    spec = config.get_launch(name)
    if spec is None:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"unknown launch {name!r}")

    # Merge registry defaults with caller overrides. Unknown keys are
    # rejected so a typo in the UI does not silently reach ``roslaunch``.
    merged = dict(spec.args)
    for key, value in (body or {}).items():
        if key not in spec.args:
            raise HTTPException(status_code=400, detail=f"unknown arg {key!r} for {name}")
        merged[key] = str(value)

    if spec.kind == "roslaunch":
        cmd = build_roslaunch_cmd(spec.package, spec.launch_file, merged)
    elif spec.kind == "shell":
        cmd = list(spec.command)
    else:  # pragma: no cover — guarded by dataclass construction
        raise HTTPException(status_code=500, detail=f"unsupported kind {spec.kind!r}")

    try:
        record = get_manager().start(name, cmd, args=merged)
    except AlreadyRunning as exc:
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail=exc.record.to_dict()) from exc
    except LaunchError as exc:  # pragma: no cover — defensive
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    return {"ok": True, "record": record.to_dict()}


@router.post("/api/launch/{name}/stop", dependencies=[Depends(check_writable)])
async def stop_launch(name: str) -> Dict[str, Any]:
    try:
        record = get_manager().stop(name, reason="user")
    except NotRunning as exc:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=str(exc)) from exc
    return {"ok": True, "record": record.to_dict()}


# ---------------------------------------------------------------------------
# Logs
# ---------------------------------------------------------------------------


@router.get("/api/logs/{name}")
async def log_tail(name: str, lines: int = 400) -> Dict[str, Any]:
    try:
        tail = get_manager().tail(name, lines=lines)
    except NotRunning as exc:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=str(exc)) from exc
    return {"name": name, "lines": tail}


@router.get("/api/logs/{name}/download")
async def log_download(name: str) -> FileResponse:
    try:
        path = get_manager().log_path(name)
    except NotRunning as exc:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=str(exc)) from exc
    return FileResponse(str(path), filename=path.name, media_type="text/plain")


@router.websocket("/api/logs")
async def log_ws(ws: WebSocket) -> None:
    await ws.accept()
    bus = get_bus()
    queue = bus.subscribe_log()
    try:
        while True:
            payload = await queue.get()
            if payload is None:
                break
            try:
                await ws.send_text(json.dumps(payload, default=str))
            except WebSocketDisconnect:
                break
            except Exception:  # pragma: no cover
                _LOG.debug("log ws send failed", exc_info=True)
                break
    finally:
        bus.unsubscribe_log(queue)
        try:
            await ws.close()
        except Exception:  # pragma: no cover
            pass


# Teleop moved to routes/nav.py in batch 2
