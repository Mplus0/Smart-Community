"""Launch control and log routes.

Endpoints:

* ``GET  /api/launches``                    — list known launches + status.
* ``POST /api/launch/{name}/start``         — spawn roslaunch.
* ``POST /api/launch/{name}/stop``          — SIGINT escalation.
* ``GET  /api/logs/{name}``                 — tail last 400 lines.
* ``GET  /api/logs/{name}/download``        — streamed log download.
* ``WS   /api/logs``                        — live log fan-out.

Starting a launch validates required args, refuses launches that would
duplicate a running component and checks prerequisites.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any, Dict, List

from fastapi import APIRouter, Body, Depends, HTTPException, WebSocket, WebSocketDisconnect, status
from fastapi.responses import FileResponse

from .. import config
from ..deps import accept_ws, check_writable
from ..launch_manager import (
    AlreadyRunning,
    LaunchError,
    NotRunning,
    build_roslaunch_cmd,
    get_manager,
)
from ..ros_bridge import get_bridge
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
                "package": spec.package,
                "launch_file": spec.launch_file,
                "args": dict(spec.args),
                "required": list(spec.required),
                "choices": {k: list(v) for k, v in config.ARG_CHOICES.items() if k in spec.args},
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


def running_components() -> Dict[str, str]:
    """Map each running component to the launch (or ROS node) providing it."""
    owners: Dict[str, str] = {}
    for record in get_manager().all_records():
        if record.status == "running":
            for comp in config.launch_components(record.name, record.args):
                owners.setdefault(comp, record.name)
    # Nodes started outside the workbench (e.g. from an SSH terminal).
    nodes = set(get_bridge().get_ros_nodes())
    for comp, node in config.COMPONENT_NODES.items():
        if node in nodes:
            owners.setdefault(comp, f"节点 {node}")
    return owners


def start_registered_launch(name: str, overrides: Dict[str, Any]):
    """Validate, check conflicts and start a registered launch.

    Shared by the Launch panel and the mission routes. Raises
    :class:`HTTPException` with a readable message on any refusal.
    """
    spec = config.get_launch(name)
    if spec is None:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"未知 launch {name!r}")

    # Unknown keys are rejected so a typo never silently reaches roslaunch.
    merged = dict(spec.args)
    for key, value in (overrides or {}).items():
        if key not in spec.args:
            raise HTTPException(status_code=400, detail=f"{name} 不支持参数 {key!r}")
        merged[key] = str(value).strip()
        # roslaunch nodes run in ~/.ros: pass file args as absolute paths.
        if key in config.FILE_ARGS and merged[key]:
            merged[key] = str(Path(merged[key]).expanduser().resolve())

    error = config.validate_launch_args(spec, merged)
    if error:
        raise HTTPException(status_code=400, detail=error)

    manager = get_manager()
    if manager.is_running(name):
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail=f"{name} 已在运行")

    owners = running_components()
    for comp in sorted(config.launch_components(name, merged)):
        if comp in owners:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail=f"{name} 与正在运行的 {owners[comp]} 重复启动 {comp}，请先停止后者"
                       "（若节点已异常退出，可在终端执行 rosnode cleanup）",
            )
    missing = sorted(c for c in config.launch_requires(name, merged) if c not in owners)
    if missing:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"{name} 需要先启动：{', '.join(missing)}",
        )

    cmd = build_roslaunch_cmd(spec.package, spec.launch_file, merged)
    try:
        return manager.start(name, cmd, args=merged)
    except AlreadyRunning as exc:
        raise HTTPException(status_code=status.HTTP_409_CONFLICT, detail=str(exc)) from exc
    except (LaunchError, OSError) as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


# Plain ``def`` handlers run in FastAPI's thread pool: stopping a launch can
# take up to 15 s and must not freeze the event loop (WebSockets, teleop).
@router.post("/api/launch/{name}/start", dependencies=[Depends(check_writable)])
def start_launch(name: str, body: Dict[str, Any] = Body(default_factory=dict)) -> Dict[str, Any]:
    record = start_registered_launch(name, body)
    return {"ok": True, "record": record.to_dict()}


@router.post("/api/launch/{name}/stop", dependencies=[Depends(check_writable)])
def stop_launch(name: str) -> Dict[str, Any]:
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
    if not await accept_ws(ws):
        return
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

