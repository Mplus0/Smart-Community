"""Dashboard routes.

Batch 1 keeps these intentionally skinny:

* ``GET /api/health`` — liveness probe for ``run.sh`` and curl checks.
* ``GET /api/state``  — latest synchronous snapshot (also usable without
  WebSocket for debugging via ``curl``).
* ``WS  /api/state``  — fan-out of state snapshots from the state bus.
* ``POST /api/estop`` — emergency stop; batch 1 only stops every live
  launch, later batches will also publish zero ``cmd_vel``.

Everything else from ``WORKBENCH.md`` §4.1 (ROS topic health, host
metrics, mission state) will be populated once ``ros_bridge`` lands in
batch 2. The frontend therefore has to tolerate missing fields.
"""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Dict

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect

from .. import config
from ..deps import check_writable
from ..launch_manager import get_manager
from ..ros_bridge import get_bridge
from ..state_bus import get_bus

_LOG = logging.getLogger(__name__)

router = APIRouter(tags=["dashboard"])


@router.get("/api/health")
async def health() -> Dict[str, Any]:
    """Minimal liveness response for ``run.sh`` / monitoring."""
    return {
        "ok": True,
        "version": __import__("infra").__version__,
        "readonly": config.READONLY,
    }


@router.get("/api/state")
async def state_snapshot() -> Dict[str, Any]:
    """Return the most recent state snapshot as plain JSON."""
    bus = get_bus()
    latest = bus.latest
    if latest is not None:
        return latest.to_dict()
    # No producer has published yet; return a stable empty shape so the
    # frontend can bind without special-casing ``null``.
    return {
        "ts": None,
        "ros": {"master": False, "nodes": []},
        "topics": {},
        "pose": None,
        "host": {},
        "launches": [r.to_dict() for r in get_manager().all_records()],
        "mission": None,
    }


@router.websocket("/api/state")
async def state_ws(ws: WebSocket) -> None:
    """Fan-out WebSocket. Sends JSON snapshots as they are published."""
    await ws.accept()
    bus = get_bus()
    queue = bus.subscribe_state()
    try:
        while True:
            payload = await queue.get()
            if payload is None:
                break  # bus closed
            try:
                await ws.send_text(json.dumps(payload, default=str))
            except WebSocketDisconnect:
                break
            except Exception:  # pragma: no cover — client gone mid-send
                _LOG.debug("state ws send failed", exc_info=True)
                break
    finally:
        bus.unsubscribe_state(queue)
        try:
            await ws.close()
        except Exception:  # pragma: no cover
            pass


@router.post("/api/estop", dependencies=[Depends(check_writable)])
async def emergency_stop() -> Dict[str, Any]:
    """Stop every live launch and send zero velocity. Batch 2: Add cmd_vel."""
    manager = get_manager()
    bridge = get_bridge()

    stopped = []
    for record in list(manager.all_records()):
        if record.status != "running":
            continue
        try:
            manager.stop(record.name, reason="estop")
            stopped.append(record.name)
        except Exception:  # pragma: no cover — logged inside manager
            _LOG.exception("estop failed for %s", record.name)

    # Batch 2: Send zero velocity to stop robot immediately
    bridge.publish_cmd_vel(0.0, 0.0)

    # Give downstream a tick so the bus can broadcast the resulting state.
    await asyncio.sleep(0)
    return {"stopped": stopped}
