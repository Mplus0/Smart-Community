"""Dashboard routes.

Endpoints:

* ``GET /api/health`` — liveness probe for ``run.sh`` and curl checks.
* ``GET /api/state``  — latest synchronous snapshot (also usable without
  WebSocket for debugging via ``curl``).
* ``WS  /api/state``  — fan-out of state snapshots from the state bus.
* ``POST /api/estop`` — emergency stop (zero velocity → cancel
  navigation → stop the mission node).
"""

from __future__ import annotations

import json
import logging
from typing import Any, Dict

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect

from .. import config
from ..deps import accept_ws
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
    if not await accept_ws(ws):
        return
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


# No read-only / token check on purpose: anyone looking at the robot must
# be able to stop it.
@router.post("/api/estop")
def emergency_stop() -> Dict[str, Any]:
    """Emergency stop: zero velocity first, then cancel navigation and mission.

    Launches (including the chassis driver) are deliberately left running
    so the driver keeps holding the zero command. Plain ``def`` so the
    blocking ROS calls run in the thread pool.
    """
    bridge = get_bridge()
    if not bridge.ready:
        # Fallback without ROS: stop launches, mission first; the chassis
        # driver sends zero velocity when it shuts down.
        manager = get_manager()
        records = [r for r in manager.all_records() if r.status == "running"]
        records.sort(key=lambda r: "mission" not in config.launch_components(r.name, r.args))
        running = [r.name for r in records]
        for name in running:
            try:
                manager.stop(name, reason="estop")
            except Exception:
                _LOG.exception("estop failed for %s", name)
        steps = ["ROS 未连接，已停止所有 launch：" + (", ".join(running) or "无")]
        _LOG.warning("EMERGENCY STOP (no ROS): %s", steps[0])
        return {"ok": True, "steps": steps}
    try:
        steps = bridge.emergency_stop()
    except Exception as exc:
        _LOG.exception("estop failed")
        raise HTTPException(status_code=503, detail=f"急停失败：{exc}") from exc
    _LOG.warning("EMERGENCY STOP: %s", "; ".join(steps))
    return {"ok": True, "steps": steps}
