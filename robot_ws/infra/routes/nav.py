"""Navigation routes: map layers, overlay data, pose/goal and teleop.

* ``GET  /api/nav/grid.png?layer=``  — localization / navigation map or
  global / local costmap as PNG. Metadata in ``X-Map-*`` headers; row 0
  of the image is the grid origin row (the frontend draws it y-up).
* ``GET  /api/nav/overlay``         — pose, laser scan, global/local plan
  (all in the map frame) and grid versions for cache busting.
* ``POST /api/nav/pose_estimate``   — 2D Pose Estimate (/initialpose).
* ``POST /api/nav/goal``            — 2D Nav Goal (/move_base_simple/goal).
* ``POST /api/nav/cancel``          — cancel every move_base goal.
* ``WS   /api/nav/teleop``          — {"linear", "angular"} at ≥ 2 Hz;
  500 ms without a message → zero velocity (ros_bridge timer).
"""

from __future__ import annotations

import asyncio
import logging
import math
import threading
from typing import Any, Dict, Tuple

import cv2
import numpy as np
from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from pydantic import BaseModel

from ..deps import accept_ws, check_writable
from ..ros_bridge import GRID_TOPICS, get_bridge

_LOG = logging.getLogger("infra.routes.nav")

router = APIRouter(prefix="/api/nav", tags=["navigation"])


class PoseRequest(BaseModel):
    x: float
    y: float
    yaw: float


# ---------------------------------------------------------------------------
# Grid rendering
# ---------------------------------------------------------------------------

_png_cache: Dict[str, Tuple[int, bytes]] = {}
_png_lock = threading.Lock()


def _render_map(data: np.ndarray) -> np.ndarray:
    """Occupancy (−1 / 0..100) → map_server style grayscale."""
    img = (254 - data.clip(0, 100).astype(np.int32) * 254 // 100).astype(np.uint8)
    img[data < 0] = 205
    return img


def _render_costmap(data: np.ndarray) -> np.ndarray:
    """Costmap → BGRA overlay: transparent free space, red lethal cells."""
    cost = data.astype(np.int32)
    img = np.zeros(data.shape + (4,), dtype=np.uint8)
    inflated = (cost > 0) & (cost < 99)
    ratio = cost[inflated] / 98.0
    img[inflated, 0] = (255 * (1 - ratio)).astype(np.uint8)   # blue → yellow-ish
    img[inflated, 1] = (200 * ratio).astype(np.uint8)
    img[inflated, 2] = (255 * ratio).astype(np.uint8)
    img[inflated, 3] = (60 + 120 * ratio).astype(np.uint8)
    lethal = cost >= 99
    img[lethal] = (60, 0, 230, 220)
    return img


@router.get("/grid.png")
def get_grid_png(layer: str = Query("navigation", description="localization / navigation / global_costmap / local_costmap")) -> Response:
    if layer not in GRID_TOPICS:
        raise HTTPException(status_code=400, detail=f"未知图层 {layer!r}")
    bridge = get_bridge()
    grid = bridge.get_grid(layer)
    if grid is None:
        raise HTTPException(status_code=404, detail=f"尚未收到 {GRID_TOPICS[layer]}")
    origin = bridge.grid_origin_in_map(grid)
    if origin is None:
        raise HTTPException(status_code=503, detail=f"缺少 TF：map → {grid.frame}")

    with _png_lock:
        cached = _png_cache.get(layer)
        if cached is not None and cached[0] == grid.version:
            png = cached[1]
        else:
            img = _render_map(grid.data) if layer in ("localization", "navigation") else _render_costmap(grid.data)
            ok, encoded = cv2.imencode(".png", img)
            if not ok:
                raise HTTPException(status_code=500, detail="PNG 编码失败")
            png = encoded.tobytes()
            _png_cache[layer] = (grid.version, png)

    h, w = grid.data.shape
    return Response(
        content=png,
        media_type="image/png",
        headers={
            "X-Map-Origin": f"{origin[0]:.4f},{origin[1]:.4f},{origin[2]:.5f}",
            "X-Map-Resolution": f"{grid.resolution:.5f}",
            "X-Map-Size": f"{w},{h}",
            "X-Map-Version": str(grid.version),
            "Cache-Control": "no-store",
        },
    )


@router.get("/overlay")
def get_overlay() -> Dict[str, Any]:
    return get_bridge().get_overlay()


# ---------------------------------------------------------------------------
# Pose / goal / cancel
# ---------------------------------------------------------------------------


def _publish(action, *args) -> None:
    try:
        action(*args)
    except RuntimeError as exc:
        raise HTTPException(status_code=503, detail=str(exc)) from exc


def _check_pose(req: PoseRequest) -> None:
    if not all(math.isfinite(v) for v in (req.x, req.y, req.yaw)):
        raise HTTPException(status_code=400, detail="坐标必须为有限数值")


@router.post("/pose_estimate", dependencies=[Depends(check_writable)])
def set_pose_estimate(req: PoseRequest) -> Dict[str, Any]:
    _check_pose(req)
    _publish(get_bridge().publish_initial_pose, req.x, req.y, req.yaw)
    return {"ok": True}


@router.post("/goal", dependencies=[Depends(check_writable)])
def set_nav_goal(req: PoseRequest) -> Dict[str, Any]:
    _check_pose(req)
    _publish(get_bridge().publish_nav_goal, req.x, req.y, req.yaw)
    return {"ok": True}


@router.post("/cancel", dependencies=[Depends(check_writable)])
def cancel_nav() -> Dict[str, Any]:
    _publish(get_bridge().cancel_navigation)
    return {"ok": True}


# ---------------------------------------------------------------------------
# Teleop WebSocket
# ---------------------------------------------------------------------------


@router.websocket("/teleop")
async def teleop_ws(websocket: WebSocket) -> None:
    """Client sends {"linear": m/s, "angular": rad/s} repeatedly while a key
    or joystick is held. Limits are applied in ros_bridge."""
    if not await accept_ws(websocket, writable=True):
        return
    bridge = get_bridge()
    try:
        while True:
            try:
                msg = await asyncio.wait_for(websocket.receive_json(), timeout=0.5)
            except asyncio.TimeoutError:
                continue  # the 500 ms heartbeat timer already sent zero
            linear = float(msg.get("linear", 0.0))
            angular = float(msg.get("angular", 0.0))
            if not (math.isfinite(linear) and math.isfinite(angular)):
                continue
            try:
                bridge.publish_cmd_vel(linear, angular)
            except RuntimeError as exc:
                await websocket.send_json({"ok": False, "error": str(exc)})
    except WebSocketDisconnect:
        pass
    except Exception:
        _LOG.exception("teleop websocket error")
    finally:
        try:
            bridge.publish_cmd_vel(0.0, 0.0)
        except RuntimeError:
            pass
