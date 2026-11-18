"""Navigation routes: map rendering, pose estimation, goal setting, and teleop.

Batch 2 scope:
- GET /api/nav/map.png: render map or navigation_map as PNG with metadata headers
- POST /api/nav/pose_estimate: publish 2D Pose Estimate
- POST /api/nav/goal: publish navigation goal
- WS /api/teleop: real-time teleop with heartbeat timeout
"""

from __future__ import annotations

import asyncio
import logging
import time
from io import BytesIO
from typing import Any, Dict

from fastapi import APIRouter, Depends, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from pydantic import BaseModel

from ..deps import check_writable
from ..ros_bridge import get_bridge

_LOG = logging.getLogger("infra.routes.nav")

router = APIRouter(prefix="/api/nav", tags=["navigation"])


# ---------------------------------------------------------------------------
# Schemas
# ---------------------------------------------------------------------------


class PoseEstimateRequest(BaseModel):
    x: float
    y: float
    yaw: float
    frame: str = "map"


class NavGoalRequest(BaseModel):
    x: float
    y: float
    yaw: float
    frame: str = "map"


class TeleopCommand(BaseModel):
    linear: float
    angular: float


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------


@router.get("/map.png")
async def get_map(
    layer: str = Query("localization", description="'localization' or 'navigation'")
) -> Response:
    """
    Render the requested map layer as PNG.

    Returns PNG with custom headers:
    - X-Map-Origin: "x,y,yaw" (map origin in meters + radians)
    - X-Map-Resolution: float (meters per pixel)

    TODO Batch 2: Currently returns stub. Need to cache OccupancyGrid from
    ros_bridge callbacks and convert to PNG using PIL or OpenCV.
    """
    # Placeholder implementation
    _LOG.warning("GET /api/nav/map.png not yet implemented (batch 2 TODO)")

    # Return a simple 1x1 PNG as placeholder (using PIL or raw bytes)
    try:
        from PIL import Image
        img = Image.new("L", (100, 100), color=128)
        buf = BytesIO()
        img.save(buf, format="PNG")
        buf.seek(0)
        png_data = buf.read()
    except ImportError:
        # Fallback: minimal PNG (1x1 gray pixel)
        # This is a valid PNG binary for a 1x1 grayscale image
        png_data = (
            b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01'
            b'\x08\x00\x00\x00\x00:~\x9bU\x00\x00\x00\nIDATx\x9cc\x00\x01\x00'
            b'\x00\x05\x00\x01\r\n-\xb4\x00\x00\x00\x00IEND\xaeB`\x82'
        )

    return Response(
        content=png_data,
        media_type="image/png",
        headers={
            "X-Map-Origin": "0.0,0.0,0.0",
            "X-Map-Resolution": "0.02",
        },
    )


@router.post("/pose_estimate")
async def set_pose_estimate(
    req: PoseEstimateRequest, _=Depends(check_writable)
) -> Dict[str, Any]:
    """Publish 2D Pose Estimate to /initialpose."""
    bridge = get_bridge()
    bridge.publish_initial_pose(req.x, req.y, req.yaw, req.frame)
    return {"ok": True, "x": req.x, "y": req.y, "yaw": req.yaw, "frame": req.frame}


@router.post("/goal")
async def set_nav_goal(
    req: NavGoalRequest, _=Depends(check_writable)
) -> Dict[str, Any]:
    """Publish navigation goal to /move_base_simple/goal."""
    bridge = get_bridge()
    bridge.publish_nav_goal(req.x, req.y, req.yaw, req.frame)
    return {"ok": True, "x": req.x, "y": req.y, "yaw": req.yaw, "frame": req.frame}


# ---------------------------------------------------------------------------
# Teleop WebSocket
# ---------------------------------------------------------------------------


@router.websocket("/teleop")
async def teleop_ws(websocket: WebSocket) -> None:
    """
    Real-time teleop WebSocket.

    Client sends JSON: {"linear": float, "angular": float}
    Backend publishes to /cmd_vel with limits and 500ms heartbeat timeout.

    If no message received for 500ms, ros_bridge automatically sends zero velocity.
    """
    await websocket.accept()
    _LOG.info("Teleop WebSocket connected")

    bridge = get_bridge()
    last_msg_time = time.monotonic()

    try:
        while True:
            try:
                # Wait for message with timeout
                msg = await asyncio.wait_for(websocket.receive_json(), timeout=0.5)

                # Parse and publish
                linear = float(msg.get("linear", 0.0))
                angular = float(msg.get("angular", 0.0))
                bridge.publish_cmd_vel(linear, angular)
                last_msg_time = time.monotonic()

            except asyncio.TimeoutError:
                # No message for 500ms - cmd_vel timeout handled by ros_bridge timer
                continue

    except WebSocketDisconnect:
        _LOG.info("Teleop WebSocket disconnected")
        # Send zero velocity on disconnect
        bridge.publish_cmd_vel(0.0, 0.0)
    except Exception as e:
        _LOG.exception(f"Teleop WebSocket error: {e}")
        bridge.publish_cmd_vel(0.0, 0.0)
