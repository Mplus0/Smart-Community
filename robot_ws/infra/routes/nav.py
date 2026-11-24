"""Navigation routes: map layers, overlay data, pose/goal and teleop.

* ``GET  /api/nav/grid.png?layer=``  — map or
  global / local costmap as PNG. Metadata in ``X-Map-*`` headers; row 0
  of the image is the grid origin row (the frontend draws it y-up).
* ``GET  /api/nav/overlay``         — pose, laser scan, global/local plan
  (all in the map frame) and grid versions for cache busting.
* ``POST /api/nav/pose_estimate``   — 2D Pose Estimate (/initialpose).
* ``POST /api/nav/goal``            — 2D Nav Goal (/move_base_simple/goal).
* ``POST /api/nav/cancel``          — cancel every move_base goal.
* ``WS   /api/nav/teleop``          — {"linear", "lateral", "angular"} at ≥ 2 Hz;
  500 ms without a message → zero velocity (ros_bridge timer).
"""

from __future__ import annotations

import asyncio
import logging
import math
import shutil
import subprocess
import tempfile
import threading
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Tuple

import cv2
import numpy as np
import yaml
from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import Response
from pydantic import BaseModel

from .. import config
from ..deps import accept_ws, check_writable
from ..ros_bridge import GRID_TOPICS, get_bridge

_LOG = logging.getLogger("infra.routes.nav")

router = APIRouter(prefix="/api/nav", tags=["navigation"])
_MAP_DIR = Path.home() / "maps" / "infra_maps"


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
def get_grid_png(layer: str = Query("localization", description="localization (/map) / global_costmap / local_costmap")) -> Response:
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
            img = _render_map(grid.data) if layer == "localization" else _render_costmap(grid.data)
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
# Saved maps and waypoint capture
# ---------------------------------------------------------------------------


@router.get("/maps")
def list_saved_maps() -> Dict[str, Any]:
    return {"maps": [{"name": path.parent.name, "path": str(path)}
                     for path in sorted(_MAP_DIR.glob("*/map.yaml"), reverse=True)
                     if path.is_file() and path.with_suffix(".pgm").is_file()]}


@router.get("/saved_grid.png")
def saved_grid(map_file: str = "") -> Response:
    if map_file:
        allowed = {item["path"] for item in list_saved_maps()["maps"]}
        if map_file not in allowed:
            raise HTTPException(status_code=404, detail="找不到所选地图")
        path = Path(map_file)
    else:
        path = config.ROBOT_WS_DIR / "src/community_bringup/maps/localization_map.yaml"
    try:
        meta = yaml.safe_load(path.read_text(encoding="utf-8"))
        image = cv2.imread(str(path.parent / meta["image"]), cv2.IMREAD_GRAYSCALE)
        resolution = float(meta["resolution"])
        x, y, yaw = map(float, meta["origin"])
        if image is None or resolution <= 0 or not all(math.isfinite(v) for v in (resolution, x, y, yaw)):
            raise ValueError("invalid map")
        ok, png = cv2.imencode(".png", cv2.flip(image, 0))
        if not ok:
            raise ValueError("PNG encoding failed")
    except (OSError, TypeError, KeyError, ValueError, yaml.YAMLError, cv2.error) as exc:
        raise HTTPException(status_code=422, detail="地图文件缺失或格式无效") from exc
    h, w = image.shape
    return Response(png.tobytes(), media_type="image/png", headers={
        "X-Map-Origin": f"{x},{y},{yaw}",
        "X-Map-Resolution": str(resolution),
        "X-Map-Size": f"{w},{h}",
        "Cache-Control": "no-store",
    })


@router.post("/maps/save", dependencies=[Depends(check_writable)])
def save_mapping_grid() -> Dict[str, Any]:
    bridge = get_bridge()
    if not {"/slam_gmapping", "/cartographer_occupancy_grid_node"}.intersection(bridge.get_ros_nodes(max_age=0.0)):
        raise HTTPException(status_code=409, detail="请先启动建图并等待地图生成")
    if bridge.get_grid("localization") is None:
        raise HTTPException(status_code=409, detail="尚未收到建图地图")
    _MAP_DIR.mkdir(parents=True, exist_ok=True)
    destination = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("%Y%m%d_%H%M%S-"), dir=_MAP_DIR))
    try:
        subprocess.run(["rosrun", "map_server", "map_saver", "-f", str(destination / "map")],
                       check=True, capture_output=True, text=True, timeout=12)
        if not all((destination / name).is_file() for name in ("map.pgm", "map.yaml")):
            raise OSError("地图文件不完整")
    except (OSError, subprocess.SubprocessError) as exc:
        shutil.rmtree(destination)
        _LOG.warning("map save failed: %s", exc)
        raise HTTPException(status_code=503, detail="保存地图失败，请确认 /map 正在发布") from exc
    return {"name": destination.name, "path": str(destination / "map.yaml")}


@router.get("/waypoint_pose")
def waypoint_pose() -> Dict[str, Any]:
    bridge = get_bridge()
    pose = bridge.get_waypoint_pose() if bridge.get_grid("localization") is not None else None
    if pose is None:
        raise HTTPException(status_code=409, detail="没有最新的地图位姿，请加载地图并完成定位，或等待建图更新")
    return pose


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
    """Client sends {"linear": m/s, "lateral": m/s, "angular": rad/s} repeatedly while a key
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
            lateral = float(msg.get("lateral", 0.0))
            angular = float(msg.get("angular", 0.0))
            if not (math.isfinite(linear) and math.isfinite(lateral) and math.isfinite(angular)):
                continue
            try:
                bridge.publish_cmd_vel(linear, angular, lateral)
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
