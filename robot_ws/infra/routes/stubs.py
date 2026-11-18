"""Mission / nav / camera route stubs.

Batch 1 only exposes placeholders so the frontend can bind URLs without
guessing. Every endpoint returns ``501 Not Implemented`` with a clear
``batch`` hint so the UI can display a disabled state.
"""

from __future__ import annotations

from typing import Any, Dict

from fastapi import APIRouter, HTTPException, status

router = APIRouter(tags=["stubs"])


def _todo(batch: int, feature: str) -> HTTPException:
    return HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail={"feature": feature, "batch": batch, "message": f"{feature} lands in batch {batch}"},
    )


# ---------------------------------------------------------------------------
# Mission
# ---------------------------------------------------------------------------


@router.get("/api/mission/runs")
async def mission_runs() -> Dict[str, Any]:
    raise _todo(4, "mission runs browser")


@router.post("/api/mission/single")
async def mission_single() -> Dict[str, Any]:
    raise _todo(4, "single-waypoint mission dispatch")


# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------


@router.get("/api/nav/map.png")
async def nav_map() -> Dict[str, Any]:
    raise _todo(2, "map layer rendering")


@router.post("/api/nav/pose_estimate")
async def nav_pose_estimate() -> Dict[str, Any]:
    raise _todo(2, "2D Pose Estimate")


@router.post("/api/nav/goal")
async def nav_goal() -> Dict[str, Any]:
    raise _todo(2, "2D Nav Goal")


# ---------------------------------------------------------------------------
# Camera / detectors
# ---------------------------------------------------------------------------


@router.get("/api/camera/sources")
async def camera_sources() -> Dict[str, Any]:
    raise _todo(3, "camera source discovery")


@router.get("/api/detectors")
async def detectors() -> Dict[str, Any]:
    raise _todo(3, "detector registry")


@router.get("/api/stream.mjpeg")
async def stream_mjpeg() -> Dict[str, Any]:
    raise _todo(3, "MJPEG stream")
