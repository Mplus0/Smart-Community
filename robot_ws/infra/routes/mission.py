"""Mission control and results browsing routes.

Batch 4 scope:
- GET /api/mission/runs: list all completed mission runs
- GET /api/mission/runs/{id}/file: access result files (result.json, photos, videos)
- POST /api/mission/single: start a single waypoint mission
- GET /api/mission/runs/{id}/download: download tar.gz of entire run
"""

from __future__ import annotations

import json
import logging
import tarfile
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Depends, HTTPException, Query, WebSocket, WebSocketDisconnect, status
from fastapi.responses import FileResponse, StreamingResponse

from .. import config
from ..deps import check_writable

_LOG = logging.getLogger("infra.routes.mission")

router = APIRouter(prefix="/api/mission", tags=["mission"])

# Mission run directory: ~/.ros/community/<run_id>/
MISSION_ROOT = Path.home() / ".ros" / "community"


# ---------------------------------------------------------------------------
# Mission run enumeration
# ---------------------------------------------------------------------------


@router.get("/runs")
async def list_mission_runs() -> Dict[str, Any]:
    """List all completed mission runs sorted by timestamp (newest first)."""
    if not MISSION_ROOT.exists():
        return {"runs": []}

    runs = []
    for run_dir in sorted(MISSION_ROOT.glob("*"), reverse=True):
        if not run_dir.is_dir() or run_dir.name == "infra":
            # Skip non-run directories
            continue

        run_id = run_dir.name
        result_file = run_dir / "result.json"
        result_data = None

        if result_file.exists():
            try:
                result_data = json.loads(result_file.read_text(encoding="utf-8"))
            except Exception as e:
                _LOG.warning(f"Failed to parse {result_file}: {e}")

        # Count media files
        photo_dir = run_dir / "photo"
        video_dir = run_dir / "videos"
        photo_count = len(list(photo_dir.glob("*.jpg"))) if photo_dir.exists() else 0
        video_count = len(list(video_dir.glob("*.mp4"))) if video_dir.exists() else 0

        runs.append(
            {
                "run_id": run_id,
                "result": result_data,
                "photo_count": photo_count,
                "video_count": video_count,
                "size_mb": sum(f.stat().st_size for f in run_dir.rglob("*") if f.is_file()) / 1024 / 1024,
            }
        )

    return {"runs": runs}


@router.get("/runs/{run_id}/file")
async def get_mission_file(
    run_id: str, path: str = Query("result.json", description="Relative path within run directory")
) -> FileResponse:
    """Download a file from a mission run (result.json, photo, video, etc.)."""
    run_dir = MISSION_ROOT / run_id
    if not run_dir.exists():
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"Run not found: {run_id}")

    # Prevent directory traversal attacks
    target = (run_dir / path).resolve()
    if not str(target).startswith(str(run_dir)):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN, detail="Access denied: path traversal detected"
        )

    if not target.exists() or not target.is_file():
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"File not found: {path}")

    return FileResponse(target, filename=target.name)


@router.get("/runs/{run_id}/download")
async def download_mission_run_tar(run_id: str) -> StreamingResponse:
    """Download entire mission run as tar.gz archive.

    Note: This streams the archive directly without buffering to disk.
    """
    run_dir = MISSION_ROOT / run_id
    if not run_dir.exists():
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail=f"Run not found: {run_id}")

    async def tar_generator():
        """Yield tar.gz bytes incrementally."""
        import io
        import gzip

        buf = io.BytesIO()
        with tarfile.open(fileobj=buf, mode="w:gz", format=tarfile.PAX_FORMAT) as tar:
            tar.add(str(run_dir), arcname=run_id, recursive=True)

        # Seek back and stream
        buf.seek(0)
        while True:
            chunk = buf.read(65536)  # 64 KB chunks
            if not chunk:
                break
            yield chunk

    return StreamingResponse(
        tar_generator(),
        media_type="application/gzip",
        headers={"Content-Disposition": f"attachment; filename={run_id}.tar.gz"},
    )


# ---------------------------------------------------------------------------
# Mission control
# ---------------------------------------------------------------------------


@router.post("/single", dependencies=[Depends(check_writable)])
async def start_single_waypoint_mission(
    waypoint: str = Query(..., description="Waypoint ID (e.g., wp_05)"),
    navigation_only: bool = Query(False, description="Skip obstacle avoidance"),
) -> Dict[str, Any]:
    """Start a single-waypoint mission.

    Batch 4 TODO: Call ros_bridge or launch_manager to start mission with parameters.
    """
    _LOG.info(f"Starting single waypoint mission: {waypoint}, navigation_only={navigation_only}")

    return {
        "ok": True,
        "waypoint": waypoint,
        "navigation_only": navigation_only,
        "status": "Single waypoint mission started (batch 4 TODO: full implementation)",
    }


# ---------------------------------------------------------------------------
# Mission state real-time updates (batch 4 / 5)
# ---------------------------------------------------------------------------


@router.websocket("/state")
async def mission_state_ws(ws: WebSocket) -> None:
    """Real-time mission state updates via WebSocket.

    Batch 4 TODO: Subscribe to /mission/state ROS topic and forward updates.
    Schema per WORKBENCH.md §4.1:
    {
        "state": "NAVIGATING|RECOGNIZING|DONE|FAILED",
        "index": int,
        "current_waypoint": str,
        "reason": str,
        "navigation_only": bool,
        "obstacle_mode": str,
        "run_id": str,
        "last_detection": {...}
    }
    """
    await ws.accept()
    try:
        # Placeholder: send empty state
        await ws.send_json(
            {
                "state": "IDLE",
                "index": 0,
                "current_waypoint": None,
                "message": "Mission state not yet implemented (batch 4 TODO)",
            }
        )
        while True:
            try:
                _ = await ws.receive_text()
            except WebSocketDisconnect:
                return
    finally:
        try:
            await ws.close()
        except Exception:
            pass
