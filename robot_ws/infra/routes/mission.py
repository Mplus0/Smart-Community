"""Mission control and result browsing.

* ``GET  /api/mission/waypoints``         — waypoint ids/tasks from route.yaml.
* ``GET  /api/mission/state``             — latest ``/mission/state`` JSON.
* ``POST /api/mission/start``             — start ``mission.launch`` (full
  route or one waypoint); hardware + navigation must already run.
* ``POST /api/mission/stop``              — stop the mission node.
* ``GET  /api/mission/runs``              — mission runs (dirs holding
  ``mission_results.json``), newest first.
* ``GET  /api/mission/runs/{id}``         — full result + media file list.
* ``GET  /api/mission/runs/{id}/file``    — one file (photo / video / log).
* ``GET  /api/mission/runs/{id}/download`` — tar.gz of the run.

``navigation_only=true`` skips **every** waypoint task (traffic-light
wait, photos, recognition, recording) and only drives the route.
"""

from __future__ import annotations

import json
import logging
import mimetypes
import os
import re
import tarfile
import tempfile
from pathlib import Path
from typing import Any, Dict, List

import yaml
from fastapi import APIRouter, Depends, HTTPException, Query
from fastapi.responses import FileResponse
from pydantic import BaseModel
from starlette.background import BackgroundTask

from .. import config
from ..deps import check_writable
from ..launch_manager import NotRunning, get_manager
from ..ros_bridge import MISSION_NODE, get_bridge
from .control import start_registered_launch

_LOG = logging.getLogger("infra.routes.mission")

router = APIRouter(prefix="/api/mission", tags=["mission"])

#: ``data_dir`` default of mission.launch; each run is ``<root>/<run_id>/``.
MISSION_ROOT = Path(os.environ.get("INFRA_MISSION_ROOT") or Path.home() / ".ros" / "community")
RESULT_FILE = "mission_results.json"
DEFAULT_ROUTE = config.ROBOT_WS_DIR / "src" / "community_mission" / "config" / "route.yaml"
_RUN_ID_RE = re.compile(r"^\w[\w.-]*$")
_MEDIA_DIRS = ("photo", "videos")


class MissionStart(BaseModel):
    waypoint: str = ""             # empty → full route
    navigation_only: bool = False
    obstacle_mode: str = "blocked"


def _route_points(route: Path = DEFAULT_ROUTE) -> List[Dict[str, Any]]:
    data = yaml.safe_load(route.read_text(encoding="utf-8")) or {}
    return [{"id": p["id"], "task": p["task"]} for p in data.get("points", [])]


# ---------------------------------------------------------------------------
# Control
# ---------------------------------------------------------------------------


@router.get("/waypoints")
def get_waypoints() -> Dict[str, Any]:
    try:
        return {"route": str(DEFAULT_ROUTE), "points": _route_points()}
    except (OSError, yaml.YAMLError, KeyError) as exc:
        raise HTTPException(status_code=500, detail=f"读取 route.yaml 失败：{exc}") from exc


def _mission_owner() -> str:
    """Name of the workbench launch running the mission node, if any."""
    for record in get_manager().all_records():
        if record.status == "running" and "mission" in config.launch_components(record.name, record.args):
            return record.name
    return ""


@router.get("/state")
def get_state() -> Dict[str, Any]:
    bridge = get_bridge()
    return {
        "state": bridge.get_mission_state(),
        "node_alive": MISSION_NODE in bridge.get_ros_nodes(),
        "launch": _mission_owner(),
    }


@router.post("/start", dependencies=[Depends(check_writable)])
def start_mission(req: MissionStart) -> Dict[str, Any]:
    if req.obstacle_mode not in ("clear", "blocked"):
        raise HTTPException(status_code=400, detail="obstacle_mode 只能是 clear 或 blocked")
    if req.waypoint:
        point = next((p for p in _route_points() if p["id"] == req.waypoint), None)
        if point is None:
            raise HTTPException(status_code=400, detail=f"route.yaml 中没有航点 {req.waypoint}")
        if point["task"] in ("record_start", "record_stop") and not req.navigation_only:
            raise HTTPException(status_code=400,
                                detail="录像航点不能单独执行任务，请勾选「仅导航」只做导航验证")
    record = start_registered_launch("mission", {
        "single_waypoint": req.waypoint,
        "navigation_only": "true" if req.navigation_only else "false",
        "obstacle_mode": req.obstacle_mode,
    })
    return {"ok": True, "record": record.to_dict()}


@router.post("/stop", dependencies=[Depends(check_writable)])
def stop_mission() -> Dict[str, Any]:
    """Stop the mission node: the standalone ``mission`` launch is stopped
    normally (the node cancels its goal on shutdown); a mission inside
    ``field``/``finals`` is killed after cancelling navigation — because the
    node is ``required`` that launch then shuts down as a whole."""
    manager = get_manager()
    if manager.is_running("mission"):
        try:
            manager.stop("mission", reason="mission stop")
        except NotRunning:
            pass
        return {"ok": True, "message": "已停止 mission"}
    bridge = get_bridge()
    if MISSION_NODE not in bridge.get_ros_nodes(max_age=0.0):
        raise HTTPException(status_code=404, detail="巡检节点未运行")
    try:
        bridge.cancel_navigation()
        bridge.kill_mission_node()
    except Exception as exc:
        raise HTTPException(status_code=503, detail=f"停止巡检失败：{exc}") from exc
    owner = _mission_owner()
    note = f"（{owner} 中的 mission 节点为 required，整套 launch 会随之退出）" if owner else ""
    return {"ok": True, "message": "已取消导航并停止巡检节点" + note}


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------


def _run_dir(run_id: str) -> Path:
    run_dir = MISSION_ROOT / run_id
    if not _RUN_ID_RE.match(run_id) or not (run_dir / RESULT_FILE).is_file():
        raise HTTPException(status_code=404, detail=f"巡检记录不存在：{run_id}")
    return run_dir


def _load_result(run_dir: Path) -> Dict[str, Any]:
    try:
        return json.loads((run_dir / RESULT_FILE).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return {"status": "UNREADABLE", "error": str(exc)}


@router.get("/runs")
def list_mission_runs() -> Dict[str, Any]:
    """Only directories written by mission_node (they hold mission_results.json);
    detections/, ocr/, videos/ and infra/ under the same root are skipped."""
    runs = []
    if MISSION_ROOT.is_dir():
        for run_dir in sorted(MISSION_ROOT.iterdir(), reverse=True):
            if not (run_dir / RESULT_FILE).is_file():
                continue
            data = _load_result(run_dir)
            runs.append({
                "run_id": run_dir.name,
                "status": data.get("status"),
                "state": data.get("state"),
                "started_at": data.get("started_at"),
                "ended_at": data.get("ended_at"),
                "duration_sec": data.get("duration_sec"),
                "current_waypoint": data.get("current_waypoint"),
                "error": data.get("error"),
                "summary": data.get("summary"),
                "photo_count": len(list((run_dir / "photo").glob("*.jpg"))),
                "video_count": len(list((run_dir / "videos").glob("*.mp4"))),
            })
    return {"root": str(MISSION_ROOT), "runs": runs}


@router.get("/runs/{run_id}")
def get_mission_run(run_id: str) -> Dict[str, Any]:
    run_dir = _run_dir(run_id)
    files = []
    for sub in _MEDIA_DIRS:
        folder = run_dir / sub
        if folder.is_dir():
            files += [f"{sub}/{f.name}" for f in sorted(folder.iterdir()) if f.is_file()]
    if (run_dir / "mission.log").is_file():
        files.append("mission.log")
    return {"run_id": run_id, "result": _load_result(run_dir), "files": files}


@router.get("/runs/{run_id}/file")
def get_mission_file(run_id: str, path: str = Query(RESULT_FILE)) -> FileResponse:
    run_dir = _run_dir(run_id).resolve()
    target = (run_dir / path).resolve()
    try:
        target.relative_to(run_dir)
    except ValueError:
        raise HTTPException(status_code=403, detail="路径越界")
    if not target.is_file():
        raise HTTPException(status_code=404, detail=f"文件不存在：{path}")
    media_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
    if target.suffix in (".log", ".json"):
        media_type += "; charset=utf-8"
    return FileResponse(target, media_type=media_type)


@router.get("/runs/{run_id}/download")
def download_mission_run(run_id: str) -> FileResponse:
    """tar.gz of the whole run, built in a temp file (videos can be large)."""
    run_dir = _run_dir(run_id)
    tmp = tempfile.NamedTemporaryFile(prefix="mission_", suffix=".tar.gz", delete=False)
    tmp.close()
    with tarfile.open(tmp.name, mode="w:gz") as tar:
        tar.add(str(run_dir), arcname=run_id)
    return FileResponse(tmp.name, media_type="application/gzip", filename=f"{run_id}.tar.gz",
                        background=BackgroundTask(os.unlink, tmp.name))
