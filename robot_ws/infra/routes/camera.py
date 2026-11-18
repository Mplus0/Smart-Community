"""Camera and detection routes.

Batch 3 scope:
- GET /api/camera/sources: list available camera sources (ROS topics, USB devices)
- GET /api/detectors: list all available detectors from detectors.yaml
- GET /api/stream.mjpeg: stream detected video frames
- POST /api/snapshot: save single frame to disk

Batch 5 enhancements:
- WS /api/camera/recognition: Real-time recognition with single/combined modes
"""

from __future__ import annotations

import asyncio
import json
import logging
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Depends, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from pydantic import BaseModel

from .. import config
from ..detector_core import DEFAULT_CONFIG

_LOG = logging.getLogger("infra.routes.camera")

router = APIRouter(prefix="/api/camera", tags=["camera"])


# ---------------------------------------------------------------------------
# Camera source enumeration
# ---------------------------------------------------------------------------


@router.get("/sources")
async def list_camera_sources() -> Dict[str, Any]:
    """List available camera sources: ROS topics and USB devices."""
    sources = {
        "ros_topics": [],
        "usb_devices": ["dji", "orbbec"],
        "special": ["thermal"],
    }

    # TODO Batch 3: Query ROS for sensor_msgs/Image topics
    # If rospy is available, call rostopic and filter for Image messages
    try:
        import rospy
        # Placeholder: would list all image topics here
        sources["ros_topics"] = ["/usb_cam/image_raw"]  # Common default
    except ImportError:
        pass

    return {"sources": sources}


@router.get("/detectors")
async def list_detectors() -> Dict[str, Any]:
    """List all available detectors from detectors.yaml."""
    # TODO Batch 3: Parse detectors.yaml and return config
    import yaml

    try:
        config_path = DEFAULT_CONFIG
        if config_path.exists():
            configs = yaml.safe_load(config_path.read_text(encoding="utf-8"))["detectors"]
            return {
                "detectors": list(configs.keys()),
                "special_modes": [
                    "__none__",
                    "__trash_pipeline__",
                    "__traffic_light_state__",
                    "__thermal_building_d__",
                    "__fall_count__",
                    "__license_plate__",
                ],
            }
    except Exception as e:
        _LOG.warning(f"Failed to load detectors.yaml: {e}")

    return {
        "detectors": [],
        "special_modes": ["__none__", "__trash_pipeline__"],
        "error": "detectors.yaml not found",
    }


# ---------------------------------------------------------------------------
# MJPEG streaming cache
# ---------------------------------------------------------------------------

_stream_cache: Dict[str, Preview] = {}


@router.get("/stream.mjpeg")
async def stream_mjpeg(
    source: str = Query("ros", description="Camera source (ros/dji/orbbec/thermal)"),
    detector: str = Query("__none__", description="Detector or special mode"),
    fps: int = Query(10, ge=1, le=30),
    conf: float = Query(0.5, ge=0.0, le=1.0),
    iou: float = Query(0.4, ge=0.0, le=1.0),
) -> StreamingResponse:
    """Stream MJPEG from selected camera+detector combination.

    Parameters:
    - source: camera source
    - detector: model name or special mode (e.g., __trash_pipeline__)
    - fps: target frames per second for the stream
    - conf: confidence threshold
    - iou: NMS IOU threshold
    """
    cache_key = f"{source}:{detector}:{fps}:{conf}:{iou}"

    # TODO Batch 3: Implement full streaming pipeline
    # - Initialize camera/ROS subscriber
    # - Run detector if needed
    # - Apply overlay
    # - Yield MJPEG boundaries

    async def generate():
        # Placeholder: return a simple "not ready" message
        frame = b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 0\r\n\r\n"
        while True:
            yield frame

    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")


# ---------------------------------------------------------------------------
# Snapshot capture
# ---------------------------------------------------------------------------


@router.post("/snapshot")
async def capture_snapshot(
    source: str = Query("ros"),
    detector: str = Query("__none__"),
) -> Dict[str, Any]:
    """Capture a single frame and save to disk.

    Returns the path and URL for download.
    """
    # TODO Batch 3: Implement snapshot logic
    # - Get current frame from camera
    # - Run detector if not __none__
    # - Apply overlay
    # - Save to ~/.ros/community/infra/snaps/<timestamp>.jpg
    # - Return path

    snap_path = config.SNAPSHOT_DIR / "placeholder.jpg"
    snap_path.parent.mkdir(parents=True, exist_ok=True)

    return {
        "ok": True,
        "path": str(snap_path),
        "url": f"/api/camera/snaps/{snap_path.name}",
        "status": "Snapshot not yet implemented (batch 3 TODO)",
    }


@router.get("/snaps/{filename}")
async def download_snapshot(filename: str) -> FileResponse:
    """Download a previously saved snapshot."""
    path = config.SNAPSHOT_DIR / filename
    if not path.exists() or not path.is_file():
        raise FileNotFoundError(f"Snapshot not found: {filename}")
    return FileResponse(path, media_type="image/jpeg", filename=filename)


# ---------------------------------------------------------------------------
# Perception debug display (batch 5)
# ---------------------------------------------------------------------------


@router.websocket("/recognition")
async def recognition_ws(ws: WebSocket) -> None:
    """WebSocket endpoint for real-time recognition (single-model or combined).

    Batch 5: Enhanced with combined recognition mode support per WORKBENCH.md §5.3.

    Client sends:
    {
        "command": "set_mode",
        "mode": "default" | "combined",
        "models": ["gate", "traffic_light", "obstacle"]  // for combined mode
    }

    or simply:
    {
        "command": "start",
        "mode": "default"
    }

    Server streams (single mode):
    {
        "ok": true,
        "class_name": "green" | "blocked" | "garbage",
        "confidence": 0.95,
        "bbox": [x_min, y_min, x_max, y_max],
        "timestamp": "2026-11-18T10:30:15.123456"
    }

    Server streams (combined mode):
    {
        "ok": true,
        "detections": [
            {"model": "gate", "class_name": "green", "confidence": 0.95, "bbox": [...]},
            {"model": "obstacle", "class_name": "blocked", "confidence": 0.88, "bbox": [...]},
        ],
        "timestamp": "..."
    }
    """
    await ws.accept()
    _LOG.info("Recognition WebSocket connected")

    preview = None
    recognizer = None
    mode = "default"
    models = ["gate"]  # default single model

    try:
        # Wait for mode configuration or start
        initial_msg = await ws.receive_text()
        config_data = json.loads(initial_msg)

        if config_data.get("command") == "set_mode":
            mode = config_data.get("mode", "default")
            models = config_data.get("models", ["gate"])
        elif config_data.get("command") == "start":
            mode = config_data.get("mode", "default")

        _LOG.info(f"Recognition mode: {mode}, models: {models}")

        # Open camera
        from ..camera_core import Preview
        from ..detector_core import Recognizer

        preview = Preview(camera_index=0)
        preview.open()

        if mode == "combined":
            # Batch 5: Combined recognition mode
            # Load multiple models and run each on every frame
            recognizers = []
            for model_name in models:
                recognizer_instance = Recognizer(model_name=model_name, conf_threshold=0.5)
                recognizers.append((model_name, recognizer_instance))

            while True:
                # Check for client messages (non-blocking)
                try:
                    msg = await asyncio.wait_for(ws.receive_text(), timeout=0.001)
                    cmd = json.loads(msg)
                    if cmd.get("command") == "stop":
                        break
                except asyncio.TimeoutError:
                    pass

                # Capture frame
                raw_frame = preview.read_frame()
                if raw_frame is None:
                    await asyncio.sleep(0.05)
                    continue

                # Run all recognizers
                detections = []
                for model_name, recognizer_instance in recognizers:
                    results = recognizer_instance.detect(raw_frame)
                    for res in results:
                        detections.append(
                            {
                                "model": model_name,
                                "class_name": res["class_name"],
                                "confidence": res["confidence"],
                                "bbox": res["bbox"],
                            }
                        )

                # Send combined results
                await ws.send_json(
                    {
                        "ok": True,
                        "detections": detections,
                        "timestamp": datetime.utcnow().isoformat(),
                    }
                )

                await asyncio.sleep(0.1)  # ~10 FPS

        else:
            # Default: single-model mode (batch 3 behavior)
            model_name = models[0]
            recognizer = Recognizer(model_name=model_name, conf_threshold=0.5)

            while True:
                # Check for client messages (non-blocking)
                try:
                    msg = await asyncio.wait_for(ws.receive_text(), timeout=0.001)
                    cmd = json.loads(msg)
                    if cmd.get("command") == "stop":
                        break
                except asyncio.TimeoutError:
                    pass

                # Capture and recognize
                raw_frame = preview.read_frame()
                if raw_frame is None:
                    await asyncio.sleep(0.05)
                    continue

                results = recognizer.detect(raw_frame)
                if results:
                    best = max(results, key=lambda r: r["confidence"])
                    await ws.send_json(
                        {
                            "ok": True,
                            "class_name": best["class_name"],
                            "confidence": best["confidence"],
                            "bbox": best["bbox"],
                            "timestamp": datetime.utcnow().isoformat(),
                        }
                    )
                else:
                    await ws.send_json({"ok": True, "class_name": None, "timestamp": datetime.utcnow().isoformat()})

                await asyncio.sleep(0.1)  # ~10 FPS

    except WebSocketDisconnect:
        _LOG.info("Recognition WebSocket disconnected")
    except Exception as e:
        _LOG.exception("Recognition WebSocket error")
        try:
            await ws.send_json({"ok": False, "error": str(e)})
        except Exception:
            pass
    finally:
        if preview:
            preview.close()
        try:
            await ws.close()
        except Exception:
            pass


@router.websocket("/perception")
async def perception_debug_ws(ws: WebSocket) -> None:
    """Real-time perception debug results.

    Batch 5 TODO: Subscribe to /perception/debug topic and broadcast latest
    detection history (last N frames) with replay capability.
    """
    await ws.accept()
    try:
        # Placeholder: send empty message
        await ws.send_json({"type": "placeholder", "message": "perception debug not yet implemented"})
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
