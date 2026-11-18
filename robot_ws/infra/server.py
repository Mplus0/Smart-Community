"""FastAPI entry point for the Smart-Community infra workbench.

Run with::

    python -m infra.server --port 8000

or via ``tools/start_infra.sh`` (``infra.launch``) / ``infra/run.sh``.

* Serves the single-page frontend under ``/`` and the API under ``/api``.
* Runs a periodic state producer (launches, ROS health, pose, mission
  state, host CPU/mem when :mod:`psutil` is available).
* Clean shutdown via ``SIGINT``/``SIGTERM`` — every live launch is
  stopped before uvicorn exits.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import sys
import threading
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, Optional

from fastapi import FastAPI
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles

from . import config
from .camera_core import get_camera
from .launch_manager import get_manager
from .routes import camera, control, dashboard, mission, nav
from .ros_bridge import get_bridge
from .state_bus import StateSnapshot, get_bus

_LOG = logging.getLogger("infra.server")


# ---------------------------------------------------------------------------
# State producer
# ---------------------------------------------------------------------------


try:  # psutil is optional; without it we leave the host dict empty.
    import psutil as _psutil  # type: ignore
except Exception:  # pragma: no cover — psutil missing on dev machines
    _psutil = None  # type: ignore[assignment]


def _collect_host() -> Dict[str, Any]:
    if _psutil is None:
        return {}
    try:
        cpu = _psutil.cpu_percent(interval=None)
        mem = _psutil.virtual_memory().percent
        disk = _psutil.disk_usage(str(Path.home())).percent
    except Exception:  # pragma: no cover — platform-specific fallbacks
        return {}
    return {"cpu": cpu, "mem": mem, "disk": disk}


async def _state_producer(stop: asyncio.Event) -> None:
    """Publish a state snapshot at :data:`config.STATE_HZ` until ``stop``."""
    bus = get_bus()
    manager = get_manager()
    bridge = get_bridge()
    period = 1.0 / max(config.STATE_HZ, 0.1)
    # Warm psutil so the first call returns a sensible value.
    if _psutil is not None:
        try:
            _psutil.cpu_percent(interval=None)
        except Exception:  # pragma: no cover
            pass

    loop = asyncio.get_running_loop()

    def collect_ros():
        # Master XML-RPC and TF lookups block: keep them off the event loop.
        ros = {"master": bridge.is_master_alive(), "nodes": bridge.get_ros_nodes()}
        return ros, bridge.get_pose(), bridge.get_mission_state()

    while not stop.is_set():
        try:
            ros, pose, mission = await loop.run_in_executor(None, collect_ros)
            snapshot = StateSnapshot(
                host=_collect_host(),
                launches=[r.to_dict() for r in manager.all_records()],
                ros=ros,
                topics=bridge.get_topics_snapshot(),
                pose=pose,
                mission=mission,
            )
            bus.publish_state(snapshot)
        except Exception:  # one bad tick must not stop the dashboard
            _LOG.exception("state producer tick failed")
        try:
            await asyncio.wait_for(stop.wait(), timeout=period)
        except asyncio.TimeoutError:
            continue
    _LOG.info("state producer stopped")


# ---------------------------------------------------------------------------
# App factory
# ---------------------------------------------------------------------------


def create_app() -> FastAPI:
    """Build the FastAPI application. Called by both the CLI and tests."""

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        config.ensure_runtime_dirs()
        get_bus().bind_loop(asyncio.get_running_loop())
        bridge = get_bridge()
        bridge.initialize()

        stop = asyncio.Event()
        task = asyncio.create_task(_state_producer(stop), name="infra-state-producer")
        _LOG.info("infra started (readonly=%s)", config.READONLY)
        try:
            yield
        finally:
            stop.set()
            try:
                await asyncio.wait_for(task, timeout=2.0)
            except asyncio.TimeoutError:  # pragma: no cover
                task.cancel()
            get_manager().stop_all(reason="shutdown")
            bridge.shutdown()
            get_bus().close()
            _LOG.info("infra stopped")

    app = FastAPI(title="Smart-Community Infra", version="0.1.0", lifespan=lifespan)

    app.include_router(dashboard.router)
    app.include_router(control.router)
    app.include_router(nav.router)
    app.include_router(camera.router)
    app.include_router(mission.router)

    # Static assets --------------------------------------------------------
    web_dir = config.WEB_DIR
    if web_dir.is_dir():
        app.mount("/static", StaticFiles(directory=str(web_dir)), name="static")

        @app.get("/", response_class=HTMLResponse)
        async def index() -> FileResponse:  # noqa: D401 — framework handler
            return FileResponse(str(web_dir / "index.html"))

    return app


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def _parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Smart-Community infra workbench")
    parser.add_argument("--host", "--bind", dest="host", default=config.DEFAULT_BIND,
                        help="interface to bind (default: %(default)s)")
    parser.add_argument("--port", type=int, default=config.DEFAULT_PORT,
                        help="HTTP port (default: %(default)s)")
    parser.add_argument("--readonly", action="store_true", default=config.READONLY,
                        help="disable all mutating endpoints")
    parser.add_argument("--log-level", default="info", choices=["critical", "error", "warning", "info", "debug"],
                        help="uvicorn log level")
    return parser.parse_args(argv)


def main(argv: Optional[list] = None) -> int:
    args = _parse_args(argv)
    # Reflect CLI into the config module so dependency checks see it.
    config.READONLY = bool(args.readonly)
    config.DEFAULT_BIND = args.host
    config.DEFAULT_PORT = args.port

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s | %(message)s",
    )

    try:
        import uvicorn  # type: ignore
    except ImportError:
        print("error: uvicorn is not installed. pip install -r requirements.txt", file=sys.stderr)
        return 2

    app = create_app()

    class _Server(uvicorn.Server):
        """Start stopping launches on the first signal.

        uvicorn first waits for open connections (MJPEG streams, WebSockets)
        before running the lifespan shutdown; roslaunch only grants
        ~15 s before SIGTERM/SIGKILL. Stopping children right away
        (in parallel) and ending the streams avoids leaking roslaunch
        processes when the workbench is killed.
        """

        _stopping = False

        def handle_exit(self, sig, frame):  # noqa: D401
            if not self._stopping:
                self._stopping = True
                _LOG.info("received signal %s, stopping launches", sig)
                get_camera().close()
                threading.Thread(target=get_manager().stop_all, kwargs={"reason": "shutdown"},
                                 name="infra-stop-all", daemon=True).start()
                bus = get_bus()
                if bus._loop is not None and not bus._loop.is_closed():
                    bus._loop.call_soon_threadsafe(bus.close)
            super().handle_exit(sig, frame)

    _Server(uvicorn.Config(app, host=args.host, port=args.port, log_level=args.log_level)).run()
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
