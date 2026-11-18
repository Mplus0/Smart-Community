"""FastAPI entry point for the Smart-Community infra workbench.

Run with::

    python -m infra.server --port 8000

or via the sibling ``run.sh`` wrapper. Later the ROS launch file
``community_bringup/launch/infra.launch`` will invoke the same
module with ``--port`` / ``--bind`` / ``--readonly`` taken from
``roslaunch`` args.

Batch 1 scope:

* Mount the four tab-shell HTMLs under ``/``.
* Serve ``/api/health``, state WS, launch control, log tail.
* Run a periodic state producer that reports launch state + host CPU/mem
  when :mod:`psutil` is available.
* Clean shutdown via ``SIGINT``/``SIGTERM`` — every live launch is
  stopped before uvicorn exits.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, Optional

from fastapi import FastAPI
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles

from . import config
from .launch_manager import get_manager
from .routes import control, dashboard, stubs, nav, camera, mission
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

    while not stop.is_set():
        snapshot = StateSnapshot(
            host=_collect_host(),
            launches=[r.to_dict() for r in manager.all_records()],
            # Batch 2: Add ROS bridge data
            ros={
                "master": bridge.is_master_alive(),
                "nodes": bridge.get_ros_nodes(),
            },
            topics=bridge.get_topics_snapshot(),
            pose=bridge.get_pose(),
        )
        bus.publish_state(snapshot)
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
        # Batch 2: Initialize ROS bridge
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
            bridge.shutdown()  # Batch 2: Shutdown ROS bridge
            get_bus().close()
            _LOG.info("infra stopped")

    app = FastAPI(title="Smart-Community Infra", version="0.1.0", lifespan=lifespan)

    app.include_router(dashboard.router)
    app.include_router(control.router)
    app.include_router(nav.router)  # Batch 2: Add navigation routes
    app.include_router(camera.router)  # Batch 3: Add camera routes
    app.include_router(mission.router)  # Batch 4: Add mission routes
    app.include_router(stubs.router)

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

    # uvicorn installs its own signal handlers, but we still want atexit
    # to fire for the LaunchManager cleanup.
    def _flush_logs(signum, _frame):  # noqa: D401
        _LOG.info("received signal %s", signum)

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _flush_logs)
        except (ValueError, OSError):  # pragma: no cover — not main thread
            pass

    uvicorn.run(app, host=args.host, port=args.port, log_level=args.log_level)
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
