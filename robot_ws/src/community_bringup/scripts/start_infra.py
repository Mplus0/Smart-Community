#!/usr/bin/env python3
"""Roslaunch shim that execs the infra workbench server.

The infra proper lives at ``robot_ws/infra/``. ``infra.launch``
cannot point directly at that location because ``<node>`` requires the
executable to live inside a ROS package; this shim therefore sits inside
``community_bringup/scripts/`` and simply forwards into
``python -m infra.server`` with the ROS-provided arguments.

Environment variables honoured (set by the launch file):

* ``INFRA_PORT``     — HTTP port (default 8000)
* ``INFRA_BIND``     — bind address (default 0.0.0.0)
* ``INFRA_READONLY`` — 1 to disable mutating endpoints
* ``INFRA_TOKEN``    — optional shared secret for mutating endpoints
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


def _robot_ws() -> Path:
    # scripts/start_infra.py → src/community_bringup/scripts/ → robot_ws/
    return Path(__file__).resolve().parents[3]


def main() -> int:
    robot_ws = _robot_ws()
    sys.path.insert(0, str(robot_ws))
    try:
        from infra.server import main as server_main  # noqa: WPS433
    except Exception as exc:
        print(f"[start_infra] failed to import infra.server: {exc}", file=sys.stderr)
        return 1

    args = [
        "--host", os.environ.get("INFRA_BIND", "0.0.0.0"),
        "--port", os.environ.get("INFRA_PORT", "8000"),
    ]
    if os.environ.get("INFRA_READONLY", "0") not in ("", "0", "false", "False"):
        args.append("--readonly")
    return server_main(args)


if __name__ == "__main__":
    sys.exit(main())
