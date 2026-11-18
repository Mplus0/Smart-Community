"""Infra runtime configuration.

Values here are deliberately plain-Python so the module can be imported
without FastAPI, rospy or any heavy dependency. Later batches may back
some of these fields with a YAML file; for the batch-1 skeleton the
defaults live in-source.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

#: Repository root ``robot_ws``. Resolved from this file's location so the
#: infra can be launched via ``python -m infra.server`` or via
#: ``roslaunch`` without additional env configuration.
INFRA_DIR: Path = Path(__file__).resolve().parent
ROBOT_WS_DIR: Path = INFRA_DIR.parent
WEB_DIR: Path = INFRA_DIR / "web"

#: Writable runtime directory (logs, snapshots, …). Mirrors the layout used
#: by the legacy ``tools/`` scripts and ``mission_node``.
RUNTIME_DIR: Path = Path(
    os.environ.get("INFRA_RUNTIME_DIR")
    or (Path.home() / ".ros" / "community" / "infra")
)
LOG_DIR: Path = RUNTIME_DIR / "logs"
SNAPSHOT_DIR: Path = RUNTIME_DIR / "snaps"


# ---------------------------------------------------------------------------
# Network / security
# ---------------------------------------------------------------------------

DEFAULT_PORT: int = int(os.environ.get("INFRA_PORT", "8000"))
DEFAULT_BIND: str = os.environ.get("INFRA_BIND", "0.0.0.0")

#: When ``True`` all mutating endpoints respond ``409 Conflict``. Driven by
#: the ``INFRA_READONLY`` env var or the ``--readonly`` CLI flag.
READONLY: bool = os.environ.get("INFRA_READONLY", "0") not in ("", "0", "false", "False")

#: Optional shared secret checked on mutating requests. Empty string means
#: "no auth" (the default for the in-vehicle network).
AUTH_TOKEN: str = os.environ.get("INFRA_TOKEN", "")


# ---------------------------------------------------------------------------
# State bus / streaming
# ---------------------------------------------------------------------------

#: Dashboard snapshot broadcast rate in Hertz. ``WORKBENCH.md`` §4.1 asks
#: for 2 Hz; keep it configurable for benchmarking.
STATE_HZ: float = float(os.environ.get("INFRA_STATE_HZ", "2.0"))

#: Per-subscriber queue size on the state bus. When the queue is full the
#: oldest snapshot is dropped so a slow WebSocket client never blocks the
#: producer.
STATE_QUEUE_MAX: int = 8


# ---------------------------------------------------------------------------
# cmd_vel limits (match DWA config, see WORKBENCH.md §4.2/§6)
# ---------------------------------------------------------------------------

MAX_LINEAR_SPEED: float = 0.25   # m/s
MAX_ANGULAR_SPEED: float = 0.30  # rad/s
TELEOP_HEARTBEAT_MS: int = 500


# ---------------------------------------------------------------------------
# Launch registry
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class LaunchSpec:
    """Metadata describing a roslaunch (or shell) target.

    The infra never shells out directly from the UI — every entry in
    the registry is validated against this allow-list before we ``Popen``
    it. ``args`` describes the editable fields shown on the Launch panel;
    empty default means "roslaunch picks its own default".
    """

    name: str
    description: str
    kind: str                 # "roslaunch" or "shell"
    package: str = ""         # for kind="roslaunch"
    launch_file: str = ""     # for kind="roslaunch"
    command: List[str] = field(default_factory=list)  # for kind="shell"
    args: Dict[str, str] = field(default_factory=dict)


#: The batch-1 registry mirrors WORKBENCH.md §4.4 but only the launches
#: that already exist in ``community_bringup`` / ``community_mission`` are
#: enabled. Batch 4 added ``field.launch`` and ``debug_perception.launch``.
LAUNCHES: Dict[str, LaunchSpec] = {
    spec.name: spec
    for spec in [
        LaunchSpec(
            name="hardware",
            description="Base driver + IMU + USB camera + chassis TF",
            kind="roslaunch",
            package="community_bringup",
            launch_file="hardware.launch",
            args={
                "base_protocol": "",
                "base_port": "/dev/carserial",
                "imu_port": "/dev/imu",
                "camera_device": "/dev/video0",
                "configure_camera": "false",
            },
        ),
        LaunchSpec(
            name="navigation",
            description="AMCL + move_base + navigation costmap",
            kind="roslaunch",
            package="community_bringup",
            launch_file="navigation.launch",
            args={
                "map": "",
                "navigation_map": "",
                "local_planner": "dwa",
                "route": "",
            },
        ),
        LaunchSpec(
            name="services",
            description="Perception / mission support services",
            kind="roslaunch",
            package="community_bringup",
            launch_file="services.launch",
            args={"data_dir": ""},
        ),
        LaunchSpec(
            name="mapping",
            description="Cartographer or gmapping mapping stack",
            kind="roslaunch",
            package="community_bringup",
            launch_file="mapping.launch",
            args={"algorithm": "cartographer", "rviz": "false"},
        ),
        LaunchSpec(
            name="community",
            description="Hardware + navigation + services (no mission)",
            kind="roslaunch",
            package="community_bringup",
            launch_file="community.launch",
            args={
                "base_protocol": "",
                "map": "",
                "navigation_map": "",
                "route": "",
                "run_mission": "false",
            },
        ),
        LaunchSpec(
            name="finals",
            description="Finals launch (hardware + navigation + mission)",
            kind="roslaunch",
            package="community_bringup",
            launch_file="finals.launch",
            args={"base_protocol": ""},
        ),
        LaunchSpec(
            name="field",
            description="Field deployment (hardware + navigation + services + mission)",
            kind="roslaunch",
            package="community_bringup",
            launch_file="field.launch",
            args={
                "base_protocol": "",
                "map": "",
                "navigation_map": "",
                "route": "",
                "navigation_only": "false",
                "obstacle_mode": "clear",
            },
        ),
        LaunchSpec(
            name="debug_perception",
            description="Perception debugging (camera + services, no hardware/navigation)",
            kind="roslaunch",
            package="community_bringup",
            launch_file="debug_perception.launch",
            args={
                "camera_device": "/dev/video0",
                "configure_camera": "false",
            },
        ),
    ]
}


def get_launch(name: str) -> Optional[LaunchSpec]:
    """Return the launch spec for ``name`` or ``None`` if unknown."""
    return LAUNCHES.get(name)


def ensure_runtime_dirs() -> None:
    """Create the runtime/log/snapshot directories if they do not exist."""
    for path in (RUNTIME_DIR, LOG_DIR, SNAPSHOT_DIR):
        path.mkdir(parents=True, exist_ok=True)
