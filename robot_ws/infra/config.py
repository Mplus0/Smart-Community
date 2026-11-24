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
from typing import Dict, FrozenSet, Optional, Tuple


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
    """Metadata describing a roslaunch target.

    Every entry is validated against this allow-list before we ``Popen``
    it. ``args`` lists the editable fields on the Launch panel; an empty
    default means "roslaunch picks its own default".

    * ``required``   — args that must be non-empty (no launch default).
    * ``components`` — robot subsystems the launch brings up; two launches
      sharing a component would start duplicate nodes and are refused.
    * ``requires``   — components that must already be running.
    """

    name: str
    description: str
    package: str
    launch_file: str
    args: Dict[str, str] = field(default_factory=dict)
    required: Tuple[str, ...] = ()
    components: FrozenSet[str] = frozenset()
    requires: FrozenSet[str] = frozenset()


#: Allowed values for enumerated launch args.
ARG_CHOICES: Dict[str, Tuple[str, ...]] = {
    "base_protocol": ("aa_bb", "legacy_11"),
    "local_planner": ("dwa", "teb"),
    "obstacle_mode": ("clear", "blocked"),
    "algorithm": ("cartographer", "gmapping"),
    "localization_only": ("true", "false"),
    "navigation_only": ("true", "false"),
    "run_mission": ("true", "false"),
    "configure_camera": ("true", "false"),
    "rviz": ("true", "false"),
}

#: Args whose non-empty value must point at an existing file.
FILE_ARGS: Tuple[str, ...] = ("map", "route")

# Component names used by the mutual-exclusion check.
#   hardware: chassis serial + IMU + lidar + chassis TF
#   camera:   main USB camera
#   localization: ekf + robot_state_publisher (navigation or SLAM mapping)
#   navigation: map servers + AMCL + move_base
#   services: perception / speech / recorder services
#   mission:  community_mission node
_HW = frozenset({"hardware", "camera"})
_NAV = frozenset({"localization", "navigation"})
_FULL = _HW | _NAV | {"services"}

#: ROS node that proves a component is already up (also catches launches
#: started from a terminal instead of the workbench).
COMPONENT_NODES: Dict[str, str] = {
    "hardware": "/community_serial",
    "camera": "/usb_cam",
    "localization": "/ekf_localization",
    "navigation": "/move_base",
    "services": "/speech_server",
    "mission": "/community_mission",
}

MAIN_CAMERA_DEVICE = (
    "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Orbbec_DaBai_DCW2_RGB_Camera_CH8X952020W-video-index0"
)

LAUNCHES: Dict[str, LaunchSpec] = {
    spec.name: spec
    for spec in [
        LaunchSpec(
            name="hardware",
            description="底盘串口 + IMU + 雷达 + 主相机和仪表相机",
            package="community_bringup",
            launch_file="hardware.launch",
            args={
                "base_protocol": "",
                "base_port": "/dev/carserial",
                "imu_port": "/dev/imu",
                "camera_device": MAIN_CAMERA_DEVICE,
                "configure_camera": "false",
            },
            required=("base_protocol",),
            components=_HW,
        ),
        LaunchSpec(
            name="navigation",
            description="地图 + AMCL + move_base（需先启动 hardware）",
            package="community_bringup",
            launch_file="navigation.launch",
            args={"map": "", "local_planner": "dwa", "localization_only": "false"},
            components=_NAV,
            requires=frozenset({"hardware"}),
        ),
        LaunchSpec(
            name="services",
            description="识别 / 语音 / 录像服务",
            package="community_bringup",
            launch_file="services.launch",
            args={"data_dir": ""},
            components=frozenset({"services"}),
        ),
        LaunchSpec(
            name="mapping",
            description="建图（需先启动 hardware，与 navigation 互斥）",
            package="community_bringup",
            launch_file="mapping.launch",
            args={"algorithm": "cartographer", "rviz": "false"},
            components=frozenset({"localization"}),
            requires=frozenset({"hardware"}),
        ),
        LaunchSpec(
            name="community",
            description="hardware + navigation + services（run_mission=true 时含巡检）",
            package="community_bringup",
            launch_file="community.launch",
            args={"base_protocol": "", "map": "", "route": "",
                  "local_planner": "dwa", "run_mission": "false"},
            required=("base_protocol",),
            components=_FULL,
        ),
        LaunchSpec(
            name="finals",
            description="决赛：hardware + navigation + services + 巡检",
            package="community_bringup",
            launch_file="finals.launch",
            args={"base_protocol": "", "map": "", "route": ""},
            required=("base_protocol",),
            components=_FULL | {"mission"},
        ),
        LaunchSpec(
            name="field",
            description="上场：hardware + navigation + services + 巡检",
            package="community_bringup",
            launch_file="field.launch",
            args={
                "base_protocol": "",
                "map": "",
                "route": "",
                "local_planner": "dwa",
                "navigation_only": "false",
                "obstacle_mode": "clear",
            },
            required=("base_protocol",),
            components=_FULL | {"mission"},
        ),
        LaunchSpec(
            name="mission",
            description="单独启动巡检任务（需 hardware + navigation 已运行）",
            package="community_mission",
            launch_file="mission.launch",
            args={"single_waypoint": "", "navigation_only": "false",
                  "obstacle_mode": "clear", "route": ""},
            components=frozenset({"mission"}),
            requires=frozenset({"hardware", "navigation"}),
        ),
        LaunchSpec(
            name="debug_perception",
            description="仅相机 + 识别服务（无底盘、无导航）",
            package="community_bringup",
            launch_file="debug_perception.launch",
            args={"camera_device": MAIN_CAMERA_DEVICE, "configure_camera": "false"},
            components=frozenset({"camera", "services"}),
        ),
    ]
}


def get_launch(name: str) -> Optional[LaunchSpec]:
    """Return the launch spec for ``name`` or ``None`` if unknown."""
    return LAUNCHES.get(name)


def launch_components(name: str, args: Dict[str, str]) -> FrozenSet[str]:
    """Components a launch brings up with the given args."""
    spec = LAUNCHES.get(name)
    if spec is None:
        return frozenset()
    if name == "navigation" and args.get("localization_only") == "true":
        return spec.components - {"navigation"}
    if name == "community" and args.get("run_mission") == "true":
        return spec.components | {"mission"}
    return spec.components


def launch_requires(name: str, args: Dict[str, str]) -> FrozenSet[str]:
    """Components that must already run before starting ``name``."""
    spec = LAUNCHES[name]
    # A full mission (not navigation_only) also needs recognition services.
    if name == "mission" and args.get("navigation_only") != "true":
        return spec.requires | {"services"}
    return spec.requires


def validate_launch_args(spec: LaunchSpec, args: Dict[str, str]) -> Optional[str]:
    """Return an error message if ``args`` cannot start ``spec``, else ``None``."""
    for key in spec.required:
        if not args.get(key):
            hint = ARG_CHOICES.get(key)
            return f"缺少必填参数 {key}" + (f"（可选：{' / '.join(hint)}）" if hint else "")
    for key, value in args.items():
        if not value:
            continue
        choices = ARG_CHOICES.get(key)
        if choices and value not in choices:
            return f"参数 {key}={value!r} 无效，可选：{' / '.join(choices)}"
        if key in FILE_ARGS and not Path(value).expanduser().is_file():
            return f"参数 {key} 指向的文件不存在：{value}"
    return None


def ensure_runtime_dirs() -> None:
    """Create the runtime/log/snapshot directories if they do not exist."""
    for path in (RUNTIME_DIR, LOG_DIR, SNAPSHOT_DIR):
        path.mkdir(parents=True, exist_ok=True)
