"""Per-robot geometry: one place for the numbers the nav stack used to bake in.

VECTOR's geometry lived as module constants in rplidar_c1, lidar_odometry,
costmap2d and recovering_planner. MOSS runs the same modules with a different
body, so those modules now read a RobotProfile at call time:

    ROBOT_PROFILE unset or "vector"  -> VECTOR, exactly today's constants
    ROBOT_PROFILE=moss               -> MOSS (placeholders, env-overridable)

The module constants stay where they were, as the VECTOR values (other code
and benches import them); tests/test_robot_profile_cold.py proves the VECTOR
profile equals them, so an unset ROBOT_PROFILE changes nothing.

Frames: base_link IS the lidar's floor projection on both robots
(lidar_xy_base_m = (0, 0)). The odometry publishes world->base_link at the
lidar's x, y; the costmap body footprint and the planner clearances centre on
it. A lidar mounted off the rotation centre would need that offset applied in
lidar_odometry first - the field exists so the number has a home, nothing
reads a non-zero value yet.

This module is pure (no dimOS import): the benches, the launcher's preflight
and the cockpit import it on any machine. It also holds MOSS's device and
state-dir resolution, for the same reason.
"""
from __future__ import annotations

import glob
import logging
import math
import os
from dataclasses import dataclass, replace
from pathlib import Path
from typing import NamedTuple

PROFILE_ENV = "ROBOT_PROFILE"

_log = logging.getLogger("vector_dimos.robot_profile")


@dataclass(frozen=True)
class RobotProfile:
    name: str                                   # "vector" | "moss"
    lidar_height_m: float                       # lidar_link above base_link
    lidar_yaw_offset_deg: float                 # lidar 0-deg mark vs the body's +x
    lidar_xy_base_m: tuple[float, float]        # lidar axis in base_link (0, 0 on both: see the module docstring)
    scan_min_range_m: float                     # returns closer than this, any direction, are dropped
    scan_mask_sectors_deg: tuple[tuple[float, float], ...]   # (start, end) sectors dropped under scan_mask_range_m; may wrap past 360
    scan_mask_range_m: float
    body_box_m: tuple[float, float]             # half extents |x|, |y| of returns that are the robot itself
    body_clear_m: tuple[float, float]           # half extents of the footprint certainly free under the body
    robot_width_m: float
    pivot_diameter_m: float
    control_margin_m: float
    track_m: float                              # track centre distance (0.0 = not a tracked/differential base)
    camera_xyz_base_m: tuple[float, float, float]   # depth camera in base_link (lidar_odometry's projection)
    camera_pitch_deg: float                     # pitched DOWN by this much
    max_speed_mps: float                        # speed watchdog ceiling
    # The costmap's end-of-mission freeze. True (VECTOR): stop_explore_cmd or
    # explore_done close map writing for the rest of the run. False (MOSS):
    # they stop the explorer only and the map keeps writing - MOSS has no
    # sonar or bumpers, the lidar map is its only obstacle source, and a
    # cockpit goal or STOP sends stop_explore_cmd mid-run.
    freeze_map_on_stop: bool = True


# Today's VECTOR constants, value for value (rplidar_c1 LIDAR_YAW_OFFSET_DEG,
# MAST_MASK_DEG, MASK_RANGE_M, MIN_RANGE_M and the scan_to_points body box;
# lidar_odometry LIDAR_HEIGHT_M, CAMERA_XYZ_BASE, CAMERA_PITCH_RAD; costmap2d
# body_clear; recovering_planner ROBOT_WIDTH_M, PIVOT_DIAMETER_M,
# CONTROL_MARGIN_M; tools/garde_vitesse LIMIT_MS). costmap2d's own
# CAMERA_HEIGHT_M / CAMERA_X_BASE_M are a different (older) mount and stay
# in costmap2d untouched.
VECTOR = RobotProfile(
    name="vector",
    lidar_height_m=0.37,
    lidar_yaw_offset_deg=2.75,
    lidar_xy_base_m=(0.0, 0.0),
    scan_min_range_m=0.15,
    scan_mask_sectors_deg=((348.0, 12.0),),
    scan_mask_range_m=0.30,
    body_box_m=(0.33, 0.25),
    body_clear_m=(0.31, 0.23),
    robot_width_m=0.50,
    pivot_diameter_m=0.78,
    control_margin_m=0.05,
    track_m=0.0,                 # mecanum: no track
    camera_xyz_base_m=(0.14, 0.0, 0.32),
    camera_pitch_deg=6.3,
    max_speed_mps=0.35,
    freeze_map_on_stop=True,     # today's costmap2d: a stopped mission freezes the map
)

# MOSS: EVERY value below is a PLACEHOLDER until measured on the rover (CAD
# nominal or a guess with margin). Each one has an env override, read at call
# time, so a flight can be re-scaled without a code change:
#   MOSS_LIDAR_HEIGHT_M       0.20          lidar scan plane above the floor
#   MOSS_LIDAR_YAW_DEG        0.0           lidar 0-deg mark vs the nose
#   MOSS_LIDAR_XY_M           "0 0"         keep 0 0 (see the module docstring)
#   MOSS_SCAN_MIN_RANGE_M     0.15
#   MOSS_SCAN_MASK_DEG        ""            sectors "a b, c d" (deg); empty = none
#   MOSS_SCAN_MASK_RANGE_M    0.30
#   MOSS_BODY_BOX_M           "0.22 0.18"   returns inside are the robot
#   MOSS_BODY_CLEAR_M         "0.20 0.16"   cleared under the body
#   MOSS_ROBOT_WIDTH_M        0.34
#   MOSS_PIVOT_DIAMETER_M     0.50
#   MOSS_CONTROL_MARGIN_M     0.05
#   MOSS_TRACK_M              0.266         CAD centre distance (moss_adapter reads the same variable)
#   MOSS_CAMERA_XYZ_M         "0.156 0 0.075"  D455F face on the nose, CAD nominal
#   MOSS_CAMERA_PITCH_DEG     0.0
#   MOSS_MAX_SPEED_MPS        0.30
MOSS = RobotProfile(
    name="moss",
    lidar_height_m=0.20,
    lidar_yaw_offset_deg=0.0,
    lidar_xy_base_m=(0.0, 0.0),
    scan_min_range_m=0.15,
    scan_mask_sectors_deg=(),
    scan_mask_range_m=0.30,
    body_box_m=(0.22, 0.18),
    body_clear_m=(0.20, 0.16),
    robot_width_m=0.34,
    pivot_diameter_m=0.50,
    control_margin_m=0.05,
    track_m=0.266,
    camera_xyz_base_m=(0.156, 0.0, 0.075),
    camera_pitch_deg=0.0,
    max_speed_mps=0.30,
    freeze_map_on_stop=False,    # not a placeholder: a design choice (see the field), no env override
)

PROFILES = {"vector": VECTOR, "moss": MOSS}

# field -> (env var, number of floats; 0 = list of (start, end) pairs)
_MOSS_ENV = {
    "lidar_height_m": ("MOSS_LIDAR_HEIGHT_M", 1),
    "lidar_yaw_offset_deg": ("MOSS_LIDAR_YAW_DEG", 1),
    "lidar_xy_base_m": ("MOSS_LIDAR_XY_M", 2),
    "scan_min_range_m": ("MOSS_SCAN_MIN_RANGE_M", 1),
    "scan_mask_sectors_deg": ("MOSS_SCAN_MASK_DEG", 0),
    "scan_mask_range_m": ("MOSS_SCAN_MASK_RANGE_M", 1),
    "body_box_m": ("MOSS_BODY_BOX_M", 2),
    "body_clear_m": ("MOSS_BODY_CLEAR_M", 2),
    "robot_width_m": ("MOSS_ROBOT_WIDTH_M", 1),
    "pivot_diameter_m": ("MOSS_PIVOT_DIAMETER_M", 1),
    "control_margin_m": ("MOSS_CONTROL_MARGIN_M", 1),
    "track_m": ("MOSS_TRACK_M", 1),
    "camera_xyz_base_m": ("MOSS_CAMERA_XYZ_M", 3),
    "camera_pitch_deg": ("MOSS_CAMERA_PITCH_DEG", 1),
    "max_speed_mps": ("MOSS_MAX_SPEED_MPS", 1),
}

_warned: set[tuple[str, str]] = set()


def _warn_once(var: str, raw: str, why: str) -> None:
    # call-time parsing runs every lidar revolution: say a bad value once
    if (var, raw) not in _warned:
        _warned.add((var, raw))
        _log.warning("%s=%r ignored (%s) - placeholder kept", var, raw, why)


def _floats(raw: str) -> list[float]:
    vals = [float(t) for t in raw.replace(",", " ").replace(";", " ").split()]
    if not all(math.isfinite(v) for v in vals):
        raise ValueError("not finite")
    return vals


def _parse(var: str, raw: str, count: int, default):
    try:
        if count == 0:
            if raw.strip().lower() in ("", "none", "-"):
                return ()
            vals = _floats(raw)
            if len(vals) % 2:
                raise ValueError("sectors come in (start, end) pairs")
            return tuple((vals[i] % 360.0, vals[i + 1] % 360.0) for i in range(0, len(vals), 2))
        vals = _floats(raw)
        if len(vals) != count:
            raise ValueError(f"expected {count} number(s)")
        return vals[0] if count == 1 else tuple(vals)
    except ValueError as exc:
        _warn_once(var, raw, str(exc))
        return default


def moss_profile(env: dict | None = None) -> RobotProfile:
    """MOSS with the MOSS_* env overrides applied (env defaults to os.environ)."""
    env = os.environ if env is None else env
    changes = {}
    for field_name, (var, count) in _MOSS_ENV.items():
        raw = env.get(var)
        if raw is None or (raw.strip() == "" and count != 0):
            continue
        value = _parse(var, raw, count, getattr(MOSS, field_name))
        if value != getattr(MOSS, field_name):
            changes[field_name] = value
    return replace(MOSS, **changes) if changes else MOSS


def current(env: dict | None = None) -> RobotProfile:
    """The profile ROBOT_PROFILE names, read NOW (not at import): VECTOR when
    unset. An unknown name falls back to VECTOR with a warning - the robot
    whose numbers every module was written against."""
    env = os.environ if env is None else env
    name = env.get(PROFILE_ENV, "vector").strip().lower() or "vector"
    if name == "vector":
        return VECTOR
    if name == "moss":
        return moss_profile(env)
    _warn_once(PROFILE_ENV, name, f"unknown profile, known: {sorted(PROFILES)}")
    return VECTOR


class Derived(NamedTuple):
    lethal_clearance_m: float
    pivot_clearance_m: float


def derived(profile: RobotProfile | None = None) -> Derived:
    """Planner/explorer clearances from the body: lethal = width/2 + margin,
    pivot = pivot diameter / 2 (VECTOR: 0.30 and 0.39, the same float
    expressions as recovering_planner's constants)."""
    p = profile or current()
    return Derived(p.robot_width_m / 2 + p.control_margin_m, p.pivot_diameter_m / 2)


# ── MOSS devices and state (pure: the preflight and the cockpit use them) ──

BYID_ROOT = "/dev/serial/by-id"
MOSS_QTPY_GLOB = "*QT_Py*"
MOSS_LIDAR_GLOB = "*CP2102N*"
# A missing device resolves to a path that cannot exist: the module that opens
# it fails loudly with this name in the log. Never a fallback to another
# device - on MOSS /dev/ttyUSB0 is the lidar, not the motor board.
MOSS_QTPY_MISSING = "/dev/serial/by-id/MISSING-QT-PY"
MOSS_LIDAR_MISSING = "/dev/serial/by-id/MISSING-C1"


def resolve_by_id(pattern: str, missing: str, root: str | None = None) -> str:
    """First match (sorted, so the choice is stable) of root/pattern, else
    the sentinel. The path returned keeps the by-id form: it survives a
    re-enumeration that renumbers ttyACM/ttyUSB."""
    base = root if root is not None else BYID_ROOT
    hits = sorted(glob.glob(os.path.join(base, pattern)))
    return hits[0] if hits else missing


def moss_qtpy_port(env: dict | None = None) -> str:
    """MOSS motor board: MOSS_PORT if set, else the QT Py by-id, else the sentinel."""
    env = os.environ if env is None else env
    return env.get("MOSS_PORT", "").strip() or resolve_by_id(MOSS_QTPY_GLOB, MOSS_QTPY_MISSING)


def moss_lidar_port(env: dict | None = None) -> str:
    """MOSS lidar: MOSS_LIDAR_PORT if set, else the CP2102N by-id, else the sentinel."""
    env = os.environ if env is None else env
    return env.get("MOSS_LIDAR_PORT", "").strip() or resolve_by_id(MOSS_LIDAR_GLOB, MOSS_LIDAR_MISSING)


def moss_state_dir(env: dict | None = None) -> Path:
    """MOSS_STATE_DIR, default ~/.local/state/moss (expanded): MOSS's own
    tree, never VECTOR's ~/.local/state/vector."""
    env = os.environ if env is None else env
    raw = env.get("MOSS_STATE_DIR", "").strip()
    # absolute, always: dimOS resolves a relative recorder path into its own
    # site-packages, and a cwd-relative state dir depends on who launched
    return Path(raw or "~/.local/state/moss").expanduser().absolute()


def moss_recording_path(env: dict | None = None) -> Path:
    """Where MossRecorder writes: <state>/recordings/moss.db (absolute - a
    relative db_path lands in dimOS's site-packages)."""
    return moss_state_dir(env) / "recordings" / "moss.db"
