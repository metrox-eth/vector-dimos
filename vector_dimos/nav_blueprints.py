"""Mapping / navigation blueprints - kept apart from blueprints.py because
dimOS's mapping modules pull torch + open3d; a missing heavy dependency must
only take `vector-dimos.nav` down, never the base / cockpit / lidar blueprints.

Which frontier explorer the `explore` blueprint builds is an A/B switch, read
once when the blueprint is built so a run is one strategy for its whole life:

    EXPLORER_V2=0     the 25-26/08 explorer (fast_explorer.VectorExplorer):
                      dimOS's weighted-sum scoring, its info-gain self-stop
    unset / anything  explorer2.Explorer2: information gain per path cost, and
    else (DEFAULT)    no way to stop but "no reachable frontier left"

Both publish the same goals on the same `goal_request` topic, so the two are
directly comparable with tools/bench_run.py on two real runs, and offline with
tools/explore_sim.py.
"""
from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

from vector_dimos.blueprints import VectorControlCoordinator, _coordinator_blueprint
from vector_dimos.explorer2 import explorer_v2_enabled


def stock_nav_enabled() -> bool:
    """STOCK_NAV=1: the plain dimOS stack.
    dimOS's own CostMapper (occupancy algo) + the stock
    wavefront frontier explorer (VectorExplorer subclasses it for speed only).
    Our custom 2D map / explorer2 / persistent map go DORMANT - parked, not
    deleted. Stays: kiss-icp pose (gyro only), bump reflexes, the deck."""
    return os.environ.get("STOCK_NAV", "0") == "1"

def camera_mount():
    """base_link -> camera_link as mounted since 24/08 (bumper build): 0.20 m
    BEHIND the lidar axis (tape-measured), 0.56 m up (floor fit), nose 1.1 deg
    down. Matches CAMERA_XYZ_BASE in lidar_odometry.py.

    Frame convention, read off dimOS 25/08: ``base_transform`` is
    base_link -> camera_link in REP-103 BODY axes (x forward, y left, z up).
    ``RealSenseCamera._publish_tf`` publishes it unchanged, then
    ``_build_mount_edges`` appends camera_link -> camera_*_frame (factory
    extrinsics, body axes) -> camera_*_optical_frame carrying OPTICAL_ROTATION
    (-0.5, 0.5, -0.5, 0.5) from dimos/hardware/sensors/camera/spec.py. dimOS
    adds the optical rotation itself, so this mount must carry the MECHANICAL
    pose only - putting the optical rotation here would apply it twice.

    Hence a pure pitch: +1.1 deg about base_link's +y (left) axis = nose down.
    Axis map, following the optical line of sight z_opt = (0, 0, 1) outwards:
        OPTICAL_ROTATION:  z_opt -> +x (forward), x_opt -> -y, y_opt -> -z
        this mount:        +x    -> (cos 1.1, 0, -sin 1.1) = (0.9998, 0, -0.0192)
    i.e. the frustum looks forward over the body, 1.1 deg down.
    """
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    return Transform(translation=Vector3(-0.20, 0.0, 0.56),
                     rotation=Quaternion(0.0, 0.00959916, 0.0, 0.99995393))


CAMERA_MOUNT = camera_mount()


# --- Rerun display -------------------------------------------------------
#
# dimOS's bridge (dimos/visualization/rerun/bridge.py, ``_on_message``) converts
# every message with a bare ``msg.to_rerun()`` - no arguments - then attaches the
# entity to its TF frame with ``rr.Transform3D(parent_frame="tf#/<frame_id>")``.
# Two layers need that default overridden. ``visual_override`` is the hook, and
# its callables must be module-level: the blueprint is pickled to the worker.

VOXEL_SIZE = 0.05
# Height band the voxel map is coloured over, metres in the world frame. Fixed
# on purpose: PointCloud2.to_rerun() normalises its turbo ramp over the map's
# OWN z range, so a map with no height in it - which is all a 2D lidar can
# produce, every voxel sitting in the scan plane - has z.max() == z.min(), every
# class_id collapses to 0, and the layer is drawn in the darkest turbo colour on
# the blueprint's black background: invisible.
VOXEL_COLOR_FLOOR = 0.0
VOXEL_COLOR_CEILING = 1.2


def voxel_map_view(cloud):
    """``global_map`` coloured by ABSOLUTE height, so a flat map stays visible."""
    import numpy as np
    import rerun as rr

    points = cloud.points_f32()
    if len(points) == 0:
        return rr.Points3D([])
    height = (points[:, 2] - VOXEL_COLOR_FLOOR) / (VOXEL_COLOR_CEILING - VOXEL_COLOR_FLOOR)
    class_ids = (np.clip(height, 0.0, 1.0) * 255).astype(np.uint8)
    # class_ids resolve through the turbo AnnotationContext dimOS logs static at
    # "/" in rerun_init(); radii = half a voxel is the stock look.
    return rr.Points3D(positions=points, radii=VOXEL_SIZE / 2, class_ids=class_ids)


def camera_frustum_view(camera_info):
    """The camera frustum on the mast instead of at the world origin.

    A rerun 0.32 Pinhole owns the frame of the entity it is logged on: it
    connects that frame to the frame of the entity-path parent, here
    ``tf#/world``. So on ``world/camera_info`` the bridge's
    ``Transform3D(parent_frame="tf#/camera_color_optical_frame")`` is a SECOND
    parent for the same frame, rerun refuses it - "Any frame is only ever
    allowed to have a single parent at any given time" (re_tf::transform_forest)
    - and keeps the pinhole edge: the frustum is drawn at the world origin along
    world +z, at the sky, where no mount transform can reach it.

    Logging ``CoordinateFrame("tf#/camera_color_optical_frame")`` instead is
    worse, and silently: the pinhole then re-parents THAT tf frame to
    ``tf#/world`` (no warning), so the frustum still sits at the origin.

    What works, measured in the viewer 25/08: pose the entity, and give the
    pinhole a child path of its own to own. Returning a list of
    (path, archetype) also makes the bridge skip its own frame attachment.
    """
    import rerun as rr

    return [
        ("world/camera_info", rr.Transform3D(parent_frame=f"tf#/{camera_info.frame_id}")),
        # 1 m (the stock plane distance) dwarfs a 0.62 m rover.
        ("world/camera_info/frustum", camera_info.to_rerun(image_plane_distance=0.4)),
    ]


# --- the DECISION costmap ------------------------------------------------
#
# `global_costmap` (VectorCostMap) is the map the planner and the explorer
# actually read: 100 occupied, 0 free (observed), -1 unknown, nothing else.
# It WAS reaching Rerun all along - the bridge logs every message that has a
# to_rerun() - but it arrived invisible, for two reasons, both in
# dimos/msgs/nav_msgs/OccupancyGrid.py:
#
#   * `_build_occupancy_lut(colormap=None, ...)` is the default LUT, and it
#     paints cost `c` as 72*(1 - c/100), 73*(1 - c/100), 129*(1 - c/100). At
#     c = 100 that is (0, 0, 0): the lethal cells, the only ones that matter,
#     are drawn PURE BLACK. Unknown (-1) is hardcoded black too. On the
#     blueprint's black background the decision map was two shades of nothing.
#   * `to_rerun()` draws one opaque textured quad over the whole crop, so the
#     free cells - most of the map - were an opaque slab lying on the floor,
#     and it decimates any grid wider than 256 cells (`grid[::step_h, ::step_w]`,
#     a stride, not a max), which drops exactly the one-cell table legs this
#     map exists to show.
#
# Alpha is not a way out. Measured 26/08 in a rerun 0.32 viewer: a Mesh3D
# albedo_texture ignores its alpha channel (a magenta half at alpha 0 rendered
# opaque magenta over the markers underneath). So "free = transparent" can only
# mean "no geometry there at all", and the layer is drawn as Points3D over the
# cells that are NOT free - one point per cell, at full grid resolution, no
# decimation, so a leg seen in a single 5 cm cell is drawn.
#
# Three child entities rather than one coloured cloud, so each can be switched
# off in the viewer tree - `unknown` is by far the biggest (~39-47k cells on a
# mapped flat, against ~4k obstacle cells) and is the one to hide first if the
# Jetson complains.
#
# `keepout` is drawn as flat solid slabs, not as one point per cell: with the
# 2026-08-26 fences around the house the zones cover ~95k cells of a mapped crop -
# two megabytes of points, 1-2 times a second, to say what a few labelled boxes
# say better. A rectangle is one slab. A polygon (the operator draws a tilted
# house with the mouse now) is one slab per horizontal run of its rasterised
# mask - a few hundred for a room-sized zone, and stair-stepped at 5 cm on the
# diagonals, which is exactly the shape the rover obeys.
COSTMAP_Z = -0.01              # a hair under the floor: never fights the voxel cloud
COSTMAP_OCCUPIED = (220, 30, 30)    # lethal: the planner will not enter
COSTMAP_KEEPOUT = (255, 140, 0)     # lethal because a HUMAN drew a keep-out zone there
COSTMAP_UNKNOWN = (70, 70, 70)      # never observed - not free, not an obstacle

_FORBIDDEN_ZONES: dict = {"mtime": None, "zones": []}
# The rasterised zones for one published crop: recomputing a polygon mask over a
# 400 x 400 crop 1-2 times a second is cheap but pointless - nothing about it
# moves until the file or the crop does.
_ZONE_RASTER: dict = {"key": None, "items": []}


def _erode1(mask):
    """Drop the one-cell border of a mask (no scipy on the Jetson build)."""
    import numpy as np

    out = np.zeros_like(mask)
    out[1:-1, 1:-1] = (mask[1:-1, 1:-1] & mask[:-2, 1:-1] & mask[2:, 1:-1]
                       & mask[1:-1, :-2] & mask[1:-1, 2:])
    return out


def _mask_runs(mask, res: float, ox: float, oy: float):
    """A cell mask as flat boxes: one per horizontal run of cells."""
    import numpy as np

    boxes = []
    for row in np.nonzero(mask.any(axis=1))[0]:
        cols = np.nonzero(mask[row])[0]
        for seg in np.split(cols, np.nonzero(np.diff(cols) > 1)[0] + 1):
            c0, c1 = int(seg[0]), int(seg[-1])
            boxes.append(((ox + (c0 + c1 + 1) * res / 2,
                           oy + (2 * int(row) + 1) * res / 2, COSTMAP_Z),
                          ((c1 - c0 + 1) * res / 2, res / 2, 0.001)))
    return boxes


def _raster_zones(zones, shape, res: float, ox: float, oy: float):
    """Each zone as (cell mask, interior mask, boxes, label), on this crop.

    The interior mask is the zone minus its one-cell border: the map floors a
    zone against its OWN origin and this floors it against the published crop's,
    and the two disagree by a cell on an edge often enough to matter (measured
    26/08 on the live map: 627 border cells out of 95k). The interior is the
    honest place to ask whether the zone is in force.
    """
    import numpy as np

    from vector_dimos import persistent_map

    h, w = shape
    items = []
    for z in zones:
        pts = persistent_map.zone_points(z)
        label = str(z.get("label", "forbidden"))
        if pts is None:
            # same arithmetic as persistent_map.keepout_mask, on the published crop
            x0, x1 = int((z["x0"] - ox) // res), int((z["x1"] - ox) // res)
            y0, y1 = int((z["y0"] - oy) // res), int((z["y1"] - oy) // res)
            if x1 < 0 or y1 < 0 or x0 > w - 1 or y0 > h - 1:
                continue                   # this zone falls outside the crop
            x0, x1 = max(0, x0), min(w - 1, x1)
            y0, y1 = max(0, y0), min(h - 1, y1)
            mask = np.zeros((h, w), dtype=bool)
            mask[y0:y1 + 1, x0:x1 + 1] = True
            core = np.zeros((h, w), dtype=bool)
            tx0, tx1 = (x0 + 1, x1 - 1) if x1 - x0 >= 2 else (x0, x1)
            ty0, ty1 = (y0 + 1, y1 - 1) if y1 - y0 >= 2 else (y0, y1)
            core[ty0:ty1 + 1, tx0:tx1 + 1] = True
            boxes = [((ox + (x0 + x1 + 1) * res / 2, oy + (y0 + y1 + 1) * res / 2, COSTMAP_Z),
                      ((x1 - x0 + 1) * res / 2, (y1 - y0 + 1) * res / 2, 0.001))]
        else:
            mask = persistent_map.polygon_mask(pts, res, ox, oy, (h, w))
            if not mask.any():
                continue                   # this zone falls outside the crop
            core = _erode1(mask)
            if not core.any():             # a zone one cell thin: it is all border
                core = mask
            boxes = _mask_runs(mask, res, ox, oy)
        items.append((mask, core, boxes, label))
    return items


def _forbidden_zones(cells, res: float, ox: float, oy: float):
    """The operator-drawn keep-out zones as (cell mask, boxes), or (None, None).

    The published grid cannot say which cells were forced - they are 100 like
    any obstacle - and it does not carry the run's frame either, while the
    zones only apply to a run that relocalized into the persistent frame
    (costmap2d._decide). The map answers that itself: ScoredGrid.occupancy()
    forces EVERY cell of a forbidden zone to 100, so a zone whose interior is
    not solid 100 is a zone that was never applied - and this returns None
    rather than paint an orange rule the rover is not actually obeying.
    """
    import numpy as np

    from vector_dimos import persistent_map

    try:
        mtime = (os.path.getmtime(persistent_map.KEEPOUT_PATH)
                 if os.path.isfile(persistent_map.KEEPOUT_PATH) else None)
        if mtime is None:
            return None, None
        if _FORBIDDEN_ZONES["mtime"] != mtime:
            _FORBIDDEN_ZONES["zones"] = persistent_map.zones_of(
                persistent_map.load_keepouts(), persistent_map.FORBIDDEN)
            _FORBIDDEN_ZONES["mtime"] = mtime
        zones = _FORBIDDEN_ZONES["zones"]
        if not zones:
            return None, None
        key = (mtime, res, ox, oy, cells.shape)
        if _ZONE_RASTER["key"] != key:
            _ZONE_RASTER["items"] = _raster_zones(zones, cells.shape, res, ox, oy)
            _ZONE_RASTER["key"] = key
    except Exception:  # noqa: BLE001 - a display layer never takes the bridge down
        return None, None

    mask = np.zeros(cells.shape, dtype=bool)
    centers, half_sizes, labels = [], [], []
    for zmask, core, boxes, label in _ZONE_RASTER["items"]:
        if not bool((cells[core] == 100).all()):
            return None, None              # not in force in this run
        mask |= zmask
        widest = max(range(len(boxes)), key=lambda i: boxes[i][1][0] * boxes[i][1][1])
        for i, (centre, half) in enumerate(boxes):
            centers.append(centre)
            half_sizes.append(half)
            labels.append(label if i == widest else "")
    if not centers:
        return None, None
    return mask, (centers, half_sizes, labels)


def _cell_points(mask, res: float, ox: float, oy: float, color):
    """One flat point per selected cell, at its centre. Empty clears the layer."""
    import numpy as np
    import rerun as rr

    if mask is None:
        return rr.Points3D(positions=np.zeros((0, 3), dtype=np.float32))
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        return rr.Points3D(positions=np.zeros((0, 3), dtype=np.float32))
    pos = np.empty((len(xs), 3), dtype=np.float32)
    pos[:, 0] = ox + (xs + 0.5) * res
    pos[:, 1] = oy + (ys + 0.5) * res
    pos[:, 2] = COSTMAP_Z
    return rr.Points3D(positions=pos, colors=color, radii=res / 2)


def _zone_boxes(boxes):
    """The keep-out zones as flat solid slabs (one per rectangle, one per
    horizontal run of a polygon). Empty clears the layer."""
    import rerun as rr

    if boxes is None:
        return rr.Boxes3D(centers=[], half_sizes=[])
    centers, half_sizes, labels = boxes
    return rr.Boxes3D(centers=centers, half_sizes=half_sizes, labels=labels,
                      show_labels=True, colors=COSTMAP_KEEPOUT,
                      fill_mode=rr.components.FillMode.Solid)


def decision_costmap_view(grid):
    """`global_costmap` as a flat, readable overlay under the voxel cloud."""
    import rerun as rr

    cells = grid.grid
    if cells.size == 0:
        return None
    res = float(grid.resolution)
    ox, oy = float(grid.origin.position.x), float(grid.origin.position.y)

    keepout, boxes = _forbidden_zones(cells, res, ox, oy)
    occupied = cells == 100
    if keepout is not None:
        occupied = occupied & ~keepout

    # Returning a list of (path, archetype) makes the bridge skip its own frame
    # attachment, so the pose is set here; the children inherit it.
    return [
        ("world/global_costmap",
         rr.Transform3D(parent_frame=f"tf#/{grid.frame_id}")),
        ("world/global_costmap/obstacle",
         _cell_points(occupied, res, ox, oy, COSTMAP_OCCUPIED)),
        ("world/global_costmap/keepout", _zone_boxes(boxes)),
        ("world/global_costmap/unknown",
         _cell_points(cells == -1, res, ox, oy, COSTMAP_UNKNOWN)),
    ]


def flight_rerun_blueprint():
    """Two tabs. "Map": only the PERSISTENT things (decision costmap + zones,
    path, pose, goals) so the view is anchored on the map being built and
    never pumps zoom with the sweeps (2026-08-27). "Live": everything,
    for when the raw sweeps are wanted. Module-level: pickled to the worker."""
    import rerun as rr
    import rerun.blueprint as rrb

    def view(name, contents):
        return rrb.Spatial3DView(
            origin="world", name=name, contents=contents,
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)))

    return rrb.Blueprint(rrb.Tabs(
        view("Map", ["+ $origin/global_costmap/**", "+ $origin/path/**",
                     "+ $origin/odom/**", "+ $origin/goal_request/**"]),
        view("Live", ["+ $origin/**"]),
    ))


RERUN_CONFIG = {
    "blueprint": flight_rerun_blueprint,
    # 512MB replay history instead of the 25% default: on the 8 GB Jetson that
    # default let the Rerun bridge grow to 2.7 GB (measured 2026-08-24).
    "memory_limit": "512MB",
    "visual_override": {
        # The voxel map is COSMETIC and never forgets (a passer-by is engraved
        # forever ON SCREEN even where the decision map corrects). What must be
        # watched is the map the planner actually reads - only that one is shown.
        "world/global_map": None,
        "world/global_costmap": decision_costmap_view,
        "world/camera_info": camera_frustum_view,
        # Aligned depth shares camera_color_optical_frame, so this is the same
        # frustum a second time - and it would stay pinned at the origin.
        "world/depth_camera_info": None,
        # The camera IMAGES never go to Rerun. The cockpit is where the
        # camera is watched; Rerun is
        # the map. Suppressed at the bridge, so the Jetson does not even
        # encode them for the viewer. Both path spellings kept: the entity is
        # "world/<channel>" today, bare "<channel>" if the prefix ever moves.
        "world/color_image": None,
        "world/depth_image": None,
        "color_image": None,
        "depth_image": None,
        # Transient floor samples (5.6 Hz, z=0) and the RAW lidar ring
        # (redundant with world/lidar) kept making the view breathe
        # vertically - the map visibly rising and falling, for two hours.
        # Display only - the mapper still consumes both streams.
        "world/camera_floor": None,
        "camera_floor": None,
        "world/pointcloud": None,
        "pointcloud": None,
    },
}


def _nav_blueprint():
    """Mapping stack: base + RealSense (colour/depth for the depth guard) +
    RPLIDAR C1 -> our lidar odometry (kiss-icp, the pose source: wheels are
    never the reference here) -> dimOS's stock VoxelGridMapper (CPU) and
    CostMapper -> Rerun. Stream names line up by construction: rplidar
    `pointcloud` -> LidarOdometry `pointcloud`; LidarOdometry `lidar` (world
    frame) -> VoxelGridMapper `lidar`; `global_map` -> CostMapper."""
    from dimos.core.global_config import global_config
    from vector_dimos.camera import VectorCamera
    from vector_dimos.costmap2d import VectorCostMap
    from dimos.mapping.costmapper import CostMapper
    from dimos.mapping.voxels.module import VoxelGridMapper
    from dimos.visualization.vis_module import vis_module
    from vector_dimos.lidar_odometry import LidarOdometry
    from vector_dimos.rplidar_c1 import RPLidarC1
    from vector_dimos.respeaker import ReSpeakerMic

    return autoconnect(
        _coordinator_blueprint(),
        # fps 15 -> 5 (MEASURED 2026-08-27): the color stream's only consumer is the
        # cockpit relay (shows 5.5 fps), depth's only consumer is lidar_odometry's
        # camera points (outputs 5.8 Hz) - publishing 15 served nobody and the bus
        # (serialize + every subscriber's deserialize) ate ~2 cores idle (profile
        # py-spy: publish 100%, lcm loops 96/75/38%). 5 is a native D455 mode.
        VectorCamera.blueprint(width=640, height=480, fps=5,
                               enable_depth=True, enable_pointcloud=False,
                               enable_imu=True, imu_hz=200,
                               base_transform=CAMERA_MOUNT),   # their 2nd (motion) pipeline fails on the RSUSB build: "No device connected"
        RPLidarC1.blueprint(),
        ReSpeakerMic.blueprint(stt_language="fr"),   # auto-detect mangles short utterances
        # gyro prior ON - MEASURED correct on 2026-08-27 (tools/gyro_sign_bench:
        # +29.8 deg gyro vs +30.3 deg lidar, -31.2 vs -31.4, mapping "-y" exact
        # on the rear mast). The rotation backbone of the classic indoor
        # lidar+IMU recipe; the 12:05 rainbow smear was the 17-bump storm, not
        # this axis (hypothesis killed by the bench, as benches are for).
        LidarOdometry.blueprint(),
        # emit_every=5: the figure from the go2 blueprint (~2 Hz here). The
        # stopgap value of 10 (~1 Hz) predated CUDA+zenoh - rates restored
        # 2026-08-27. Earlier note: the upstream health norm is a ~7 Hz costmap;
        # at 1 Hz the path-clearance sees obstacles up to 1 s late (unacceptable
        # for autonomy). Raised from 3 because the full-map costmap recompute at
        # 3.3 Hz ate two cores parked (load 41, 2026-08-27). The real fix is the
        # upstream answer.
        VoxelGridMapper.blueprint(voxel_size=0.08, device="CUDA:0", frame_id="world", emit_every=10),
        (CostMapper.blueprint() if stock_nav_enabled() else
         VectorCostMap.blueprint()),   # STOCK_NAV=1 -> dimOS's CostMapper exactly as their go2 ships it
                                       # (defaults; "occupancy" was an INVENTED registry key - valid keys are
                                       # height_cost/general/simple, each map KeyError'd the Rx chain and
                                       # global_costmap fell silent, found 2026-08-27). Default path = our 2D
                                       # map: the PROVEN recipe (git 73cc2c6 ran VectorCostMap everywhere;
                                       # upstream's CostMapper had erased table legs on VECTOR)
        vis_module(viewer_backend=global_config.viewer, rerun_config=RERUN_CONFIG),
    ).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
    ])


nav_blueprint = _nav_blueprint()


def _explorer_blueprint():
    """The frontier explorer, old or new - see the module docstring for the flag.

    goal_timeout=45 in both: 15 s was sized for the stock 0.55 m/s follower, and
    at the capped 0.149 m/s a 3 m frontier needs ~25 s, so every goal timed out
    mid-drive (4 goals, 0 reached, 25/08 21:31). min_frontier_perimeter=0.3 in
    both: 0.5 m of perimeter hid the doorway-sized frontiers in this flat.
    """
    if explorer_v2_enabled() and not stock_nav_enabled():
        from vector_dimos.explorer2 import Explorer2
        # No info_gain_threshold, no num_no_gain_attempts, no lookahead_distance,
        # no max_explored_distance: v2 has no self-stop and no distance-from-
        # explored-goals term. Passing them would be silently ignored, so they
        # are left out on purpose.
        return Explorer2.blueprint(min_frontier_perimeter=0.3, goal_timeout=45.0)
    from vector_dimos.fast_explorer import VectorExplorer
    return VectorExplorer.blueprint(safe_distance=0.35, lookahead_distance=4.0,
                                    min_frontier_perimeter=0.3,
                                    # 1 % gain per goal made it quit once the first room was
                                    # known while the workshop was still unknown (23/08 18:36,
                                    # after 29 m): 0.1 %, 6 tries
                                    info_gain_threshold=0.001, num_no_gain_attempts=6,
                                    max_explored_distance=12.0,
                                    goal_timeout=45.0)


def _explore_blueprint():
    """Autonomous exploration with dimOS's own stack on top of `nav`:
    CostMapper -> ReplanningAStarPlanner (A* + its P controller -> nav_cmd_vel)
    -> MovementManager (-> cmd_vel, the coordinator's twist) and a frontier
    explorer choosing goals on the costmap - explorer2 by default, the 25-26/08
    wavefront one under EXPLORER_V2=0 (see the module docstring). Start it by
    publishing Bool(True) on `explore_cmd`; cap the speed with
    `dimos --nerf-speed 0.3` (the local planner's default is 0.55 m/s).

    STOCK_NAV=1 FLIES WITHOUT KEEP-OUT ZONES. The zones are enforced in exactly
    one place - VectorCostMap.ScoredGrid.occupancy() forcing zone cells to 100
    (costmap2d.py) - and stock mode swaps that module for dimOS's CostMapper,
    which has no zone concept: keepout.json is never read, the bathroom is not
    forbidden, and no line of the run says so (2026-08-28 audit). This is the
    documented cost of the A/B, not a bug to fix here; tools/fly.sh warns at
    gate 2/7 when zones exist on the rover and STOCK_NAV=1."""
    from dimos.core.global_config import global_config
    from vector_dimos.camera import VectorCamera
    from vector_dimos.costmap2d import VectorCostMap
    from dimos.mapping.costmapper import CostMapper
    from dimos.mapping.voxels.module import VoxelGridMapper
    from vector_dimos.esp_sensors import EspSensors
    from vector_dimos.memory import VectorMemory, VectorMemoryLight
    from vector_dimos.relocalization import VectorRelocalization
    from dimos.navigation.movement_manager.movement_manager import MovementManager
    from vector_dimos.recovering_planner import RecoveringPlanner
    from dimos.visualization.vis_module import vis_module
    from vector_dimos.lidar_odometry import LidarOdometry
    from vector_dimos.rplidar_c1 import RPLidarC1
    from vector_dimos.respeaker import ReSpeakerMic

    extra = []
    if os.environ.get("GAMEPAD", "0") == "1":
        # the day-one teleop module rides along: a human drives on
        # tele_cmd_vel while the SAME stack maps (the map-quality test).
        from vector_dimos.gamepad import GamepadTeleop
        extra = [GamepadTeleop.blueprint()]
    return autoconnect(
        *extra,
        _coordinator_blueprint(),
        # fps 15 -> 5 (MEASURED 2026-08-27): the color stream's only consumer is the
        # cockpit relay (shows 5.5 fps), depth's only consumer is lidar_odometry's
        # camera points (outputs 5.8 Hz) - publishing 15 served nobody and the bus
        # (serialize + every subscriber's deserialize) ate ~2 cores idle (profile
        # py-spy: publish 100%, lcm loops 96/75/38%). 5 is a native D455 mode.
        VectorCamera.blueprint(width=640, height=480, fps=5,
                               enable_depth=True, enable_pointcloud=False,
                               enable_imu=True, imu_hz=200,
                               base_transform=CAMERA_MOUNT),
        RPLidarC1.blueprint(),
        ReSpeakerMic.blueprint(stt_language="fr"),   # auto-detect mangles short utterances
        # gyro prior ON - MEASURED correct on 2026-08-27 (tools/gyro_sign_bench:
        # +29.8 deg gyro vs +30.3 deg lidar, -31.2 vs -31.4, mapping "-y" exact
        # on the rear mast). The rotation backbone of the classic indoor
        # lidar+IMU recipe; the 12:05 rainbow smear was the 17-bump storm, not
        # this axis (hypothesis killed by the bench, as benches are for).
        LidarOdometry.blueprint(),
        # carve_columns, the FULL story (claude-mem, three flips on 23/08 alone):
        #   16:52  False - True was carving camera floor points, algo 'simple'
        #          could not mark floor FREE (obs 29473; 'simple' era only)
        #   21:14  True (metrox) - False left ghost trails of every passer-by;
        #          nav had moved to costmap2d, this map became visual-only
        #   22:30  False - True carves camera table tops in the visual (29658)
        # The CLIP (545fd68, 15:53) flew BEFORE all three: factory True.
        # Under STOCK_NAV=1 this voxel map IS the navigation map again
        # (module docstring: global_map -> CostMapper), and False turned every
        # living being into a permanent wall - 28/08 21:55 the cat got
        # engraved, flight aborted. So: stock flies the clip's True (the map
        # heals itself); our own arm - where nav runs on costmap2d and this
        # really is the Rerun visual only - keeps the detail trade of 22:30.
        VoxelGridMapper.blueprint(voxel_size=0.08, device="CUDA:0", frame_id="world", emit_every=10, carve_columns=stock_nav_enabled()),   # ~1 Hz: the Rerun bridge re-sends the whole map each time (load 25, viewer 2.5 GB at 3 Hz)
        # dimOS's height-cost defaults are for a quadruped (can_climb 0.15 m, pass
        # under 0.6 m): VECTOR climbs nothing (3 cm) and its camera top is ~0.85 m.
        (CostMapper.blueprint() if stock_nav_enabled() else
         VectorCostMap.blueprint()),   # STOCK_NAV=1 -> dimOS's CostMapper exactly as their go2 ships it
                                       # (defaults; "occupancy" was an INVENTED registry key - valid keys are
                                       # height_cost/general/simple, each map KeyError'd the Rx chain and
                                       # global_costmap fell silent, found 2026-08-27). Default path = our 2D
                                       # map: the PROVEN recipe (git 73cc2c6 ran VectorCostMap everywhere;
                                       # upstream's CostMapper had erased table legs on VECTOR)
        # the rover is 54x46 cm but its corners sweep 0.71 m when it pivots and the
        # camera mast stands at the front bumper: give the planner a footprint with margin
        # width 0.46 + 4 cm: in discovery mode the rover drives along its length and turns in place,
        # so its lateral clearance is what matters (no strafing while mapping);
        # 0.62 (a disc for the corners) walled off every 60-70 cm gap - 11 'no path' from a 50 cm clearance
        # 62.5 x 46 cm with the bumper bars: real diagonal 0.776, so 0.78 is honest.
        # Measured 2026-08-25 on the newest costmap checkpoint (20260825-221223/costmap_224656.npz, 19.4 m2
        # observed): 3084 observed free cells clear an inflation radius of 0.39 m, 3278 clear 0.36 m -
        # 0.78 costs 5.9 % of the goal-capable space, not the walling-off it was suspected of. Same
        # verdict on all 18 recorded runs (worst case 13.9 %), so the honest number stays.
        RecoveringPlanner.blueprint(robot_width=0.50, robot_rotation_diameter=0.78),
        MovementManager.blueprint(
            # pass-through: the module's own ceilings are the guard now (the
            # hidden halving made the pad crawl at 0.10 m/s, 2026-08-27)
            tele_cmd_vel_scaling=Twist(Vector3(1.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0))),
        _explorer_blueprint(),
        # contact corners + sonar via the ESP32 USB bridge. Neither writes the map (sensor
        # doctrine, 25/08): front bump = back off, rear bump = move forward, sonar = the forward
        # brake in adapter.py (creep under 0.55 m, stop under 0.30 m).
        EspSensors.blueprint(),
        # Upstream's anti-doubling engine (wired in 2026-08-27): matches the
        # live voxel map against the saved reference every 2 s and publishes the
        # map->world TF + fitness logs. RELOC_MAP unset = module dormant
        # (upstream's own no-map_file behaviour). First milestone: OBSERVE the
        # measured drift; wiring the correction into consumers is the next step,
        # designed against the upstream go2 flow.
        VectorRelocalization.blueprint(map_file=os.environ.get("RELOC_MAP") or None),
        # every run recorded (dimOS memory): replay, draw, tune planners without
        # the robot. RECORD_CLOUDS=0 = teleop-phase light recording (trajectory
        # + costmap only, all run_autopsy needs): the raw clouds were the
        # recorder's write volume and it ate a full core (measured 27/08,
        # one sqlite commit per observation).
        (VectorMemory.blueprint() if os.environ.get("RECORD_CLOUDS", "1") == "1"
         else VectorMemoryLight.blueprint()),
        vis_module(viewer_backend=global_config.viewer, rerun_config=RERUN_CONFIG),
    ).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
    ])


explore_blueprint = _explore_blueprint()

