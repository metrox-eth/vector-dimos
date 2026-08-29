#!/usr/bin/env python3
"""Load dimOS's WavefrontFrontierExplorer twice - stock, and with PR #2830 - and
drive both from an offline harness, WITHOUT running a dimOS node.

Two source files live in pr2830/, both fetched from GitHub, unmodified:

    selector_base.py   dimensionalOS/dimos @ 6fcc4e2 -- the PR's base commit.
                       Byte-identical (md5 e77c3286...) to the file installed in
                       /home/openclaw/dimos-rig/.venv, i.e. our "stock wavefront".
    selector_head.py   samuelokpor/dimos @ ff9d5ae -- the PR's head commit.

Everything the scorer calls -- min_cost_astar (C++ extension), OccupancyGrid,
Vector3, get_distance -- is the REAL installed dimos package. The only thing
this file adds is a way to hold an instance of the class without an LCM bus:

  * the class is built with object.__new__ instead of __init__, and the plain
    attributes __init__ would have set are set here by hand (they are listed in
    _STATE below and copied from the shipped __init__).
  * `stop_exploration` is replaced on the instance by a flag-setter, because the
    shipped one talks to the module's threads. It is only ever reached through
    the self-stop branch of get_exploration_goal, and the flag reproduces what
    that branch means: "this explorer has declared itself finished".

No other method is touched. detect_frontiers, _rank_frontiers,
_compute_comprehensive_frontier_score, _compute_path_cost and
get_exploration_goal all run as written by their authors.
"""

from __future__ import annotations

import importlib.util
import logging
import os
import sys
import threading
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PR_DIR = os.path.join(HERE, "pr2830")

# dimOS's scorer logs one INFO line per frontier per decision.
logging.getLogger().setLevel(logging.ERROR)
for name in list(logging.root.manager.loggerDict):
    logging.getLogger(name).setLevel(logging.ERROR)


def _load(name: str, path: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


sel_stock = _load("dimos_sel_stock", os.path.join(PR_DIR, "selector_base.py"))
sel_pr = _load("dimos_sel_pr2830", os.path.join(PR_DIR, "selector_head.py"))

from dimos.msgs.geometry_msgs.Pose import Pose  # noqa: E402
from dimos.msgs.geometry_msgs.Vector3 import Vector3  # noqa: E402
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid  # noqa: E402

# The attribute set WavefrontFrontierExplorer.__init__ creates, minus the LCM
# plumbing that Module.__init__ owns. Copied from the shipped __init__ body.
_STATE = "explored_goals exploration_direction last_costmap no_gain_counter"


def make_explorer(module, **config_overrides):
    """An instance of `module`'s WavefrontFrontierExplorer, ready to be asked
    for goals, with no bus attached."""
    cls = module.WavefrontFrontierExplorer
    self = object.__new__(cls)
    self.config = module.WavefrontConfig(**config_overrides)
    self._cache = module.FrontierCache()
    self.explored_goals = []
    self.exploration_direction = Vector3(0.0, 0.0, 0.0)
    self.last_costmap = None
    self.no_gain_counter = 0
    self.latest_costmap = None
    self.latest_odometry = None
    self.goal_reached_event = threading.Event()
    self.exploration_active = True
    self.exploration_thread = None
    self.stop_event = threading.Event()

    self.self_stopped = False

    def stop_exploration(_self=self):
        _self.self_stopped = True
        _self.exploration_active = False
        return True

    self.stop_exploration = stop_exploration
    assert all(hasattr(self, a) for a in _STATE.split())
    return self


def to_occupancy_grid(grid: np.ndarray, res: float, ox: float, oy: float,
                      ts: float = 0.0) -> OccupancyGrid:
    """Our costmap (int8, 100 / 0 / -1, [y, x]) as a dimOS OccupancyGrid.

    Same convention on both sides: CostValues.FREE = 0, OCCUPIED = 100,
    UNKNOWN = -1, grid[y][x], origin at the lower-left cell. No rescaling, no
    reprojection - this is a wrapper, not a conversion.
    """
    return OccupancyGrid(grid=grid.astype(np.int8), resolution=res,
                         origin=Pose(position=[ox, oy, 0.0]), frame_id="world", ts=ts)


def selector_frontiers(explorer, module, robot_xy, costmap):
    """detect_frontiers + the sizes it computed, for plotting/diagnostics.

    detect_frontiers returns only centroids, but ranks them internally; this
    re-runs the same call so the harness can report how many clusters existed.
    """
    pose = Vector3(robot_xy[0], robot_xy[1], 0.0)
    return explorer.detect_frontiers(pose, costmap)


__all__ = ["sel_stock", "sel_pr", "make_explorer", "to_occupancy_grid",
           "Vector3", "OccupancyGrid", "Pose", "deque"]
