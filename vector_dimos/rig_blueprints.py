"""Blueprints meant to run on the RIG, not the rover - kept import-light.

nav_blueprints pulls the whole cockpit chain (socketio, starlette...) at
import time; the rig's lean dimos install has none of that and the reloc
offload needs none of it (found 27/08 22h26, two ModuleNotFoundError deep).
This module imports ONLY what the offloaded engine touches.

Launch on the rig:
    ROBOT_IP=<rover> TRANSPORT=zenoh RELOC_MAP=<.pc2.lcm> \
        dimos run vector-dimos.reloc-rig
The rover holds the zenoh rendezvous (tools/zenoh_rendezvous.py, port 7447);
gossip completes the mesh. Doctrine: this is a spectator - if the link dies,
the rover keeps driving and mapping, only the drift measurements stop.
"""
from __future__ import annotations

import os

from dimos.core.coordination.blueprints import autoconnect

from vector_dimos.relocalization import VectorRelocalization


def _reloc_rig_blueprint():
    return autoconnect(
        VectorRelocalization.blueprint(map_file=os.environ.get("RELOC_MAP") or None),
    )


reloc_rig_blueprint = _reloc_rig_blueprint()
