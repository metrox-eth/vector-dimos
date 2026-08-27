"""dimOS's RelocalizationModule with ONE number adapted for a thin planar map.

Their anti-doubling engine: every RELOC_INTERVAL the LIVE voxel map is matched
against a SAVED reference map (.pc2.lcm); a fitness-gated map->world TF pins
the frame so drift cannot accumulate. Exactly the piece our own guards tried
to be (and froze everything instead - retired 27/08).

The single adaptation: their MIN_LOCAL_POINTS = 50_000 is calibrated for
dense 3D lidar maps (quadrupeds, Mid-360). Our planar map reached 43 289
points after a FULL 16-minute lap (measured 27/08 22h10) - the stock filter
would silently skip every relocalization we will ever attempt. Same engine,
same fitness gate, one threshold changed - the VectorExplorer pattern.
"""
from __future__ import annotations

from typing import Any

from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Enough structure to match a room outline; our lap map = 43k points,
# mid-run ~10-25k. Below this a match against the reference is noise.
MIN_LOCAL_POINTS_2D = 8_000


class VectorRelocalization(RelocalizationModule):
    def _has_enough_points(self, msg: PointCloud2) -> bool:
        return len(msg) >= MIN_LOCAL_POINTS_2D

    # Instrumentation only (27/08 23h35): merged_map never reached the bus while
    # every layer tested clean in isolation. An exception in the merge callback
    # dies silently (that Rx subscribe has no error handler), so make the merge
    # path observable in the run journal.
    def _on_merge_input(self, pair: Any) -> None:
        local, tf = pair
        logger.info(f"merge_input: n_pts={len(local)} tf={'none' if tf is None else 'set'}")
        try:
            super()._on_merge_input(pair)
        except Exception:
            logger.exception("merge FAILED")
            raise
        if tf is not None:
            logger.info("merge published")
