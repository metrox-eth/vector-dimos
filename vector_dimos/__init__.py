"""vector-dimos: the VECTOR mecanum platform as a dimOS external package."""
from .adapter import VectorBaseAdapter
from .kinematics import MecanumGeometry, forward, inverse

__all__ = ["VectorBaseAdapter", "MecanumGeometry", "forward", "inverse"]
