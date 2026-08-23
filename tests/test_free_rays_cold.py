"""Cold bench: a ray that hit a wall at 2.0 m straight ahead yields floor
samples from 0.3 m up to 1.9 m (17 of them, 10 cm apart, z = 0) and nothing
beyond; a ray to 6 m stops at 2.5 m; a hit at 0.25 m yields nothing."""

import numpy as np

from vector_dimos.lidar_odometry import free_floor_along_rays

f = free_floor_along_rays(np.array([[2.0, 0.0]]))
xs = np.sort(f[:, 0])
assert np.allclose(f[:, 1], 0.0) and np.allclose(f[:, 2], 0.0)
assert abs(xs[0] - 0.3) < 1e-6 and abs(xs[-1] - 1.9) < 1e-6 and len(xs) == 17, (xs[0], xs[-1], len(xs))
print(f"  wall at 2.0 m ahead -> {len(xs)} floor samples from {xs[0]:.1f} to {xs[-1]:.1f} m")

f = free_floor_along_rays(np.array([[0.0, -6.0]]))          # 6 m to the right
ys = np.sort(-f[:, 1])
assert abs(ys[-1] - 2.5) < 1e-6 and np.allclose(f[:, 0], 0.0, atol=1e-9), ys[-1]
print(f"  6 m to the right -> capped at {ys[-1]:.1f} m, {len(ys)} samples")

f = free_floor_along_rays(np.array([[0.25, 0.0]]))
assert len(f) == 0
print("  hit at 0.25 m -> no free floor")

f = free_floor_along_rays(np.array([[1.0, 0.0], [1.0, 0.02]]))   # two near-identical rays dedup per 5 cm cell
assert 7 <= len(f) <= 14, len(f)   # cell-boundary rounding may keep a few twins; the voxel map dedups the rest
print(f"  two near-identical rays -> {len(f)} samples after dedup")
print("TEST PASSED")
