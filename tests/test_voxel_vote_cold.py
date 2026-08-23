"""Cold bench for VoxelVote: a stable return enters the map on its 3rd
revolution, a one-off noisy return never does, a 1-in-9 flicker never does."""

import numpy as np

from vector_dimos.lidar_odometry import VoxelVote

v = VoxelVote(0.05)
stable = np.array([[1.00, 0.00, 0.37]])
noise = np.array([[2.00, 0.00, 0.37]])
seen = []
for frame in range(12):
    pts = stable if frame != 4 else np.vstack([stable, noise])     # noise once, on frame 4
    if frame % 9 == 0:
        pts = np.vstack([pts, [[3.0, 0.0, 0.37]]])                  # a 1-in-9 flicker
    out = v.vote(pts)
    seen.append({(float(x),) for x in np.round(out[:, 0], 2)})
first = next(i for i, s in enumerate(seen) if (1.0,) in s)
assert first == 2, f"stable point should pass on the 3rd frame, passed on frame {first + 1}"
assert all((1.0,) in s for s in seen[2:]), "stable point must stay in"
assert not any((2.0,) in s for s in seen), "one-off noise must never pass"
assert not any((3.0,) in s for s in seen), "1-in-9 flicker must never pass"
print(f"  stable: in from frame 3; one-off noise: never; 1-in-9 flicker: never; tracked voxels now {len(v._score)}")
print("TEST PASSED")
