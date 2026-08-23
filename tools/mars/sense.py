"""The rover's guard, v2 (after pushing the laser crate on 2026-08-23).

Reads dimOS's own streams: colour (photo), depth (camera bands) and the lidar
cloud (front sectors). Prints ONE line the explore loop parses:

  guard cam L=.. C=.. R=.. valid=..%  lidar F=.. L=.. R=..  ahead=..  sides=..

Camera geometry MEASURED: 0.80 m high, level, fy~386 px. Rows 260..480 cover
heights 0.70 m (at 3 m) down to the lowest the camera can see; below ~0.5 m
of range it sees nothing under 0.5 m high - that blind zone is the lidar's
and, later, the bumpers'. Too few valid depth pixels = BLIND = blocked.
Distances per band = minimum of per-column medians (thin legs cannot hide).
"""
import sys, time, threading
import numpy as np, cv2
from dimos.core.transport_factory import make_transport
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

tag = sys.argv[1] if len(sys.argv) > 1 else time.strftime("%H%M%S")
wait_s = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
MIN_VALID = 0.5; BLIND = 0.0
last = {}; ev = threading.Event()
def keep(name):
    def cb(msg):
        last[name] = msg
        if all(k in last for k in ("color_image", "depth_image", "pointcloud")): ev.set()
    return cb
make_transport("color_image", Image).subscribe(keep("color_image"))
make_transport("depth_image", Image).subscribe(keep("depth_image"))
make_transport("pointcloud", PointCloud2).subscribe(keep("pointcloud"))
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
make_transport("camera_info", CameraInfo).subscribe(keep("camera_info"))
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
make_transport("global_map", PointCloud2).subscribe(keep("global_map"))
make_transport("odom", PoseStamped).subscribe(keep("odom"))
ev.wait(wait_s)
if "color_image" in last:
    cv2.imwrite(f"/home/metrox/mars/{tag}.png", last["color_image"].to_opencv())
cam = {"L": BLIND, "C": BLIND, "R": BLIND}; valid_pct = 0.0
# --- camera: full 3D check. Every 4th depth pixel -> point in the ROVER frame
# (camera 0.80 m up, 0.21 m ahead of the lidar, level; optical x right, y down,
# z forward). Floor removed by geometry (z < FLOOR_Z), everything else up to
# the mast top counts, whatever its height (metrox 14h12: "toutes les hauteurs").
CAM_H, CAM_X = 0.80, 0.30; FLOOR_Z, TOP_Z = 0.04, 0.90; HALF_W = 0.33   # rover 54x46 cm, camera at the front bumper, highest point
if "depth_image" in last:
    K = list(last["camera_info"].K) if "camera_info" in last else [386.6, 0, 320.6, 0, 386.0, 245.0, 0, 0, 1]   # measured 2026-08-23
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    d = np.asarray(last["depth_image"].data); d = d[..., 0] if d.ndim == 3 else d
    h, w = d.shape; vs, us = np.mgrid[0:h:4, 0:w:4]
    z = d[vs, us].astype(np.float32) / 1000.0; ok = (z > 0.15) & (z < 6.0)
    valid_pct = 100 * ok[:, us.shape[1]//3:2*us.shape[1]//3].mean()
    z, us_, vs_ = z[ok], us[ok], vs[ok]
    X = z + CAM_X; Y = -(us_ - cx) * z / fx; Z = CAM_H - (vs_ - cy) * z / fy
    floor_z = 0.02 + 0.03 * np.clip(X - 1.0, 0.0, None)   # 2 cm near, growing with range (depth noise)
    obst = (Z > floor_z) & (Z < TOP_Z) & (X > 0.2)
    X, Y = X[obst], Y[obst]
    def nearest(m): return float(np.sort(X[m])[2]) if m.sum() >= 3 else 9.0   # 3rd-nearest: thin objects count, lone noise pixels do not
    if valid_pct >= 100 * MIN_VALID:
        cam["C"] = nearest(np.abs(Y) <= HALF_W)
        cam["L"] = nearest((Y > HALF_W) & (Y <= 1.0))
        cam["R"] = nearest((Y < -HALF_W) & (Y >= -1.0))
lid = {"F": BLIND, "L": BLIND, "R": BLIND}
if "pointcloud" in last:
    out = last["pointcloud"].as_numpy(); p = np.asarray(out[0] if isinstance(out, tuple) else out)
    if len(p):
        ang = np.degrees(np.arctan2(p[:, 1], p[:, 0])); r = np.hypot(p[:, 0], p[:, 1])
        for name, (a, b) in (("F", (-30, 30)), ("L", (30, 90)), ("R", (-90, -30))):
            m = (ang >= a) & (ang <= b) & (r > 0.2)
            lid[name] = float(r[m].min()) if m.any() else 9.0
# --- the map's memory: global voxel map (world) -> rover frame via the lidar-odometry pose.
# What the camera saw from afar (a crate, a chair) stays known when the camera goes blind up close.
mem = {"F": 9.0, "L": 9.0, "R": 9.0}
if "global_map" in last and "odom" in last:
    out = last["global_map"].as_numpy(); g = np.asarray(out[0] if isinstance(out, tuple) else out)
    o = last["odom"]; pos = o.position if hasattr(o, "position") else o.pose.position; q = o.orientation if hasattr(o, "orientation") else o.pose.orientation
    yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    dx, dy = g[:, 0] - pos.x, g[:, 1] - pos.y; c, s_ = np.cos(-yaw), np.sin(-yaw)
    bx, by = c * dx - s_ * dy, s_ * dx + c * dy          # world -> rover frame (lidar at the origin)
    near = (bx > 0.45) & (bx < 2.0) & (g[:, 2] > 0.04) & (g[:, 2] < 0.90)   # > 0.45 m: outside the rover itself
    for name, m in (("F", near & (np.abs(by) <= HALF_W)), ("L", near & (by > HALF_W) & (by <= 1.0)), ("R", near & (by < -HALF_W) & (by >= -1.0))):
        if m.sum() >= 3: mem[name] = float(bx[m].min())
ahead = min(cam["C"], lid["F"], mem["F"]); sides = min(cam["L"], cam["R"], lid["L"], lid["R"], mem["L"], mem["R"])
print(f"guard cam L={cam['L']:.2f} C={cam['C']:.2f} R={cam['R']:.2f} valid={valid_pct:.0f}%  "
      f"lidar F={lid['F']:.2f} L={lid['L']:.2f} R={lid['R']:.2f}  map F={mem['F']:.2f} L={mem['L']:.2f} R={mem['R']:.2f}  ahead={ahead:.2f}  sides={sides:.2f}")
