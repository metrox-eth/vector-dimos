#!/usr/bin/env python3
"""Walking speed of the Go2 that recorded the dimOS floors, from their own odom.

Reads the `odom` table of a dimOS SqliteStore recording. That table carries
ts / pose_x / pose_y as plain SQL columns, so nothing is decoded and no pose
maths of ours is involved: the numbers below are the recorded trajectory.

Method, declared before looking at any result:

  1. poses sorted by ts, duplicates on ts dropped.
  2. the trace is cut into SEGMENTS wherever the gap between two consecutive
     samples exceeds GAP_S (1.0 s): a recording pause is not a standstill and
     must not be averaged over.
  3. inside a segment, speed is measured over a sliding WINDOW of WIN_S (0.5 s)
     of wall time: straight-line displacement between the two ends divided by
     their elapsed time. A 0.5 s window is long enough that per-sample odom
     jitter (a few mm at ~20 Hz) does not dominate, and short enough that a
     turn-in-place is not smeared into a walk.
  4. a window is MOVING when its speed is at least MOVE_MS. The headline uses
     0.05 m/s; 0.03 and 0.10 are printed beside it so the choice is visible.
  5. median and p90 are taken over the moving windows only.

SECOND DERIVATION, THE LIDAR SPACING. explore_sim couples two constants:
SPEED_MPS (simulated time, hence how far a goal timeout truncates a walk) and
SCAN_EVERY_M, one lidar revolution per 0.25 m of travel - a value calibrated for
a 0.15 m/s rover, i.e. about 1.7 Hz. A Go2 walks several times faster and its
lidar does not spin faster, so PER METRE it sees less. Raising the speed while
leaving SCAN_EVERY_M at 0.25 m would be an optimistic discovery bias, not a
detail, so the spacing is derived from the same recordings the same way:

  6. the timestamps of the `lidar` frames are read from the store's own lidar
     table; the odom trace is interpolated at each of them (linearly, inside a
     segment only - a frame that falls in a recording gap is skipped).
  7. the distance between two consecutive lidar frame positions is the metres
     of travel per revolution. A pair counts as MOVING when its implied speed
     d/dt is at least MOVE_MS, the same standstill threshold as above, so a
     robot standing still does not drag the median to zero.
  8. median and p90 over the moving pairs.

THIRD DERIVATION, THE TURN-IN-PLACE RATE. explore_sim turns in place at
TURN_RATE = 0.5 rad/s, and that consumes goal-timeout budget: a robot that
spends 6 s rotating has 6 s less to walk. A Go2 pivots faster than our rover, so
the number is derived too, and the trick is to isolate a PIVOT from a curve: a
window where the body is turning (yaw rate at or above TURN_THRESHOLD) while the
body is NOT translating (window speed below MOVE_MS) is a turn in place. Median
and p90 over those windows. The yaw-while-walking figure is printed beside it so
the difference is visible.

Straight-line displacement over the window slightly UNDER-states speed on a
curve, so the derived speed is a lower bound on path speed.
"""
from __future__ import annotations
import argparse, json, math, os, sqlite3, sys
import numpy as np

GAP_S = 1.0
WIN_S = 0.5
MOVE_THRESHOLDS = (0.03, 0.05, 0.10)
HEADLINE = 0.05
TURN_THRESHOLD = 0.10           # rad/s, the same idea applied to yaw


def _clean(a):
    if len(a) == 0:
        return a
    a = a[np.argsort(a[:, 0], kind="stable")]
    keep = np.concatenate([[True], np.diff(a[:, 0]) > 0])
    return a[keep]


def _yaw(qx, qy, qz, qw):
    return np.arctan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def load_odom_sql(path):
    """ts / pose_x / pose_y / yaw straight out of the SQL columns of odom."""
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    rows = con.execute(
        "select ts, pose_x, pose_y, pose_qx, pose_qy, pose_qz, pose_qw from odom "
        "where pose_x is not null and pose_y is not null order by ts").fetchall()
    con.close()
    a = np.asarray(rows, dtype=np.float64).reshape(-1, 7)
    if len(a) == 0:
        return a[:, :4]
    yaw = _yaw(a[:, 3], a[:, 4], a[:, 5], a[:, 6])
    return _clean(np.column_stack([a[:, 0], a[:, 1], a[:, 2], yaw]))


def load_odom_store(path):
    """Same trace, decoded from the odom blobs by dimOS' own store reader.

    Some recordings leave the pose_x/pose_y INDEX columns zeroed or null while
    the real PoseStamped payload sits in odom_blob. Those are read here with
    dimOS' own SqliteStore, so no decoding of ours is involved either.
    """
    try:
        from dimos.memory.store.sqlite import SqliteStore
    except ModuleNotFoundError:
        from dimos.memory2.store.sqlite import SqliteStore
    st = SqliteStore(path=path)
    rows = []
    frames = set()
    for obs in st.streams.odom:
        d = obs.data
        frames.add(getattr(d, "frame_id", None))
        rows.append((float(obs.ts), float(d.x), float(d.y), float(d.yaw)))
    if frames - {"world"}:
        raise SystemExit(f"REFUSED {path}: odom frame_id {frames}, not world")
    return _clean(np.asarray(rows, dtype=np.float64).reshape(-1, 4))


def load_odom(path):
    """SQL columns when they carry a real trace, the blobs otherwise."""
    a = load_odom_sql(path)
    if len(a) >= 2 and np.hypot(np.diff(a[:, 1]), np.diff(a[:, 2])).sum() > 0.5:
        return a, "sql"
    return load_odom_store(path), "blob"


def window_yaw_rates(a, win_s=WIN_S):
    """|unwrapped yaw change| / elapsed over the same sliding window."""
    ts, yaw = a[:, 0], np.unwrap(a[:, 3])
    out = []
    for s, e in segments(ts):
        t = ts[s:e]
        if len(t) < 2:
            continue
        j = np.searchsorted(t, t + win_s, side="left")
        i = np.arange(len(t))
        ok = j < len(t)
        i, j = i[ok], j[ok]
        dt = t[j] - t[i]
        good = dt > 1e-6
        i, j, dt = i[good], j[good], dt[good]
        out.append(np.abs(yaw[s:e][j] - yaw[s:e][i]) / dt)
    return np.concatenate(out) if out else np.zeros(0)


def segments(ts, gap_s=GAP_S):
    breaks = np.nonzero(np.diff(ts) > gap_s)[0]
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks + 1, [len(ts)]])
    return list(zip(starts.tolist(), ends.tolist()))


def window_speeds(a, win_s=WIN_S):
    ts, x, y = a[:, 0], a[:, 1], a[:, 2]
    out = []
    for s, e in segments(ts):
        t = ts[s:e]
        if len(t) < 2:
            continue
        j = np.searchsorted(t, t + win_s, side="left")
        i = np.arange(len(t))
        ok = j < len(t)
        i, j = i[ok], j[ok]
        dt = t[j] - t[i]
        good = dt > 1e-6
        i, j, dt = i[good], j[good], dt[good]
        d = np.hypot(x[s:e][j] - x[s:e][i], y[s:e][j] - y[s:e][i])
        out.append(d / dt)
    return np.concatenate(out) if out else np.zeros(0)


def lidar_spacing(path, a, move_ms=HEADLINE):
    """Metres of travel between consecutive lidar frames, while moving.

    The lidar frame timestamps come from the store's own `lidar` table; the
    position at each of them is the odom trace interpolated at that instant,
    inside one continuous odom segment only.
    """
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    lts = np.asarray([r[0] for r in con.execute(
        "select ts from lidar order by ts")], dtype=np.float64)
    con.close()
    if len(lts) < 2 or len(a) < 2:
        return None
    ts, x, y = a[:, 0], a[:, 1], a[:, 2]
    segs = segments(ts)
    out, gaps = [], 0
    for s, e in segs:
        t = ts[s:e]
        if e - s < 2:
            continue
        sel = lts[(lts >= t[0]) & (lts <= t[-1])]
        if len(sel) < 2:
            continue
        px = np.interp(sel, t, x[s:e])
        py = np.interp(sel, t, y[s:e])
        d = np.hypot(np.diff(px), np.diff(py))
        dt = np.diff(sel)
        good = dt > 1e-6
        d, dt = d[good], dt[good]
        moving = (d / dt) >= move_ms
        gaps += int((~moving).sum())
        out.append(d[moving])
    if not out:
        return None
    d = np.concatenate(out)
    if not len(d):
        return None
    return {"lidar_frames": int(len(lts)),
            "moving_pairs": int(len(d)), "standstill_pairs": gaps,
            "m_per_scan_median": round(float(np.median(d)), 4),
            "m_per_scan_p90": round(float(np.percentile(d, 90)), 4),
            "m_per_scan_mean": round(float(d.mean()), 4),
            "scan_hz_median": round(float(1.0 / np.median(np.diff(lts))), 3)
            if len(lts) > 2 else None}


def _spacing_raw(path, a, move_ms=HEADLINE):
    """The moving inter-scan distances themselves, for pooling."""
    con = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    lts = np.asarray([r[0] for r in con.execute("select ts from lidar order by ts")],
                     dtype=np.float64)
    con.close()
    if len(lts) < 2 or len(a) < 2:
        return None
    ts, x, y = a[:, 0], a[:, 1], a[:, 2]
    out = []
    for s, e in segments(ts):
        t = ts[s:e]
        if e - s < 2:
            continue
        sel = lts[(lts >= t[0]) & (lts <= t[-1])]
        if len(sel) < 2:
            continue
        px, py = np.interp(sel, t, x[s:e]), np.interp(sel, t, y[s:e])
        dd, dt = np.hypot(np.diff(px), np.diff(py)), np.diff(sel)
        good = dt > 1e-6
        dd, dt = dd[good], dt[good]
        out.append(dd[(dd / dt) >= move_ms])
    if not out:
        return None
    r = np.concatenate(out)
    return r if len(r) else None


def path_length(a):
    tot = 0.0
    for s, e in segments(a[:, 0]):
        if e - s < 2:
            continue
        tot += float(np.hypot(np.diff(a[s:e, 1]), np.diff(a[s:e, 2])).sum())
    return tot


def report(name, path):
    a, src = load_odom(path)
    if len(a) == 0:
        return {"map": name, "error": "no odom rows"}
    ts = a[:, 0]
    segs = segments(ts)
    span = float(sum(ts[e - 1] - ts[s] for s, e in segs))
    sp = window_speeds(a)
    wr = window_yaw_rates(a)
    d = {"map": name, "file": os.path.basename(path), "source": src,
         "poses": int(len(a)),
         "segments": len(segs), "recorded_s": round(span, 1),
         "sample_hz": round(len(a) / span, 2) if span > 0 else None,
         "path_m": round(path_length(a), 2),
         "windows": int(len(sp))}
    sp_info = lidar_spacing(path, a)
    if sp_info:
        d.update(sp_info)
    turning = wr[wr >= TURN_THRESHOLD]
    n = min(len(sp), len(wr))
    pivot = wr[:n][(sp[:n] < HEADLINE) & (wr[:n] >= TURN_THRESHOLD)]
    d["pivot_windows"] = int(len(pivot))
    d["pivot_median"] = round(float(np.median(pivot)), 4) if len(pivot) else None
    d["pivot_p90"] = round(float(np.percentile(pivot, 90)), 4) if len(pivot) else None
    d["pivot_p99"] = round(float(np.percentile(pivot, 99)), 4) if len(pivot) else None
    d["pivot_max"] = round(float(pivot.max()), 4) if len(pivot) else None
    d["turn_thr_rad_s"] = TURN_THRESHOLD
    d["turning_frac"] = round(float(len(turning)) / max(1, len(wr)), 3)
    d["yaw_median"] = round(float(np.median(turning)), 4) if len(turning) else None
    d["yaw_p90"] = round(float(np.percentile(turning, 90)), 4) if len(turning) else None
    d["yaw_p99"] = round(float(np.percentile(turning, 99)), 4) if len(turning) else None
    for thr in MOVE_THRESHOLDS:
        m = sp[sp >= thr]
        d[f"moving_frac@{thr}"] = round(float(len(m)) / max(1, len(sp)), 3)
        d[f"median@{thr}"] = round(float(np.median(m)), 4) if len(m) else None
        d[f"p90@{thr}"] = round(float(np.percentile(m, 90)), 4) if len(m) else None
        d[f"p99@{thr}"] = round(float(np.percentile(m, 99)), 4) if len(m) else None
        d[f"max@{thr}"] = round(float(m.max()), 4) if len(m) else None
    return d


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", action="append", required=True,
                    help="name=path, repeatable")
    ap.add_argument("--out", default=None)
    ap.add_argument("--dump-windows", default=None,
                    help="npz of every moving-window speed, per map, for pooling")
    args = ap.parse_args()
    res = []
    dump = {}
    for spec in args.db:
        name, _, path = spec.partition("=")
        if not os.path.exists(path):
            print(f"MISSING {name} {path}", file=sys.stderr)
            continue
        r = report(name, path)
        res.append(r)
        print(json.dumps(r), flush=True)
        if args.dump_windows:
            a, _src = load_odom(path)
            if len(a):
                sp = window_speeds(a)
                dump[name] = sp[sp >= HEADLINE]
                _sp = _spacing_raw(path, a)
                if _sp is not None:
                    dump[name + "__scan"] = _sp
                _w = window_yaw_rates(a)
                dump[name + "__yaw"] = _w[_w >= TURN_THRESHOLD]
                _n = min(len(sp), len(_w))
                dump[name + "__pivot"] = _w[:_n][(sp[:_n] < HEADLINE)
                                                 & (_w[:_n] >= TURN_THRESHOLD)]
    if args.dump_windows and dump:
        np.savez_compressed(args.dump_windows, **dump)
    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f, indent=1)
