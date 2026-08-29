# What changed in the harness, line by line

Base: `../sim_2830_resid/bench_2830.py` (the go2 bench's file plus the residual
job's two report-only trace fields), copied unchanged into this workspace.
`dimos_selector.py`, `midstarts.py`, `fix_*.py` and `pr2830/` are byte-identical
copies; the two vendored upstream selector files were never opened for writing
and still md5 to `e77c328643c959a49077115e8a341f2c` and
`1ccc0c69fe88a72e402565feca988d26`.

Nothing upstream is touched by any of this. Every change is in the harness'
own loop, its own robot profile, or its own reporting.

## 1. `go2_profile.py`: one new profile, `go2ctrl`

Identical to `go2` in every dimension, margin and rate. One field differs:

```
SPEED_MPS   0.60  ->  0.55
```

0.55 is `dimos/navigation/replanning_a_star/local_planner.py:67`
(`_speed: float = 0.55`), the speed the autonomous stack drives the Go2 at when
the wavefront selector is the thing publishing goals. 0.60 was derived from
teleoperated recordings and measures a human's thumb. `go2` is left in the file
untouched so the reproduction check below can run against it.

## 2. `bench_2830.py`: the faithful loop

### 2.1 New module constant and CLI flag

`T_SEL_S = 0.0`, `--t-sel-s`. Zero is the old behaviour in full and is the
default, so any invocation of this file that does not ask for the fix gets
exactly the old bench.

### 2.2 New helper, `_drive_during(sim, dt, goal_xy)`

The whole mechanism, in one function. `dt` simulated seconds pass; if
`goal_xy` is not None the robot spends them still driving to it, by re-entering
`explore_sim.Sim.drive` on the same goal with `ES.GOAL_TIMEOUT_S` temporarily
set to `dt`. That reuses their drive loop exactly as the main pursuit does:
same replanning per lidar revolution, same bump reflex, same sub-stepping, same
scan spacing. If the robot arrives, is refused, or exhausts its path inside
`dt`, it stands still for the remainder (`_elapse`).

Returns `(outcome, metres walked, seconds consumed)`; all three are reported and
none of them is read by any policy.

### 2.3 The decision loop, re-ordered

```
    pose, costmap  <- read at the top            (selector lines 794, 799)
    dec_x, dec_y   <- remember that pose
    goal = decide(pose, costmap)                 (selector line 800)
    _drive_during(sim, T_sel, live_goal)         <-- NEW: the compute costs time
    publish; sim.drive(goal)                     (selector lines 812, 823)
```

The new call sits between the decision and the publish, which is where the
compute sits in `_run_exploration_loop`. Two consequences fall out of it and
both are the point:

* the robot walks toward a goal the selector has already abandoned, and
* the goal that is published was chosen for the pose the robot had `T_sel`
  seconds earlier.

### 2.4 `live_goal`: which goal the navigator still holds

Set and cleared exactly where dimOS sets and clears `GlobalPlanner._current_goal`:

| harness | dimOS | live? |
|---|---|---|
| `sim.drive` returned `"reached"` | `_handle_stop_message("arrived")` -> `cancel_goal(arrived=True)` -> `stop_planning()` (GP:273-275, 149-170) | no |
| `sim.drive` returned `"timeout"` | selector logs a warning and loops; nothing cancelled (SEL:823-828) | **yes** |
| `sim.drive` returned `"blocked"` | `_plan_path` found no safe goal or no path -> `cancel_goal()` (GP:327-341) | no |
| `sim.drive` returned `"timeout"` **before the budget** (path exhausted, explore_sim:801-805) | the local planner reached the end of its path -> `final_rotation` -> `"arrived"` (LP:277-310) -> stopped | no |

The last row was WRONG in the first version of this harness and is the one
change made after runs had started. `_still_pursuing(outcome, elapsed, budget)`
separates the two `"timeout"` returns by the clock: a clock timeout can only
happen at or past the budget, path exhaustion only before it. In config
`scoring`, where the budget is 1e9 s, this makes every `"timeout"` path
exhaustion, which is what config `scoring` means. `rec.outcome` is deliberately
left at `"timeout"` so a T_sel = 0 run stays byte-identical to the recorded
benches; the distinction is carried by the new `path_exhausted` per-goal flag and
`goals_path_exhausted` per-run count.

**How it was caught, and by what.** The pre-declared `scoring` identity check of
`hypotheses_fidelity.txt` ("in `scoring` no goal ends on a timeout, so T_sel is
pure standstill, so T_sel 0 and 15 must be identical in space"). It failed on 1
of 6 pairs in the first partial results. The 19 runs produced before the fix are
parked at `/root/sim_2830_fidelity/out_v1_conflated_timeout/` on the box and are
used for nothing.

### 2.5 The 2 s retry wait is a driving window too

When the selector finds no frontier it waits 2 s and loops (SEL:841-848).
No goal is published in that iteration, so the navigator still holds the old
one and the robot keeps walking through the wait. `_drive_during` is used there
as well, gated on the same switch.

### 2.6 The switch is one switch

`faithful = t_sel_s > 0.0`. At `T_sel = 0` the selection window and the retry
window are both standstill, i.e. the old bench bit for bit - not a half-fixed
hybrid. Verified, see section 4.

### 2.7 New recorded fields (reported, never fed back)

Per goal: `dec_x`, `dec_y` (the pose the selector saw), `d_robot_dec`,
`sel_outcome`, `sel_moved_m`, `sel_s`. `from_x`/`from_y` keep their meaning -
where the robot was when the goal took effect - which at `T_sel = 0` is the same
pose and is what every earlier bench recorded.

Per run: `t_sel_s`, `sel_windows`, `sel_driving_windows`, `sel_moved_m`,
`sel_time_s`, `sel_arrived`, `sel_blocked`, `sel_drift_median_m`,
`d_robot_dec_median_m`.

Per trace record (only with `--trace`): `robot` stays the pose the selector saw,
`robot_pub` is added for the publish pose, `sel_moved_m` for the window.

### 2.8 Which pose the class-N filter uses

`analyse_cand2.swings_of` is imported unchanged and measures displacement from
`from_x`/`from_y`, i.e. the publish pose. That is the physically right pose:
class N asks whether the robot had to travel to serve the goal, and it starts
serving it at the publish. At `T_sel = 0` the two poses coincide, so no earlier
number moves because of this choice.

## 3. `analyse_fidelity.py`: new, and it publishes class N

Imports `swings_of` and `round_trips` from `../sim_2830_cand2/analyse_cand2.py`
unchanged, so a crossing here is the same object as a crossing in the go2 and
residual reports. What it adds: an **N-churn column in every table** (class N
is goal churn without displacement and is never subtracted in silence), the
faithful-temporality columns, the per-`T_sel` G1 verdict, the pre-declared
`scoring` identity check, and the Stage B budget table.

## 4. Reproduction check, run before any Stage A run

`bench_2830.py --profile go2 --lidar-range 12 --maps hk_park --configs shipped
--arms stock --t-sel-s 0`, 6 middle starts, against the recorded
`../sim_2830_go2/go2_12m_hk_park.json`:

**6 runs, 13 summary fields plus every goal coordinate, 0 disagreements.**
Also checked on those runs: `from_x == dec_x` and `from_y == dec_y` on every
goal, and `sel_moved_m == sel_s == 0` everywhere. The plumbing is inert at
`T_sel = 0`.

Raw output: `repro_go2_12m_hk_park.json`.
