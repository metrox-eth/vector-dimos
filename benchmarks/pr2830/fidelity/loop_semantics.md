# What the dimOS exploration loop actually does in time

Read off the installed package, not from memory and not from the PR. Everything
below has a file and a line number, and every line number was opened and read in
this job.

```
package   dimos 0.0.14b1
path      /home/openclaw/dimos-rig/.venv/lib/python3.12/site-packages/dimos/
resolved  import dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector
          -> .../site-packages/dimos/navigation/frontier_exploration/wavefront_frontier_goal_selector.py
          (checked at runtime: the editable vector_dimos-0.2.0 finder that is also
           installed does NOT shadow it)
md5       wavefront_frontier_goal_selector.py = e77c328643c959a49077115e8a341f2c
          = pr2830/selector_base.py, the PR's base, in every bench of this workspace
```

Short names below: **SEL** = `wavefront_frontier_goal_selector.py`,
**GP** = `navigation/replanning_a_star/global_planner.py`,
**LP** = `navigation/replanning_a_star/local_planner.py`,
**MOD** = `navigation/replanning_a_star/module.py`,
**CTL** = `navigation/replanning_a_star/controllers.py`.

---

## 1. The two threads, and who owns the robot

The selector and the navigator are separate modules connected by one message.

| | publishes | subscribes |
|---|---|---|
| `WavefrontFrontierExplorer` | `goal_request: Out[PoseStamped]` (SEL:122) | `goal_reached: In[Bool]` (SEL:116) |
| `ReplanningAStarPlanner` | `goal_reached: Out[Bool]` (MOD:53) | `goal_request: In[PoseStamped]` (MOD:48) |

`goal_request` lands on `GlobalPlanner.handle_goal_request` (MOD:92 wires it;
GP:134). `goal_reached` is fed from `GlobalPlanner.goal_reached` (MOD:115) and
lands on `WavefrontFrontierExplorer._on_goal_reached` (SEL:165, body SEL:193).

The exploration loop runs on its own thread (SEL:721). The navigator runs two
more: a monitor thread (GP:107) and, while a path is being followed, the local
planner's control thread at 10 Hz (LP:119, LP:68). **Nothing the selector does
between two publishes touches those threads.** The only selector-side call that
stops the robot is `stop_exploration`, which publishes the robot's own current
pose as a goal (SEL:750-758), and that is the user pressing stop, not the loop.

## 2. The loop, statement by statement

`_run_exploration_loop` (SEL:780-848):

| line | statement | what it costs in time |
|---|---|---|
| SEL:787 | `while self.exploration_active ...` | - |
| SEL:794-796 | `robot_pose = Vector3(self.latest_odometry...)` | **the pose is read HERE**, at the top |
| SEL:799 | `costmap = simple_inflate(self.latest_costmap, 0.25)` | the costmap is read HERE too |
| SEL:800 | `goal = self.get_exploration_goal(robot_pose, costmap)` | **T_sel**: their pure-python frontier BFS. Seconds. See section 5 |
| SEL:804-812 | build `PoseStamped`, `self.goal_request.publish(goal_msg)` | the goal takes effect HERE |
| SEL:819 | `self.goal_reached_event.clear()` | after the publish, so an arrival that landed during the compute is discarded |
| SEL:823 | `goal_reached = self.goal_reached_event.wait(timeout=self.config.goal_timeout)` | up to `goal_timeout` |
| SEL:825-828 | `if goal_reached: ... else: logger.warning("Goal timeout ...")` | **nothing else**. No cancel, no stop, no re-publish |
| back to SEL:787 | | |

`goal_timeout` is `WavefrontConfig.goal_timeout: float = 15.0` (SEL:93). The Go2
"smart" blueprint instantiates `WavefrontFrontierExplorer.blueprint()` with no
override (`robot/unitree/go2/blueprints/smart/unitree_go2.py:46`), so 15.0 s is
what runs on their robot. (The warning string at SEL:828 says "30 seconds"; it
is stale text, the value is the config field. Worth reporting as a one-line
upstream nit.)

**The finding, stated exactly.** Between SEL:823 returning False and SEL:812
publishing the next goal there are three statements that read state and one that
computes for seconds, and not one of them cancels anything. `_current_goal`
inside `GlobalPlanner` is still the old goal (GP:51, set at GP:137) and the local
planner's control thread is still running its 10 Hz loop against the old path
(LP:196-230). **The robot keeps walking to the old goal for the whole of the
next selection.** It stops walking to it only at GP:134-140, when the new goal
arrives and `handle_goal_request` calls `_plan_path`, whose first statement is
`cancel_goal(but_will_try_again=True)` (GP:313).

## 3. The three ways a goal pursuit can end, and what the robot is doing after each

| pursuit ends because | selector sees | navigator state | is the robot moving during the NEXT selection? |
|---|---|---|---|
| **arrived** | `goal_reached_event` set (SEL:196) via GP:170 `goal_reached.on_next(Bool(True))` after `_handle_stop_message("arrived")` (GP:273-275) | `cancel_goal(arrived=True)` -> `_local_planner.stop_planning()` (GP:167), `cmd_vel` zeroed (LP:123) | **no**, standing still |
| **15 s elapsed** | `wait()` returns False (SEL:823), warning logged (SEL:828) | untouched. `_current_goal` still set, control thread still following the path | **YES, still driving to the OLD goal** |
| **planner refused it** (no safe goal GP:327-332, or no path GP:336-341, or the replan limiter gave up GP:304-306) | `goal_reached.on_next(Bool(False))` at GP:170 - and `_on_goal_reached` **ignores a False** (SEL:195: `if msg.data`) | `cancel_goal()`, local planner stopped | **no**, standing still - and the selector is still blocked in its 15 s wait, unaware |

Row 2 is the audit's finding and it is the one this bench had wrong. Row 3 is a
second, smaller infidelity in our harness, in the opposite direction; it is
listed in section 6 and is not modelled.

## 4. What the robot walks at

`LocalPlanner._speed: float = 0.55` (LP:67). `GlobalPlanner` builds the local
planner with it (GP:84-86), and `LocalPlanner.__init__` passes it to the
controller unchanged unless `global_config.nerf_speed < 1.0` (LP:89-97).
`nerf_speed` defaults to 1.0 (`core/global_config.py:107`) and the Go2 smart
blueprint does not set it. **0.55 m/s is the autonomous speed of the robot this
bench is about.**

That is not the 0.60 m/s the Go2 bench used. 0.60 was derived from the odom of
dimOS' own recordings, which are a human teleoperating; the number is right for
what it measured and it is the wrong number for this loop, which never
teleoperates. This job runs 0.55, and 0.55 is also the conservative choice for
the premise the bench rests on: `15 s x 0.55 = 8.25 m` of walk before the loop
pulls the robot off a goal, against 9.00 m at 0.60.

**Where the real controller is slower than our motion model, and we do not model
it** (fidelity note, direction stated, not corrected):

- `linear_velocity = self._speed * (1.0 - abs(yaw_error) / 90 deg)` (CTL:74), with
  a floor of `_min_linear_velocity = 0.2` m/s (CTL:43). The real Go2 therefore
  slows down through every curve of the path; our sim walks every straight
  segment at a flat 0.55 and charges a separate in-place pivot per segment
  (`explore_sim.Sim.turn_to`, `TURN_RATE`).
- If the heading error exceeds 90 deg the controller rotates in place and does not
  translate at all (CTL:70-71).
- Angular velocity is `0.5 * yaw_error` clipped to `+/- _speed` (CTL:87-88), i.e.
  at most 0.55 rad/s, with a 0.2 rad/s floor; in `simulation` mode it is floored
  at 0.8 rad/s instead (CTL:108-109). Our `TURN_RATE = 0.63` rad/s (derived from
  the recordings' pivot windows) sits between those.
- The path itself is resampled and smoothed (GP:343) and planned on a gradient
  costmap at 1.1x robot width (GP:349-363); our sim plans on the ported voronoi
  planner and simplifies to straight segments.

Net direction: **our robot covers a metre of path in less time than theirs
would**, so if anything this bench gives the loop MORE walk inside its 15 s than
the real one gets. That is conservative for the fidelity question being asked
(does the swing behaviour survive when the robot is not frozen), because more
walk per goal means fewer goals and fewer chances to ping-pong.

## 5. T_sel: what the selection costs, measured

`get_exploration_goal` runs `detect_frontiers`, which is a full-grid pure-python
wavefront BFS over the whole costmap (`SEL:274-...`, `_is_frontier_point` at
SEL:248 does 8 neighbour lookups per cell in python), then ranks every cluster
with `_compute_comprehensive_frontier_score`. There is no C, no numpy vectorised
path, and no incremental update: every decision re-scans the entire grid.

Isolated measurement, this rig, one worker, no contention: see
`t_sel_measurement.txt`. It is the anchor for the swept values.

`T_sel` is swept over **{0, 5, 15} s**:

- **0 s** is not a claim about dimOS. It is the OLD arm - the robot frozen while
  the selection runs - kept so the fidelity change can be read as a difference
  rather than swapped in silently. Every earlier bench in this workspace is this
  arm.
- **5 s** and **15 s** bracket the measurement. 15 s also has a second meaning
  worth naming: at T_sel = 15 s the compute costs as much as the timeout itself,
  so the robot walks 15 s toward the old goal and then 15 s toward the new one,
  and half of all walking is spent on goals that have already been superseded.

## 6. Residual infidelities of this harness, after the fix

Named, with the direction each one pushes, none of them modelled:

1. **A refused goal costs 1 s here and up to 15 s there.** Section 3 row 3: the
   selector's `wait()` ignores `Bool(False)` (SEL:195), so the real robot stands
   still for the remainder of the 15 s after the planner refuses a goal, while
   this harness charges `FAIL_BREATH_S = 1.0` s and moves on. Affects simulated
   time only (the robot is stationary either way, so no motion and no lidar
   revolution is at stake), and simulated time binds nothing here: no run in
   this workspace has ever ended on the 6000 s time budget.
2. **Impact budget resets across a selection window.** `MAX_IMPACTS_PER_GOAL = 3`
   is counted inside one `Sim.drive` call; continuing the same goal through a
   selection window re-enters `drive` and hands it a fresh budget of 3. Bounds:
   at most one extra window per goal, and impacts are rare on these floors.
3. **The selection window overshoots by up to one cell-step plus one pivot**,
   because `explore_sim` checks its clock between sub-steps. Same overshoot the
   main 15 s pursuit already has.
4. **Perfect odometry.** The real loop reads `latest_odometry` (SEL:794), which
   on the robot is a relocalising SLAM pose with jumps. Here it is exact.
5. **Our body margins are stricter than dimOS' own defaults.** `GlobalConfig`
   ships `robot_width = 0.3` and `robot_rotation_diameter = 0.6`
   (`core/global_config.py:105-106`); this harness runs a 0.35 m planner width
   and a 0.766 m pivot circle for the same Go2. Ours is the more conservative
   pair, it is the one every earlier bench used, and changing it would break
   comparability, so it stays and is declared.
6. ~~**`goal_reached` semantics on path exhaustion.**~~ **FIXED during this job,
   and the fix is worth recording because our own pre-declared check found it.**
   `explore_sim.Sim.drive` returns `"timeout"` for two different events: the
   clock ran out while walking (explore_sim:793) and the path ran out or a whole
   replan produced no motion (explore_sim:801-805). Only the first is SEL:823
   timing out; the second is, in dimOS, the local planner reaching the end of
   its path, doing its final rotation and reporting `"arrived"` (LP:277-310),
   which stops the robot and publishes `goal_reached = True`. The first version
   of the faithful arm treated both as "still pursuing".

   It was caught by the pre-declared `scoring` identity check: in config
   `scoring` the goal timeout is 1e9 s, so no goal can ever end on the clock, so
   T_sel there must be pure standstill and the T_sel = 0 and T_sel = 15 runs
   must be identical. One pair out of six was not (`12 m hk_entrance/mid5`), and
   the reason was exactly this conflation. `bench_2830._still_pursuing` now
   separates the two by the clock - a clock timeout can only happen at or past
   the budget, path exhaustion only before it - and the per-goal
   `path_exhausted` flag and the per-run `goals_path_exhausted` count report how
   often it happens. The 19 runs produced before the fix are parked on the box
   at `/root/sim_2830_fidelity/out_v1_conflated_timeout/` and are used for
   nothing.

## 7. The one-paragraph version

dimOS publishes a frontier goal, waits 15 s, and if the robot has not arrived it
recomputes the next goal **without cancelling anything**. The recompute is a
full-grid python BFS that costs seconds, and through all of it the navigator is
still walking the robot toward the goal the selector has already given up on. The
new goal is chosen for the pose the robot had when the compute started and is
handed over at the pose it has when the compute ends. Our benches froze the robot
for that whole interval, which removed both the extra walking and the staleness.
This job puts them back and asks whether the cross-map swing behaviour survives.
