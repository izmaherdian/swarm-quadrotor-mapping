# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Research project on **swarm quadrotor area mapping**: multiple `iris` drones partition
a 2D arena via Centroidal Voronoi Tessellation and cover their cells with
boustrophedon ("lawnmower") sweeps, with fault-tolerance (dead-cell merging +
helper reallocation), dynamic obstacle avoidance (ORCA / PPO), and two swappable
low-level controllers (PID-LQR and PID-H∞). Simulation runs in Gazebo (gz-sim 10)
+ RViz2. Text/logs are largely in Indonesian; keep that style when editing.

The `matlabtopython/` and `docs/` trees are the academic side (MATLAB/Simulink
controller synthesis, LaTeX papers) — not part of the runtime.

## Environment

- ROS 2 distro **`lyrical`** — scripts `source /opt/ros/lyrical/setup.bash`. Python 3.14.
- Python venv at repo-root `.venv/` (numpy, scipy, shapely, matplotlib, torch,
  stable-baselines3, gymnasium, onnx). The run scripts auto-source it.
- All work happens in the `swarm_ws/` colcon workspace.

## Build & run

```bash
cd swarm_ws
colcon build                                   # or --packages-select <pkg>
source install/setup.bash
```

From `swarm_ws/`:

```bash
./launch_mapping_demo.sh -s 1|2|3|4 [--pid-lqr|--pid-hinf] [--headless] \
    [--results DIR] [--region rect|l_shape|u_shape|plus|<path.yaml>] [--sweep-speed M/S]
./compare_lqr_hinf.sh -s 1 -d 2400 [--region u_shape]   # both controllers + table + JSON
./kill_drone.sh 4                    # fault injection, from a second terminal
```

Schemes: 1 nominal, 2 Dryden wind, **3 static obstacles only**, 4 static +
2 dynamic. Schemes 1 & 2 × {LQR, H∞} are done and reported; **scheme 3 is the
current focus**. Scheme 4 stays deferred (CBF for moving cylinders not yet
stable — see `results/compare/scheme4_*`); `obstacles.world` is left untouched
so it still works. Avoidance is always CBF-QP. Pass `--results DIR` so each
run's CSVs and log stay paired.

**Scheme 3 = static obstacles only.** `enable_dynamic_obstacles` is a separate
param from `enable_obstacles` and defaults to `scheme == 4`, so scheme 3 never
subscribes to, drives, or feeds the QP the moving cylinders. Scheme 3 also uses
its own world, `worlds/obstacles_<region>.world`, which contains no dynamic
models at all: those cylinders span z 0.25–3.85 m while the drones cruise at
2.0 m, so leaving them spawned-but-unmoved would put two stationary obstacles in
the arena that neither the planner nor the QP can see.

**Obstacles are SENSED, not looked up** (`use_lidar_obstacles`, default true when
obstacles are on). Until 2026-08-31 the LiDAR was dead: `lidar_callback` stored
`lidar_ranges` and *nothing ever read it*, while the planner and the QP both took
obstacle coordinates from a table. Scheme 3 was therefore "avoidance with a
perfect a-priori map", and the paper must not claim sensing for those runs.

Now `swarm_mid_level/perception/obstacle_map.py` turns each scan into obstacle
detections (reject returns near other drones and near the arena wall → cluster →
fit a circle) and merges them into **one shared `ObstacleMap`** for the swarm; a
track is confirmed after 4 sightings. The mission starts with an *empty* map.

The split that keeps this honest:

  * `self.static_obstacles` — what the system **knows**. Empty at takeoff, filled
    by LiDAR. The planner and `_cbf_obstacles` see only this.
  * `self.truth_obstacles` — the Gazebo table. Used **only for scoring**: the
    coverage denominator and collision detection. Grading a drone against
    obstacles it has not yet discovered is the entire point.

`_replan_rows_for_discoveries` re-plans a drone's *remaining* rows when a newly
confirmed obstacle threatens them. Only rows above the current sweep line are
replaced — sweeps run bottom→top, so re-planning completed rows would sweep them
twice. Between discovery and re-plan, the QP is what keeps the drone safe.

Because the LiDAR sits at cruise altitude (mount z +0.08, cruise 2.0 m) and the
cylinders span z 0–4 m, a 360-ray, 12 m horizontal scan sees them; a 0.40 m
cylinder subtends ~9 rays at 5 m.

**Obstacles are per-region and generated, not hand-written.**
`world/obstacles.py :: OBSTACLES_BY_REGION` gives each region 9 cylinders
*inside* that region (the old 9 only all fit `rect`; the non-convex presets
contained just 6). `tools/gen_obstacle_worlds.py` derives the `.world` files
from that table — run it after changing any coordinate, and
`test_obstacle_paths.py` parses the SDF back and fails if the two drift.
`rect` deliberately keeps its historical nine so older runs stay comparable.

**Mapping region** is a polygon set by the `region` param
(`swarm_high_level/world/region.py` presets or a YAML vertex list). `rect` = the
old 28×28 active area, so schemes 1/2 stay comparable to earlier runs. The
Voronoi/Lloyd partition, boustrophedon, and coverage grid are all polygon-aware.

**Boustrophedon** (`swarm_high_level/world/coverage_path.py`) returns ONE flat
`[s0,e0,s1,e1,...]` list, always bottom→top:

  * the **first and last rows are the cell's own bottom/top edge chains**, not
    horizontal lines — that is what covers the wedges a fixed-pitch horizontal
    sweep misses at tapered corners. Interior rows stay horizontal.
  * `entry_point` (the drone's position) picks **which end** of the bottom chain
    the drone enters at, so transit doesn't loop around to the far side.
  * chain vertices closer than **0.60 m** are merged (`_chain_segments`). A
    segment shorter than the 0.58 m stopping distance at 1.6 m/s *cannot* be
    braked into and shows up as end-of-row overshoot; sensor radius 0.95 m still
    covers the merged vertices, so coverage is unaffected.
  * `clip_voronoi_margin` is **0.35 m**.

Cell coverage measured 98.2–100% across all four presets.

**Obstacle avoidance in the path is planned, not left to the QP.** Two things
run when `obstacles` is passed:

  * `_split_interval_for_obstacles` **splits** a scanline interval on the
    keep-out disc, returning a *list*. The multi-interval machinery for concave
    cells already handles the pieces. The old `_trim_…` could only push an
    interval's *ends*: an obstacle mid-row was ignored outright, and one near an
    end pushed that end the wrong way, *extending* the row across the obstacle.
    Measured before the fix: 10 of 92 planned sweep segments in `rect` passed
    within physical collision distance, hitting all nine cylinders.
  * `route_around_obstacles` is a single final pass over the assembled flat
    list, so it covers interior rows, connectors **and the cell edge chains**
    (which were not obstacle-aware at all). It inserts arcs at
    `OBSTACLE_KEEP_OUT = 1.30 m`, then `_simplify_chain` string-pulls points back
    out while proving each removal still clears `OBSTACLE_CLEAR_MIN = 1.10 m`.

Why 1.30: physical need is 0.87 m (0.40 cylinder + 0.22 drone + 0.25
`delta_static`), and an arc chord subtending ≤70° dips to `1.30·cos35° = 1.065`
m — still clear. The 70° cap exists to keep chords *long* (1.49 m ≫ the 0.60 m
`min_seg`); a finely-sampled arc gets merged and the corner cut back into the
keep-out zone. The arc doubles as the sweeper: at 1.30 m with a 0.95 m sensor,
the ring around each cylinder is still mapped, so detouring costs no coverage.
Measured after the fix: closest approach 1.066–1.096 m, coverage 98.2–99.8%.

**Obstacle→cell assignment is by intersection, never `contains`.** A drone plans
around every obstacle whose keep-out disc touches *its swept cell*
(`_obstacles_near_cell`), so one obstacle can belong to several drones. The old
exclusive `contains_point` left the neighbouring drone sweeping through a
keep-out zone with no detour planned — it happens in all four presets. Runtime
safety never depended on this: `_cbf_obstacles` always feeds every obstacle to
every drone's QP.

**Start-point deconfliction is combinatorial, not a flip loop.** Neighbouring
cells can put two drones' "nearest entry" 0.70 m apart — exactly the V2V hard
limit — and then the second drone can never *arrive*, so `wait_all_start` hangs
the whole swarm forever (observed: 317 s stuck, coverage 0%). Each drone has two
candidate entries (near/far end), so all 2⁷ combinations are scored: maximise the
minimum pairwise start distance (capped at 2.5 m), then minimise total transit.
One flipped drone typically buys 0.7 m → 3–7 m for ~8% extra transit.

**Sweep speed**: `nominal_speed` is now the `sweep_speed` param, default **1.6
m/s** (was a hardcoded 2.85 that was never achieved and forced the feedforward
tilt past the 15° limit → end-of-row overshoot).

**During `sweeping_row` in schemes 1–2 the QP is bypassed entirely** unless a
neighbour is within 0.85 m, and `ref_pos` is **projected fully onto the row
line** (`_row_clamp_ref`). Keeping the lateral component let wind blow the
reference off the line, so the low-level position loop chased a bent line and the
wind corrupted the map instead of merely costing control effort. With the
reference pinned to the line, wind appears only as roll/pitch. `ct_corr_max`
(default 0.90 m/s, was 0.45) sets how hard the drone fights back onto the line.
Note this makes `|ref−pos|` *larger* (15 → 25 cm) — that is the honest
cross-track error finally being visible, not a regression.

## Verification — use the fast path first

**Gazebo RTF is ~0.31** with 7 drones (re-measured 2026-08-30 on an idle machine:
156 s of sim in 501 s of wall). A full mission is ~156 s of **sim** time and
~8.5 min of **wall** time.

**Do not read sim time off the coordinator's log timestamps.** Those `[1788…]`
stamps are UNIX epoch — i.e. WALL clock — even with `use_sim_time:=true`. Sim
time comes from the CSVs' `Time_s`, which is derived from the Gazebo odometry
`header.stamp`. Comparing log stamps against wall clock trivially yields
"RTF ≈ 1.0" and is wrong; that mistake was made and corrected on 2026-08-30.

**Always pass `--exit-after N`** for unattended runs. The coordinator otherwise
never exits (`main()` just spins), so a caller's `timeout` burns its full budget
even though the mission finished: the 2026-08-29 sweep spent 241 min on 6
missions that each completed in ~8 min of sim. `exit_after_success` (seconds of
sim time) shuts the node down once **every live drone is `done`** — not merely
when coverage ≥ 97%, so a mission is never truncated. Default `0.0` keeps the
old never-exit behaviour.

Note `-p` DOUBLE params must carry a decimal point (`12` is rejected as INTEGER);
`launch_mapping_demo.sh` formats them with `printf` for you.

```bash
# L1: unit — CBF library + region/boustrophedon geometry, no ROS, no Gazebo — ~20s
source install/setup.bash    # or add src/swarm_low_level to PYTHONPATH
PYTHONPATH=src/swarm_high_level:src/swarm_mid_level:src/swarm_low_level \
  pytest src/swarm_high_level/test src/swarm_mid_level/test -q

# L2 standalone CBF scenario, tunable duration and speed (~15x realtime)
#     --static-only drops the two moving cylinders, matching scheme 3
PYTHONPATH=src/swarm_mid_level:src/swarm_low_level \
  python3 src/swarm_mid_level/test/scenario_scheme3.py 200 1.6 --static-only

# L3: confirm in the real simulator, once
./launch_mapping_demo.sh -s 1 --headless --region u_shape

# L4: scheme 3 sweep — regions × {lqr, hinf}, ~11 min wall each
./tools/sweep_scheme3.sh                      # l_shape u_shape plus × both
PYTHONPATH=src/swarm_high_level python3 -m swarm_high_level.metrics.region_report <out>
```

`test_obstacle_paths.py` is the guard for scheme 3's planner: no path segment may
come within 0.62 m of a cylinder centre, coverage must survive the detours, the
waypoint list must stay even-length, and the generated `.world` files must still
match `OBSTACLES_BY_REGION`. Every one of its assertions fails on the pre-fix
code, which is the point.

`tools/sweep_scheme3.sh` wipes `/dev/shm` between missions and kills stale
processes with `pkill -f "[g]z sim"` — the bracket matters, since a plain
`"gz sim"` pattern matches the invoking shell's own command line and kills it.

`swarm_ws/tests/verify_09_fault_tolerance_recovery.sh` (fault injection, exit
code) and `verify_11_benchmark_all_schemes.sh` (4-scheme sweep, feeds
`plot_benchmark_schemes.py`) are the surviving integration scripts.

Quantitative reporting goes through `swarm_high_level/metrics/`:
`run_report.py` (one or two runs — the 4-group table + JSON) and
`region_report.py` + `region_figures.py` (scheme × region × controller matrix
plus PNG figures). Both print `n/a` for missing data and **never** fill in a
substitute value.

**Attitude metrics must be filtered to the sweeping phase.** `region_report.py`
keeps only samples where `Ref_Yaw` has been stable ≥1.5 s and the drone is moving
>0.6 m/s. Unfiltered whole-mission percentiles are dominated by the ~180° pivots
at row ends and will make you "diagnose" a wind-induced yaw wobble that does not
exist — during steady sweeping the yaw error is ~1° RMS and wind shows up in
roll/pitch (+11°), exactly as intended.

**Benchmark tooling used to fabricate data.** `plot_benchmark_schemes.py`
silently substituted `np.random`-generated telemetry when CSVs were missing (42
of 56 benchmark CSVs were 0 bytes, so 6 of 8 scheme×controller panels were
fiction), hardcoded `ov_max = 0.00`, and printed fixed "conclusions" regardless
of the data; `verify_10` printed "PASS" in every cell unconditionally. All fixed
or removed, and the fabricated figures were deleted on 2026-08-29. Every number
destined for the paper must trace to an actual CSV.

## Package architecture (`swarm_ws/src/`)

Layered controller stack; drones are namespaced `/iris_1`.../iris_7`, communicating
over `.../odometry`, `.../cmd_vel`, `.../scan`.

- **`swarm_sim`** (ament_cmake) — Gazebo assets: `worlds/` (`empty.world`,
  `obstacles.world`), `models/iris_1..7` + `iris_base`, `rviz/`, and the
  `launch/` files. `spawn_drones_launch.py` is the workhorse: takes
  `num_drones`, `controller`, `spawn_x{,1..7}`, `results_base`, `use_mid_level`,
  and wires up per-drone controller + gz↔ros bridges + mid-level node. It parses
  `sys.argv` for `foo:=bar` directly (not just via LaunchConfiguration) to pick
  the CSV results directory.

- **`swarm_low_level`** (ament_python) — attitude/position control producing
  motor `Actuators` from `cmd_vel`/setpoints. Nodes: `pid_lqr_node`,
  `pid_hinf_node` (the two benchmarked controllers), `dryden_wind_node` (wind
  disturbance injection), `tf_prefix_node`. `solver_pid_{lqr,hinf}.py` hold the
  gain math; `config/quadrotor_params.yaml` the physical params. The many
  `simulator_pid_*` files are standalone offline plant simulations for the paper
  plots, not ROS nodes.

- **`swarm_mid_level`** (ament_python) — **CBF-QP collision avoidance**. Pure
  numpy, no ROS dependency, which is why its suite runs in seconds. One QP per
  drone per tick folds static obstacles, moving obstacles and reciprocal
  drone-to-drone spacing into a single set of linear constraints on commanded
  velocity, replacing four stacked hand-tuned layers.

  - `cbf/plant_model.py` — derives `k_v`, `a_max`, `T_lead`, `v_c` from the LQR
    gains + `quadrotor_params.yaml`. **Nothing downstream hardcodes physics.**
  - `cbf/barrier.py` — `φ(h)`, the actuator-consistent class-K function. It is
    the whole method: the approach speed allowed at clearance `h` is the fastest
    one that can still stop in time given the real 15° tilt limit and dead time.
    A linear `α(h) = γh` provably cannot work near `h → 0`.
  - `cbf/qp2d.py` — exact Euclidean projection onto a 2D polyhedron by
    active-set enumeration. No solver dependency (deliberate: cp314 wheels for
    osqp/quadprog aren't guaranteed and the launcher calls bare `python3`).
    ~138 µs at 24 rows. Validated against SLSQP + a HiGHS LP referee on 3000
    random instances — SLSQP failed 320 of them, this solver none.
  - `cbf/deadlock.py` — symmetry breaking. **Only ever touches the objective,
    never `A` or `b`**, so liveness is never traded against safety. Note CBF
    constraints only *prevent* approach; they never *restore* separation, so a
    packed cluster freezes permanently without the recovery term here.
  - `cbf/avoidance.py` — `solve_all()` facade plus the Tier 0–3 infeasibility
    ladder. Use `solve_all`, not seven `solve()` calls: the reciprocal split
    `λ_ij + λ_ji = 1` is only exact from one snapshot.

  Integration point is deliberately narrow: `send_world_twist()` in the
  coordinator filters the state machine's desired velocity through the QP. The
  state machine proposes, the QP disposes. With `use_cbf=false` the legacy path
  is byte-for-byte unchanged.

- **`swarm_high_level`** (ament_python) — swarm coordination support, pure
  numpy/shapely (no ROS), so it is unit-tested in `src/swarm_high_level/test/`:
  - `world/region.py` — mapping-region presets + YAML loader, in-region seed
    points for Lloyd, grid region mask.
  - `world/coverage_path.py` — `poly_centroid`, scanline intersections,
    non-convex-aware `generate_boustrophedon`, `clip_poly_to_region`.
  - `world/obstacles.py` — the 9 static + 2 dynamic obstacles, one source of truth.
  - `metrics/{telemetry,run_report}.py` — the 4-group quantitative table + JSON.
  The deployed ROS coordinator is `experiments/test_7drone_voronoi_mapping.py`,
  which drives the state machine on live odometry and imports the `world/` helpers.

## Traps worth knowing

- **The low level is a position controller, not a velocity one.**
  `target_pose_callback` latches `target_pose_received = True` permanently
  ([pid_lqr_node.py:577](swarm_ws/src/swarm_low_level/swarm_low_level/pid_lqr_node.py#L577));
  after that `cmd_vel` can never move the reference again. Velocity enters only
  as attitude feedforward (`k_ff = 0.15 rad per m/s`).
- **Commanded speeds are not achieved.** The old `nominal_speed = 2.85` never
  materialised — measured max across all 7 drones was **1.86 m/s** — and
  `k_ff·2.85 = 0.43 rad` exceeded the 15° tilt limit, causing end-of-row
  overshoot. `sweep_speed` now defaults to **1.6 m/s** (achievable; tilt ~13.7°).
- **`ref_pos` is written in one place under CBF** (`_cbf_filter` →
  `pos + T_lead·v_safe`, `T_lead = 0.3287 s`). During `sweeping_row` that filter
  then clamps the *longitudinal* projection of `ref_pos` to `[0, line_len]` so
  the drone cannot overshoot the row end; the lateral component is untouched.
- The coordinator writes **no files** and **never exits** — `main()` just spins.
  Callers must kill it; success only sets `mission_completed`.
- Inter-drone collision is never detected; only `d_min` is tracked.

## Coordinator script (the actual mapping logic)

`swarm_ws/experiments/test_7drone_voronoi_mapping.py` is where the mission
behavior lives — state machine over `pivot_to_transit` → `transit_to_start` →
`wait_all_start` → `align_start_yaw` → `sweeping_row` → `delay_at_corner_end` →
`stepping_vertical` → `delay_at_new_row` → … → `return_to_centroid` → `done`,
plus dead-cell merging + helper reallocation. Pure geometry lives in
`swarm_high_level/world/`; edit the coordinator for mapping/coverage behavior and
`swarm_low_level` nodes for tracking/stability. RViz2 `MarkerArray` on
`/mapping/markers` (region boundary, cells, carrot, HUD sidebar, compass).
