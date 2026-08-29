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

Schemes: 1 nominal, 2 Dryden wind, 3 obstacles (9 static + 2 dynamic), 4 both.
**Current focus is schemes 1 & 2 × {LQR, H∞}** on a configurable, possibly
**non-convex** mapping region (accepted abstract: "Non-Convex Geodetic Mapping").
Scheme 3 static-only is next; dynamic obstacles / scheme 4 are deferred (CBF for
moving cylinders not yet stable — see `results/compare/scheme4_*`). Avoidance is
always CBF-QP. Pass `--results DIR` so each run's CSVs and log stay paired.

**Mapping region** is a polygon set by the `region` param
(`swarm_high_level/world/region.py` presets or a YAML vertex list). `rect` = the
old 28×28 active area, so schemes 1/2 stay comparable to earlier runs. The
Voronoi/Lloyd partition, boustrophedon, and coverage grid are all polygon-aware.

**Boustrophedon** (`swarm_high_level/world/coverage_path.py`): interior rows stay
horizontal; `generate_boustrophedon` returns `(rows, meta)` where `meta` carries
`connectors` (cell-edge vertices between rows) and `cap_pre`/`cap_post` (edge arcs
over the tapered top/bottom of the cell — those wedges are missed by fixed-pitch
horizontal rows). `expand_path(rows, meta)` inlines all of it into one flat
waypoint list; the drone hugs the Voronoi cell edge at every turn and cap. This
took cell coverage from ~92–96% to ~98–99.8%. `clip_voronoi_margin` is **0.25 m**
(down from 0.45) so adjacent drones' sweeps nearly meet; CBF handles the rare
simultaneous border contact. Short (<2.5 m) cap/connector segments use a fast
pivot settle (6 ticks) so the extra segments don't lengthen the mission.

**Sweep speed**: `nominal_speed` is now the `sweep_speed` param, default **1.6
m/s** (was a hardcoded 2.85 that was never achieved and forced the feedforward
tilt past the 15° limit → end-of-row overshoot). The endpoint clamp
(`s_target ≤ line_len`) is restored inside `_cbf_filter`: it clamps only the
*longitudinal* component of `ref_pos` to the current row, leaving the lateral
(cross-track + V2V) component from the QP intact.

## Verification — use the fast path first

**Gazebo RTF is ~0.26** with 7 drones (measured; disabling the GPU lidar only
gets it to 0.32 — the cost is quadrotor physics, not sensors). A full mission is
8–13 minutes of wall clock. Do not iterate against Gazebo.

```bash
# L1: unit — CBF library + region/boustrophedon geometry, no ROS, no Gazebo — ~20s
source install/setup.bash    # or add src/swarm_low_level to PYTHONPATH
PYTHONPATH=src/swarm_high_level:src/swarm_mid_level:src/swarm_low_level \
  pytest src/swarm_high_level/test src/swarm_mid_level/test -q

# L2 standalone CBF scenario, tunable duration and speed (~15x realtime)
PYTHONPATH=src/swarm_mid_level:src/swarm_low_level \
  python3 src/swarm_mid_level/test/scenario_scheme3.py 200 2.85

# L3: confirm in the real simulator, once
./launch_mapping_demo.sh -s 1 --headless --region u_shape
```

`swarm_ws/tests/verify_09_fault_tolerance_recovery.sh` (fault injection, exit
code) and `verify_11_benchmark_all_schemes.sh` (4-scheme sweep, feeds
`plot_benchmark_schemes.py`) are the surviving integration scripts.

Quantitative reporting goes through
`swarm_high_level/metrics/run_report.py` — tracking & safety, mission time &
coverage, control effort, and CBF diagnostics. It prints `n/a` for missing data
and **never** fills in a substitute value.

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
