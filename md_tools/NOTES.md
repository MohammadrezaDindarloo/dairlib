# NOTES — Push Anything internals, baseline and our changes

Running log of understanding, decisions and findings. Line references are to commit
`5371ac12a` (`md-reachability`, = `push_anything_dev` + one `.gitignore` commit).

Abbreviations: `ctrl.cc` = `systems/controllers/sampling_based_c3_controller.cc`,
`main.cc` = `examples/sampling_c3/franka_sampling_c3_controller.cc`,
`gen.cc` = `examples/sampling_c3/generate_samples.cc`,
`goal.cc` = `examples/sampling_c3/goal_generator.cc`,
`params/` = `examples/sampling_c3/anything/parameters/`.

State layout used everywhere (n objects): `x_lcs = [p_ee(3), {q_i(4), p_i(3)}×n, v_ee(3), {ω_i(3), v_i(3)}×n]`,
so `n_q = 3+7n`, `n_v = 3+6n`. Object i position is `x.segment(7+7i, 3)`, quaternion `x.segment(3+7i, 4)`.

---

## 0. Findings that matter before Part B (read first)

1. **`multiyaml_rewrite.py` edits tracked files.** It rewrites `sampling_c3_controller_params.yaml`
   (object_models), `sim_params.yaml`, `vis_params.yaml`, both `lcm_channels_*.yaml`,
   `goal_params.yaml`, `reposition_params.yaml`, `sampling_params.yaml`,
   `progress_params_c3plus.yaml` and `sampling_c3plus_options.yaml`
   (`multiyaml_rewrite.py:476-598`). The committed files are exactly its output for the
   default 4-object `base_names: [G_shape_video, R_shape_texture, A_shape_video, S_shape_texture]`
   (`git status` was clean after the earlier `run_sim.sh` run). So the experiment runner must
   `git checkout --` these files after every non-default run, or `git status` will not be clean.
2. **"Defaults" depend on the number of objects.** The rewrite overwrites tuning parameters
   from per-count tables (`multiyaml_rewrite.py:14-36`, applied at `:233-264`, `:457-473`):

   | objects n | 1 | 2 | 3 | 4 |
   |---|---|---|---|---|
   | horizon `N` | 10 | 15 | 7 | 7 |
   | `num_outer_threads` | 6 | 5 | 4 | 5 |
   | `include_walls` | **true** | **true** | **true** | false |
   | `w_R` / `w_G` (pose) | 6 / 0.18 | 6 / 0.18 | 10 / 0.15 | 10 / 0.2 |
   | `num_additional_samples_c3` | 5 | 4 | 3 | 4 |
   | `num_additional_samples_repos` | 4 | 3 | 2 | 3 |
   | `progress_enforced_over_n_loops` | 35 | 25 | 18 | 16 |
   | `cost_switching_threshold_distance` | 0.05 | 0.05 | 0.05 | 0.05 |

   It also sets initial object poses `x = 0.4+0.02i, y = -0.3+0.2i`, yaw quat `[0.393,0,0,0.92]`
   (`:561-564`), the first goal `x = 0.5, y = 0.2i − 0.1(n−1)` plus per-letter offsets/orientations
   from `letter_settings.yaml` (`:432-454`), resting heights and `pwl_waypoint_height` (`:567-570`).
   With walls on (n ≤ 3), C3 has wall contacts and **no** linear workspace constraints on the
   objects (`ctrl.cc:977-1006` only runs when `include_walls` is false).
3. **Not seedable.** Sample generation and goal generation use `std::random_device`
   (`gen.cc:492,625,630`, `goal.cc:255,335`, `common/math_utils.h:13`). "Same seeds" across
   baseline/ours is impossible without a (guarded) change to their files. Until then, compare
   distributions over several runs with the same objects, first goal and duration.
4. **Compute-coupled.** The sim runs at `realtime_rate: 1` (`params/sim_params.yaml`) and the
   controller is an `LcmDrivenLoop` on `FRANKA_STATE_SIMULATION` (`main.cc:748-765`): each loop
   takes the latest state and solves as fast as it can. Slower CPUs mean fewer control loops
   per simulated second, and the progress test counts **loops**, not seconds (§1.6). The
   earlier `run_sim.sh` run (`sim_logs/c3.log`, 4 objects, ~15 min) logged 116 loop gaps
   > 1 s sim time (mean 1.15 s, max 1.51 s; `lcm_driven_loop.h:329`), 58 "unproductive" switches,
   57 "reached repositioning target" switches, **0** "found lower-cost sample" switches, and
   **no goal completed** (only the start-up goal detection). `sacct` shows that run was job
   `mlpGPU` on crannog05 with **4 CPUs**, while the controller requests
   `num_outer_threads × num_threads` = 5 × 5 threads (nested OpenMP, `ctrl.cc:410-419`) — so
   that run was CPU-starved and is not a valid baseline. Decision: baseline runs use
   partition `Open-Research` (crannog, 64 CPUs/node), `--cpus-per-task=24`, no GPU; node and
   CPU count are recorded per run.
5. **`start_logging.py` is broken twice**: `/opt/lcm/1.4.0/bin/lcm-logger` does not exist in the
   container (it is at `/usr/bin/lcm-logger`), and it reads `controller_params['object_model']`
   / `sim_params['object_model']` (`start_logging.py:43-44`), keys that no longer exist
   (now `object_models` lists) → `KeyError` before logging starts. We will not edit it; Part B
   writes our own wrapper.
6. **Compute time per step is not published.** `filtered_solve_time_` (`ctrl.cc:1360-1367`) is
   internal. It can be recovered from logs: the execution trajectories' first knot time is
   `t_plan_start + filtered_solve_time_` (`ctrl.cc:1669`, `:1779`), and the wall-clock spacing of
   `SAMPLING_C3_DEBUG` messages (one per loop) gives the raw loop period.
7. **The controller aborts on EE workspace violation**: `CheckForWorkspaceLimitViolations`
   (`ctrl.cc:1452-1470`) uses `DRAKE_DEMAND`, so the process dies (no graceful failure).
   Also `MeshNormalSamplingMultiObject` throws after 100 failed attempts (`gen.cc:751`).
   The runner must detect a dead controller and record it as a failure.

---

## A1. Control loop of `franka_sampling_c3_controller`

### Diagram (`main.cc`)
- Inputs: Franka state + one `lcmt_object_state` per object → `FrankaKinematics` builds `x_lcs`
  (`main.cc:369-387, 621-637`). Radio (xbox) on `SAMPLING_C3_RADIO` (`:379-381`).
- Goal generator `SamplingC3GoalGeneratorPlanar` (`main.cc:391-406`) feeds two muxes:
  `x_lcs_des` = EE target + per-object **lookahead sub-goal** + velocity targets
  (`:420-463`), and `x_lcs_final_des` = EE target + per-object **final goal** + zero velocities
  (`:465-487`).
- Controller `SamplingC3Controller` (`main.cc:490-492`); all publishers are forced-publish once
  per loop (`:494-577`), including `SAMPLING_C3_DEBUG`, `IS_C3_MODE`, `SAMPLE_LOCATIONS`,
  `SAMPLE_COSTS`, `C3_ACTUAL`, `C3_TARGET`, `C3_FINAL_TARGET`, `TRACKING_TRAJECTORY_ACTOR`.
- Loop: `LcmDrivenLoop` triggered by `FRANKA_STATE_SIMULATION`, waits until each object channel
  has >1 message (`main.cc:748-765`).

### One loop: `ComputePlan` (`ctrl.cc:719-1375`)
1. Read inputs; optionally replace EE state by the prediction from the last plan
   (`ResolvePredictedEEState`, `ctrl.cc:751-753`, `:1382-1448`; on for both modes in C3+ options).
2. `CheckForWorkspaceLimitViolations` (`:759`) — aborts on violation (see §0.7).
3. Current errors summed over objects vs **final** goal: position (3-D) and angle (`:761-779`).
4. Goal-change detection and pose/position cost switch (§A2).
5. Build C3 options and cost matrices for the current mode (`:833-842`, `UpdateCostMatrices`
   `:1474-1536`, quaternion-dependent Hessian cost in pose mode).
6. `object_on_target[i]` vs the **lookahead** target `x_lcs_des`: position < 0.02 m (xy) and,
   in pose mode, angle < 0.1 rad (`:844-864`). Only used to choose which objects to sample on
   and for fixed-goal mode.
7. **Sample generation** (`:866-889`): `GenerateSampleStates` returns
   `num_additional_samples_c3` (C3 mode) or `num_additional_samples_repos` (repositioning mode)
   candidate states that differ from `x_lcs` only in EE position (§A1.1).
8. Previous repositioning target: rejected if within `sample_projection_clearance` (0.027 m) of
   any object/wall (`:891-909`). In repositioning mode it is inserted at index 1 if not rejected
   (`:911-919`). Index 0 is always the current state (`:921`).
   So the candidate list is
   - C3 mode: `[current, s_1..s_k]` (+ later possibly `buffer_best`), k = `num_additional_samples_c3`;
   - repositioning: `[current, prev_target?, s_1..s_k]`, k = `num_additional_samples_repos`.
9. One LCS per candidate (and a finer one for cost) with contact pairs resolved **at that
   candidate's state** (`CreateLCSObjectsForSamples`, `:1575-1654`; §A3).
10. **Parallel C3+ solve per candidate** (`:950-1061`, OpenMP): each gets
    `x_desired = x_lcs_des` (the lookahead sub-goal) for all N+1 knots (`:963`), EE/object
    workspace constraints if no walls (`:977-1006`), EE velocity limits (`:1008-1016`), input
    limits (`:1018-1034`), `Solve` (`:1037-1038`), then a cost (§A1.2).
11. Sample buffers updated / best buffered sample appended (C3 mode only) (`:1066-1073`; §A1.3).
12. Mode decision with hysteresis (`:1075-1286`; §A1.4).
13. Execution: C3 mode tracks the **current-location** C3 plan (EE position + feed-forward force)
    (`UpdateC3ExecutionTrajectory`, `:1656-1746`; for planar demo the EE height is pinned to
    `z_height`). Repositioning mode tracks a contact-free path to the best sample
    (`UpdateRepositioningExecutionTrajectory`, `:1749-1804` → `reposition.cc`). Both are
    computed every loop; `traj_execute_actor` publishes the one for the current mode
    (`:2652-2680`). `prev_repositioning_target_` is set to the best sample every loop (`:1754`).

### A1.1 How samples are generated (`gen.cc`)
- Strategy in use: `sampling_strategy: 7` = `kMeshNormalMultiObject`
  (`GenerateSampleStates`, `gen.cc:147-158` → `MeshNormalSamplingMultiObject`, `:562-753`).
- Mesh faces are loaded once per object from `urdf/<name>/<name>.obj`; faces whose normals are
  too vertical are dropped (`ctrl.cc:426-486`, filter at `:465-475`).
- Per sample:
  1. Pick an object **uniformly at random** among objects not `object_on_target`
     (`gen.cc:611-633`; the comment says "weighted by area" but it is uniform; `dis_obj` is unused).
  2. Pick a face weighted by area, a uniform point on it (`:644-666`).
  3. Offset by `buffer_distance` (0.035 m) along the face normal (`:668-669`).
  4. Set z to `z_height` (0.002 m), or `z_height + 0.01` within 3 cm of a wall when walls and
     `sample_on_wall` are on (`:671-705`).
  5. Reject if within `sample_projection_clearance` (0.027 m) of any object (or wall, if
     `sample_on_wall` is false) via `ComputeSignedDistanceToPoint` (`:715-743`); retry up to
     `max_attempts` (100), then **throw** (`:751`).
- Then the outer `do … while (!SampleIsAcceptable(...))` (`gen.cc:148-157`) re-samples until:
  - `IsSampleInWorkspace` (`gen.cc:771-798`): EE xy box and radius annulus, shrunk by
    `workspace_margins` (0.03): x ∈ [0.26, 0.77], y ∈ [−0.477, 0.477], z ∈ [−0.04, 0.30],
    r = √(x²+y²) ∈ [0.31, 0.72] (from `sampling_c3plus_options.yaml:29-33`);
  - `SampleAvoidsBadSpots` (`gen.cc:187-205`): not within `unsuccessful_radius` (0.01 m) of any
    entry in the unsuccessful buffer.
  The outer loop has no attempt cap (it can spin if the acceptable set is empty).
- Note: **only EE samples** are checked against the workspace. Nothing checks whether the
  *object* will stay pushable; that is the gap behind "all failures = object out of reach".

### A1.2 How each candidate is scored
- `CalcCost` (`ctrl.cc:490-717`): takes the C3+ plan (x, λ, u from `z`), appends x_N, and with
  `cost_type: 5` = `kSimImpedanceObjectCostOnly` (both pose and position, `progress_params_c3plus.yaml:18-19`)
  re-simulates an impedance-tracked rollout on the finer cost LCS (`N·lcs_dt_resolution` steps,
  `dt/4`) and zeroes the EE and input weights (`:589-606`). Cost =
  Σ_k (x_k − x_des)ᵀ Q_k (x_k − x_des) over **all objects**, x_des = lookahead sub-goal (`:609-611`).
- Total sample cost = C3 cost + `travel_cost_per_meter` × xy distance from current EE
  (`:1050-1054`). `travel_cost_per_meter: 0`, so **no travel cost in practice**.
- In repositioning mode, if the previous repositioning target was reached
  (`finished_reposition_flag_`, set by `reposition.cc` when the remaining path fits in one step),
  `finished_reposition_cost` (1e9) is added to index 1 (`:1056-1060`), which forces the
  switch to C3 next loop (reason 2).
- `best_sample_index_` = argmin over indices ≥ 1 (`:1093-1110`).

### A1.3 Sample buffers
- **Sample buffer** (`MaintainSampleBuffers`, `:1865-1965`): stores up to `N_sample_buffer` (200)
  past samples with their costs; entries are dropped when any object has moved more than
  4 mm or 0.05 rad since the sample was scored (`PruneOutdatedSamplesFromBuffer`, `:1807-1860`).
  Lowest-cost entry is kept last.
- In C3 mode, if `consider_best_buffer_sample_when_leaving_c3`, the best buffered sample (with
  its **stale** cost and plan, no new solve) is appended as an extra candidate (`:1969-2021`).
  Choosing it removes it from the buffer (`:1168-1173`).
- **Unsuccessful buffer** (`AddToUnsuccessfulBuffer`, `:2024-2068`): every repositioning→C3 switch
  stores the current EE location; future samples within 1 cm are rejected and buffered samples
  within 1 cm are removed. Size 10 (overflow warns and drops the oldest). Pruned when objects
  move > 6 mm / 0.05 rad.
- Both buffers are cleared on goal change and on crossing the pose threshold (`:807`, `:826`).

### A1.4 Mode switching and every `mode_switch_reason`
Enum `ModeSwitchReason` (`sampling_based_c3_controller.h:58-65`); reset to 0 every loop
(`ctrl.cc:1122`), so a non-zero value appears in exactly one debug message per switch.
Hysteresis set chosen by pose/position mode (`:1077-1091`); `use_relative_hysteresis: true`.
Let c₀ = current-location cost, c* = best other cost, c_r = cost of the previous repos target.

**In C3 mode** (`:1126-1177`):
- First `KeepTrackOfC3ModeProgress` (`:2071-2161`) with `track_c3_progress_via: 3`
  (`kConfigCostDrop`): keeps the last `progress_enforced_over_n_loops` values of the object
  configuration cost Σ_i e_iᵀ Q₀ e_i, e_i = pose error to the **final** goal (`:2081-2089`).
  When the window is full, progress is insufficient unless the cost dropped by at least
  `progress_enforced_cost_drop` = 50% from the oldest to newest entry (`:2116-2127`, `:2153-2157`).
  (`num_control_loops_to_wait*` are unused with this metric.)
- Fixed-goal mode and all objects done → repositioning, reason stays 0 (`:1135-1140`).
- **4 `kToReposUnproductive`**: progress insufficient, not forced by xbox, and C3 samples > 0
  (`:1141-1147`).
- **3 `kToReposCost`**: c₀ > c* + f·c₀, i.e. c* < (1−f)·c₀ with f = 0.6 (pose) / 0.7 (position):
  the best other sample must be < 40% / 30% of the current cost (`:1149-1160`).
- On leaving C3: progress metrics reset; `source_of_pursued_target` = 3 (buffer) if the chosen
  index is the appended buffer sample, else 2 (new sample) (`:1162-1177`).

**In repositioning mode** (`:1178-1286`):
- Repos→repos (no reason code): keep the previous target (source 1) unless a new sample is
  < (1−0.7)·c_r; if the previous target is now in collision, take the new best (source 2)
  (`:1179-1220`). When switching to a new sample, c* is inflated by the hysteresis before the
  C3 comparison (`:1212-1219`).
- **5 `kToC3Xbox`**: radio channel 12 forces C3 (`:1243-1253`). Not used in our sim.
- Fixed-goal reached: stay repositioning (`:1254-1259`).
- To C3 when c* > c₀ + f·c* (i.e. c₀ < (1−f)·c*, f = 0.9 pose / 0.5 position) **and** the EE is
  low: z < `z_height + c3_min_clearance` (+0.01 near walls) = 0.012 m (`:1260-1269`):
  - **2 `kToC3ReachedReposTarget`** if c_r > 1e9, i.e. the target-reached penalty was added (`:1272-1275`);
  - **1 `kToC3Cost`** otherwise (`:1276-1279`).
  Either way, the current EE location goes into the unsuccessful buffer (`:1281-1284`).

Note on the earlier run: reasons were almost all 4 and 2, i.e. "push until the config cost
stops halving, then go to a fresh sample and push from there". **Hypothesis (unverified):**
because the progress cost sums over all objects, pushing one object rarely halves the total
cost in a 16-loop window when n = 4, so C3 is cut off often.

### A1.5 Repositioning motion (`reposition.cc`)
- `traj_type: 3` = piecewise linear (`reposition.cc:393-466`): up to `pwl_waypoint_height`,
  across above the target, down; speed 0.18 m/s; straight line once within 8 mm in xy
  (`:44-53`, `:78-110`). Sets `finished_reposition_flag` when the target is reached within one
  step. (Other trajectory types use `x_lcs.segment(n_q−3,3)` as "the object", i.e. only the
  **last** object — irrelevant for type 3.)

---

## A2. Goals, lookahead sub-goals, success and goal changes

### Final goals (`goal.cc`, `goal_params.yaml`)
- Initial final goal = `fixed_target_positions/orientations` (`goal.cc:87-89`), written by the
  rewrite script (§0.2).
- `CalcObjectTarget` (`goal.cc:111-187`), evaluated every loop per object:
  - success for object i: xy distance (`only_use_xy_position: true`) < 0.02 m **and** angle
    < 0.1 rad to its **final** goal (`:130-152`);
  - only when **all** objects succeed at the same time (`:154-177`) are new goals drawn for all
    objects (`OnGoalReached`, `:393-404`).
- `goal_mode: 0` (random):
  - the y range [−0.30, 0.30] is split into n equal strips (`goal_params.h:85-94`); strips are
    re-assigned to objects by a random derangement (`goal.cc:244-269`);
  - position (`:271-326`): x ∈ [0.50, 0.51]; first object (leftmost strip) at a random y in its
    strip (n ≤ 2) or at the strip's left edge (n ≥ 3); each next object 0.20 m
    (`pairwise_goal_distance`) from the previous one, at an angle ≤ asin(0.01/0.2) — i.e. goals
    form a near-horizontal row;
  - orientation (`:331-359`): flat plus a random yaw; after the first random goal the yaw is
    drawn from [90°, 270°] relative to the previous goal (so every goal needs ≥ 90° rotation).
  - `random_goal_radius_limits` is read but never used.
- The goal **never** moves an object goal to keep it reachable; goals lie on x ≈ 0.5.

### Lookahead sub-goal (what C3 actually tracks)
- `GenerateLineTrajectoryWithLookahead` (`goal.cc:406-452`): position = current +
  min(0.15 m, distance) toward the final goal along a **straight line** (3-D, incl. z);
  orientation = rotate the current orientation toward the final one by at most 2 rad along the
  shortest axis, with axis hysteresis near 180° (`angle_hysteresis` 0.4 rad, `:433-439`).
- Velocity targets: zero (because `angle_err_to_vel_factor: 0`, `:191-229`).
- EE target: object 0 position + 0.06 m in z (`:98-109`); EE cost weight is ~0 anyway.
- This sub-goal is `x_lcs_des` → C3 `x_desired` (`ctrl.cc:963`) and the reference for sample costs.
  It is published on `C3_TARGET`; the final goal on `C3_FINAL_TARGET`.

### What the controller does with goals (`ctrl.cc`)
- **Goal change**: final target (excluding EE) differs by > 1e-5 → print "Detected goal change!",
  `detected_goal_changes_++`, back to position mode, buffers cleared (`:781-808`).
  `detected_goal_changes_` starts at −1 and becomes 0 on the first loop (start-up), so
  **goals completed = last value of `detected_goal_changes`**, and time to goal = time between
  increments (first goal: from first control loop).
- **Position → pose mode**: when Σ_i ‖p_i,xy − g_i,xy‖ < 0.05·n (`:810-831`). Before that,
  orientation weights are 0 (`q_vector_position`); after, full pose. Planning dt is 0.075 s in both.
- Success detection is only in the goal generator; the controller's own `object_on_target`
  (vs lookahead target) is used for sampling only.

---

## A3. Multiple objects

- One joint LCS with all objects. Contact pair **groups** (`main.cc:240-356`, order at `:347-356`):
  1. EE–ground (resolve 0);
  2. EE–every convex piece of every object (resolve **1**);
  3. object–ground: 3 spheres per object (resolve all 3n);
  4. for each object pair: all convex-piece pairs (resolve 1 per pair);
  5. walls–object per object (resolve 1 each) if `include_walls`.
  Counts per group come from `resolve_contacts_to_lists` (rewritten per n).
- For each candidate, `GetResolvedContactPairs` (`ctrl.cc:1538-1571`) keeps the N closest pairs
  per group **at the candidate's state** (`LCSFactory::GetNClosestContactPairs`). Hence the LCS
  for a sample has exactly **one EE contact: the convex piece closest to the sample**. This is
  how a sample "targets" an object — implicitly, by where it was placed (on a uniformly chosen
  not-yet-on-target object, §A1.1). The C3 cost still sums over all objects.
- All objects are pursued simultaneously: no object ordering, no explicit "current object".
  The success test requires all objects on goal at once (§A2).
- Object–object contacts are planar (2 friction directions, `resolve_as_planar_contacts_list`).

---

## A4. Parameters affecting sampling, repositioning, progress and goals

Values are the committed (4-object) values; † = overwritten by `multiyaml_rewrite.py` per n (§0.2).

### `sampling_params.yaml`
| parameter | value | effect |
|---|---|---|
| `sampling_strategy` | 7 | `kMeshNormalMultiObject` |
| `filter_samples_for_safety` | true | enforce `IsSampleInWorkspace` |
| `num_additional_samples_repos` † | 3 | new samples per loop in repositioning (+current, +prev target) |
| `num_additional_samples_c3` † | 4 | new samples per loop in C3 (+current, +buffer best) |
| `consider_best_buffer_sample_when_leaving_c3` | true | append buffer best in C3 mode |
| `N_sample_buffer` | 200 | buffer size |
| `pos_error_sample_retention` / `ang_…` | 0.004 m / 0.05 rad | prune buffer when objects move |
| `avoid_choosing_unsuccessful_samples` | true | reject samples near past unsuccessful starts |
| `N_unsuccessful_sample_buffer` | 10 | |
| `unsuccessful_pos_error_sample_retention` / `ang_…` | 0.006 m / 0.05 rad | prune unsuccessful buffer |
| `unsuccessful_radius` | 0.01 m | rejection radius |
| `sample_projection_clearance` | 0.027 m | min distance from sample to object surface |
| `buffer_distance` | 0.035 m | offset along face normal; also used in face filter |
| `max_attempts` | 100 | then `throw` |
| `barycentric_bias` | 1 | unused by strategy 7 |
| `z_height` † | 0.002 m | sample height (planar) |
| `gen_planar_samples` | true | project samples to `z_height` |
| `ee_z_close` | true | require low EE before entering C3 |
| `sample_on_wall` | true | allow samples near walls (raised +1 cm) |
| `c3_min_clearance` | 0.01 m | EE z tolerance for entering C3 |
| `sampling_radius`, `sampling_height`, `min/max_angle_from_vertical`, `fixed_sample_locations`, `grid_*_limits`, `min/max_sampling_radius` | – | unused by strategy 7 |

### `reposition_params.yaml`
| parameter | value | effect |
|---|---|---|
| `traj_type` | 3 | piecewise linear |
| `speed` | 0.18 m/s | repositioning speed |
| `use_straight_line_traj_under_piecewise_linear` | 0.008 m | final straight segment |
| `pwl_waypoint_height` † | 0.0774 m | travel height |
| `max_tilt_angle` | 20° | only if `include_end_effector_orientation` (false) |
| spline/sphere/circle params | – | unused for type 3 |

### `progress_params_c3plus.yaml` (the one linked from the controller params)
| parameter | value | effect |
|---|---|---|
| `cost_type`, `cost_type_position` | 5, 5 | object-only impedance-rollout cost |
| `track_c3_progress_via` | 3 | `kConfigCostDrop` |
| `progress_enforced_cost_drop` | 0.5 | required relative drop over the window |
| `progress_enforced_over_n_loops` † | 16 | window length in **loops** |
| `num_control_loops_to_wait(_position)` | 5, 5 | unused with metric 3 |
| `cost_switching_threshold_distance` † | 0.05 m (×n) | position → pose mode |
| `travel_cost_per_meter` | 0 | no travel penalty |
| `finished_reposition_cost` | 1e9 | forces switch to C3 at target |
| `use_relative_hysteresis` | true | fractions below are used, absolute values are not |
| `hyst_c3_to_repos_frac` / `_position` | 0.6 / 0.7 | |
| `hyst_repos_to_c3_frac` / `_position` | 0.9 / 0.5 | |
| `hyst_repos_to_repos_frac` / `_position` | 0.7 / 0.7 | |
| `hyst_*` absolute | 3e5, 2e5, 6e13 | unused (relative mode) |

### `goal_params.yaml`
| parameter | value | effect |
|---|---|---|
| `goal_mode` | 0 | random |
| `position_success_threshold` | 0.02 m | |
| `orientation_success_threshold` | 0.1 rad | |
| `only_use_xy_position` | true | |
| `resting_object_heights` † | per object | goal z |
| `ee_target_z_offset_above_object` | 0.06 m | EE target |
| `lookahead_step_size` | 0.15 m | sub-goal step |
| `lookahead_angle` | 2 rad | sub-goal rotation step |
| `angle_hysteresis` | 0.4 rad | |
| `angle_err_to_vel_factor` | 0 | zero velocity target |
| `fixed_target_positions/orientations` † | per object | first goal |
| `random_goal_x_limits` | [0.50, 0.51] | |
| `random_goal_y_limits` | [−0.30, 0.30] | split into n strips |
| `random_goal_radius_limits` | [0.50, 0.52] | **unused** |
| `random_goal_gen_max_attempts` | 100 | |
| `pairwise_goal_distance` | 0.20 m | goal spacing |

### `sampling_c3plus_options.yaml` (parts relevant here)
`workspace_limits` x [0.23, 0.80], y [−0.507, 0.507], z [−0.04, 0.30]; `robot_radius_limits`
[0.28, 0.75]; `workspace_margins` 0.03; `ee_velocity_limits` ±0.14 m/s;
`u_horizontal_limits` ±10 N; `planning_dt_position/pose` 0.075 s; `N` † 7; `num_outer_threads` † 5,
`num_threads` 5; `include_walls` † false (n=4); `use_predicted_x0_c3/repos` true.

### `sim_params.yaml`
`dt` 0.001, `realtime_rate` 1, `object_publish_rate` 10 Hz (object state at 10 Hz — the
controller's object estimate is up to 0.1 s old), `q_init_objects` †.

---

## A5. Where a planning layer can plug in without touching C3+

Constraint from the code: the controller treats `x_lcs_final_des` as *the* goal for progress
(§A1.4), goal-change resets (§A2) and the pose switch; it treats `x_lcs_des` as the C3 reference.
So **sub-goals must go through `x_lcs_des`, never through the final-target port**, or every
sub-goal would wipe the buffers and reset the mode. Conversely, progress is measured toward the
**final** goal: a sub-goal that temporarily moves an object away from its final goal (e.g. to keep
it reachable) will be cut off as "unproductive" by `kConfigCostDrop`. Level 3 will need either
sub-goals that monotonically reduce final-goal cost, or a guarded option to measure progress
against `x_lcs_des`.

### (a) Sample generation and acceptance (levels 1–2)
1. `SampleIsAcceptable` (`gen.cc:167-185`): add a third, parameter-guarded condition
   (e.g. "pushing from here keeps the object inside the reach set" / "lies in a useful motion
   cone"). Smallest change; applies to every strategy. Needs an attempt cap because the outer
   `do-while` (`gen.cc:148-157`) has none.
2. Object choice in `MeshNormalSamplingMultiObject` (`gen.cc:611-633`): replace the uniform object
   pick by planner weights / the planner's current object. Needs a new input (e.g. a weight
   vector) threaded through `GenerateSampleStates` — small signature change in their code.
3. Between generation and the C3 solves in `ComputePlan` (`ctrl.cc:883-889` → `:921`):
   over-generate, then filter/rank candidates (reach, motion cone) and keep the best k. This is
   where wasted C3 solves are saved; it is controller code, so it needs a guarded call into our
   module under `examples/sampling_c3/reachability/`.
4. Cost shaping next to the travel cost (`ctrl.cc:1050-1054`): add a guarded reachability
   penalty to `all_sample_costs_`. Does not touch the C3 solve; changes ranking and switching.
5. Workspace definition used today is EE-only (`IsSampleInWorkspace`, `gen.cc:771-798`) and,
   without walls, a box constraint on objects inside C3 (`ctrl.cc:991-1005`). Our robot-reach set
   should be a strictly better replacement for (1), not a change to C3 constraints.

### (b) Goal and sub-goal generation (level 3)
1. `GenerateLineTrajectoryWithLookahead` (`goal.cc:406-452`) is the existing sub-goal
   generator: a straight line with fixed steps. A guarded option can replace it with sub-goals
   from our planner (sequence of intermediate poses over reachable sets). Lives entirely in the
   goal generator; C3 sees it as an ordinary `x_lcs_des`.
2. Cleaner alternative: a new `LeafSystem` (our file) between the goal generator's
   `object_target` ports and `target_state_mux` (`main.cc:437-447`), enabled by a flag. It reads
   object states and final goals and outputs sub-goals; the only change in their file is
   the guarded wiring in `main.cc`.
3. Object order (multi-object): the sub-goal of objects that are "not active yet" can be set to
   their current pose (hold), so the cost focuses on the active object; combined with (a)2 so
   samples go to the active object. The goal generator's all-at-once success test (`goal.cc:154-177`)
   stays unchanged.
4. Goal sampling itself (`goal.cc:271-359`) is part of the benchmark task definition; we should
   not change it for comparisons.

---

## Plan (agreed 2026-10-13)

Each stage ends with a check that can be read from logs, so understanding is backed by evidence.

- **Stage 0 — setup.** Commit docs; `.gitignore` for `sim_logs/`; commit `run_sim.sh`.
  Slurm: Open-Research, 24 CPUs. Runs are not seedable → compare distributions over several
  runs per configuration (no change to their code for now).
- **Stage 1 — reproduce baseline, unchanged code.** Headless runner + own `lcm-logger`
  wrapper (`md_tools/run_experiment.sh`), Slurm batch (`md_tools/experiments.sbatch`),
  metrics (`md_tools/analyze_logs.py`), `md_tools/BASELINE_REPORT.md`.
  Check: single object reaches goals reliably; multi-object times follow the paper's trend
  (paper numbers are hardware; sim will not match exactly).
- **Stage 2 — learn each mechanism by switching it off** (parameter changes on copies only):
  sample buffer, unsuccessful buffer, progress window, hysteresis, lookahead step, sample count.
  Check: one NOTES entry per ablation with metrics.
- **Stage 3a — re-implement the decision layer in Python** (`md_tools/replay/`): sampling
  acceptance, cost ranking, hysteresis, progress test, buffers, lookahead sub-goal, goal success.
  Driven by logged states and logged C3 costs. Check: identical `mode_switch_reason`,
  best-sample index and sub-goal at every logged step.
- **Stage 3b — re-implement C3+ on a toy problem** (one object, one contact): LCS, ADMM,
  C3+ projection. Check: plan matches theirs for logged states.
- **Stage 4 — our levels 1 → 2 → 3**, each behind a parameter, compared against Stage 1.

## Log

- 2026-10-13: Stage 1 tooling. `run_experiment.sh` (headless run, own LCM port, params
  snapshot/restore under a cluster-wide `flock`, restore as soon as the controller has loaded
  its config), `analyze_logs.py`, `experiments.sbatch` (array, one config per task, analysis
  inside the job). Pitfalls hit and fixed:
  - `srun` from inside the interactive job tries to add a step to that 4-CPU job → use `sbatch`.
  - `lcm-logger` rejects `-l`; use `--lcm-url=`.
  - Logging straight to Lustre (~2.9 MB/s, ~790k msgs per 3 min) is too slow: the logger's
    100 MB queue filled and it **silently dropped** everything after ~36 s. Now logs to node-local
    `/tmp` and moves the file at the end; `analyze_logs.py` flags logs covering < 90% of the run.
  - Log size ≈ 165 MB/min (≈ 4 GB per 25-min run).
- 2026-10-13: Smoke test, 1 object `G_shape_video`, 3 min, crannog04, 24 CPUs
  (`~/pa_runs/smoke_G_3min`): 5 goals, time to goal mean 33.3 s (median 32.6, 25.4–40.8);
  paper hardware single-object mean ≈ 31 s. Loop period median 69 ms (p95 112 ms), real-time
  factor 1.00, no clock resets (vs. the 4-CPU run: >1 s steps). Time in C3 63%. Switches: 34×
  reason 4 (unproductive) and 34× reason 2 (reached repos target), 0× reasons 1/3.
  Observation from `timeline.png`: per goal, the coarse move takes ~5 s; most of the time is a
  flat fine-alignment phase with repeated C3 → unproductive → reposition cycles.
- 2026-10-13: Part A written. No code changed. Findings §0.1–0.7 need decisions before Part B
  (restore tracked YAMLs after runs; no seeding; fix CPU count per run; own logger wrapper).
