# CLAUDE.md — reachability-aware planning on top of Push Anything

## 1. What this project is

- Owner: Mohammadreza (PhD student, University of Edinburgh). Background in robotics and
  optimisation; prefers clear explanations, minimal and clean code changes, and evidence
  before claims.
- Base work: **Push Anything** (Bui, Gao, Yang, …, Posa; ICRA 2026; arXiv:2510.19974),
  DAIR Lab, UPenn. Non-prehensile planar pushing of 1–4 objects with a Franka arm using
  **C3+** (contact-implicit MPC) plus a sampling layer that chooses where to push.
- Paper results (hardware): single object 700/701 success, ~31 s mean time to goal;
  multi-object success 98% (2), 96.8% (3), 79.3% (4); mean time to goal ~96 s (2),
  191 s (3), 316 s (4). All failures: an object was pushed out of the robot's reach.
  Stated limitations: "lacks high-level planning", inefficient as task complexity grows.
  Their future work: "combine our method with higher-level reasoning".
- **Our goal:** add a reachability-aware *planning layer* on top of their controller to make
  planning more efficient and more reasonable, WITHOUT changing their C3+ solver, their
  perception or their core controller logic, and (for now) WITHOUT any learning. Levels:
  1. Robot-reach set: offline region of the table the pusher can reach at pushing height,
     shrunk by object size; used to reject samples and keep objects pushable.
  2. Motion cones: for each candidate contact, the set of object motions a quasi-static push
     can produce; used to filter and rank candidate push locations before spending a C3+ solve.
  3. High-level planner: a sequence of intermediate object poses (plus pushing face) over
     reachable sets, given to C3+ as sub-goals; for several objects, also the object order.

  Success is measured by efficiency (time to goal, number of repositionings, wasted samples,
  compute per step, multi-object success), not single-object success (already ~100%).

## 2. Git rules

- Fork: `origin = https://github.com/MohammadrezaDindarloo/dairlib.git`.
- Work ONLY on branch `md-reachability`. `git push` is configured to push only this branch.
- Never modify, commit to or push `push_anything_dev` (their branch, our baseline).
- Do not commit logs, `sim_logs/`, `*.sif`, build outputs or large data.
- Small, descriptive commits. Never force-push without asking.

## 3. Compute environment (important)

- Edinburgh Informatics cluster. Head node `mlp1.inf.ed.ac.uk` (internal name `stanger`).
  Compute nodes (e.g. `crannog05`) are reached only through Slurm (`srun`/`sbatch`); direct
  SSH into compute nodes is forbidden. No sudo anywhere.
- Heavy work (builds, simulations) must run inside a Slurm job, not on the head node.
- All software runs inside an Apptainer image (the authors' Docker image):
  `~/containers/push-anything.sif` (Ubuntu 24.04, Drake prerequisites, Bazel, Java, LCM,
  procman, Gurobi, SNOPT). The repo is `~/repos/dairlib` on the host and is visible inside
  the container.
- Run any command in the container non-interactively with:
  `apptainer exec ~/containers/push-anything.sif bash -lc "cd ~/repos/dairlib && <command>"`.
  Inside the container, `echo $APPTAINER_NAME` prints `push-anything.sif`.
- Python: the container's Python already has trimesh, ruamel.yaml, lxml, etc. For extra
  packages use a venv that sees the container's packages:
  `python3 -m venv --system-site-packages ~/venvs/pa` (create and activate inside the container).
- Never compile C++ outside the container; never `apt install`.
- Slurm defaults for simulations: partition `Open-Research` (crannog, 64 CPUs/node),
  `--cpus-per-task=24`, no GPU. The controller needs ~25 threads; a 4-CPU job starves it
  and invalidates results.
- Experiment outputs (LCM logs, plots) go to `~/pa_runs/`, outside the repo.
- Each run uses its own LCM port (`--lcm_url=udpm://239.255.76.67:<port>?ttl=0`) so
  runs cannot hear other LCM traffic on the node.
- Plan and stage status: `md_tools/NOTES.md` → "Plan".

## 4. Build

`bazel build //examples/sampling_c3/... --jobs=16`
Incremental rebuilds are fast. New C++ files need entries in the nearby `BUILD.bazel`.
Binaries appear under `bazel-bin/examples/sampling_c3/`.

## 5. Running the simulation

- `run_sim.sh` (repo root, ours) launches the stack without procman, writes logs to
  `sim_logs/`, finds the Meshcat port and prints copy-paste instructions for viewing it from
  the laptop (Slurm relay through the head node). Equivalent to their procman script
  `start_experiment_no_logs` in `examples/sampling_c3/anything/franka_sim_anything.pmd`.
- Processes (all take `--demo_name=anything`; controllers also `--is_simulation=true`):
  - `franka_sim`: Drake simulation of arm and objects.
  - `franka_osc_controller`: operational-space controller tracking the planned EE motion.
  - `franka_sampling_c3_controller`: the method (sampling, C3+, mode switching).
  - `franka_visualizer`: Meshcat (for a human; Claude Code cannot see it).
- Before running, `python3 examples/sampling_c3/multiyaml_rewrite.py` applies the object
  selection (`base_names`) to all configs; `run_sim.sh` does this.
- Other demos: `push_t`, `jacktoy` (folders under `examples/sampling_c3/`).
- **Claude Code cannot see Meshcat.** Judge runs only from LCM logs and text logs.
- The processes talk over LCM (UDP multicast) on one node; this works on crannog.

## 6. Code map

- `examples/sampling_c3/anything/`: the Push Anything demo (`parameters/*.yaml`, `*.pmd`).
  - `sampling_c3_controller_params.yaml`: `base_names` (exactly one active line, 1–4
    objects; 44 object models in `examples/sampling_c3/urdf/`) and links to the other files.
  - `goal_params.yaml`: goal mode (0 random, 1 orientation sequence, 2 fixed), success
    thresholds (0.02 m, 0.1 rad), goal sampling limits, and the existing **lookahead
    sub-goal** (`lookahead_step_size`, `lookahead_angle`, `angle_hysteresis`). Natural place
    for our high-level planner.
  - `sampling_params.yaml`, `reposition_params.yaml`, `progress_params*.yaml`,
    `sampling_c3*_options.yaml`, `sim_params.yaml`, `vis_params.yaml`.
- `examples/sampling_c3/generate_samples.{h,cc}`: candidate EE samples
  (`GenerateSampleStates`, `SampleIsAcceptable`, `IsSampleInWorkspace`,
  `SampleAvoidsBadSpots`, `ProjectSampleOutsideObject`). Hook for levels 1–2.
- `examples/sampling_c3/goal_generator.{h,cc}`: goals and lookahead sub-goals. Hook for level 3.
- `examples/sampling_c3/reposition.{h,cc}`: repositioning (contact-free) motion.
- `examples/sampling_c3/franka_sampling_c3_controller.cc` and
  `systems/controllers/sampling_based_c3_controller.{h,cc}`: the controller and mode logic.
- `lcmtypes/lcmt_sampling_c3_debug.lcm`: per-step debug message on `SAMPLING_C3_DEBUG` with
  `is_c3_mode`, `mode_switch_reason` (0 none; 1 to C3, lower cost; 2 to C3, reached
  repositioning target; 3 to repositioning, found lower-cost sample; 4 to repositioning, C3
  unproductive; 5 to C3, forced by xbox), `source_of_pursued_target`,
  `detected_goal_changes`, `best_progress_steps_ago`, `lowest_cost`, `current_pos_error`,
  `current_rot_error`, …. Also `C3_FINAL_TARGET`, `C3_TARGET` and `C3_ACTUAL` (`lcmt_c3_state`).
- `examples/sampling_c3/start_logging.py`, `process_lcm_logs.py`,
  `test/lcm_log_loader.cc`, `cost_visualization.py`: their logging and analysis tools.

- **C3 / C3+ is an external Bazel module, not in this repo**: `MODULE.bazel:90-95` pins
  `github.com/DAIRLab/c3` at commit `5c08cb2e`. Source (read-only, in the Bazel cache):
  `bazel-dairlib/external/c3+/` — `core/` (`c3.cc`, `c3_plus.cc`, `lcs.cc`, `traj_eval.cc`),
  `multibody/` (`lcs_factory`), `systems/`. Included as `#include "c3/core/..."`.

## 7. How to work

- Read and understand before editing. Prefer adding new files over changing theirs.
  Our new code goes under `examples/sampling_c3/reachability/`; our scripts and analysis
  under `md_tools/` at the repo root.
- Any change to their files must be minimal, guarded by a parameter whose default keeps the
  original behaviour, and listed in `md_tools/NOTES.md`.
- Always compare against the unmodified baseline with the same objects, goals, seeds and
  durations. Report negative or neutral results honestly.
- Keep a running log of decisions and findings in `md_tools/NOTES.md`.
- Ask before: changing C3+ or solver code, changing perception, deleting files, long
  (> 2 h) compute, or anything irreversible.

## 8. Verified facts that refine the above (see md_tools/NOTES.md §0 for details)

- `multiyaml_rewrite.py` rewrites **tracked** YAML files in place and also overwrites tuning
  parameters per object count (horizon N, weights, sample counts, walls, progress window).
  The committed YAMLs equal its output for the 4-object default `base_names`. Any run with
  other objects must restore the tracked files afterwards (`git checkout -- <files>`).
- `lcm-logger` exists in the container at `/usr/bin/lcm-logger`; `/opt/lcm/...` does not.
  `start_logging.py` is also broken independently (reads a removed `object_model` key).
- All randomness (samples, goals) uses `std::random_device`; runs are **not seedable**
  without a code change. Sim runs at real-time rate and the controller is LCM-driven, so
  results depend on CPU speed: compare baseline and ours on the same Slurm node type and CPU count.
