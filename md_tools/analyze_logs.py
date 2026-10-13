"""Baseline metrics from one or more run directories made by run_experiment.sh.

Usage (inside the container):
  python3 md_tools/analyze_logs.py <run_dir> [<run_dir> ...] [--plots]

Writes <run_dir>/metrics.json (and PNGs with --plots) and prints a summary.

Sources:
  SAMPLING_C3_DEBUG  one message per control loop: mode, switch reason, goal changes
  C3_ACTUAL          object poses (lcmt_c3_state with state names)
  C3_FINAL_TARGET    final goals
"""
import argparse
import json
import math
import os
import os.path as op
import sys

import numpy as np
import yaml

REPO = op.abspath(op.join(op.dirname(__file__), ".."))
sys.path.append(op.join(REPO, "bazel-bin", "lcmtypes"))
import dairlib  # noqa: E402
from lcm import EventLog  # noqa: E402

REASONS = {1: "to_c3_lower_cost", 2: "to_c3_reached_repos_target",
           3: "to_repos_lower_cost_sample", 4: "to_repos_c3_unproductive",
           5: "to_c3_xbox"}
DECODERS = {"SAMPLING_C3_DEBUG": dairlib.lcmt_sampling_c3_debug,
            "C3_ACTUAL": dairlib.lcmt_c3_state,
            "C3_FINAL_TARGET": dairlib.lcmt_c3_state}


def stats(x):
  x = np.asarray(x, dtype=float)
  if x.size == 0:
    return {"n": 0}
  return {"n": int(x.size), "mean": float(x.mean()), "median": float(np.median(x)),
          "std": float(x.std()), "min": float(x.min()), "max": float(x.max())}


def read_log(path):
  out = {k: [] for k in DECODERS}
  for ev in EventLog(path, "r"):
    if ev.channel in DECODERS:
      # (wall time [s], decoded message)
      out[ev.channel].append((ev.timestamp * 1e-6, DECODERS[ev.channel].decode(ev.data)))
  return out


def load_limits(run_dir):
  """EE workspace from the run's own copy of the C3 options."""
  ctrl = yaml.safe_load(open(op.join(run_dir, "params", "sampling_c3_controller_params.yaml")))
  opts = yaml.safe_load(open(op.join(run_dir, "params",
                                     op.basename(ctrl["sampling_c3_options_file"]))))
  wl = opts["workspace_limits"]
  return {"x": (wl[0][3], wl[0][4]), "y": (wl[1][3], wl[1][4]),
          "r": tuple(opts["robot_radius_limits"]), "margin": opts["workspace_margins"]}


def dist_to_region_edge(x, y, lim, margin):
  """Signed distance of (x, y) to the edge of the EE region shrunk by margin
  (box ∩ annulus). Positive = inside. This is the region where samples may lie
  (IsSampleInWorkspace); a proxy for 'pushable', refined later by level 1."""
  r = math.hypot(x, y)
  d = [x - (lim["x"][0] + margin), (lim["x"][1] - margin) - x,
       y - (lim["y"][0] + margin), (lim["y"][1] - margin) - y,
       r - (lim["r"][0] + margin), (lim["r"][1] - margin) - r]
  return min(d)


def analyze(run_dir, plots=False):
  meta = dict(l.strip().split("=", 1) for l in open(op.join(run_dir, "meta.txt")) if "=" in l)
  log = read_log(op.join(run_dir, "lcmlog"))
  dbg = log["SAMPLING_C3_DEBUG"]
  if len(dbg) < 2:
    return {"run": run_dir, "meta": meta, "error": "fewer than 2 SAMPLING_C3_DEBUG messages"}

  wall = np.array([w for w, _ in dbg])
  t = np.array([m.utime for _, m in dbg]) * 1e-6           # sim time of each loop
  c3 = np.array([m.is_c3_mode for _, m in dbg], dtype=bool)
  reason = np.array([m.mode_switch_reason for _, m in dbg])
  goals = np.array([m.detected_goal_changes for _, m in dbg])
  pos_err = np.array([m.current_pos_error for _, m in dbg])
  rot_err = np.array([m.current_rot_error for _, m in dbg])
  dt = np.diff(t, append=t[-1])                             # duration of each loop step

  # The controller's clock can be reset by LcmDrivenLoop; keep it monotone.
  resets = int(np.sum(np.diff(t) < 0))
  dt = np.clip(dt, 0, None)
  duration = float(dt.sum())

  # ---- goals: detected_goal_changes is 0 after start-up, +1 per completed goal.
  inc = np.flatnonzero(np.diff(goals) > 0) + 1
  bounds = [0] + list(inc)
  times_to_goal = [t[b] - t[a] for a, b in zip(bounds[:-1], bounds[1:])]
  unfinished = float(t[-1] - t[bounds[-1]])

  # ---- modes and switch reasons.
  reason_counts = {REASONS[k]: int(np.sum(reason == k)) for k in REASONS}
  repos_per_goal = []
  segs = bounds + [len(t)]
  for a, b in zip(segs[:-1], segs[1:]):
    repos_per_goal.append(int(np.sum(np.isin(reason[a:b], [3, 4]))))

  # C3 episodes: from entering C3 to the switch out, grouped by exit reason.
  c3_episodes = {"unproductive": [], "lower_cost_sample": [], "open_at_end": []}
  start = 0 if c3[0] else None
  for i in range(1, len(t)):
    if c3[i] and not c3[i - 1]:
      start = i
    elif not c3[i] and c3[i - 1] and start is not None:
      key = {4: "unproductive", 3: "lower_cost_sample"}.get(int(reason[i]), "other")
      c3_episodes.setdefault(key, []).append(float(t[i] - t[start]))
      start = None
  if start is not None:
    c3_episodes["open_at_end"].append(float(t[-1] - t[start]))

  # ---- compute per control loop.
  loop_wall = np.diff(wall)
  loop_sim = np.diff(t)[np.diff(t) > 0]

  # ---- time per goal spent in pose-tracking mode (after Σ xy error < 0.05·n).
  pose = np.array([m.in_pose_tracking_mode for _, m in dbg], dtype=bool)
  pose_frac_per_goal = []
  for a, b in zip(bounds[:-1], bounds[1:]):
    pose_frac_per_goal.append(float(dt[a:b][pose[a:b]].sum() / max(dt[a:b].sum(), 1e-9)))

  # ---- object distances to the edge of the EE sampling region.
  lim = load_limits(run_dir)
  act = log["C3_ACTUAL"]
  reach = {}
  per_object = {}
  stall = {}
  if act:
    names = list(act[0][1].state_names)
    n_obj = sum(1 for s in names if s.startswith("object_x_"))
    ta = np.array([m.utime for _, m in act]) * 1e-6
    A = np.array([m.state for _, m in act])

    # Per-object error to the final goal (C3_FINAL_TARGET, matched by time).
    fin = log["C3_FINAL_TARGET"]
    tf = np.array([m.utime for _, m in fin]) * 1e-6
    F = np.array([m.state for _, m in fin])
    jf = np.clip(np.searchsorted(tf, ta, side="right") - 1, 0, len(tf) - 1)
    for i in range(n_obj):
      p = [names.index(f"object_{c}_{i}") for c in ("x", "y")]
      q = [names.index(f"object_q{c}_{i}") for c in ("w", "x", "y", "z")]
      e_pos = np.linalg.norm(A[:, p] - F[jf][:, p], axis=1)
      qa = A[:, q] / np.linalg.norm(A[:, q], axis=1, keepdims=True)
      qf = F[jf][:, q] / np.linalg.norm(F[jf][:, q], axis=1, keepdims=True)
      e_rot = 2 * np.arccos(np.clip(np.abs(np.sum(qa * qf, axis=1)), 0, 1))
      per_object[f"object_{i}"] = {"final_pos_err_m": float(e_pos[-1]),
                                   "final_rot_err_rad": float(e_rot[-1]),
                                   "_series": (ta, e_pos, e_rot)}

    # Stall: 30 s windows in which no object moved > 5 mm or rotated > 0.05 rad.
    W = 30.0
    stalled_windows = 0
    edges = np.arange(ta[0], ta[-1] - W, W)
    for w0 in edges:
      sel = (ta >= w0) & (ta < w0 + W)
      if sel.sum() < 2:
        continue
      moved = False
      for i in range(n_obj):
        p = [names.index(f"object_{c}_{i}") for c in ("x", "y")]
        q = [names.index(f"object_q{c}_{i}") for c in ("w", "x", "y", "z")]
        P, Q = A[sel][:, p], A[sel][:, q]
        Q = Q / np.linalg.norm(Q, axis=1, keepdims=True)
        dpos = np.linalg.norm(P - P[0], axis=1).max()
        drot = (2 * np.arccos(np.clip(np.abs(Q @ Q[0]), 0, 1))).max()
        if dpos > 0.005 or drot > 0.05:
          moved = True
          break
      stalled_windows += 0 if moved else 1
    stall = {"window_s": W, "windows": int(len(edges)), "stalled_windows": stalled_windows,
             "frac_time_stalled": float(stalled_windows / max(len(edges), 1))}

    for i in range(n_obj):
      ix, iy = names.index(f"object_x_{i}"), names.index(f"object_y_{i}")
      d = np.array([dist_to_region_edge(m.state[ix], m.state[iy], lim, lim["margin"])
                    for _, m in act])
      outside = d < 0
      events = int(np.sum(outside[1:] & ~outside[:-1]) + outside[0])
      reach[f"object_{i}"] = {"min_dist_to_edge_m": float(d.min()),
                              "median_dist_to_edge_m": float(np.median(d)),
                              "frac_time_outside": float(outside.mean()),
                              "outside_events": events}
      if plots:
        reach[f"object_{i}"]["_series"] = (ta, d)

  # The supervised run lasts `minutes` after the controller is up; a log that
  # covers much less means the logger dropped messages.
  coverage = float(wall[-1] - wall[0]) / (60.0 * float(meta.get("minutes", "nan")))

  res = {
      "run": run_dir, "meta": meta, "log_coverage": coverage,
      "log_coverage_ok": bool(coverage > 0.9),
      "sim_duration_s": duration, "wall_duration_s": float(wall[-1] - wall[0]),
      "realtime_factor": duration / float(wall[-1] - wall[0]),
      "controller_clock_resets": resets,
      "goals_reached": int(len(times_to_goal)),
      "time_to_goal_s": stats(times_to_goal), "times_to_goal_s": times_to_goal,
      "unfinished_goal_elapsed_s": unfinished,
      "frac_time_c3": float(dt[c3].sum() / duration),
      "frac_time_repos": float(dt[~c3].sum() / duration),
      "switch_reason_counts": reason_counts,
      "repositionings_per_goal": repos_per_goal,
      "c3_episode_duration_s": {k: stats(v) for k, v in c3_episodes.items()},
      "time_in_c3_before_unproductive_switch_total_s": float(sum(c3_episodes["unproductive"])),
      "loop_period_wall_s": stats(loop_wall),
      "loop_period_wall_p95_s": float(np.percentile(loop_wall, 95)),
      "loop_period_sim_s": stats(loop_sim),
      "reach": {k: {kk: vv for kk, vv in v.items() if not kk.startswith("_")}
                for k, v in reach.items()},
      "per_object": {k: {kk: vv for kk, vv in v.items() if not kk.startswith("_")}
                     for k, v in per_object.items()},
      "stall": stall,
      "pose_mode_frac_per_goal": pose_frac_per_goal,
      "final_pos_error_m": float(pos_err[-1]), "final_rot_error_rad": float(rot_err[-1]),
  }
  if plots:
    make_plots(run_dir, t, c3, goals, pos_err, rot_err, loop_wall, reach, inc, per_object)
  with open(op.join(run_dir, "metrics.json"), "w") as f:
    json.dump(res, f, indent=1)
  return res


def make_plots(run_dir, t, c3, goals, pos_err, rot_err, loop_wall, reach, inc, per_object):
  import matplotlib
  matplotlib.use("Agg")
  import matplotlib.pyplot as plt
  fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
  for a in ax:
    a.fill_between(t, 0, 1, where=~c3, transform=a.get_xaxis_transform(),
                   color="0.85", step="post", linewidth=0, label="repositioning")
    for i in inc:
      a.axvline(t[i], color="g", lw=1)
  # Per-object errors to the final goal; dashed lines = success thresholds.
  for k, v in per_object.items():
    ta, ep, er = v["_series"]
    ax[0].plot(ta, ep, lw=1, label=k)
    ax[1].plot(ta, er, lw=1, label=k)
  ax[0].axhline(0.02, color="k", ls="--", lw=0.8); ax[1].axhline(0.1, color="k", ls="--", lw=0.8)
  ax[0].set_ylabel("pos error [m]"); ax[1].set_ylabel("rot error [rad]")
  ax[0].legend(fontsize=7)
  for k, v in reach.items():
    ta, d = v["_series"]
    ax[2].plot(ta, d, lw=1, label=k)
  ax[2].axhline(0, color="r", lw=1)
  ax[2].set_ylabel("dist to EE-region edge [m]"); ax[2].set_xlabel("sim time [s]")
  ax[2].legend(fontsize=7)
  ax[0].set_title(op.basename(run_dir.rstrip("/")) + "  (grey = repositioning, green = goal reached)")
  fig.tight_layout(); fig.savefig(op.join(run_dir, "timeline.png"), dpi=110); plt.close(fig)

  fig, a = plt.subplots(figsize=(6, 3.5))
  a.hist(loop_wall, bins=60); a.set_xlabel("control loop period, wall [s]"); a.set_ylabel("count")
  fig.tight_layout(); fig.savefig(op.join(run_dir, "loop_period.png"), dpi=110); plt.close(fig)


def main():
  p = argparse.ArgumentParser()
  p.add_argument("runs", nargs="+")
  p.add_argument("--plots", action="store_true")
  args = p.parse_args()
  for r in args.runs:
    res = analyze(r, args.plots)
    if "error" in res:
      print(f"{r}: {res['error']}"); continue
    print(f"\n== {r}  [{res['meta'].get('base_names')}]  status={res['meta'].get('status')}")
    if not res["log_coverage_ok"]:
      print(f"  !! LOG COVERS ONLY {res['log_coverage']:.0%} OF THE RUN — metrics unreliable")
    print(f"  sim {res['sim_duration_s']:.0f}s  wall {res['wall_duration_s']:.0f}s  "
          f"realtime x{res['realtime_factor']:.2f}  clock resets {res['controller_clock_resets']}")
    print(f"  goals reached {res['goals_reached']}  time-to-goal {res['time_to_goal_s']}")
    print(f"  unfinished goal elapsed {res['unfinished_goal_elapsed_s']:.1f}s  "
          f"final Σerr {res['final_pos_error_m']:.3f} m / {res['final_rot_error_rad']:.2f} rad")
    print(f"  time in C3 {res['frac_time_c3']:.2f}  repos {res['frac_time_repos']:.2f}")
    print(f"  switch reasons {res['switch_reason_counts']}")
    print(f"  repositionings per goal {res['repositionings_per_goal']}")
    lp = res["loop_period_wall_s"]
    print(f"  loop period wall: median {lp['median']*1e3:.0f} ms  "
          f"p95 {res['loop_period_wall_p95_s']*1e3:.0f} ms  max {lp['max']*1e3:.0f} ms")
    for k, v in res["reach"].items():
      po = res["per_object"].get(k, {})
      print(f"  {k}: min dist to edge {v['min_dist_to_edge_m']:.3f} m  "
            f"outside {v['frac_time_outside']:.2%} ({v['outside_events']} events)  "
            f"final err {po.get('final_pos_err_m', float('nan')):.3f} m / "
            f"{po.get('final_rot_err_rad', float('nan')):.2f} rad")
    st = res["stall"]
    print(f"  stalled (no object moved >5 mm/0.05 rad in {st['window_s']:.0f} s): "
          f"{st['stalled_windows']}/{st['windows']} windows")
    print(f"  pose-mode fraction per goal: "
          f"{[round(x, 2) for x in res['pose_mode_frac_per_goal']]}")


if __name__ == "__main__":
  main()
