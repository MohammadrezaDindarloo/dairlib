#!/bin/bash
# Headless Push Anything run with LCM logging.
#
#   md_tools/run_experiment.sh <base_names> <minutes> <out_dir>
#
#   base_names : comma-separated object names, e.g. G_shape_video or
#                G_shape_video,R_shape_texture
#   minutes    : wall-clock duration of the controlled run (sim runs at real time)
#   out_dir    : created; receives lcmlog, process logs, parameters, metadata
#
# Their tracked parameter files are rewritten by multiyaml_rewrite.py for the
# chosen objects and restored from a snapshot on exit, so the repo ends unchanged.
# Runs on the same node are serialised with a lock (they share those files).
# Optional env: PA_LCM_PORT (default: random), PA_SIF (container image).
set -u
SIF="${PA_SIF:-$HOME/containers/push-anything.sif}"

# Re-run inside the container if we are outside it.
if [ -z "${APPTAINER_NAME:-}" ]; then exec apptainer exec "$SIF" bash "$0" "$@"; fi

if [ $# -ne 3 ]; then
  echo "usage: $0 <base_names comma-separated> <minutes> <out_dir>"; exit 2
fi
BASE_NAMES="$1"; MINUTES="$2"; OUT="$(realpath -m "$3")"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
PARAMS_DIR="examples/sampling_c3/anything/parameters"
BIN="bazel-bin/examples/sampling_c3"
PORT="${PA_LCM_PORT:-$(( 8000 + RANDOM % 2000 ))}"
LCM_URL="udpm://239.255.76.67:${PORT}?ttl=0"

if [ -e "$OUT" ]; then echo "out_dir exists: $OUT"; exit 2; fi
mkdir -p "$OUT/params" "$OUT/proc"
cd "$REPO" || exit 1

# Files that multiyaml_rewrite.py may write (anything under the parameters dir).
mkdir -p "$HOME/pa_runs"
exec 9>"$HOME/pa_runs/.params.lock"
flock 9
SNAP="$(mktemp -d)"
cp -a "$PARAMS_DIR/." "$SNAP/"

PIDS=()
restore_params() {                         # restore their tracked files (idempotent)
  [ -d "$SNAP" ] || return 0
  cp -a "$SNAP/." "$REPO/$PARAMS_DIR/"
  rm -rf "$SNAP"
  flock -u 9
}
cleanup() {
  for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 3
  for p in "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null; done
  wait 2>/dev/null
  restore_params
  if [ -n "${LOCAL_LOG:-}" ] && [ -f "$LOCAL_LOG/lcmlog" ]; then
    mv "$LOCAL_LOG/lcmlog" "$OUT/lcmlog" && rmdir "$LOCAL_LOG"
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# ---- select objects and apply their per-object-count configuration ----------
python3 - "$PARAMS_DIR/sampling_c3_controller_params.yaml" "$BASE_NAMES" <<'EOF' || exit 1
import re, sys
path, names = sys.argv[1], [n.strip() for n in sys.argv[2].split(",") if n.strip()]
assert 1 <= len(names) <= 4, "1-4 objects"
text = open(path).read()
new, n = re.subn(r"(?m)^base_names:.*$", "base_names: [" + ", ".join(names) + "]", text)
assert n == 1, "expected exactly one active base_names line"
open(path, "w").write(new)
EOF
python3 examples/sampling_c3/multiyaml_rewrite.py > "$OUT/proc/rewrite.log" 2>&1 \
  || { echo "multiyaml_rewrite.py failed, see $OUT/proc/rewrite.log"; exit 1; }
cp -a "$PARAMS_DIR/." "$OUT/params/"
cp -a examples/sampling_c3/shared_parameters "$OUT/params/shared_parameters"

# ---- metadata ---------------------------------------------------------------
{
  echo "base_names=$BASE_NAMES"
  echo "minutes=$MINUTES"
  echo "lcm_url=$LCM_URL"
  echo "git_commit=$(git rev-parse HEAD)"
  echo "git_branch=$(git rev-parse --abbrev-ref HEAD)"
  echo "host=$(hostname -s)"
  echo "slurm_job=${SLURM_JOB_ID:-none}"
  echo "cpus_allocated=$(nproc)"
  echo "start_wall=$(date -Iseconds)"
} > "$OUT/meta.txt"
git status --porcelain > "$OUT/git_status.txt"
git diff HEAD > "$OUT/git_diff.patch"

# ---- launch -----------------------------------------------------------------
# Log to node-local disk: writing ~2 MB/s straight to Lustre is too slow, the
# logger's queue fills and it silently drops messages. Moved to $OUT on exit.
LOCAL_LOG="$(mktemp -d /tmp/pa_run.XXXXXX)"
lcm-logger --lcm-url="$LCM_URL" --flush-interval=1000 -f "$LOCAL_LOG/lcmlog" \
  > "$OUT/proc/logger.log" 2>&1 &
PIDS+=($!); LOGGER=$!
sleep 1
"$BIN/franka_sim" --demo_name=anything --lcm_url="$LCM_URL" > "$OUT/proc/sim.log" 2>&1 &
PIDS+=($!); SIM=$!
"$BIN/franka_osc_controller" --is_simulation=true --demo_name=anything \
  --lcm_url="$LCM_URL" > "$OUT/proc/osc.log" 2>&1 &
PIDS+=($!); OSC=$!
sleep 1
"$BIN/franka_sampling_c3_controller" --is_simulation=true --demo_name=anything \
  --lcm_url="$LCM_URL" > "$OUT/proc/c3.log" 2>&1 &
PIDS+=($!); C3=$!

# Every process reads its parameters at start-up. Once the controller has built
# its loop (last start-up step) restore their files and release the lock, so
# parallel runs on other nodes can proceed and the repo is dirty only briefly.
for i in $(seq 1 120); do
  grep -q "constructed loop" "$OUT/proc/c3.log" 2>/dev/null && break
  kill -0 "$C3" 2>/dev/null || break
  sleep 1
done
restore_params

# ---- supervise until time is up or a process dies ---------------------------
END=$(( $(date +%s) + MINUTES * 60 ))
STATUS="completed"
while [ "$(date +%s)" -lt "$END" ]; do
  for name in SIM OSC C3 LOGGER; do
    if ! kill -0 "${!name}" 2>/dev/null; then
      wait "${!name}"; code=$?
      STATUS="died:${name,,}:exit=${code}:after_s=$(( $(date +%s) - END + MINUTES * 60 ))"
      break 2
    fi
  done
  sleep 2
done
{
  echo "status=$STATUS"
  echo "end_wall=$(date -Iseconds)"
} >> "$OUT/meta.txt"
echo "[$BASE_NAMES] $STATUS -> $OUT"
