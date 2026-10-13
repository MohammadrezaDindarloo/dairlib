#!/bin/bash
# Push Anything simulation without procman + copy-paste viewer instructions.
# Ctrl-C stops everything (simulation and tunnel).
SIF="$HOME/containers/push-anything.sif"
GATEWAY_ALIAS="${PA_SSH_GATEWAY:-mlp1}"            # how YOUR LAPTOP reaches the head node (~/.ssh/config)
HEAD_HOST="${PA_HEAD_HOST:-mlp1.inf.ed.ac.uk}"     # head node as seen from the cluster

# Re-run inside the container automatically if we are outside it
if [ -z "$APPTAINER_NAME" ]; then exec apptainer exec "$SIF" bash "$0" "$@"; fi

cd "$(dirname "$0")"
mkdir -p sim_logs
python3 examples/sampling_c3/multiyaml_rewrite.py || exit 1
trap 'kill $(jobs -p) 2>/dev/null' EXIT

bazel-bin/examples/sampling_c3/franka_sim --demo_name=anything                                         > sim_logs/sim.log 2>&1 &
bazel-bin/examples/sampling_c3/franka_osc_controller --is_simulation=true --demo_name=anything         > sim_logs/osc.log 2>&1 &
bazel-bin/examples/sampling_c3/franka_visualizer --is_simulation=true --demo_name=anything             > sim_logs/vis.log 2>&1 &
sleep 1
bazel-bin/examples/sampling_c3/franka_sampling_c3_controller --is_simulation=true --demo_name=anything > sim_logs/c3.log 2>&1 &

# ---- wait until Meshcat is listening; read its port from the log -------------
PORT=""
for i in $(seq 1 60); do
  PORT=$(grep -oE 'localhost:[0-9]+' sim_logs/vis.log 2>/dev/null | head -1 | cut -d: -f2)
  if [ -n "$PORT" ] && (echo > /dev/tcp/127.0.0.1/$PORT) 2>/dev/null; then break; fi
  PORT=""; sleep 0.5
done
if [ -z "$PORT" ]; then
  echo "Meshcat did NOT start. Check:  tail -n 30 sim_logs/vis.log sim_logs/c3.log"
  wait; exit 1
fi

NODE=${SLURMD_NODENAME:-$(hostname -s)}
LINE="=============================================================================="
rand_port() { echo $(( 20000 + RANDOM % 40000 )); }   # 20000-59999, avoids 7000 (macOS AirPlay)

if [ -n "$SLURM_JOB_ID" ]; then
  # ---- Slurm compute node: relay through the head node on a random free port ----
  RELAY=""; TUNNEL_OK=no
  if command -v ssh >/dev/null; then
    for try in 1 2 3 4 5; do
      CAND=$(rand_port)
      ssh -o BatchMode=yes -o ExitOnForwardFailure=yes -o ConnectTimeout=10 \
          -N -R ${CAND}:localhost:${PORT} ${HEAD_HOST} > sim_logs/tunnel.log 2>&1 &
      PID=$!; sleep 3
      if kill -0 $PID 2>/dev/null; then RELAY=$CAND; TUNNEL_OK=yes; break; fi
      grep -qiE "permission denied|password|host key" sim_logs/tunnel.log && break   # login problem, not a busy port
    done
  fi
  RELAY=${RELAY:-$(rand_port)}

  echo "$LINE"
  echo "  Simulation running on ${NODE}, Slurm job ${SLURM_JOB_ID}. Meshcat port ${PORT}."
  echo "  Logs: sim_logs/    Ctrl-C in THIS terminal stops everything."
  echo "$LINE"
  echo
  echo "  TERMINAL A (this one): the simulation. Leave it running."
  echo
  if [ "$TUNNEL_OK" = yes ]; then
    echo "  TERMINAL B: not needed -- the relay ${NODE}:${PORT} -> ${HEAD_HOST}:${RELAY}"
    echo "              was started automatically."
  else
    echo "  TERMINAL B: a NEW terminal on the head node (e.g. new VS Code terminal tab)."
    echo "    1) attach to your job on ${NODE}:"
    echo
    echo "         srun --jobid=${SLURM_JOB_ID} --overlap --pty bash"
    echo
    echo "    2) when the prompt shows ${NODE} (do NOT open the container), run and leave open:"
    echo
    echo "         ssh -N -o ExitOnForwardFailure=yes -R ${RELAY}:localhost:${PORT} ${HEAD_HOST}"
    echo
    echo "       (type your password if asked; it prints nothing when working."
    echo "        'remote port forwarding failed' = port taken -> rerun this script.)"
  fi
  echo
  echo "  TERMINAL C: on YOUR LAPTOP, run and leave open:"
  echo
  echo "         ssh -N -L ${RELAY}:localhost:${RELAY} ${GATEWAY_ALIAS}"
  echo
  echo "     then open in your browser:   http://localhost:${RELAY}"
  echo "     (instead of Terminal C you may use VS Code PORTS panel -> Forward a Port -> ${RELAY})"
else
  # ---- directly on a machine you SSH into: random local port on the laptop ----
  LOCAL=$(rand_port)
  echo "$LINE"
  echo "  Simulation running on ${NODE} (no Slurm job). Meshcat port ${PORT}.  Logs: sim_logs/"
  echo "$LINE"
  echo "  TERMINAL A (this one): the simulation. Leave it running."
  echo
  echo "  TERMINAL B: on YOUR LAPTOP, run and leave open:"
  echo
  echo "         ssh -N -L ${LOCAL}:localhost:${PORT} ${GATEWAY_ALIAS}"
  echo
  echo "     then open in your browser:   http://localhost:${LOCAL}"
fi
echo
echo "------------------------------------------------------------------------------"
echo "  Blank page or 'Can \"Upgrade\" only to \"WebSocket\"'?  Append /static/ to the URL."
echo "  To stop: Ctrl-C here, then Ctrl-C in the other terminals."
echo "$LINE"
wait
