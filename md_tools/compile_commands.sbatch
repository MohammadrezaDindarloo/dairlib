#!/bin/bash
# Generate compile_commands.json for the whole repo (editor code navigation).
#
#   sbatch md_tools/compile_commands.sbatch
#
# Shares the Bazel cache (~/.cache/bazel) with interactive sessions: run
# `bazel shutdown` in any open container session first, and do not run bazel
# elsewhere until this job ends.
#
#SBATCH -J pa_ccmds
#SBATCH -p Open-Research
#SBATCH --cpus-per-task=32
#SBATCH --mem=64G
#SBATCH --time=08:00:00
#SBATCH -o /home/%u/pa_runs/compile_commands_%j.out

set -u
REPO="$HOME/repos/dairlib"
echo "start $(date -Iseconds) on $(hostname -s), $(nproc) CPUs"
apptainer exec "$HOME/containers/push-anything.sif" bash -lc "
  cd '$REPO' &&
  bazel run //md_tools:refresh_compile_commands ;
  status=\$? ;
  bazel shutdown ;
  exit \$status"
status=$?
echo "end $(date -Iseconds), exit $status"
if [ -f "$REPO/compile_commands.json" ]; then
  echo "compile_commands.json: $(du -h "$REPO/compile_commands.json" | cut -f1)," \
       "$(grep -c '"file"' "$REPO/compile_commands.json") entries"
fi
exit $status
