#!/bin/bash
# Baseline experiments: one configuration per array task, run in parallel.
#
#   sbatch md_tools/experiments.sbatch            # all configurations
#   sbatch --array=0-2 md_tools/experiments.sbatch  # a subset
#
# Each task runs md_tools/run_experiment.sh on its own node allocation and LCM port.
# Outputs: ~/pa_runs/baseline/<tag>_<objects>_<jobid>_<task>/
#
#SBATCH -J pa_base
#SBATCH -p Open-Research
#SBATCH --cpus-per-task=24
#SBATCH --mem=16G
#SBATCH --time=00:45:00
#SBATCH --array=0-5
#SBATCH -o /home/%u/pa_runs/baseline/slurm_%A_%a.out

# tag | base_names | minutes
CONFIGS=(
  "1obj|G_shape_video|15"
  "1obj|T_shape_video|15"
  "1obj|H_shape_texture|15"
  "2obj|H_shape_texture,I_shape_texture|25"
  "3obj|H_shape_texture,E_shape_video,N_shape_video|25"
  "4obj|G_shape_video,R_shape_texture,A_shape_video,S_shape_texture|25"
)

IFS='|' read -r TAG NAMES MINUTES <<< "${CONFIGS[$SLURM_ARRAY_TASK_ID]}"
SHORT=$(echo "$NAMES" | sed 's/_shape_[a-z]*//g; s/_video//g; s/,/-/g')
OUT="$HOME/pa_runs/baseline/${TAG}_${SHORT}_${SLURM_ARRAY_JOB_ID}_${SLURM_ARRAY_TASK_ID}"
PA_LCM_PORT=$(( 8000 + SLURM_ARRAY_TASK_ID )) \
  "$HOME/repos/dairlib/md_tools/run_experiment.sh" "$NAMES" "$MINUTES" "$OUT"

# Analyse here, inside the allocation (writes metrics.json and PNGs into $OUT).
apptainer exec "$HOME/containers/push-anything.sif" bash -lc \
  "cd $HOME/repos/dairlib && python3 md_tools/analyze_logs.py '$OUT' --plots"
