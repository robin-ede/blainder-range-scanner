#!/usr/bin/env bash
set -euo pipefail

# Define variables for both Blender and its internal Python
BLENDER_BIN="${BLENDER_BIN:-/opt/blender/blender}"
# The path to Blender's bundled Python based on your Dockerfile setup
BLENDER_PYTHON="/opt/blender/5.0/python/bin/python3.11"

print_help() {
  cat <<'EOF'
Usage:
  help
  blender [args...]
  python <script.py> [args...]
  run-fixed-scene --scene <scene.blend> --config <config.json> [--summary-dir DIR]
  run-pose-noise --scene <scene.blend> --config <config.json>
  run-multi-seed --config <config.json>
  run-animated-path --base-scene <scene.blend> --config <config.json>
  visualize <report.json> [more_reports...] [--output-dir DIR]

Notes:
  - The container is designed for headless Blender execution only.
  - Mount your repo, scenes, configs, and output directories into /workspace or pass absolute mounted paths.
  - Blender inside the container is pinned to the same major/minor release as the local project setup.
EOF
}

if [ "$#" -eq 0 ]; then
  print_help
  exit 0
fi

case "$1" in
  help|-h|--help)
    print_help
    ;;
  blender)
    shift
    exec "${BLENDER_BIN}" "$@"
    ;;
  python)
    shift
    # Now uses Blender's Python instead of the system Python
    exec "${BLENDER_PYTHON}" "$@"
    ;;
  run-fixed-scene)
    shift
    exec "${BLENDER_PYTHON}" /workspace/scripts/run_fixed_scene_validation_batch.py --blender "${BLENDER_BIN}" "$@"
    ;;
  run-pose-noise)
    shift
    exec "${BLENDER_PYTHON}" /workspace/scripts/run_pose_then_noise_validation.py --blender "${BLENDER_BIN}" "$@"
    ;;
  run-multi-seed)
    shift
    exec "${BLENDER_PYTHON}" /workspace/scripts/run_multi_seed_pose_then_noise_validation.py --blender "${BLENDER_BIN}" "$@"
    ;;
  run-animated-path)
    shift
    exec "${BLENDER_PYTHON}" /workspace/scripts/run_animated_path_validation.py --blender "${BLENDER_BIN}" "$@"
    ;;
  visualize)
    shift
    exec "${BLENDER_PYTHON}" /workspace/scripts/visualize_validation_report.py "$@"
    ;;
  *)
    exec "$@"
    ;;
esac