#!/usr/bin/env bash
# Unified CLI with an isolated USB-capable overlay of the existing teleop env.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON_BIN="${LITCHIBOT_TELEOP_PYTHON:-/home/user/litchibot_glove/.venv-teleop/bin/python}"
source "$REPO_ROOT/ops/lib/deployment_env.sh"
CPUSET="$(deployment_env_value "$REPO_ROOT/docker/.env" HOUSEKEEPING_CPUSET)"
DATA_ROOT="$(deployment_env_value "$REPO_ROOT/docker/.env" TELEOP_DATA_ROOT)"
[[ -n "$CPUSET" && -n "$DATA_ROOT" ]] || {
  echo "HOUSEKEEPING_CPUSET/TELEOP_DATA_ROOT missing from docker/.env" >&2; exit 1;
}
export PYTHONPATH="$REPO_ROOT:$REPO_ROOT/adapters/pico/src:$REPO_ROOT/adapters/manus/python${PYTHONPATH:+:$PYTHONPATH}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
cd "$REPO_ROOT"
exec taskset -c "$CPUSET" "$PYTHON_BIN" -m teleop_runtime.cli \
  --config config/modes/pico.yaml --arm-source gello --hand-source litchibot \
  --preset-data-root "$DATA_ROOT" "$@"
