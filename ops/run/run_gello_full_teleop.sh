#!/usr/bin/env bash
# Host orchestration only. Old arms/hands wrappers own their existing runtimes.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
CONDA_BASE="$(conda info --base)"
TELEOP_CONDA_ENV="${TELEOP_CONDA_ENV:-gello-upper-body-teleop}"
exec "$CONDA_BASE/envs/$TELEOP_CONDA_ENV/bin/python" -m teleop_runtime.full_launcher "$@"
