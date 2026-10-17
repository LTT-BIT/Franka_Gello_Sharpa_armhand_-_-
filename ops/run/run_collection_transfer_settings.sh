#!/usr/bin/env bash
# Preferences only: no robot, recorder, watcher, upload, or deletion is started.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CONDA_BASE="$(conda info --base)"
export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec "$CONDA_BASE/bin/python" -m apps.operator_gui.transfer_panel
