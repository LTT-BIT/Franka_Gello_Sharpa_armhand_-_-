#!/usr/bin/env bash
# Independent real-glove dry-run. No ROS/GELLO/FR3/Sharpa initialization.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO_ROOT/ops/lib/deployment_env.sh"
deployment_export_litchibot_env "$REPO_ROOT/docker/.env"
PYTHON_BIN="${LITCHIBOT_PYTHON:-/home/user/litchibot_glove/.venv/bin/python}"
cd "$REPO_ROOT"
exec "$PYTHON_BIN" -m adapters.litchibot.cli --hand-source litchibot --dry-run "$@"
