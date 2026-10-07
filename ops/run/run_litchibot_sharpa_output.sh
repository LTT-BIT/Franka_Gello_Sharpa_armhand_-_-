#!/usr/bin/env bash
# Deprecated and disabled: production uses the original Terminal 2 entry.
set -euo pipefail
echo "DEPRECATED AND DISABLED: use run_sharpa_hands_cyclonedds.sh --hand-source litchibot --dry-run. No second Sharpa driver is permitted." >&2
exit 2
if [[ "${1:-}" != "--enable-output" ]]; then
  echo "Hardware output is disabled. Requires explicit --enable-output." >&2
  exit 2
fi
shift
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO_ROOT/ops/lib/deployment_env.sh"
LITCHI_REPO="$(deployment_env_value "$REPO_ROOT/docker/.env" LITCHI_REPO)"
CPUSET="$(deployment_env_value "$REPO_ROOT/docker/.env" HOUSEKEEPING_CPUSET)"
DOMAIN="$(deployment_env_value "$REPO_ROOT/docker/.env" TELEOP_ROS_DOMAIN_ID)"
[[ -d "$LITCHI_REPO" && -f "$LITCHI_REPO/ros2/install/setup.bash" ]] || {
  echo "Existing litchi_hardware ROS installation missing: $LITCHI_REPO" >&2; exit 1;
}
[[ -n "$CPUSET" ]] || { echo "HOUSEKEEPING_CPUSET missing" >&2; exit 1; }
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$REPO_ROOT/config/cyclonedds.xml"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${DOMAIN:-0}"
cd "$LITCHI_REPO"
exec taskset -c "$CPUSET" pixi run -e ros2 bash -c \
  'source ros2/install/setup.bash; exec ros2 launch "$@"' -- \
  "$REPO_ROOT/adapters/litchibot/sharpa_output.launch.py" enable_output:=true "$@"
