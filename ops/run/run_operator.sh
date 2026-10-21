#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO_ROOT/ops/lib/deployment_env.sh"
DEPLOYMENT_ENV="$REPO_ROOT/docker/.env"
CONTROL_HOST="${TELEOP_CONTROL_HOST:-127.0.0.1}"
CONTROL_PORT="${TELEOP_CONTROL_PORT:-$(deployment_env_default "$DEPLOYMENT_ENV" TELEOP_CONTROL_PORT 5590)}"
COLLECTION_CONTROL_HOST="${COLLECTION_CONTROL_HOST:-127.0.0.1}"
COLLECTION_CONTROL_PORT="${COLLECTION_CONTROL_PORT:-$(deployment_env_default "$DEPLOYMENT_ENV" COLLECTION_CONTROL_PORT 5592)}"
export PRESET_IK_PORT="${PRESET_IK_PORT:-$(deployment_env_default "$DEPLOYMENT_ENV" PRESET_IK_PORT 5591)}"
CONDA_BASE="$(conda info --base)"
backend_pid=""
gui_pid=""

# Keep the native GELLO/MANUS backend and GUI off the isolated Franka cores.
# The checked deployment file is authoritative so an old exported shell value
# cannot silently override Docker and host processes differently.
configured_housekeeping=""
if [[ -f "$REPO_ROOT/docker/.env" ]]; then
  configured_housekeeping="$(
    awk -F= '$1 == "HOUSEKEEPING_CPUSET" {
      sub(/^[^=]*=/, ""); print; exit
    }' "$REPO_ROOT/docker/.env"
  )"
fi
if [[ -z "$configured_housekeeping" ]]; then
  echo "HOUSEKEEPING_CPUSET is missing from docker/.env" >&2
  exit 1
fi
if [[ -n "${HOUSEKEEPING_CPUSET:-}" && \
      "$HOUSEKEEPING_CPUSET" != "$configured_housekeeping" ]]; then
  echo "Exported HOUSEKEEPING_CPUSET conflicts with docker/.env:" >&2
  echo "  exported=$HOUSEKEEPING_CPUSET" >&2
  echo "  configured=$configured_housekeeping" >&2
  exit 1
fi
HOUSEKEEPING_CPUSET="$configured_housekeeping"
if ! taskset -c "$HOUSEKEEPING_CPUSET" true >/dev/null 2>&1; then
  echo "Invalid HOUSEKEEPING_CPUSET: $HOUSEKEEPING_CPUSET" >&2
  exit 1
fi

group_alive() {
  # kill -0 also succeeds for an exited child that is waiting to be reaped.
  # Treat a process group containing only zombies as stopped; otherwise every
  # clean backend exit waits for the full timeout and is needlessly escalated
  # to SIGTERM before the final wait(1) can reap it.
  ps -eo pgid=,stat= | awk -v pgid="$1" '
    $1 == pgid && $2 !~ /^Z/ { alive = 1 }
    END { exit(alive ? 0 : 1) }
  '
}

wait_for_group() {
  local pid=$1
  local attempts=$2
  local attempt
  for ((attempt = 0; attempt < attempts; attempt++)); do
    group_alive "$pid" || return 0
    sleep 0.1
  done
  return 1
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$backend_pid" ]]; then
    kill -INT -- "-$backend_pid" 2>/dev/null || true
  fi
  if [[ -n "$gui_pid" ]]; then
    kill -TERM -- "-$gui_pid" 2>/dev/null || true
  fi
  # Native MANUS/Wuji shutdown can take several seconds while subscriptions,
  # the device connection, and Core Integrated stop.  Let that cleanup finish
  # so WujiHand2Backend.close() can disable the hand before escalating.
  if [[ -n "$backend_pid" ]] && ! wait_for_group "$backend_pid" 100; then
    echo "Backend did not stop after SIGINT; sending SIGTERM..." >&2
    kill -TERM -- "-$backend_pid" 2>/dev/null || true
    wait_for_group "$backend_pid" 50 || {
      echo "Backend did not stop after SIGTERM; sending SIGKILL..." >&2
      kill -KILL -- "-$backend_pid" 2>/dev/null || true
    }
  fi
  [[ -z "$backend_pid" ]] || wait "$backend_pid" 2>/dev/null || true
  [[ -z "$gui_pid" ]] || wait "$gui_pid" 2>/dev/null || true
  exit "$status"
}
trap cleanup EXIT INT TERM

# Query sockets directly: lsof can block while inspecting an unrelated FUSE mount.
if ! occupied="$(timeout 5 ss -H -ltnp "sport = :$CONTROL_PORT")"; then
  echo "Cannot check control port $CONTROL_PORT; socket query failed or timed out." >&2
  exit 1
fi
if [[ -n "$occupied" ]]; then
  echo "Control port $CONTROL_PORT already has a listener: $occupied" >&2
  echo "Stop the previous operator before starting another one." >&2
  exit 1
fi

backend_command=(
  "$REPO_ROOT/ops/run/run_teleop.sh"
  --control-host "$CONTROL_HOST"
  --control-port "$CONTROL_PORT"
  "$@"
)
if [[ " $(id -nG) " != *" dialout "* ]]; then
  printf -v backend_shell '%q ' "${backend_command[@]}"
  backend_command=(sg dialout -c "exec $backend_shell")
fi

setsid taskset -c "$HOUSEKEEPING_CPUSET" "${backend_command[@]}" &
backend_pid=$!

setsid taskset -c "$HOUSEKEEPING_CPUSET" \
  "$CONDA_BASE/bin/python" "$REPO_ROOT/apps/operator_gui/operator_gui.py" \
  --host "$CONTROL_HOST" --port "$CONTROL_PORT" \
  --collection-host "$COLLECTION_CONTROL_HOST" \
  --collection-port "$COLLECTION_CONTROL_PORT" &
gui_pid=$!

set +e
wait -n "$backend_pid" "$gui_pid"
status=$?
set -e
exit "$status"
