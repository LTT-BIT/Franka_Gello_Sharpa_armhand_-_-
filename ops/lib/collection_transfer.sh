#!/usr/bin/env bash
# Host data lifecycle only. This process is not owned by full teleop or Docker.
collection_transfer_pid=""
collection_transfer_start() {
  local transfer_log_dir
  transfer_log_dir="$(PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}" /usr/bin/python3 -c \
    'from teleop_runtime.transfer_settings import state_root; print(state_root())')" || return 0
  mkdir -p -- "$transfer_log_dir" || return 0
  local transfer_cpuset
  local -a transfer_affinity=()
  transfer_cpuset="$(deployment_env_default "$REPO_ROOT/docker/.env" HOUSEKEEPING_CPUSET '')"
  if [[ -n "$transfer_cpuset" ]] && command -v taskset >/dev/null; then
    transfer_affinity=(taskset -c "$transfer_cpuset")
  fi
  setsid env PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}" \
    "${transfer_affinity[@]}" /usr/bin/python3 -m teleop_runtime.collection_transfer --owner-pid "$$" \
    >>"$transfer_log_dir/service.log" 2>&1 &
  collection_transfer_pid=$!
  echo "Independent upload/delete watcher PID: $collection_transfer_pid (startup files excluded)"
}
collection_transfer_drain() {
  if [[ -n "$collection_transfer_pid" ]]; then
    kill -TERM "$collection_transfer_pid" 2>/dev/null || true
    echo "Upload/delete worker is finishing its queue independently; recorder need not wait."
  fi
}
