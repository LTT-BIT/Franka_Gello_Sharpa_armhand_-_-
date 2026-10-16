#!/usr/bin/env bash
# Container-side supervisor for persistent Sharpa episode collection.
set -euo pipefail

: "${COLLECTION_RECORD_CONFIG:?}"
: "${COLLECTION_OUTPUT_DIR:?}"
: "${COLLECTION_QUALITY_DIR:?}"
: "${COLLECTION_WORKCELL_HASH:?}"
: "${COLLECTION_CONTROL_PORT:?}"
: "${COLLECTION_WITH_CAMERAS:?}"
: "${COLLECTION_INCLUDE_HANDS:?}"
: "${COLLECTION_CAM0_SERIAL:?}"
: "${COLLECTION_CAM0_SEMANTIC:?}"
: "${COLLECTION_CAM1_SERIAL:?}"
: "${COLLECTION_CAM1_SEMANTIC:?}"
: "${COLLECTION_CAM2_SERIAL:?}"
: "${COLLECTION_CAM2_SEMANTIC:?}"

camera_pid=""
readiness_pid=""
collector_pid=""

pid_alive() {
  [[ -n "$1" ]] && kill -0 -- "$1" 2>/dev/null
}

stop_process_group() {
  local pid=$1
  if pid_alive "$pid"; then
    kill -INT -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_process_group "$collector_pid"
  stop_process_group "$readiness_pid"
  stop_process_group "$camera_pid"
  exit "$status"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

case "$COLLECTION_WITH_CAMERAS" in
  true)
    : "${COLLECTION_CAMERA_CONFIG:?}"
    : "${COLLECTION_CAMERA_READY_TIMEOUT_SEC:?}"
    : "${ORBBEC_SDK_LICENSE_ACCEPTED:?}"
    setsid ros2 launch teleop_camera_bringup triple_camera.launch.py \
      deployment_config:="$COLLECTION_CAMERA_CONFIG" &
    camera_pid=$!

    source_readiness_args=()
    # Only the continuity-enabled profile adds source-time gating; ordinary
    # recording retains its existing reception-only readiness behavior.
    continuity_enabled="$(python3 - "$COLLECTION_RECORD_CONFIG" <<'PY_CONFIG'
import sys
import yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream)["teleop_data_collector"]["ros__parameters"]
print("true" if config.get("continuity_enabled", False) else "false")
PY_CONFIG
)"
    if [[ "$continuity_enabled" == true ]]; then
      source_readiness_args+=(--require-source-timestamps)
    fi
    echo "Waiting for four camera streams to deliver stable frames..."
    setsid python3 -m teleop_data_collector.camera_stream_health wait \
      --timeout-sec "$COLLECTION_CAMERA_READY_TIMEOUT_SEC" \
      --stable-sec 3 \
      --max-gap-ms 150 "${source_readiness_args[@]}" &
    readiness_pid=$!

    camera_exited=false
    while pid_alive "$readiness_pid"; do
      if ! pid_alive "$camera_pid"; then
        camera_exited=true
        stop_process_group "$readiness_pid"
        break
      fi
      sleep 0.2
    done
    set +e
    wait "$readiness_pid"
    readiness_status=$?
    set -e
    readiness_pid=""
    if [[ "$camera_exited" == true ]]; then
      echo "Three-camera bringup exited during startup." >&2
      exit 1
    fi
    if [[ $readiness_status -ne 0 ]]; then
      echo "Camera streams did not pass the recording readiness gate." >&2
      exit "$readiness_status"
    fi
    if ! pid_alive "$camera_pid"; then
      echo "Three-camera bringup exited during startup." >&2
      exit 1
    fi
    ;;
  false) ;;
  *)
    echo "COLLECTION_WITH_CAMERAS must be true or false" >&2
    exit 2
    ;;
esac

optional_overrides=()
case "$COLLECTION_INCLUDE_HANDS" in
  true) ;;
  false)
    optional_overrides+=(
      -p topics.left_hand_action.required:=false
      -p topics.right_hand_action.required:=false
      -p topics.left_hand_state.required:=false
      -p topics.right_hand_state.required:=false
    )
    ;;
  *)
    echo "COLLECTION_INCLUDE_HANDS must be true or false" >&2
    exit 2
    ;;
esac

if [[ "$COLLECTION_WITH_CAMERAS" == false ]]; then
  optional_overrides+=(
    -p topics.cam0.required:=false
    -p topics.cam0_depth.required:=false
    -p topics.cam1.required:=false
    -p topics.cam2.required:=false
  )
fi

echo "Sharpa collector is starting. Use the operator GUI to record episodes."
collector_executable="/workspace/franka_upper_body_teleop/ros_ws/install/teleop_data_collector/lib/teleop_data_collector/rosbag_data_collector"
[[ -x "$collector_executable" ]] || {
  echo "Collector executable is missing; rebuild the ROS workspace." >&2
  exit 2
}
setsid "$collector_executable" --ros-args \
  --params-file "$COLLECTION_RECORD_CONFIG" \
  -p output_dir:="$COLLECTION_OUTPUT_DIR" \
  -p quality_dir:="$COLLECTION_QUALITY_DIR" \
  -p control_port:="$COLLECTION_CONTROL_PORT" \
  -p workcell_config_hash:="$COLLECTION_WORKCELL_HASH" \
  -p device_identities.cam0_serial:="$COLLECTION_CAM0_SERIAL" \
  -p device_identities.cam0_semantic:="$COLLECTION_CAM0_SEMANTIC" \
  -p device_identities.cam1_serial:="$COLLECTION_CAM1_SERIAL" \
  -p device_identities.cam1_semantic:="$COLLECTION_CAM1_SEMANTIC" \
  -p device_identities.cam2_serial:="$COLLECTION_CAM2_SERIAL" \
  -p device_identities.cam2_semantic:="$COLLECTION_CAM2_SEMANTIC" \
  "${optional_overrides[@]}" &
collector_pid=$!

set +e
wait "$collector_pid"
collector_status=$?
set -e
collector_pid=""
exit "$collector_status"
