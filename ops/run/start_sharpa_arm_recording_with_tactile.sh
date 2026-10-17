#!/usr/bin/env bash
# Record dual-FR3 + dual-Sharpa + ten-finger tactile streams without opening
# another libfranka/FCI connection. This is separate from the original launcher.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO_ROOT/ops/lib/deployment_env.sh"
DATA_ROOT=""
CAMERA_CONFIG="$REPO_ROOT/data_collection/config/cameras.yaml"
DISCOVERY_TIMEOUT_SEC=45
CAMERA_READY_TIMEOUT_SEC=90
COLLECTION_CONTROL_PORT="${COLLECTION_CONTROL_PORT:-$(deployment_env_default "$REPO_ROOT/docker/.env" COLLECTION_CONTROL_PORT 5592)}"
WITH_CAMERAS=false
INCLUDE_HANDS=true

usage() {
  cat >&2 <<'EOF'
Usage: ./ops/run/start_sharpa_arm_recording_with_tactile.sh [--with-cameras]
       [--data-root ABSOLUTE_PATH] [--camera-config YAML]
       [--camera-ready-timeout SECONDS]

The default data root is TELEOP_DATA_ROOT from docker/.env. Start the arms and
the CycloneDDS Sharpa wrapper first. All 20 tactile streams are required. This
command starts a persistent collector; use the operator GUI to start and stop
each episode. Rate each completed episode before starting the next one.
Use Ctrl-C only to exit the collector after the current episode is stopped.
EOF
}

while (($#)); do
  case "$1" in
    --data-root)
      (($# >= 2)) || { usage; exit 2; }
      DATA_ROOT=$2
      shift 2
      ;;
    --camera-config)
      (($# >= 2)) || { usage; exit 2; }
      CAMERA_CONFIG=$2
      shift 2
      ;;
    --with-cameras)
      WITH_CAMERAS=true
      shift
      ;;
    --without-hands)
      echo "--without-hands is not supported by the tactile recorder." >&2
      exit 2
      ;;
    --discovery-timeout)
      (($# >= 2)) || { usage; exit 2; }
      DISCOVERY_TIMEOUT_SEC=$2
      shift 2
      ;;
    --camera-ready-timeout)
      (($# >= 2)) || { usage; exit 2; }
      CAMERA_READY_TIMEOUT_SEC=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

ENV_FILE="$REPO_ROOT/docker/.env"
python3 "$REPO_ROOT/teleop_runtime/dds_buffers.py"
[[ -f "$ENV_FILE" ]] || {
  echo "Missing deployment file: $ENV_FILE" >&2
  exit 2
}

if [[ -z "$DATA_ROOT" ]]; then
  DATA_ROOT="$(awk -F= '$1 == "TELEOP_DATA_ROOT" {
    sub(/^[^=]*=/, ""); gsub(/\r/, ""); print; exit
  }' "$ENV_FILE")"
fi
[[ "$DATA_ROOT" == /* ]] || {
  echo "--data-root must be an absolute path" >&2
  exit 2
}
[[ "$DISCOVERY_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ ]] || {
  echo "--discovery-timeout must be a positive integer" >&2
  exit 2
}
[[ "$CAMERA_READY_TIMEOUT_SEC" =~ ^[1-9][0-9]*$ ]] || {
  echo "--camera-ready-timeout must be a positive integer" >&2
  exit 2
}
[[ "$COLLECTION_CONTROL_PORT" =~ ^[1-9][0-9]*$ ]] && \
  ((COLLECTION_CONTROL_PORT <= 65535)) || {
  echo "COLLECTION_CONTROL_PORT must be an integer in 1..65535" >&2
  exit 2
}

DATA_ROOT="$(realpath -m -- "$DATA_ROOT")"
case "$DATA_ROOT/" in
  "$REPO_ROOT/"*)
    echo "Data output must be outside the Git repository: $DATA_ROOT" >&2
    exit 2
    ;;
esac

CAMERA_VALUES=(
  not-recorded not-recorded
  not-recorded not-recorded
  not-recorded not-recorded
)
if [[ "$WITH_CAMERAS" == true ]]; then
  [[ -f "$CAMERA_CONFIG" ]] || {
    echo "Camera deployment config not found: $CAMERA_CONFIG" >&2
    exit 2
  }
  CAMERA_CONFIG="$(realpath -- "$CAMERA_CONFIG")"
  ORBBEC_SDK_LICENSE_ACCEPTED="$(awk -F= '
    $1 == "ORBBEC_SDK_LICENSE_ACCEPTED" {
      sub(/^[^=]*=/, ""); gsub(/\r/, ""); print; exit
    }' "$ENV_FILE")"
  if [[ "$ORBBEC_SDK_LICENSE_ACCEPTED" != "YES" ]]; then
    echo "Orbbec runtime is license-gated." >&2
    echo "Review ros_ws/src/orbbec_camera/SDK/End User License Agreement.txt." >&2
    echo "If accepted, set ORBBEC_SDK_LICENSE_ACCEPTED=YES in docker/.env." >&2
    exit 2
  fi
  mapfile -t CAMERA_VALUES < <(python3 - "$CAMERA_CONFIG" <<'PY'
import sys
from pathlib import Path

import yaml

config = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8")) or {}
cameras = config.get("camera_bringup") or {}
serials = []
semantics = []
for name in ("cam0", "cam1", "cam2"):
    entry = cameras.get(name) or {}
    serial = str(entry.get("serial_number", "")).strip()
    semantic = str(entry.get("semantic", "")).strip()
    if not serial or serial.startswith("REPLACE_"):
        raise SystemExit(f"{name} serial_number is not deployed")
    if not semantic or semantic.startswith("REPLACE_"):
        raise SystemExit(f"{name} semantic is not deployed")
    serials.append(serial)
    semantics.append(semantic)
if len(set(serials)) != 3:
    raise SystemExit("Camera serial numbers must be unique")
if len(set(semantics)) != 3:
    raise SystemExit("Camera semantics must be unique")
for serial, semantic in zip(serials, semantics):
    print(serial)
    print(semantic)
PY
  )
  [[ ${#CAMERA_VALUES[@]} -eq 6 ]] || {
    echo "Camera deployment config did not produce three identities." >&2
    exit 2
  }
fi

TOPICS=(
  /teleop/validated_arm_commands
  /teleop/arm_command_status
  /left/franka/joint_states
  /right/franka/joint_states
)

EXPECTED_TYPES=(
  sensor_msgs/msg/JointState
  teleop_interfaces/msg/ArmCommandStatus
  sensor_msgs/msg/JointState
  sensor_msgs/msg/JointState
)
if [[ "$INCLUDE_HANDS" == true ]]; then
  TOPICS+=(
    /sharpa/left/command
    /sharpa/right/command
    /sharpa/left/joint_states
    /sharpa/right/joint_states
  )
  EXPECTED_TYPES+=(
    sensor_msgs/msg/JointState
    sensor_msgs/msg/JointState
    sensor_msgs/msg/JointState
    sensor_msgs/msg/JointState
  )
fi

TACTILE_FINGERS=(thumb index middle ring pinky)
for side in left right; do
  for finger in "${TACTILE_FINGERS[@]}"; do
    TOPICS+=(
      "/sharpa/${side}/tactile/${finger}/wrench"
      "/sharpa/${side}/tactile/${finger}/deformation"
    )
    EXPECTED_TYPES+=(
      geometry_msgs/msg/WrenchStamped
      sensor_msgs/msg/Image
    )
  done
done
readonly -a TOPICS EXPECTED_TYPES TACTILE_FINGERS

cd "$REPO_ROOT/docker"
running="$(docker compose ps --services --status running)"
if ss -H -ltn "sport = :$COLLECTION_CONTROL_PORT" | awk 'NF { found=1 } END { exit !found }'; then
  echo "Refusing to start: collector UI control port 127.0.0.1:$COLLECTION_CONTROL_PORT is already in use." >&2
  exit 1
fi
for service in franka-control teleop-control gello-bridge; do
  if ! grep -qx "$service" <<<"$running"; then
    echo "Required arm service is not running: $service" >&2
    echo "Start ./ops/run/run_gello_arms_only.sh first." >&2
    exit 1
  fi
done
active_collectors="$(
  docker ps \
    --filter label=com.docker.compose.service=data-collection \
    --format '{{.Names}}'
)"
if [[ -n "$active_collectors" ]]; then
  echo "Refusing to start a second data-collection container." >&2
  echo "Already running: $active_collectors" >&2
  exit 1
fi
if [[ "$WITH_CAMERAS" == true ]]; then
  if grep -qx orbbec <<<"$running"; then
    echo "Refusing to start: compose service 'orbbec' already owns a camera." >&2
    exit 1
  fi
  if pgrep -x OrbbecViewer >/dev/null 2>&1; then
    echo "Refusing to start: close OrbbecViewer before three-camera capture." >&2
    exit 1
  fi
fi

if [[ "$INCLUDE_HANDS" == true ]]; then
  echo "Waiting for dual-arm and dual-Sharpa topics..."
else
  echo "Waiting for dual-arm topics (--without-hands)..."
fi
deadline=$((SECONDS + DISCOVERY_TIMEOUT_SEC))
missing=()
wrong_types=()
while ((SECONDS < deadline)); do
  graph="$(
    docker compose exec -T teleop-control \
      /entrypoint.sh ros2 topic list -t --no-daemon 2>/dev/null || true
  )"
  missing=()
  wrong_types=()
  for index in "${!TOPICS[@]}"; do
    topic=${TOPICS[$index]}
    expected_type=${EXPECTED_TYPES[$index]}
    line="$(awk -v topic="$topic" '$1 == topic { print; exit }' <<<"$graph")"
    if [[ -z "$line" ]]; then
      missing+=("$topic")
    elif [[ "$line" != *"[$expected_type]"* ]]; then
      wrong_types+=("$topic expected $expected_type; discovered: $line")
    fi
  done
  ((${#missing[@]} == 0 && ${#wrong_types[@]} == 0)) && break
  sleep 1
done

if ((${#missing[@]} || ${#wrong_types[@]})); then
  ((${#missing[@]} == 0)) || printf 'Missing topic: %s\n' "${missing[@]}" >&2
  ((${#wrong_types[@]} == 0)) || printf 'Wrong topic type: %s\n' "${wrong_types[@]}" >&2
  if [[ "$INCLUDE_HANDS" == true ]]; then
    echo "Launch Sharpa with ./ops/run/run_sharpa_hands_cyclonedds.sh, then retry." >&2
  fi
  exit 1
fi

bag_group=gello_sharpa_tactile
mkdir -p -- "$DATA_ROOT/bags/$bag_group"
mkdir -p -- "$DATA_ROOT/数据分类/$bag_group"
WORKCELL_HASH="$(sha256sum "$REPO_ROOT/config/workcell/current.yaml" | awk '{print $1}')"
container_output_dir="/collection_data/bags/$bag_group"
container_quality_dir="/collection_data/数据分类/$bag_group"

echo "All ${#TOPICS[@]} arm/hand topics discovered with the expected types."
echo "Episode directory: $DATA_ROOT/bags/$bag_group"
echo "Wait for READY, then use the operator GUI to start/stop each episode."
echo "After stopping, wait for validation and select a quality rating."
echo "Press Ctrl-C only when no episode is recording and you are finished collecting."

camera_mount=()
if [[ "$WITH_CAMERAS" == true ]]; then
  echo "Starting cam0/cam1/cam2 from: $CAMERA_CONFIG"
  camera_mount=(-v "$CAMERA_CONFIG:/collection_camera_config.yaml:ro")
fi
collector_name="sharpa-tactile-data-collection-$$"
source "$REPO_ROOT/ops/lib/collection_storage.sh"
source "$REPO_ROOT/ops/lib/collection_transfer.sh"
collection_transfer_start
docker compose run -d --name "$collector_name" --no-deps \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e ORBBEC_SDK_LICENSE_ACCEPTED="${ORBBEC_SDK_LICENSE_ACCEPTED:-NO}" \
  -e COLLECTION_RECORD_CONFIG=/workspace/franka_upper_body_teleop/data_collection/config/record_gello_sharpa_tactile.yaml \
  -e COLLECTION_OUTPUT_DIR="$container_output_dir" \
  -e COLLECTION_HOST_DATA_ROOT="$DATA_ROOT" \
  -e COLLECTION_STORAGE_ROOTS="$COLLECTION_STORAGE_ROOTS" \
  -e COLLECTION_QUALITY_DIR="$container_quality_dir" \
  -e COLLECTION_WORKCELL_HASH="$WORKCELL_HASH" \
  -e COLLECTION_CONTROL_PORT="$COLLECTION_CONTROL_PORT" \
  -e COLLECTION_WITH_CAMERAS="$WITH_CAMERAS" \
  -e COLLECTION_CAMERA_CONFIG=/collection_camera_config.yaml \
  -e COLLECTION_CAMERA_READY_TIMEOUT_SEC="$CAMERA_READY_TIMEOUT_SEC" \
  -e COLLECTION_INCLUDE_HANDS="$INCLUDE_HANDS" \
  -e COLLECTION_CAM0_SERIAL="${CAMERA_VALUES[0]}" \
  -e COLLECTION_CAM0_SEMANTIC="${CAMERA_VALUES[1]}" \
  -e COLLECTION_CAM1_SERIAL="${CAMERA_VALUES[2]}" \
  -e COLLECTION_CAM1_SEMANTIC="${CAMERA_VALUES[3]}" \
  -e COLLECTION_CAM2_SERIAL="${CAMERA_VALUES[4]}" \
  -e COLLECTION_CAM2_SEMANTIC="${CAMERA_VALUES[5]}" \
  -v "$DATA_ROOT:/collection_data" \
  "${collection_storage_mounts[@]}" \
  "${camera_mount[@]}" \
  data-collection \
  /workspace/franka_upper_body_teleop/ops/run/run_sharpa_camera_recording_container.sh \
  >/dev/null

stop_requested=false
stop_collector() {
  if [[ "$stop_requested" == false ]]; then
    stop_requested=true
    echo "Stopping the Sharpa tactile collector after active bag finalization..."
    docker kill --signal=TERM "$collector_name" >/dev/null 2>&1 || true
  fi
}
trap stop_collector INT TERM

docker logs --follow "$collector_name" &
logs_pid=$!
while [[ "$(docker inspect --format '{{.State.Running}}' "$collector_name")" == true ]]; do
  sleep 0.2 || true
done
collector_status="$(docker inspect --format '{{.State.ExitCode}}' "$collector_name")"
wait "$logs_pid" 2>/dev/null || true
collection_transfer_drain
trap - INT TERM
docker rm "$collector_name" >/dev/null

if [[ $collector_status -eq 0 || $collector_status -eq 130 || $collector_status -eq 143 ]]; then
  echo "Sharpa collector stopped. Completed episodes remain in $DATA_ROOT/bags/$bag_group"
  exit 0
fi
echo "Sharpa collector exited with status $collector_status." >&2
exit "$collector_status"
