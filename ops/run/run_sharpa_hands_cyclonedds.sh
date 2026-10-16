#!/usr/bin/env bash
# Production Terminal 2: one shared Sharpa driver in the arms/recorder DDS domain.
# Default Manus argv, calibration and transport remain unchanged.
# LitchiBot selects a separate Python 3.10 worker and ROS command bridge.
# Dry-run forces fake; real acceptance requires a separate supervised permit/guard.
set -euo pipefail

# Preserve the original Manus argv when no new options are supplied.
HAND_SOURCE=manus
DRY_RUN=false
VALIDATION=false
VALIDATION_PERMIT=""
FULL_CONFIG=""
source_selected=false
dry_selected=false
extra_args=()
while (($#)); do
  case "$1" in
    --hand-source)
      [[ $# -ge 2 ]] || { echo "Missing --hand-source value" >&2; exit 2; }
      HAND_SOURCE="$2"; source_selected=true; shift 2 ;;
    --hand-source=*) HAND_SOURCE="${1#*=}"; source_selected=true; shift ;;
    --full-config)
      [[ $# -ge 2 ]] || { echo "Missing full config" >&2; exit 2; }
      FULL_CONFIG="$2"; shift 2 ;;
    full_config:=*) FULL_CONFIG="${1#full_config:=}"; shift ;;
    --dry-run) DRY_RUN=true; dry_selected=true; shift ;;
    --supervised-hardware-validation) VALIDATION=true; shift ;;
    --validation-permit)
      [[ $# -ge 2 ]] || { echo "Missing validation permit" >&2; exit 2; }
      VALIDATION_PERMIT="$2"; shift 2 ;;
    supervised_hardware_validation:=*) VALIDATION="${1#supervised_hardware_validation:=}"; shift ;;
    validation_permit:=*) VALIDATION_PERMIT="${1#validation_permit:=}"; shift ;;
    hand_source:=*) HAND_SOURCE="${1#hand_source:=}"; source_selected=true; shift ;;
    dry_run:=*) DRY_RUN="${1#dry_run:=}"; dry_selected=true; shift ;;
    *) extra_args+=("$1"); shift ;;
  esac
done
[[ "$HAND_SOURCE" == manus || "$HAND_SOURCE" == litchibot ]] || {
  echo "hand source must be manus or litchibot" >&2; exit 2;
}
[[ "$DRY_RUN" == true || "$DRY_RUN" == false ]] || {
  echo "dry_run must be true or false" >&2; exit 2;
}
[[ "$VALIDATION" == true || "$VALIDATION" == false ]] || { echo "Invalid validation mode" >&2; exit 2; }
if [[ "$VALIDATION" == true && ( "$HAND_SOURCE" != litchibot || "$DRY_RUN" == true || -z "$VALIDATION_PERMIT" ) ]]; then
  echo "Supervised validation requires LitchiBot, an explicit permit, and no --dry-run." >&2
  exit 2
fi
if [[ "$HAND_SOURCE" == litchibot && "$DRY_RUN" != true && "$VALIDATION" != true && -z "$FULL_CONFIG" ]]; then
  echo "BLOCKER FOR REAL HARDWARE MOTION: ordinary LitchiBot hardware startup prohibited; use dry-run or supervised validation with permit." >&2
  exit 2
fi

GELLO_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$GELLO_REPO/ops/lib/deployment_env.sh"
if [[ "$HAND_SOURCE" == litchibot ]]; then
  deployment_export_litchibot_env "$GELLO_REPO/docker/.env"
fi
configured_litchi="$(deployment_env_value "$GELLO_REPO/docker/.env" LITCHI_REPO)"
legacy_litchi=/home/descfly/ZH/manus_sharpa_record/litchi_hardware
if [[ -n "${LITCHI_REPO:-}" ]]; then
  LITCHI_REPO="$LITCHI_REPO"
elif [[ -n "$configured_litchi" && "$configured_litchi" != /absolute/path/* ]]; then
  LITCHI_REPO="$configured_litchi"
else
  LITCHI_REPO="$legacy_litchi"
fi
CYCLONEDDS_XML="$GELLO_REPO/config/cyclonedds.xml"
HOUSEKEEPING_CPUSET=""

# Match the arms' DDS domain from the authoritative deployment file so a domain
# drift cannot silently split hands from arms again.
ARMS_DOMAIN=""
if [[ -f "$GELLO_REPO/docker/.env" ]]; then
  ARMS_DOMAIN="$(awk -F= '$1 == "TELEOP_ROS_DOMAIN_ID" {
    sub(/^[^=]*=/, ""); gsub(/\r/, ""); print; exit
  }' "$GELLO_REPO/docker/.env")"
fi
ARMS_DOMAIN="${ARMS_DOMAIN:-0}"

# Keep the MANUS SDK, V4 optimizer, and Sharpa SDK off the FR3 realtime cores.
# docker/.env is authoritative for both the containers and host processes so a
# stale exported value cannot silently create two different CPU layouts.
if [[ -f "$GELLO_REPO/docker/.env" ]]; then
  HOUSEKEEPING_CPUSET="$(awk -F= '$1 == "HOUSEKEEPING_CPUSET" {
    sub(/^[^=]*=/, ""); gsub(/\r/, ""); print; exit
  }' "$GELLO_REPO/docker/.env")"
fi
[[ -n "$HOUSEKEEPING_CPUSET" ]] || {
  echo "HOUSEKEEPING_CPUSET is missing from $GELLO_REPO/docker/.env" >&2
  exit 1
}
if ! taskset -c "$HOUSEKEEPING_CPUSET" true >/dev/null 2>&1; then
  echo "Invalid HOUSEKEEPING_CPUSET: $HOUSEKEEPING_CPUSET" >&2
  exit 1
fi

[[ -f "$CYCLONEDDS_XML" ]] || {
  echo "Missing CycloneDDS config: $CYCLONEDDS_XML" >&2
  exit 1
}
[[ -d "$LITCHI_REPO" ]] || {
  echo "litchi_hardware repo not found: $LITCHI_REPO (set LITCHI_REPO=...)" >&2
  exit 1
}
[[ -f "$LITCHI_REPO/ros2/install/setup.bash" ]] || {
  echo "litchi ros2 workspace is not built. Run: (cd '$LITCHI_REPO' && pixi run -e ros2 ros-build)" >&2
  exit 1
}

if [[ -n "$FULL_CONFIG" ]]; then
  # An explicit managed session uses the SAME runtime guard, initially DISABLED.
  # This never changes standalone defaults and never converts dry-run into hardware.
  PYTHONPATH="$GELLO_REPO${PYTHONPATH:+:$PYTHONPATH}" "$LITCHI_REPO/.pixi/envs/ros2/bin/python" - "$FULL_CONFIG" "$HAND_SOURCE" "$DRY_RUN" <<'CHECK'
import sys
from adapters.litchibot.full_control import load_full_config
config=load_full_config(sys.argv[1])
if config['source']!=sys.argv[2] or (sys.argv[3]=='true' and config['mode']!='fake'):
    raise SystemExit('Full source/dry-run conflict')
CHECK
  extra_args+=("full_teleop:=true" "full_config:=$FULL_CONFIG"
    "validation_driver_script:=$GELLO_REPO/adapters/litchibot/validation_driver.py")
fi

# Keep the familiar entry point while allowing this deployment to select its
# MANUS bridge and calibration. Explicit launch arguments take precedence.
have_calibration=false
have_native_library=false
for argument in "${extra_args[@]}"; do
  case "$argument" in
    manus_calibration_directory:=*) have_calibration=true ;;
    manus_native_library:=*) have_native_library=true ;;
  esac
done
manus_calibration="$LITCHI_REPO/calibration/metaglove"
if [[ "$HAND_SOURCE" == manus ]]; then
  configured_native="$(deployment_env_value "$GELLO_REPO/docker/.env" MANUS_NATIVE_LIBRARY)"
  configured_calibration="$(deployment_env_value "$GELLO_REPO/docker/.env" MANUS_CALIBRATION_DIRECTORY)"
  if [[ "$have_native_library" == false && -n "$configured_native" ]]; then
    [[ -f "$configured_native" ]] || { echo "Missing MANUS bridge: $configured_native" >&2; exit 1; }
    extra_args+=("manus_native_library:=$configured_native")
  fi
  if [[ "$have_calibration" == false && -n "$configured_calibration" ]]; then
    for side in left right; do
      [[ -f "$configured_calibration/Calibration_${side}.mcal" ]] || {
        echo "Missing MANUS calibration: $configured_calibration/Calibration_${side}.mcal" >&2; exit 1;
      }
    done
    manus_calibration="$configured_calibration"
  fi
fi
if [[ "$have_calibration" == false ]]; then
  extra_args+=("manus_calibration_directory:=$manus_calibration")
fi
if [[ "$source_selected" == true && "$HAND_SOURCE" != manus ]]; then
  extra_args+=("hand_source:=$HAND_SOURCE")
fi
if [[ "$dry_selected" == true ]]; then
  extra_args+=("dry_run:=$DRY_RUN")
fi
if [[ "$HAND_SOURCE" == litchibot ]]; then
  extra_args+=("litchibot_bridge_script:=$GELLO_REPO/adapters/litchibot/terminal2_bridge.py")
fi
if [[ "$VALIDATION" == true ]]; then
  extra_args+=("supervised_hardware_validation:=true" "validation_permit:=$VALIDATION_PERMIT"
    "validation_driver_script:=$GELLO_REPO/adapters/litchibot/validation_driver.py")
fi

echo "Sharpa hands -> CycloneDDS, ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID=$ARMS_DOMAIN"
echo "Sharpa CPU isolation -> $HOUSEKEEPING_CPUSET (housekeeping cores)"
echo "Calibration/args: ${extra_args[*]}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CYCLONEDDS_XML"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="$ARMS_DOMAIN"

cd "$LITCHI_REPO"
if [[ -n "$FULL_CONFIG" && "$HAND_SOURCE" == manus ]]; then
  # Project-owned launch selects ordinary MANUS forwarding independently of
  # the bundled validation launch. No changes to standalone or LitchiBot.
  exec taskset -c "$HOUSEKEEPING_CPUSET" pixi run -e ros2 bash -c \
    'source "$1/ros2/install/setup.bash"; shift; exec ros2 launch "$@"' -- \
    "$LITCHI_REPO" "$GELLO_REPO/adapters/manus/teleop.launch.py" "${extra_args[@]}"
fi
exec taskset -c "$HOUSEKEEPING_CPUSET" pixi run -e ros2 teleop "${extra_args[@]}"
