#!/usr/bin/env bash
# NEW / EXPERIMENTAL (Route A DDS unification) -- safe to delete to revert.
#
# Launches the standalone litchi_hardware MANUS->Sharpa hand teleop, but forces
# it onto the SAME DDS world the FR3 arms and the data recorder use:
#   CycloneDDS + ROS_LOCALHOST_ONLY=1 + ROS_DOMAIN_ID=<arms domain>.
#
# The stock command
#   pixi run -e ros2 teleop manus_calibration_directory:=$PWD/calibration/metaglove
# runs on FastRTPS / domain 0 / multicast, which is a DIFFERENT DDS world than
# the arms (CycloneDDS/localhost-only). That split is why hands and arms are two
# separate programs today, and it is also what prevents one recorder from seeing
# both. This wrapper only overrides the transport env vars, then defers to the
# unchanged litchi `teleop` task, passing through all arguments verbatim.
#
# It modifies NO existing file. Delete this script to return to the stock path.
set -euo pipefail

GELLO_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$GELLO_REPO/ops/lib/deployment_env.sh"
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

# Default to the metaglove calibration when the caller passes none, matching the
# documented stock invocation.
have_calibration=false
for argument in "$@"; do
  case "$argument" in
    manus_calibration_directory:=*) have_calibration=true ;;
  esac
done
extra_args=("$@")
if [[ "$have_calibration" == false ]]; then
  extra_args+=("manus_calibration_directory:=$LITCHI_REPO/calibration/metaglove")
fi

echo "Sharpa hands -> CycloneDDS, ROS_LOCALHOST_ONLY=1, ROS_DOMAIN_ID=$ARMS_DOMAIN"
echo "Sharpa CPU isolation -> $HOUSEKEEPING_CPUSET (housekeeping cores)"
echo "Calibration/args: ${extra_args[*]}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CYCLONEDDS_XML"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="$ARMS_DOMAIN"

cd "$LITCHI_REPO"
exec taskset -c "$HOUSEKEEPING_CPUSET" pixi run -e ros2 teleop "${extra_args[@]}"
