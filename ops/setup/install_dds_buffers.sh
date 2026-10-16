#!/usr/bin/env bash
# Host deployment only; does not start ROS or connect to any device.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ $EUID -ne 0 ]]; then
  echo "Run: sudo bash $REPO_ROOT/ops/setup/install_dds_buffers.sh" >&2
  exit 1
fi
target=/etc/sysctl.d/90-gello-dds-buffers.conf
minimum=8388608
current="$(cat /proc/sys/net/core/rmem_max)"
# Preserve a larger deployed limit; only raise the allowed maximum.
if ((current > minimum)); then minimum=$current; fi
if [[ -e "$target" ]] && ! [[ "$(cat "$target")" =~ ^#\ GELLO\ DDS\ receive\ buffer ]]; then
  echo "Refusing to overwrite an unrelated sysctl file: $target" >&2
  exit 1
fi
install -d -m 0755 /etc/sysctl.d
printf '# GELLO DDS receive buffer (camera fragment bursts)\nnet.core.rmem_max = %s\n' "$minimum" > "$target"
chmod 0644 "$target"
sysctl -p "$target"
python3 "$REPO_ROOT/teleop_runtime/dds_buffers.py"
echo "Installed $target; retained across reboot. Restart ROS processes to recreate their sockets."
