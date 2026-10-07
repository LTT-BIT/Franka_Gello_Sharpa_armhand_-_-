#!/usr/bin/env bash
# Legacy selector; LitchiBot production uses run_sharpa_hands_cyclonedds.sh.
set -euo pipefail
REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
HAND_SOURCE=manus
if [[ "${1:-}" == "--hand-source" ]]; then
  HAND_SOURCE="${2:?Missing hand source}"
  shift 2
elif [[ "${1:-}" == --hand-source=* ]]; then
  HAND_SOURCE="${1#*=}"
  shift
fi
case "$HAND_SOURCE" in
  manus) exec "$REPO_ROOT/ops/run/run_sharpa_hands_cyclonedds.sh" "$@" ;;
  litchibot)
    echo "Deprecated Terminal 2 selector: use run_sharpa_hands_cyclonedds.sh --hand-source litchibot --dry-run" >&2
    exit 2 ;;
  *) echo "Unsupported hand source: $HAND_SOURCE" >&2; exit 2 ;;
esac
