#!/usr/bin/env bash
# Make GUI-selected host data paths available at the same path in the collector.
# Sourced by the independent recording launchers after DATA_ROOT is resolved.
collection_storage_mounts=()
collection_storage_roots=()
declare -A collection_storage_seen=()
for collection_storage_root in "$HOME" "$DATA_ROOT" /media /mnt; do
  collection_storage_root="$(realpath -m -- "$collection_storage_root")"
  if [[ -z "${collection_storage_seen[$collection_storage_root]:-}" ]]; then
    collection_storage_seen[$collection_storage_root]=1
    collection_storage_mounts+=(-v "$collection_storage_root:$collection_storage_root")
    collection_storage_roots+=("$collection_storage_root")
  fi
done
COLLECTION_STORAGE_ROOTS="$(python3 - "${collection_storage_roots[@]}" <<'PY'
import json
import sys
print(json.dumps(sys.argv[1:]))
PY
)"
