#!/usr/bin/env bash

# Read a single KEY=value from docker/.env without sourcing arbitrary shell.
deployment_env_value() {
  local env_file=$1
  local key=$2
  [[ -f "$env_file" ]] || return 0
  awk -F= -v key="$key" '
    $1 == key {
      sub(/^[^=]*=/, "")
      sub(/\r$/, "")
      print
      exit
    }
  ' "$env_file"
}

deployment_env_default() {
  local env_file=$1
  local key=$2
  local fallback=$3
  local value
  value="$(deployment_env_value "$env_file" "$key")"
  printf '%s\n' "${value:-$fallback}"
}

# Headless LitchiBot uses a separate CPython 3.10 runtime. Read only these
# deployment keys; an explicitly exported value still takes precedence.
deployment_export_litchibot_env() {
  local env_file=$1
  local key value
  for key in LITCHIBOT_PYTHON LITCHIBOT_SDK_ROOT LITCHIBOT_DATA_DIR; do
    if [[ -z "${!key:-}" ]]; then
      value="$(deployment_env_value "$env_file" "$key")"
      [[ -z "$value" ]] || export "$key=$value"
    fi
  done
}
