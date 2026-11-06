#!/usr/bin/env bash
set +x
set -euo pipefail
umask 077

if [[ $(id -u) == 0 ]]; then
  printf '%s\n' 'Run as comma, not root (tmux and Params belong to comma).' >&2
  exit 1
fi
if [[ ! ${PYTHONPATH+x} ]]; then
  set +u
  source /etc/profile
  set -u
  set +x
fi
export PYTHONDONTWRITEBYTECODE=1
OPENPILOT_DIR=${OPENPILOT_DIR:-/data/openpilot}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
params_dir=${MODEM_PARAMS_DIR:-/data/params/d}
at_port=${MODEM_AT_PORT:-/dev/modem_at0}
at_lock=${MODEM_AT_LOCK:-/dev/shm/modem.lock}

redact() {
  python3 "$script_dir/modem_at_read.py" --redact-stdin --secrets-from-params --params-dir "$params_dir"
}

# Check secret access before collecting anything.
printf '' | redact >/dev/null
stamp=$(date -u +%Y%m%dT%H%M%SZ)
bundle="$HOME/modem_logs_$stamp"
mkdir -- "$bundle"

collect() {
  local filename=$1
  shift
  local statuses
  set +e
  "$@" 2>&1 | redact >"$bundle/$filename"
  statuses=("${PIPESTATUS[@]}")
  set -e
  if [[ ${statuses[1]} != 0 ]]; then
    printf '%s\n' 'Redaction failed; collection stopped. Do not share the incomplete bundle.' >&2
    exit 1
  fi
  if [[ ${statuses[0]} != 0 ]]; then
    printf '%s\n' '[Collection command failed or returned a non-OK modem response.]' | redact >>"$bundle/$filename"
  fi
}

branch_commit() {
  git -C "$OPENPILOT_DIR" branch --show-current
  git -C "$OPENPILOT_DIR" rev-parse HEAD
}

modemmanager_tail() {
  sudo journalctl -b -u ModemManager --no-pager | tail -100
}

modem_lines() {
  # Filter the capture before any pane output reaches disk.
  env TERM=xterm tmux capture-pane -p -J -S - -t comma 2>&1 | awk 'index($0, " modem: ")'
}

collect version.txt cat /VERSION
collect openpilot_git.txt branch_commit
collect tmux_modem.txt modem_lines
collect modem.json cat /dev/shm/modem
collect pppd_chat.txt sudo journalctl -b -t pppd -t chat --no-pager
collect modemmanager.txt modemmanager_tail
collect ip_addr.txt ip -br addr
collect ip_route.txt ip route
collect ip_rule.txt ip rule
collect dev_shm.txt ls -l /dev/shm
collect at_probe.txt python3 "$script_dir/modem_at_read.py" --preset probe \
  --port "$at_port" --lock "$at_lock" --secrets-from-params --params-dir "$params_dir"
if [[ $(awk '/^> AT\+GMR: / { print ($0 ~ /^> AT\+GMR: EG25/); exit }' "$bundle/at_probe.txt") == 1 ]]; then
  collect at_eg25.txt python3 "$script_dir/modem_at_read.py" --preset eg25 \
    --port "$at_port" --lock "$at_lock" --secrets-from-params --params-dir "$params_dir"
fi
collect apn_auth.txt env OPENPILOT_DIR="$OPENPILOT_DIR" bash "$script_dir/set_apn_auth.sh" show

archive="$bundle.tar.gz"
if ! tar -czf "$archive" -C "$HOME" "${bundle##*/}" 2>/dev/null; then
  printf '%s\n' 'Failed to create the archive.' >&2
  exit 1
fi
printf 'Bundle: %s\nArchive: %s\nCheck every file before sharing.\n' "$bundle" "$archive" | redact
