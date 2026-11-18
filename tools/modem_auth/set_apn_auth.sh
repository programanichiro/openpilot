#!/usr/bin/env bash
set +x
set -euo pipefail

if [[ $(id -u) == 0 ]]; then
  printf '%s\n' 'Refusing to run as root. Run as comma; root-owned params break modem.py and the manager.' >&2
  exit 1
fi
if [[ $# != 1 ]]; then
  printf '%s\n' 'Usage: set_apn_auth.sh chap | pap | none | show | clear' >&2
  exit 2
fi
case "$1" in
  chap|pap|none|show|clear) ;;
  *) printf '%s\n' 'Usage: set_apn_auth.sh chap | pap | none | show | clear' >&2; exit 2 ;;
esac

if [[ ! ${PYTHONPATH+x} ]]; then
  # The device profile selects the openpilot venv.
  set +u
  source /etc/profile
  set -u
  set +x
fi
OPENPILOT_DIR=${OPENPILOT_DIR:-/data/openpilot}
export PYTHONDONTWRITEBYTECODE=1

# Code is public; credential values travel only on stdin.
params_code=$(cat <<'PY'
import contextlib
import os
import sys

# Order matters: GsmAuth is last for chap/pap writes and clear removals.
KEYS = ("GsmUsername", "GsmPassword", "GsmAuth")

@contextlib.contextmanager
def quiet():
    saved = (os.dup(1), os.dup(2))
    try:
        with open(os.devnull, "w") as sink:
            os.dup2(sink.fileno(), 1)
            os.dup2(sink.fileno(), 2)
            yield sink
    finally:
        for fd, original in zip((1, 2), saved):
            os.dup2(original, fd)
            os.close(original)

def text(value):
    if value is None:
        return ""
    return value.decode("utf-8", errors="replace") if isinstance(value, bytes) else value

def run():
    output = []
    # Params errors and logging can include values. Keep them off the terminal.
    with quiet() as sink:
        with contextlib.redirect_stdout(sink), contextlib.redirect_stderr(sink):
            from openpilot.common.params import Params, UnknownKeyName

            params = Params()
            try:
                for key in KEYS:
                    if params.check_key(key) is False:
                        raise UnknownKeyName(key)
            except UnknownKeyName:
                return 1, ["Install and build the test branch first; the APN auth keys are unknown. Nothing was written."]

            action = sys.argv[1]
            if action in ("chap", "pap"):
                lines = sys.stdin.buffer.read().split(b"\n")
                if len(lines) != 3 or lines[-1] != b"":
                    return 1, ["Credential input rejected."]
                user, password = lines[:2]
                for value in (user, password):
                    if len(value) > 64:
                        return 1, ["GsmUsername/GsmPassword rejected: maximum 64 bytes."]
                    if any(c < 0x21 or c > 0x7e or c in b'"\\,' for c in value):
                        return 1, ["GsmUsername/GsmPassword rejected: invalid characters."]
                params.put("GsmUsername", user.decode("ascii"))
                params.put("GsmPassword", password.decode("ascii"))
                params.put("GsmAuth", action)
                output.append("APN auth params saved. Wait for modem.py to apply them.")
            elif action == "none":
                params.put("GsmAuth", "none")
                params.remove("GsmUsername")
                params.remove("GsmPassword")
                output.append("modem.py will now clear the modem's CID1 auth. Wait for the cleared-auth log line before running clear.")
            elif action == "clear":
                for key in KEYS:
                    params.remove(key)
                output.append("APN auth params removed.")
            elif action == "show":
                values = {key: text(params.get(key)) for key in (*KEYS, "GsmApn")}
                for key in ("GsmApn", "GsmAuth"):
                    output.append(f"{key}: {ascii(values[key])}")
                for key in KEYS[:2]:
                    value = values[key]
                    output.append(f"{key}: set ({len(value)} chars)" if value else f"{key}: not set")
                if not values["GsmAuth"] and any(values[key] for key in KEYS[:2]):
                    output.append("note: GsmAuth is empty, so modem.py ignores the credentials")
    return 0, output

try:
    status, messages = run()
except (Exception, KeyboardInterrupt):
    status, messages = 1, ["APN params operation failed. Check that the test branch is installed and built; no error details are shown."]
for message in messages:
    print(message, file=sys.stdout if status == 0 else sys.stderr)
sys.exit(status)
PY
)

run_params() {
  PYTHONPATH="$OPENPILOT_DIR" python3 -c "$params_code" "$1"
}

case "$1" in
  chap|pap)
    run_params check
    unset apn_username apn_password
    printf 'Username: ' >&2
    if ! IFS= read -r apn_username; then
      printf '\n%s\n' 'Credential input cancelled.' >&2
      exit 1
    fi
    printf 'Password: ' >&2
    if ! IFS= read -rs apn_password; then
      printf '\n%s\n' 'Credential input cancelled.' >&2
      unset apn_username apn_password
      exit 1
    fi
    printf '\n' >&2
    result=0
    printf '%s\n%s\n' "$apn_username" "$apn_password" | run_params "$1" || result=$?
    unset apn_username apn_password
    exit "$result"
    ;;
  *) run_params "$1" ;;
esac
