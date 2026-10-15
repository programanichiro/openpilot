# APN authentication diagnostics

Run these tools over ssh as `comma` on the device. Install and build the APN auth
test branch before setting credentials. The setter checks all three auth keys
before writing and refuses root execution.

```bash
./set_apn_auth.sh chap       # Or pap; enter credentials at the prompts.
./set_apn_auth.sh show
./modem_at_read.py --preset probe --secrets-from-params
./collect_modem_logs.sh
```

Credentials may be empty. Otherwise each must contain at most 64 ASCII bytes,
from `!` through `~`, excluding double quote, backslash and comma. Input stays
on stdin; credentials do not enter command arguments or exported environment
variables. `show` displays credential lengths and the stored public APN/auth
values, including any text that overlaps with credentials.

Only `GsmAuth=chap` or `pap` opts in to credentials. With empty `GsmAuth`, modem.py
ignores any stored credentials, and `show` prints a note. The setter writes
`GsmAuth` after both credentials; `clear` removes it after both credentials.

Before changing SIMs or leaving the test branch, run `./set_apn_auth.sh none`,
wait for modem.py's log confirming that CID1 auth was cleared, then run
`./set_apn_auth.sh clear`. Removing the params alone does not clear modem auth.

The AT reader uses the same exclusive lock as modem.py and releases it after
each command. It waits up to 10 seconds for the lock; queries have a 5 second
deadline, or 10 seconds for the MBN list. Examples:

```bash
./modem_at_read.py 'AT+CGAUTH?' 'AT+QICSGP=1' --secrets-from-params
./modem_at_read.py --preset eg25 --json --secrets-from-params
./modem_at_read.py --redact-stdin --secrets-from-params < captured_log.txt
```

Identity commands, writes, chained commands and unsupported syntax are refused.
Auth fields are redacted even when the credentials are absent from params.
With `--secrets-from-params`, literal username/password occurrences of at least
three characters are also masked. Identity numbers, IPv4 addresses, registration
locations, modem identity log values and modem JSON identity fields are masked.
Device hostnames and serving-cell identities are also masked. LTE cellID, PCID
and TAC are removed while radio metrics stay; other RATs or unexpected layouts
are redacted after the RAT.
Other hex/decimal runs of at least 16 characters keep only their first four
characters plus `<id>`. Existing IMEI/ICCID masks and redaction placeholders stay.
Modem `ERROR` replies, query failures and timeouts produce a nonzero exit status;
the remaining queries still run. JSON output is an array of `command`/`lines`
objects with the same redaction.

Developers checking known dummy credentials may use `--show-auth-values` to
disable only CGAUTH/QICSGP user/password field masking. It warns on stderr,
refuses `--secrets-from-params`, and refuses a non-empty `GsmPassword` in the
params directory. If that file cannot be checked, it also refuses. All other
redaction stays active, including identity numbers, IPv4 and registration
locations. This flag also applies to `--json` and `--redact-stdin`. The collector
never uses it.

The collector creates a private `~/modem_logs_<UTC timestamp>/` directory and
matching `.tar.gz`. Every captured stream, including stderr, passes through the
redactor before being saved. The tmux pane is captured with `TERM=xterm` and
streamed through a filter keeping only lines containing ` modem: `, then through
redaction into `tmux_modem.txt`. The full pane is never written to disk, and the
bundle contains no `tmux_full.txt`. It collects pppd/chat and ModemManager journals;
it never requests the sudo journal. Check every file before sharing the bundle.
Missing diagnostics are recorded; a redaction failure stops collection.

`OPENPILOT_DIR` defaults to `/data/openpilot`. The AT reader accepts `--port`,
`--lock`, `--lock-timeout` and `--params-dir`. The collector accepts the test
overrides `MODEM_AT_PORT`, `MODEM_AT_LOCK` and `MODEM_PARAMS_DIR`.

Offline tests use pytest; the tools themselves require only bash and Python's
standard library. The setter additionally uses openpilot's installed Params API.
Test temporary files stay under `tests/` and are removed after each test.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -p no:cacheprovider -q tests
bash -n set_apn_auth.sh collect_modem_logs.sh
RUFF_NO_CACHE=true ruff check .
```
