#!/usr/bin/env python3
"""Read-only modem queries and a reusable, conservative log redactor."""

import argparse
import contextlib
import fcntl
import json
import math
import os
import re
import select
import sys
import termios
import time
from pathlib import Path

PRESETS = {
    "probe": (
        "AT+GMR", "AT+CGAUTH=?", "AT+CGAUTH?", "AT+QICSGP=?", "AT+QICSGP=1",
        "AT+CGDCONT?", "AT+CREG?", "AT+CGREG?", "AT+CEREG?", "AT+COPS?",
        "AT+CGACT?", "AT+QNWINFO", "AT+CSQ", "AT+CEER", "AT+CSCS?",
    ),
    "eg25": (
        'AT+QMBNCFG="List"', 'AT+QCFG="noauthcheck"', 'AT+QCFG="ims"', "AT+CGCONTRDP=1",
    ),
}
BARE_READS = {"ATI", "AT+GMR", "AT+QGMR", "AT+CGMR", "AT+CEER", "AT+QNWINFO", "AT+CSQ"}
IDENTITY = {"AT+QCCID", "AT+CNUM", "AT+CIMI", "AT+CGSN"}


def is_allowed_command(command: str) -> bool:
    cmd = command.upper()
    if not cmd.isascii() or any(c in cmd for c in "\r\n;\\"):
        return False
    if cmd.rstrip("=?") in IDENTITY:
        return False
    if cmd in BARE_READS or cmd == 'AT+QMBNCFG="LIST"':
        return True
    if re.fullmatch(r"AT[+&^$%]?[A-Z][A-Z0-9_]*(?:\?|=\?)", cmd):
        return True
    if re.fullmatch(r"AT\+QICSGP=(?:[1-9]|1[0-6])", cmd):
        return True
    if re.fullmatch(r"AT\+CGCONTRDP=[0-9]+", cmd):
        return True
    return bool(re.fullmatch(r'AT\+QCFG="[A-Z0-9_./ -]+"', cmd))


def command_timeout(command: str) -> float:
    return 10.0 if command.upper() == 'AT+QMBNCFG="LIST"' else 5.0


def _fields(payload: str):
    """Return CSV value spans, retaining the original punctuation."""
    fields = []
    pos = 0
    while pos < len(payload):
        while pos < len(payload) and payload[pos] in " \t":
            pos += 1
        if pos == len(payload):
            break
        quoted = payload[pos] == '"'
        if quoted:
            start = pos + 1
            pos = start
            while pos < len(payload):
                if payload[pos] == '"':
                    if pos + 1 < len(payload) and payload[pos + 1] == '"':
                        pos += 2
                        continue
                    break
                pos += 1
            if pos == len(payload):
                raise ValueError("malformed fields")
            end = pos
            value = payload[start:end].replace('""', '"')
            pos += 1
        else:
            start = pos
            while pos < len(payload) and payload[pos] != ",":
                pos += 1
            end = pos
            while end > start and payload[end - 1] in " \t":
                end -= 1
            value = payload[start:end]
        fields.append((start, end, value, quoted))
        while pos < len(payload) and payload[pos] in " \t":
            pos += 1
        if pos == len(payload) or payload[pos:].startswith("|"):
            break
        if payload[pos] != ",":
            raise ValueError("malformed fields")
        pos += 1
    return fields


def _replace_fields(payload, fields, replacements):
    for index, replacement in sorted(replacements.items(), reverse=True):
        if index < len(fields):
            start, end, _, _ = fields[index]
            payload = payload[:start] + replacement + payload[end:]
    return payload


def _redact_records(line, pattern, replace):
    # Find the next record only after replacing this record's values.
    position = 0
    while match := pattern.search(line, position):
        end = len(line)
        for boundary in "\r\n":
            found = line.find(boundary, match.end())
            if found >= 0:
                end = min(end, found)
        payload = line[match.end():end]
        line = line[:match.end()] + replace(match, payload) + line[end:]
        position = match.end()
    return line


def redact_auth(line: str) -> str:
    def replace(match, payload):
        name = match.group(1)
        if payload.lstrip().startswith("("):
            return payload
        try:
            fields = _fields(payload)
            start = 2
            if name.upper() == "QICSGP" and len(fields) >= 5 and not fields[1][3] and fields[1][2].isdigit():
                start = 3
            replacements = {}
            for index, label in ((start, "user"), (start + 1, "password")):
                if index < len(fields):
                    value = fields[index][2]
                    replacements[index] = value if re.fullmatch(rf"<{label}:\d+>", value) else f"<{label}:{len(value)}>"
            return _replace_fields(payload, fields, replacements)
        except ValueError:
            return "<redacted>"

    return _redact_records(line, re.compile(r"\+(CGAUTH|QICSGP):[ \t]*", re.IGNORECASE), replace)


def redact_location(line: str) -> str:
    def replace(match, payload):
        try:
            fields = _fields(payload)
            if len(fields) < 3:
                return payload
            second = fields[1]
            query = not second[3] and second[2].isdigit() and int(second[2]) <= 10 and len(fields) >= 4
            start = 2 if query else 1
            return _replace_fields(payload, fields, {start: "<loc>", start + 1: "<loc>"})
        except ValueError:
            return "<redacted>"

    return _redact_records(line, re.compile(r"\+(?:CREG|CGREG|CEREG):[ \t]*", re.IGNORECASE), replace)


def redact_servingcell(line: str) -> str:
    def redact_value(value):
        header = re.match(
            r'^\s*(?:"servingcell"|servingcell)\s*,\s*(?:"[^"\r\n]*"|[^,"\r\n]*)\s*,\s*(?:"[^"\r\n]*"|[^,"\r\n]*)(?=,|$)',
            value, flags=re.IGNORECASE,
        )
        fallback = header.group(0) + ",<redacted>" if header else "<redacted>"
        try:
            fields = _fields(value)
        except ValueError:
            return fallback
        if len(fields) != 18 or fields[2][2].upper() != "LTE":
            return fallback
        return _replace_fields(value, fields, {6: "<cell>", 7: "<pci>", 12: "<tac>"})

    def extra(match):
        try:
            value = json.loads(match.group(2))
        except json.JSONDecodeError:
            return match.group(1) + '"<redacted>"'
        if not re.match(r'^\s*"?servingcell\b', value, flags=re.IGNORECASE):
            return match.group(0)
        return match.group(1) + json.dumps(redact_value(value))

    def qeng(match, payload):
        terminal = re.search(r"\s+\|\s+(?:OK|ERROR)\s*$", payload)
        if terminal:
            return redact_value(payload[:terminal.start()]) + terminal.group(0)
        return redact_value(payload)

    line = re.sub(r'("extra"\s*:\s*)("(?:\\.|[^"\\])*")', extra, line)
    return _redact_records(
        line, re.compile(r'\+QENG:[ \t]*(?="?servingcell\b)', re.IGNORECASE), qeng,
    )


def redact_line(line: str, secrets: dict[str, str] | None = None, *, show_auth_values: bool = False) -> str:
    line = redact_location(line if show_auth_values else redact_auth(line))
    line = redact_servingcell(line)
    line = re.sub(r"\bcomma-[0-9a-fA-F]{6,}\b", "comma-<host>", line)
    line = re.sub(
        r"\b(imei|iccid|mcc_mnc)=(?:\"[^\"]*\"|'[^']*'|[^\s,;|]+)",
        r"\1=<redacted>", line, flags=re.IGNORECASE,
    )
    line = re.sub(
        r'("(?:ip_address|iccid|imei|mcc_mnc)"\s*:\s*)("(?:\\.|[^"\\])*"|[^\s,}\]]+)',
        r'\1"<redacted>"', line, flags=re.IGNORECASE,
    )
    replacements = {}
    for name, value in (secrets or {}).items():
        if len(value) >= 3:
            for variant in (value, json.dumps(value, ensure_ascii=True)[1:-1]):
                replacements.setdefault(variant, f"<{name}>")
    if replacements:
        pattern = "|".join(re.escape(value) for value in sorted(replacements, key=len, reverse=True))
        line = re.sub(pattern, lambda match: replacements[match.group(0)], line)

    def long_id(match):
        value = match.group(0)
        # Keep the established IMEI/ICCID masks below.
        if value.startswith("<") or re.fullmatch(r"[0-9]{16,17}|89[0-9]{17,18}|89[0-9]{16,17}[Ff]", value):
            return value
        return value[:4] + "<id>"

    line = re.sub(
        r"<(?:redacted|id|host|username|password|loc|cell|pci|tac|(?:user|password):[0-9]+)>|[0-9a-fA-F]{16,}",
        long_id, line,
    )
    line = re.sub(
        r"(?<![0-9])(?:89[0-9]{17,18}|89[0-9]{16,17}[Ff])(?![0-9Ff])",
        lambda match: match.group(0)[:7] + "<redacted>", line,
    )
    line = re.sub(r"(?<![0-9])[0-9]{14,17}(?![0-9])", lambda match: match.group(0)[:5] + "<redacted>", line)

    def ipv4(match):
        octets = match.group(0).split(".")
        return ".".join(octets[:2]) + ".x.x" if all(int(n) <= 255 for n in octets) else match.group(0)

    return re.sub(r"(?<![\d.])(?:[0-9]{1,3}\.){3}[0-9]{1,3}(?![\d.])", ipv4, line)


def load_secrets(params_dir: str | Path) -> dict[str, str]:
    secrets = {}
    for key, name in (("GsmUsername", "username"), ("GsmPassword", "password")):
        try:
            secrets[name] = (Path(params_dir) / key).read_bytes().decode("utf-8", errors="replace")
        except FileNotFoundError:
            continue
    return secrets


@contextlib.contextmanager
def modem_lock(path: str | Path, timeout: float = 10.0, poll_interval: float = 0.05):
    if not math.isfinite(timeout) or timeout < 0:
        raise ValueError("invalid lock timeout")
    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
    try:
        deadline = time.monotonic() + timeout
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("modem lock timeout") from None
                time.sleep(min(poll_interval, remaining))
        try:
            yield
        finally:
            fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


@contextlib.contextmanager
def serial_port(path: str | Path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        attrs[4] = attrs[5] = termios.B115200
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIFLUSH)
        yield fd
    finally:
        os.close(fd)


def _terminal(line: str) -> bool:
    return line in ("OK", "ERROR") or line.startswith(("+CME ERROR:", "+CMS ERROR:"))


def exchange(command: str, port="/dev/modem_at0", lock="/dev/shm/modem.lock", lock_timeout=10.0, timeout=None) -> list[str]:
    if not is_allowed_command(command):
        raise ValueError("command is not allowed")
    timeout = command_timeout(command) if timeout is None else timeout
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("invalid command timeout")
    with modem_lock(lock, lock_timeout), serial_port(port) as fd:
        deadline = time.monotonic() + timeout
        pending = memoryview((command + "\r").encode("ascii"))
        while pending:
            _, writable, _ = select.select([], [fd], [], max(0.0, deadline - time.monotonic()))
            if not writable:
                return ["TIMEOUT"]
            try:
                pending = pending[os.write(fd, pending):]
            except BlockingIOError:
                continue
        lines = []
        buffer = bytearray()
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if buffer.strip():
                    lines.append(buffer.decode("utf-8", errors="replace").strip())
                return lines + ["TIMEOUT"]
            readable, _, _ = select.select([fd], [], [], remaining)
            if not readable:
                continue
            try:
                data = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if not data:
                return lines + ["DISCONNECTED"]
            buffer.extend(data)
            while b"\n" in buffer:
                raw, _, rest = buffer.partition(b"\n")
                buffer = bytearray(rest)
                line = raw.decode("utf-8", errors="replace").strip()
                if line and line.upper() != command.upper():
                    lines.append(line)
                    if _terminal(line):
                        return lines


class SafeParser(argparse.ArgumentParser):
    def error(self, message):
        self.exit(2, "Invalid arguments. Use --help for usage.\n")


def nonnegative(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("invalid timeout")
    return number


def main(argv=None) -> int:
    parser = SafeParser(description=__doc__)
    parser.add_argument("commands", nargs="*", help="allowlisted AT queries")
    parser.add_argument("--preset", choices=tuple(PRESETS), action="append", default=[])
    parser.add_argument("--port", default="/dev/modem_at0")
    parser.add_argument("--lock", default="/dev/shm/modem.lock")
    parser.add_argument("--lock-timeout", type=nonnegative, default=10.0)
    parser.add_argument("--params-dir", default="/data/params/d")
    parser.add_argument("--secrets-from-params", action="store_true")
    parser.add_argument("--show-auth-values", action="store_true", help="developer only: show auth fields with dummy values")
    parser.add_argument("--redact-stdin", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if args.show_auth_values:
        print("showing APN auth fields unredacted: use only with dummy values", file=sys.stderr, flush=True)
        if args.secrets_from_params:
            parser.exit(2, "Cannot combine --show-auth-values with --secrets-from-params.\n")
        try:
            with (Path(args.params_dir) / "GsmPassword").open("rb") as password:
                configured = bool(password.read(1))
        except FileNotFoundError:
            configured = False
        except OSError:
            parser.exit(2, "Cannot check GsmPassword; refusing --show-auth-values.\n")
        if configured:
            parser.exit(2, "GsmPassword is non-empty; refusing --show-auth-values.\n")
    commands = [cmd for preset in args.preset for cmd in PRESETS[preset]] + args.commands
    if args.redact_stdin and (commands or args.json):
        parser.error("incompatible options")
    if not args.redact_stdin and (not commands or not all(is_allowed_command(cmd) for cmd in commands)):
        parser.exit(2, "Only allowlisted read-only AT commands are accepted.\n")
    secrets = load_secrets(args.params_dir) if args.secrets_from_params else {}
    if args.redact_stdin:
        for raw in sys.stdin.buffer:
            sys.stdout.write(redact_line(raw.decode("utf-8", errors="replace"), secrets, show_auth_values=args.show_auth_values))
        return 0
    results = []
    failed = False
    for command in commands:
        try:
            lines = exchange(command, args.port, args.lock, args.lock_timeout)
        except TimeoutError:
            lines = ["LOCK TIMEOUT"]
        except (OSError, ValueError):
            lines = ["QUERY FAILED"]
        failed |= not lines or lines[-1] != "OK"
        shown_command = redact_line(command, secrets, show_auth_values=args.show_auth_values)
        shown_lines = [redact_line(line, secrets, show_auth_values=args.show_auth_values) for line in lines]
        results.append({"command": shown_command, "lines": shown_lines})
        if not args.json:
            print(f"> {shown_command}: " + " | ".join(shown_lines))
    if args.json:
        print(json.dumps(results, ensure_ascii=True))
    return int(failed)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (Exception, KeyboardInterrupt):  # noqa: BLE001 -- Never expose modem data in a traceback.
        print("modem_at_read: operation failed; error details suppressed.", file=sys.stderr)
        sys.exit(1)
