#!/usr/bin/env python3
import csv
import fcntl
import glob
import json
import logging
import os
import select
import signal
import string
import struct
import subprocess
import tempfile
import termios
import time

from contextlib import contextmanager
from ipaddress import IPv4Address, AddressValueError

from enum import Enum

from openpilot.common.esim.base import WEBBING_ICCID_PREFIX

logging.basicConfig(
  level=logging.INFO,
  format="%(asctime)s.%(msecs)03d %(levelname)-7s modem: %(message)s",
  datefmt="%H:%M:%S",
)

AT_PORT = "/dev/modem_at0"
PPP_PORT = "/dev/modem_at1"
STATE_PATH = "/dev/shm/modem"
AT_LOCK = "/dev/shm/modem.lock"  # shared with LPA
AT_INIT = [
  "ATE0",       # disable command echo
  "ATV1",       # verbose result codes (CONNECT/BUSY/NO CARRIER, not numeric)
  "AT+CMEE=1",  # numeric +CME ERROR codes on failures (per 3GPP 27.007)
  "ATX4",       # extended result codes: busy + dial tone detection, line speed in CONNECT
  "AT&C1",      # DCD pin follows carrier state (V.250 default)
  "AT+CREG=2",  # registration URCs include location info
  "AT+CGREG=2", # GPRS registration URCs include location info
]
CREG = {0: "not_registered", 1: "home", 2: "searching", 3: "denied", 4: "unknown", 5: "roaming"}
# 3GPP TS 27.007 +COPS <AcT> -> network type
NETWORK_TYPE = {0: "gsm", 1: "gsm", 3: "gsm", 8: "gsm",
                2: "utran", 4: "utran", 5: "utran", 6: "utran",
                7: "lte", 9: "lte", 10: "lte",
                11: "nr", 12: "nr", 13: "nr"}

DIAL_CID = 1
AUTH_STALL_TIMEOUT = 60.0
AUTH_PROTOCOLS = {0: "none", 1: "pap", 2: "chap"}

PPPD_CMD = [
  "sudo", "pppd", PPP_PORT, "460800", "noauth", "nodetach", "noipdefault", "usepeerdns",
  "nodefaultroute", "connect",
  "/usr/sbin/chat -v ABORT 'NO CARRIER' ABORT 'NO DIALTONE' ABORT 'BUSY' " +
  f"ABORT 'NO ANSWER' ABORT 'ERROR' TIMEOUT 5 '' AT OK ATD*99***{DIAL_CID}# CONNECT ''",
  "lcp-echo-interval", "30", "lcp-echo-failure", "4", "mtu", "1500", "mru", "1500",
  "novj", "novjccomp", "ipcp-accept-local", "ipcp-accept-remote", "nomagic",
  "user", '""', "password", '""',
]
INITIAL_STATE: dict[str, object] = {
  "seconds_since_boot": 0,
  "state": "INITIALIZING",
  "connected": False, "ip_address": "",
  "iccid": "", "mcc_mnc": "", "imei": "", "modem_version": "",
  "signal_strength": 0, "signal_quality": 0,
  "network_type": "unknown", "operator": "", "band": "", "channel": 0,
  "registration": "unknown", "temperatures": [], "extra": "",
  "tx_bytes": 0, "rx_bytes": 0,
}


def _resolve_auth(auth: str, user: str, password: str) -> tuple[int, str, str] | None:
  if not auth:
    return None
  if auth == "none":
    return 0, "", ""
  if auth not in ("chap", "pap"):
    raise ValueError("GsmAuth not understood")
  for field, value in (("GsmUsername", user), ("GsmPassword", password)):
    if any(not 0x21 <= ord(c) <= 0x7e or c in '"\\,' for c in value):
      raise ValueError(f"{field} rejected (unsupported character)")
    if len(value) > 64:  # validated ASCII, so bytes equal characters
      raise ValueError(f"{field} rejected (more than 64 bytes)")
  return (1 if auth == "pap" else 2), user, password


def _cleanup_pppd_auth():
  for path in glob.glob("/dev/shm/modem_pppd_auth_*"):
    try:
      os.remove(path)
    except FileNotFoundError:
      pass
    except OSError:
      logging.warning("failed to remove stale PPP auth file")


@contextmanager
def _serial_port(port: str, baudrate: int):
  fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
  try:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = attrs[5] = getattr(termios, f"B{baudrate}")
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    yield fd
  finally:
    os.close(fd)


def _read_line(fd: int, timeout: float) -> bytes:
  data = bytearray()
  deadline = time.monotonic() + timeout
  while True:
    readable, _, _ = select.select([fd], [], [], max(0.0, deadline - time.monotonic()))
    if not readable:
      return bytes(data)
    byte = os.read(fd, 1)
    if not byte:
      return bytes(data)
    data.extend(byte)
    if byte == b"\n":
      return bytes(data)


class State(Enum):
  INITIALIZING = "INITIALIZING"
  SEARCHING = "SEARCHING"
  CONNECTING = "CONNECTING"
  CONNECTED = "CONNECTED"
  DISCONNECTING = "DISCONNECTING"


STATE_WAIT = 1.0  # seconds to wait after each state handler returns


class PPPSession:
  """Owns pppd lifecycle, fail tracking, and PPP routing."""
  MAX_FAILS = 3

  def __init__(self):
    self._proc: subprocess.Popen | None = None
    self._fails = 0
    self._peer = ""
    self._auth_path: str | None = None

  def start(self, auth: tuple[int, str, str] | None = None):
    self._remove_auth_file()
    cmd = PPPD_CMD
    if auth is not None and auth[0] in (1, 2):
      fd = None
      try:
        fd, self._auth_path = tempfile.mkstemp(dir="/dev/shm", prefix="modem_pppd_auth_")
        with os.fdopen(fd, "w") as f:
          f.write(f'user "{auth[1]}"\npassword "{auth[2]}"\n')
        cmd = PPPD_CMD[:-4] + ["file", self._auth_path]
      except Exception:
        logging.warning("failed to create PPP auth file; using default options")
        if fd is not None:
          try:
            os.close(fd)
          except OSError:
            pass
        self._remove_auth_file()
    self._proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    self._peer = ""
    logging.info(f"PPP dialing CID {DIAL_CID}")

  def kill(self):
    self._remove_auth_file()
    subprocess.run(["sudo", "killall", "-9", "pppd"], capture_output=True)
    self._peer = ""

  def _remove_auth_file(self):
    if self._auth_path is not None:
      try:
        os.remove(self._auth_path)
        self._auth_path = None
      except FileNotFoundError:
        self._auth_path = None
      except OSError:
        logging.warning("failed to remove PPP auth file")

  @staticmethod
  def reset_data_port():
    """Drop DTR on PPP_PORT so the modem terminates any stuck PPP session."""
    try:
      with _serial_port(PPP_PORT, 460800) as fd:
        dtr = struct.pack("I", termios.TIOCM_DTR)
        fcntl.ioctl(fd, termios.TIOCMBIC, dtr)
        time.sleep(0.2)
        fcntl.ioctl(fd, termios.TIOCMBIS, dtr)
    except Exception as e:
      logging.warning(f"data port reset failed: {e}")

  def has_exited(self) -> bool:
    return self._proc is not None and self._proc.poll() is not None

  def reset_fail_counter(self):
    self._fails = 0

  def record_fail(self) -> bool:
    """Bump fail counter; return True if at the give-up limit."""
    self._fails += 1
    return self._fails >= self.MAX_FAILS

  @property
  def fails(self) -> int:
    return self._fails

  def maybe_install_routes(self, ip: str, peer: str) -> bool:
    """Install routes if peer changed; kill the session on failure so the state machine reconnects."""
    if not peer or peer == self._peer:
      return False
    try:
      IPv4Address(ip)
      IPv4Address(peer)
    except AddressValueError:
      logging.warning(f"refusing route install with non-IPv4 ip={ip!r} peer={peer!r}")
      self.kill()
      return False
    self.cleanup_routes()
    cmds = [
      ["sudo", "ip", "route", "add", "default", "via", peer, "dev", "ppp0", "metric", "1000"],
      ["sudo", "ip", "route", "add", "default", "via", peer, "dev", "ppp0", "table", "1000"],
      ["sudo", "ip", "rule", "add", "from", ip, "table", "1000"],
    ]
    for cmd in cmds:
      r = subprocess.run(cmd, capture_output=True, text=True)
      if r.returncode != 0:
        logging.warning(f"route install failed ({' '.join(cmd[1:])}): {r.stderr.strip()}")
        self.cleanup_routes()
        self.kill()
        return False
    logging.info(f"route set up for {ip} via {peer}")
    self._peer = peer
    return True

  def maybe_install_dns(self, dns_servers: list[str]) -> bool:
    """Register DNS servers with systemd-resolved; kill the session on failure to force a retry."""
    if not dns_servers:
      return False
    for cmd in (["sudo", "resolvectl", "dns", "ppp0", *dns_servers],
                ["sudo", "resolvectl", "default-route", "ppp0", "yes"]):
      r = subprocess.run(cmd, capture_output=True, text=True)
      if r.returncode != 0:
        logging.warning(f"resolvectl failed ({' '.join(cmd[1:])}): {r.stderr.strip()}")
        self.kill()
        return False
    logging.info(f"resolvectl: ppp0 DNS = {dns_servers}")
    return True

  @staticmethod
  def cleanup_routes():
    subprocess.run(["sudo", "ip", "route", "del", "default", "dev", "ppp0"], capture_output=True)
    subprocess.run(["sudo", "ip", "route", "flush", "table", "1000"], capture_output=True)
    # rules don't have a flush; delete until none remain
    while subprocess.run(["sudo", "ip", "rule", "del", "table", "1000"], capture_output=True).returncode == 0:
      pass
    subprocess.run(["sudo", "resolvectl", "revert", "ppp0"], capture_output=True)


class Modem:
  def __init__(self):
    self._ppp = PPPSession()
    self._sim_change = False
    self._apn = ""  # blank = network-provided via PCO
    self._roaming_allowed = True
    self._reattach_tries = 0  # re-registrations left to apply APN/auth to DIAL_CID
    self._auth: tuple[int, str, str] | None = None
    self._auth_command: str | None = None
    self._auth_probe_tries = 0
    self._auth_unavailable = False
    self._auth_warning = ""
    self._auth_changed = False
    self._reattach_on_stall = False
    self._stall_since: float | None = None
    self.running = True
    self.S = INITIAL_STATE.copy()

  @staticmethod
  def _read_param(key):
    try:
      with open(f"/data/params/d/{key}") as f:
        return f.read().strip()
    except FileNotFoundError:
      return ""

  @staticmethod
  def _parse_reg(v: str) -> str:
    try:
      return CREG.get(int(v.split(",")[1].strip('"')), "unknown")
    except (ValueError, IndexError):
      return "unknown"

  @staticmethod
  def _has_modem_manager() -> bool:
    return os.path.isfile("/lib/systemd/system/ModemManager.service")

  def _is_roaming_allowed(self) -> bool:
    if self.S["iccid"].startswith(WEBBING_ICCID_PREFIX):
      return True
    return self._read_param("GsmRoaming") == "1"

  def _publish_state(self, **kwargs):
    reg = kwargs.get("registration")
    if reg != self.S["registration"] and reg in ("denied", "searching", "not_registered"):
      logging.info(f"last network cause (AT+CEER, may be stale): {self._at('AT+CEER')}")
    self.S.update(kwargs)
    self.S["seconds_since_boot"] = time.monotonic()
    with tempfile.NamedTemporaryFile(mode="w", dir="/dev/shm", delete=False) as f:
      json.dump(self.S, f, indent=2)
    os.chmod(f.name, 0o644)
    os.replace(f.name, STATE_PATH)

  def _at(self, cmd, shown=None):
    """Send AT command, return response lines. [] on error or if LPA holds port."""
    fd = os.open(AT_LOCK, os.O_CREAT | os.O_RDWR, 0o666)
    try:
      fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
      os.close(fd)
      return []
    try:
      with _serial_port(AT_PORT, 9600) as serial_fd:
        termios.tcflush(serial_fd, termios.TCIFLUSH)
        command = (cmd + "\r").encode()
        while command:
          command = command[os.write(serial_fd, command):]
        lines = []
        while True:
          raw = _read_line(serial_fd, 5)
          if not raw:
            raise TimeoutError("AT timeout")
          line = raw.decode(errors="ignore").strip()
          if not line:
            continue
          if line == "OK":
            break
          if line == "ERROR" or line.startswith("+CME ERROR"):
            raise RuntimeError(line)
          lines.append(line)
        return lines
    except (RuntimeError, TimeoutError, OSError) as e:
      logging.info(f"AT {shown if shown is not None else cmd} failed: {type(e).__name__ if shown is not None else e}")
      return []
    finally:
      fcntl.flock(fd, fcntl.LOCK_UN)
      os.close(fd)

  def _atv(self, cmd, pfx):
    for line in self._at(cmd):
      if pfx in line and ":" in line:
        return line.split(":", 1)[1].strip()
    return None

  def _init_at_channel(self) -> bool:
    """Run AT_INIT and confirm ATE0 took effect. Returns False if echo is still on."""
    for c in AT_INIT:
      self._at(c)
    r = self._at("AT+CGMI")
    return bool(r) and not r[0].startswith("AT")

  def _configure_modem(self, modem_version: str):
    if not modem_version.startswith("EG25"):
      return
    cmds = [
      # clear initial EPS bearer APN (some carriers reject the default)
      'AT+CGDCONT=0,"IP",""',

      # SIM hot swap
      'AT+QSIMDET=1,0',
      'AT+QSIMSTAT=1',

      # configure modem as data-centric
      'AT+QNVW=5280,0,"0102000000000000"',
      'AT+QNVFW="/nv/item_files/ims/IMS_enable",00',
      'AT+QNVFW="/nv/item_files/modem/mmode/ue_usage_setting",01',
    ]
    for c in cmds:
      self._at(c)

  def _do_initializing(self):
    if not os.path.exists(AT_PORT):
      return State.INITIALIZING
    logging.info("port found, initializing")
    self._ppp.kill()
    self._ppp.cleanup_routes()

    if not self._init_at_channel():
      logging.warning("AT echo still on, retrying")
      return State.INITIALIZING

    # an interrupted re-registration can leave the modem deregistered
    if self._cops_mode() == "2":
      self._at("AT+COPS=0")

    identity = self._read_identity()
    if not identity["iccid"] or not identity["imei"]:
      logging.warning(f"identity read incomplete: {identity}, retrying")
      return State.INITIALIZING

    self._configure_modem(identity["modem_version"])

    self.S.update(identity)
    self._apn = self._read_param("GsmApn")
    self._roaming_allowed = self._is_roaming_allowed()
    self._auth = self._read_auth()
    if self._auth is None:
      self._reattach_on_stall = False
    self._auth_command = None
    if self._auth_changed and self._auth is not None:
      self._arm_auth_reattach()
    self._auth_changed = False
    if self._auth is not None and not self._auth_unavailable:
      self._auth_command = self._select_auth_command(identity["modem_version"])
      if self._auth_command is None:
        if self._auth_probe_tries < 3:
          self._auth_probe_tries += 1
          return State.INITIALIZING
        logging.warning("modem has no APN auth command")
        self._auth_unavailable = True
      else:
        self._auth_probe_tries = 0
        profile = self._read_auth_profile()
        if profile is not None and (profile[:2] != self._auth[:2] or (profile[2] is not None and profile[2] != self._auth[2])):
          self._arm_auth_reattach()
    # the modem attaches with the APN it has stored, so a new one only applies on its next attach
    if self._read_dial_apn() != self._apn:
      self._reattach_tries = 2
    # blank APN lets the carrier supply one via PCO
    self._at(f'AT+CGDCONT={DIAL_CID},"IP","{self._apn}"')
    self._write_auth()
    logging.info(f"APN '{self._apn or '(network-provided)'}' written to CID {DIAL_CID}, roaming={'on' if self._roaming_allowed else 'off'}")

    self._sim_change = False  # clear since we just re-read identity with the new SIM
    self._publish_state(**identity)
    return State.SEARCHING

  def _read_identity(self):
    def first_line(cmd):
      r = self._at(cmd)
      return r[0].strip() if r else ""

    imei = first_line("AT+CGSN")
    if not (imei.isdigit() and 14 <= len(imei) <= 17):  # 3GPP TS 23.003
      imei = ""

    iccid = (self._atv("AT+QCCID", "+QCCID:") or "").rstrip("F")
    if not all(c in string.hexdigits for c in iccid):
      iccid = ""

    imsi = first_line("AT+CIMI")
    mcc_mnc = imsi[:6] if imsi.isdigit() and len(imsi) >= 6 else ""

    modem_version = first_line("AT+GMR")

    logging.info(f"imei={imei} iccid={iccid} mcc_mnc={mcc_mnc} ver={modem_version}")
    return {"imei": imei, "iccid": iccid, "mcc_mnc": mcc_mnc, "modem_version": modem_version}

  def _read_dial_apn(self) -> str | None:
    v = self._atv("AT+CGDCONT?", f"+CGDCONT: {DIAL_CID},")
    fields = v.split(",") if v else []
    return fields[2].strip('"') if len(fields) > 2 else None

  def _read_auth(self) -> tuple[int, str, str] | None:
    try:
      auth_param, user, password = (self._read_param(key) for key in ("GsmAuth", "GsmUsername", "GsmPassword"))
      auth = _resolve_auth(auth_param, user, password)
      if not auth_param and (user or password):
        raise ValueError("GsmUsername/GsmPassword set but GsmAuth is empty")
    except ValueError as e:
      warning = str(e)
      if warning != self._auth_warning:
        logging.warning(warning)
      self._auth_warning = warning
      return None
    self._auth_warning = ""
    return auth

  def _select_auth_command(self, modem_version: str) -> str | None:
    commands = ("QICSGP", "CGAUTH") if modem_version.startswith("EG25") else ("CGAUTH", "QICSGP")
    for command in commands:
      if any(line.startswith(f"+{command}:") for line in self._at(f"AT+{command}=?")):
        return command
    return None

  def _read_auth_profile(self) -> tuple[int, str, str | None] | None:
    cmd = "AT+CGAUTH?" if self._auth_command == "CGAUTH" else f"AT+QICSGP={DIAL_CID}"
    prefix = f"+CGAUTH: {DIAL_CID}," if self._auth_command == "CGAUTH" else "+QICSGP:"
    for line in self._at(cmd, shown=cmd):
      if not line.startswith(prefix):
        continue
      try:
        fields = next(csv.reader([line.split(":", 1)[1].strip()], skipinitialspace=True, strict=True))
        if self._auth_command == "CGAUTH":
          protocol, user = int(fields[1]), fields[2] if len(fields) > 2 else ""
          password = fields[3] if len(fields) > 3 else None
        else:
          protocol, user, password = int(fields[4]), fields[2], fields[3]
        return protocol, user, password if protocol == 0 else password or None
      except (ValueError, IndexError, csv.Error):
        return None
    return None

  def _arm_auth_reattach(self):
    self._reattach_tries = 2
    self._reattach_on_stall = True
    self._stall_since = None
    logging.info("APN auth requires re-attach")

  def _write_auth(self):
    if self._auth is None or self._auth_command is None:
      return
    protocol, user, password = self._auth
    if self._auth_command == "CGAUTH":
      cmd = f'AT+CGAUTH={DIAL_CID},0,"",""' if protocol == 0 else f'AT+CGAUTH={DIAL_CID},{protocol},"{user}","{password}"'
    else:
      cmd = f'AT+QICSGP={DIAL_CID},1,"{self._apn}","{user}","{password}",{protocol}'
    self._at(cmd, shown=f"AT+{self._auth_command}={DIAL_CID},<redacted>")
    profile = self._read_auth_profile()
    result = "readback failed"
    if profile is not None:
      password_ok = f"password ok={profile[2] == password}" if profile[2] is not None else "password not shown"
      result = f"readback protocol ok={profile[0] == protocol}, user ok={profile[1] == user}, {password_ok}"
    logging.info(f"APN auth via {self._auth_command}: protocol={AUTH_PROTOCOLS[protocol]}, {result}")

  def _do_searching(self):
    new_roaming = self._is_roaming_allowed()
    if new_roaming != self._roaming_allowed:
      logging.info(f"roaming changed: {self._roaming_allowed} -> {new_roaming}")
      self._roaming_allowed = new_roaming

    # roaming was just synced; INITIALIZING writes the new APN and auth
    if self._params_changed():
      return State.DISCONNECTING

    v = self._atv("AT+CREG?", "+CREG:")
    if not v:
      return self._searching_idle()

    reg = self._parse_reg(v)
    greg = self._parse_reg(self._atv("AT+CGREG?", "+CGREG:") or "")
    logging.debug(f"creg={reg} cgreg={greg} roaming_allowed={self._roaming_allowed}")

    if reg not in ("searching", "not_registered"):
      self._stall_since = None
    elif self._reattach_on_stall and self._stall_since is None:
      self._stall_since = time.monotonic()

    if reg == "denied" and self._reattach_tries > 0:
      self._publish_state(registration=reg)
      return self._reattach()

    if self._reattach_on_stall and self._reattach_tries > 0 and self._stall_since is not None:
      if time.monotonic() - self._stall_since >= AUTH_STALL_TIMEOUT:
        self._publish_state(registration=reg)
        return self._reattach(stalled=True)

    if reg == "roaming" and not self._roaming_allowed:
      self._publish_state(registration=reg)
      return State.SEARCHING

    if reg in ("home", "roaming") and greg in ("home", "roaming"):
      self._publish_state(registration=reg)
      return State.CONNECTING

    if reg != self.S.get("registration"):
      self._publish_state(registration=reg)
    return self._searching_idle()

  def _cops_mode(self) -> str | None:
    v = self._atv("AT+COPS?", "+COPS:")
    return v.split(",", 1)[0].strip() if v else None

  def _reattach(self, stalled=False):
    reason = "stalled" if stalled else "denied"
    logging.info(f"registration {reason}, re-attaching with APN '{self._apn or '(network-provided)'}'")
    self._reattach_tries -= 1
    self._at("AT+COPS=2")  # deregister
    if self._cops_mode() == "2":
      self._reattach_tries = 0
    # context is inactive while deregistered; the INITIALIZING write may have been lost to the AT lock
    self._at(f'AT+CGDCONT={DIAL_CID},"IP","{self._apn}"')
    self._write_auth()
    self._at("AT+COPS=0")  # automatic registration, attaches with the APN in DIAL_CID
    self._reattach_on_stall &= self._reattach_tries > 0
    self._stall_since = time.monotonic() if self._reattach_on_stall else None
    if self._cops_mode() != "0":
      return State.DISCONNECTING  # AT port busy; INITIALIZING sends AT+COPS=0
    return State.SEARCHING

  def _searching_idle(self):
    if self._sim_change or not os.path.exists(AT_PORT):
      logging.info(f"-> reconnecting (sim_change={self._sim_change} port={os.path.exists(AT_PORT)})")
      return State.DISCONNECTING
    return State.SEARCHING

  def _do_connecting(self):
    logging.info("starting pppd")
    self._ppp.reset_fail_counter()
    self._sim_change = False
    self._reattach_tries = 0  # registered, nothing left to apply
    self._reattach_on_stall = False
    self._stall_since = None
    self._ppp.start(self._auth)
    return State.CONNECTED

  def _handle_pppd_exit(self):
    if self._sim_change or not os.path.exists(AT_PORT):
      return State.DISCONNECTING
    give_up = self._ppp.record_fail()
    if give_up:
      logging.warning(f"PPP fail {self._ppp.fails}/{self._ppp.MAX_FAILS}, reconnecting")
      return State.DISCONNECTING
    logging.warning(f"PPP fail {self._ppp.fails}/{self._ppp.MAX_FAILS}, retrying")
    self._ppp.reset_data_port()
    if not os.path.exists(AT_PORT):
      return State.DISCONNECTING
    self._ppp.start(self._auth)
    return State.CONNECTED

  def _params_changed(self) -> bool:
    auth_changed = self._read_auth() != self._auth
    if auth_changed:
      logging.info("APN auth settings changed")
      self._auth_changed = True
    new_apn = self._read_param("GsmApn")
    if new_apn != self._apn:
      logging.info(f"GsmApn changed: '{self._apn}' -> '{new_apn}'")
      return True
    new_roaming = self._is_roaming_allowed()
    if new_roaming != self._roaming_allowed:
      logging.info(f"roaming changed: {self._roaming_allowed} -> {new_roaming}")
      return True
    return auth_changed

  def _check_iccid(self, state):
    if state in (State.INITIALIZING, State.DISCONNECTING) or not self.S["iccid"]:
      return
    iccid = (self._atv("AT+QCCID", "+QCCID:") or "").rstrip("F")
    if iccid and iccid != self.S["iccid"]:
      logging.warning(f"iccid changed: {self.S['iccid']} -> {iccid}")
      self._sim_change = True

  def _do_connected(self):
    if self._ppp.has_exited():
      return self._handle_pppd_exit()

    if self._sim_change or not os.path.exists(AT_PORT) or self._params_changed():
      return State.DISCONNECTING

    self._poll()
    return State.CONNECTED

  def _do_disconnecting(self):
    logging.warning("reconnecting")
    self._publish_state(**INITIAL_STATE)
    self._ppp.kill()
    self._ppp.cleanup_routes()
    self._ppp.reset_data_port()
    self._sim_change = False
    self._stall_since = None
    return State.INITIALIZING

  def _poll_signal(self) -> dict:
    v = self._atv("AT+CSQ", "+CSQ:")
    if not v:
      return {}
    try:
      rssi = int(v.split(",")[0])
      if rssi == 99:
        return {}
      return {"signal_strength": rssi, "signal_quality": min(100, int(rssi / 31 * 100))}
    except (ValueError, IndexError):
      return {}

  def _poll_operator(self) -> dict:
    v = self._atv("AT+COPS?", "+COPS:")
    if not v:
      return {}
    p = v.split(",")
    out: dict = {}
    try:
      if len(p) >= 3:
        out["operator"] = p[2].strip('"')
      if len(p) >= 4:
        out["network_type"] = NETWORK_TYPE.get(int(p[3]), "unknown")
    except (ValueError, IndexError):
      pass
    return out

  def _poll_band(self) -> dict:
    v = self._atv("AT+QNWINFO", "+QNWINFO:")
    if not v:
      return {}
    info = v.replace('"', '').split(",")
    try:
      if len(info) >= 4:
        return {"band": info[2], "channel": int(info[3])}
    except ValueError:
      pass
    return {}

  def _poll_extra(self) -> dict:
    v = self._atv('AT+QENG="servingcell"', "+QENG:")
    return {"extra": v.replace('"', '')} if v else {}

  def _poll_temps(self) -> dict:
    v = self._atv("AT+QTEMP", "+QTEMP:")
    if not v:
      return {}
    try:
      return {"temperatures": [t for t in (int(x) for x in v.split(",") if x.strip()) if t != 255]}
    except (ValueError, IndexError):
      return {}

  def _poll_iface(self) -> dict:
    try:
      r = subprocess.run(["ip", "-4", "addr", "show", "ppp0"], capture_output=True, text=True, timeout=2)
      ip, peer = "", ""
      for line in r.stdout.splitlines():
        # `inet 10.x.x.x peer 10.64.64.64/32 ...`
        parts = line.strip().split()
        if "inet" in parts:
          i = parts.index("inet")
          ip = parts[i + 1].split("/")[0]
          if "peer" in parts:
            peer = parts[parts.index("peer") + 1].split("/")[0]
          break
      if ip:
        if self._ppp.maybe_install_routes(ip, peer):
          self._ppp.maybe_install_dns(self._read_cellular_dns())
        return {"ip_address": ip, "connected": True}
      if self.S["connected"]:
        return {"connected": False, "ip_address": ""}
    except Exception:
      pass
    return {}

  def _read_cellular_dns(self) -> list[str]:
    v = self._atv(f"AT+CGCONTRDP={DIAL_CID}", "+CGCONTRDP:")
    if not v:
      return []
    # +CGCONTRDP: <cid>,<bearer_id>,<apn>,<local_addr>,<gw_addr>,<dns_prim>,<dns_sec>,...
    fields = [f.strip().strip('"') for f in v.split(",")]
    dns_servers = []
    for d in fields[5:7]:
      try:
        dns_servers.append(str(IPv4Address(d)))
      except (AddressValueError, ValueError):
        pass
    if not dns_servers:
      dns_servers = [
        "8.8.8.8",  # Google
        "1.1.1.1",  # Cloudflare
      ]
      logging.warning(f"no cellular DNS servers reported by modem: {v!r}; using fallback {dns_servers}")
    return dns_servers

  def _poll_byte_counters(self) -> dict:
    try:
      with open("/sys/class/net/ppp0/statistics/tx_bytes") as f:
        tx = int(f.read().strip())
      with open("/sys/class/net/ppp0/statistics/rx_bytes") as f:
        rx = int(f.read().strip())
    except Exception:
      return {}
    return {"tx_bytes": tx, "rx_bytes": rx}

  def _poll(self):
    s: dict = {}
    for fn in (self._poll_signal, self._poll_operator, self._poll_band,
               self._poll_extra, self._poll_temps, self._poll_iface,
               self._poll_byte_counters):
      s.update(fn())
    if s:
      self._publish_state(**s)

  def run(self):
    logging.info("starting")
    _cleanup_pppd_auth()
    self._publish_state(state=State.INITIALIZING.value)
    if self._has_modem_manager():
      subprocess.run(["sudo", "systemctl", "mask", "--runtime", "ModemManager"], capture_output=True)
      subprocess.run(["sudo", "systemctl", "stop", "ModemManager"], capture_output=True)
    self._ppp.kill()

    state = State.INITIALIZING

    handlers = {
      State.INITIALIZING: self._do_initializing,
      State.SEARCHING: self._do_searching,
      State.CONNECTING: self._do_connecting,
      State.CONNECTED: self._do_connected,
      State.DISCONNECTING: self._do_disconnecting,
    }

    while self.running:
      try:
        self._check_iccid(state)
        prev = state
        state = handlers[state]()
        if state != prev:
          self._publish_state(state=state.value)
          logging.info(f"{prev.value} -> {state.value}")
      except Exception:
        logging.exception(f"error in {state.value}")
        state = State.DISCONNECTING
      time.sleep(STATE_WAIT)

  def stop(self):
    self.running = False
    self._ppp.kill()
    self._ppp.cleanup_routes()
    try:
      os.remove(STATE_PATH)
    except FileNotFoundError:
      pass
    if self._has_modem_manager():
      subprocess.run(["sudo", "systemctl", "unmask", "--runtime", "ModemManager"], capture_output=True)
      subprocess.run(["sudo", "systemctl", "start", "ModemManager"], capture_output=True)


def main():
  m = Modem()

  def _sig(*_):
    m.running = False

  signal.signal(signal.SIGINT, _sig)
  signal.signal(signal.SIGTERM, _sig)
  m.run()
  m.stop()


if __name__ == "__main__":
  main()
