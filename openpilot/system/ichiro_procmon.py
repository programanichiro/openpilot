#!/usr/bin/env python3
# modeld の資源の漏れを調べるための監視プロセス（イチロウパイロット独自）。
# tinygrad の更新後、big モデル（chestnut）で走ると modeld が毎回約9分で SIGSEGV になる。
# フレームごとに何かが漏れて上限に達している疑いがあるので、走行中に30秒ごとに次を CSV に書き出す。
#   - modeld と dmonitoringmodeld の mmap の数（/proc/<pid>/maps の行数）、開いているファイルの数、メモリ、スレッド数
#   - 端末全体の空きメモリ、開いているファイルの総数、mmap の数が最も多いプロセス
# modeld が落ちても記録が残るよう、1行ごとにディスクへ書き出す。
import os
import time
from datetime import datetime

OUT_DIR = "/data/media/0/ichiro_procmon"
INTERVAL = 30.0
KEEP_FILES = 20
TARGETS = ("openpilot.selfdrive.modeld.modeld", "openpilot.selfdrive.modeld.dmonitoringmodeld")

COLUMNS = [
  "time", "uptime_s",
  "modeld_pid", "modeld_age_s", "modeld_maps", "modeld_fds", "modeld_rss_kb", "modeld_vsz_kb", "modeld_threads",
  "dm_pid", "dm_maps", "dm_fds", "dm_rss_kb",
  "mem_available_kb", "files_open", "top_maps_name", "top_maps_pid", "top_maps",
]


def _read(path: str) -> str:
  try:
    with open(path, "rb") as f:
      return f.read().decode(errors="replace")
  except OSError:
    return ""


def _count_lines(path: str) -> int:
  try:
    with open(path, "rb") as f:
      return sum(1 for _ in f)
  except OSError:
    return -1


def _count_fds(pid: int) -> int:
  try:
    return len(os.listdir(f"/proc/{pid}/fd"))
  except OSError:
    return -1


def _status(pid: int) -> dict[str, str]:
  values = {}
  for line in _read(f"/proc/{pid}/status").splitlines():
    key, _, value = line.partition(":")
    values[key] = value.strip().split(" ")[0]
  return values


def _cmdline(pid: int) -> str:
  return _read(f"/proc/{pid}/cmdline").replace("\0", " ").strip()


def _uptime() -> float:
  try:
    return float(_read("/proc/uptime").split()[0])
  except (IndexError, ValueError):
    return -1.0


def _age(pid: int, uptime: float) -> float:
  # /proc/<pid>/stat の22番目（起動時刻、クロック単位）からプロセスの経過秒を出す
  stat = _read(f"/proc/{pid}/stat")
  try:
    start_ticks = int(stat.rsplit(")", 1)[1].split()[19])
    return round(uptime - start_ticks / os.sysconf("SC_CLK_TCK"), 1)
  except (IndexError, ValueError):
    return -1.0


def _pids() -> list[int]:
  return [int(name) for name in os.listdir("/proc") if name.isdigit()]


def sample() -> dict[str, object]:
  uptime = _uptime()
  row: dict[str, object] = {"time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"), "uptime_s": round(uptime, 1)}
  found: dict[str, int] = {}
  top = ("", -1, -1)
  for pid in _pids():
    cmd = _cmdline(pid)
    for target in TARGETS:
      if target in cmd and target not in found and "ichiro_procmon" not in cmd:
        found[target] = pid
    maps = _count_lines(f"/proc/{pid}/maps")
    if maps > top[2]:
      top = (cmd.split(" ")[0][-40:], pid, maps)

  if (pid := found.get(TARGETS[0])) is not None:
    status = _status(pid)
    row |= {"modeld_pid": pid, "modeld_age_s": _age(pid, uptime), "modeld_maps": _count_lines(f"/proc/{pid}/maps"),
            "modeld_fds": _count_fds(pid), "modeld_rss_kb": status.get("VmRSS", ""), "modeld_vsz_kb": status.get("VmSize", ""),
            "modeld_threads": status.get("Threads", "")}
  if (pid := found.get(TARGETS[1])) is not None:
    status = _status(pid)
    row |= {"dm_pid": pid, "dm_maps": _count_lines(f"/proc/{pid}/maps"), "dm_fds": _count_fds(pid), "dm_rss_kb": status.get("VmRSS", "")}

  for line in _read("/proc/meminfo").splitlines():
    if line.startswith("MemAvailable:"):
      row["mem_available_kb"] = line.split()[1]
  row["files_open"] = _read("/proc/sys/fs/file-nr").split("\t")[0].strip()
  row["top_maps_name"], row["top_maps_pid"], row["top_maps"] = top
  return row


def _rotate() -> None:
  files = sorted(name for name in os.listdir(OUT_DIR) if name.startswith("procmon_") and name.endswith(".csv"))
  for name in files[:-KEEP_FILES]:
    try:
      os.remove(os.path.join(OUT_DIR, name))
    except OSError:
      pass


def main() -> None:
  os.makedirs(OUT_DIR, exist_ok=True)
  path = os.path.join(OUT_DIR, f"procmon_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
  with open(path, "w", buffering=1) as f:
    f.write(f"# vm.max_map_count={_read('/proc/sys/vm/max_map_count').strip()} ulimit_nofile={os.sysconf('SC_OPEN_MAX')}\n")
    f.write(",".join(COLUMNS) + "\n")
    _rotate()
    while True:
      try:
        row = sample()
        f.write(",".join(str(row.get(column, "")) for column in COLUMNS) + "\n")
        os.fsync(f.fileno())
      except Exception as e:
        f.write(f"# error: {e!r}\n")
      time.sleep(INTERVAL)


if __name__ == "__main__":
  main()
