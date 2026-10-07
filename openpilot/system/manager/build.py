#!/usr/bin/env python3
import os
import subprocess
import time
from pathlib import Path

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.spinner import Spinner
from openpilot.common.text_window import TextWindow
from openpilot.common.hardware import HARDWARE, AGNOS
from openpilot.common.time_helpers import system_time_valid

# NTP 同期の待ち時間。実測では起動から約28秒で同期する。
TIME_SYNC_TIMEOUT = 60
# ネットワークに一度も繋がらないまま諦めるまでの秒数
NO_NETWORK_TIMEOUT = 10


def has_default_route() -> bool:
  try:
    with open("/proc/net/route") as f:
      return any(len(cols) > 1 and cols[1] == "00000000" for cols in (l.split() for l in f.readlines()[1:]))
  except OSError:
    return False


def pull_big_models() -> None:
  # ビッグモデルは LFS 配布。実体が無いとポインタ(131〜134バイト)のまま存在し、modeld が
  # 読み込みに失敗して小モデルに落ちる。実体は最小の warp でも 863KB あるので桁で判別できる。
  models = Path(BASEDIR) / "openpilot/selfdrive/modeld/models"
  if not any(f.stat().st_size < 1024 for f in models.glob("big_*_tinygrad.pkl")):
    return

  # RTC のバックアップが無いので、起動直後の時刻は systemd のビルド時刻(約2ヶ月前)になる。
  # そのままだと TLS 証明書が「まだ有効でない」と判定され、LFS の取得が必ず失敗する。
  offline_since = None
  for _ in range(TIME_SYNC_TIMEOUT):
    if system_time_valid():
      break
    if has_default_route():
      offline_since = None
    else:
      # 起動直後は Wi-Fi 接続前でルートが無い。繋がる見込みが無いときだけ諦める。
      offline_since = offline_since or time.monotonic()
      if time.monotonic() - offline_since > NO_NETWORK_TIMEOUT:
        return
    time.sleep(1)

  subprocess.run(["git", "lfs", "pull"], cwd=BASEDIR, check=False)


def build() -> None:
  spinner = Spinner()
  spinner.update_progress(0, 100)

  HARDWARE.set_power_save(False)
  if AGNOS:
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  # launch_chffrplus.sh ではなくここで走らせるのは、800MB の取得をスピナーの内側に入れるため。
  pull_big_models()

  # building with all cores can result in using too much memory, so retry serially
  compile_output: list[bytes] = []
  for parallelism in ([], ["-j4"], ["-j1"]):
    compile_output.clear()
    with subprocess.Popen(["scons", *parallelism], cwd=BASEDIR, env={**os.environ, "PWD": BASEDIR}, stderr=subprocess.PIPE) as scons:
      assert scons.stderr is not None

      # Read progress from stderr and update spinner
      while scons.poll() is None:
        try:
          line = scons.stderr.readline()
          if line is None:
            continue
          line = line.rstrip()

          prefix = b'progress: '
          if line.startswith(prefix):
            progress = float(line[len(prefix):])
            spinner.update_progress(100 * min(1., progress / 100.), 100.)
          elif len(line):
            compile_output.append(line)
            print(line.decode('utf8', 'replace'))
        except Exception:
          pass

      # Drain and close the pipe before retrying or returning.
      for line in scons.stderr.read().split(b'\n'):
        line = line.rstrip()
        if len(line):
          compile_output.append(line)

    if scons.returncode == 0:
      break

  os.sync()

  if scons.returncode != 0:
    # Build failed log errors
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')

    # Show TextWindow
    spinner.close()
    if not os.getenv("CI"):
      with TextWindow("openpilot failed to build\n \n" + error_s) as t:
        t.wait_for_exit()
    exit(1)

if __name__ == "__main__":
  build()
