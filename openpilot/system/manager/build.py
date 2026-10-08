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

# NTP 同期を待つ上限。実測では起動から約28秒で同期する。Wi-Fi の接続自体に約12秒かかるので、
# 「ルートが無いから諦める」という早期判定は入れない。オフラインでもこの秒数で頭打ちになる。
TIME_SYNC_TIMEOUT = 60


def pull_big_models() -> None:
  # ビッグモデルは LFS 配布。実体が無いとポインタ(131〜134バイト)のまま存在し、modeld が
  # 読み込みに失敗して小モデルに落ちる。実体は最小の warp でも 863KB あるので桁で判別できる。
  models = Path(BASEDIR) / "openpilot/selfdrive/modeld/models"
  pointers = sorted(f.name for f in models.glob("big_*_tinygrad.pkl") if f.stat().st_size < 1024)
  if not pointers:
    return
  print(f"big model: {len(pointers)} pointer(s) found, pulling from LFS")

  # RTC のバックアップが無いので、起動直後の時刻は systemd のビルド時刻(約2ヶ月前)になる。
  # そのままだと TLS 証明書が「まだ有効でない」と判定され、LFS の取得が必ず失敗する。
  for i in range(TIME_SYNC_TIMEOUT):
    if system_time_valid():
      if i:
        print(f"big model: system time valid after {i}s")
      break
    time.sleep(1)
  else:
    print("big model: system time still invalid, pulling anyway")

  r = subprocess.run(["git", "lfs", "pull"], cwd=BASEDIR, check=False)
  print(f"big model: git lfs pull returned {r.returncode}")


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
