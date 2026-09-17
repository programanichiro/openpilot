#!/usr/bin/env python3
import datetime
import os
import signal
import sys
import time
import traceback

from openpilot.cereal import log
import openpilot.cereal.messaging as messaging
from openpilot.common.utils import atomic_write
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.text_window import TextWindow
from openpilot.common.hardware import HARDWARE
from openpilot.system.manager.helpers import unblock_stdout, save_bootlog
from openpilot.system.manager.process import ensure_running
from openpilot.system.manager.process_config import managed_processes
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.common.version import get_build_metadata
from openpilot.common.hardware.hw import Paths

# 大モデルの GPU 呼び出しで modeld がブロックすると例外が飛ばず、modeld 内のフォールバックも
# 走らないまま modelV2 が止まる。SIGINT では抜けられないので SIGKILL で落として再起動させる。
# modelV2 がこの秒数止まり続けたら異常とみなす。
MODELD_STALL_THRESHOLD = 3.0


def manager_init() -> None:
  save_bootlog()

  build_metadata = get_build_metadata()

  params = Params()
  params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
  params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)
  if build_metadata.release_channel:
    params.clear_all(ParamKeyFlag.DEVELOPMENT_ONLY)

  if params.get_bool("RecordFrontLock"):
    params.put_bool("RecordFront", True, block=True)

  # set unset params to their default value
  for k in params.all_keys():
    default_value = params.get_default_value(k)
    if default_value is not None and params.get(k) is None:
      params.put(k, default_value, block=True)

  # Create folders needed for msgq
  try:
    os.mkdir(Paths.shm_path())
  except FileExistsError:
    pass
  except PermissionError:
    print(f"WARNING: failed to make {Paths.shm_path()}")

  # set params
  serial = HARDWARE.get_serial()
  params.put("Version", build_metadata.openpilot.version, block=True)
  params.put("GitCommit", build_metadata.openpilot.git_commit, block=True)
  params.put("GitCommitDate", build_metadata.openpilot.git_commit_date, block=True)
  params.put("GitBranch", build_metadata.channel, block=True)
  params.put("GitRemote", build_metadata.openpilot.git_origin, block=True)
  params.put_bool("IsTestedBranch", build_metadata.tested_channel, block=True)
  params.put_bool("IsReleaseBranch", build_metadata.release_channel, block=True)
  params.put("HardwareSerial", serial, block=True)

  # set dongle id
  reg_res = register(show_spinner=True)
  if reg_res:
    dongle_id = reg_res
  else:
    raise Exception(f"Registration failed for device {serial}")
  os.environ['DONGLE_ID'] = dongle_id  # Needed for swaglog
  os.environ['GIT_ORIGIN'] = build_metadata.openpilot.git_normalized_origin # Needed for swaglog
  os.environ['GIT_BRANCH'] = build_metadata.channel # Needed for swaglog
  os.environ['GIT_COMMIT'] = build_metadata.openpilot.git_commit # Needed for swaglog

  if not build_metadata.openpilot.is_dirty:
    os.environ['CLEAN'] = '1'

  # init logging
  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

def manager_cleanup() -> None:
  # send signals to kill all procs
  for p in managed_processes.values():
    p.stop(block=False)

  # ensure all are killed
  for p in managed_processes.values():
    p.stop(block=True)

  cloudlog.info("everything is dead")


def manager_thread() -> None:
  cloudlog.bind(daemon="manager")
  cloudlog.info("manager start")
  cloudlog.info({"environ": os.environ})

  params = Params()

  ignore: list[str] = []
  if params.get("DongleId") in (None, UNREGISTERED_DONGLE_ID):
    ignore += ["manage_athenad", "uploader"]
  if os.getenv("NOBOARD") is not None:
    ignore.append("pandad")
  ignore += [x for x in os.getenv("BLOCK", "").split(",") if len(x) > 0]

  sm = messaging.SubMaster(['deviceState', 'carParams', 'pandaStates', 'modelV2'], poll='deviceState')
  pm = messaging.PubMaster(['managerState'])

  params.put_bool("IsOffroad", True, block=True)
  ensure_running(managed_processes.values(), False, params=params, CP=sm['carParams'], not_run=ignore)

  started_prev = False
  ignition_prev = False
  modeld_alive_prev = False
  modeld_ready = False
  modeld_stall_t = None

  while True:
    sm.update(1000)

    started = sm['deviceState'].started

    if started and not started_prev:
      params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
    elif not started and started_prev:
      params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)

    ignition = any(ps.ignitionLine or ps.ignitionCan for ps in sm['pandaStates'] if ps.pandaType != log.PandaState.PandaType.unknown)
    if ignition and not ignition_prev:
      params.clear_all(ParamKeyFlag.CLEAR_ON_IGNITION_ON)

    # update offroad state for services that don't subscribe to deviceState
    if started != started_prev:
      params.put_bool("IsOffroad", not started, block=True)

    started_prev = started
    ignition_prev = ignition

    # modeld が大モデルの GPU 待ちで固まると modelV2 が止まったまま復帰しない。判定に deviceState の
    # chestnutPresent は使えない。USB が抜けた瞬間に False になり、まさに救いたい場面で監視が外れる。
    # 代わりに ChestnutActive を見る。これは大モデルを読み込んだ modeld だけが書くので、chestnut を
    # 積んでいない機体では None のままとなり、この監視は作動しない。
    modeld = managed_processes['modeld']
    modeld_alive = modeld.proc is not None and modeld.proc.is_alive()
    if modeld_alive and not modeld_alive_prev:
      modeld_ready = False  # 再起動したらモデルのロード完了待ちに戻す
    modeld_alive_prev = modeld_alive
    if modeld_alive and sm.alive['modelV2']:
      modeld_ready = True  # 一度でも modelV2 が出ればロード完了。以降の停止は異常とみなす

    now = time.monotonic()
    stalled = (started and modeld_alive and modeld_ready and not sm.alive['modelV2']
               and params.get("ChestnutActive") is not None)
    if not stalled:
      modeld_stall_t = None
    elif modeld_stall_t is None:
      modeld_stall_t = now
    elif now - modeld_stall_t > MODELD_STALL_THRESHOLD:
      cloudlog.error(f"modelV2 stopped for {now - modeld_stall_t:.1f}s, killing modeld to recover")
      # signal() で直接殺すと proc が残り、start() の早期 return で二度と再起動されない。
      # proc を None に戻すのは stop() の中だけなので、こちらを使う。
      modeld.stop(sig=signal.SIGKILL)
      modeld_stall_t = None

    # 大モデルの失敗後、小モデルへ切り替えた直後に modeld が落ちることがある。openpilot は自然死した
    # プロセスを再起動しない（proc が残り start() が早期 return する）ので、modeld に限って回収する。
    # ChestnutActive で絞っているので、再起動した modeld が大モデルを飛ばせば None になり救済は1回で
    # 止まる。クラッシュが続いても無限ループにならない。upstream のフォールバックが直るまでの繋ぎ。
    if started and not modeld_alive and modeld.proc is not None and params.get("ChestnutActive") is not None:
      cloudlog.error("modeld died, reaping so it can restart")
      modeld.stop()

    ensure_running(managed_processes.values(), started, params=params, CP=sm['carParams'], not_run=ignore)

    running = ' '.join("{}{}\u001b[0m".format("\u001b[32m" if p.proc.is_alive() else "\u001b[31m", p.name)
                       for p in managed_processes.values() if p.proc)
    print(running)
    cloudlog.debug(running)

    # send managerState
    msg = messaging.new_message('managerState', valid=True)
    msg.managerState.processes = [p.get_process_state_msg() for p in managed_processes.values()]
    pm.send('managerState', msg)

    # kick AGNOS power monitoring watchdog
    try:
      if sm.all_checks(['deviceState']):
        with atomic_write("/var/tmp/power_watchdog", "w", overwrite=True) as f:
          f.write(str(time.monotonic()))
    except Exception:
      pass

    # Exit main loop when uninstall/shutdown/reboot is needed
    shutdown = False
    for param in ("DoUninstall", "DoShutdown", "DoReboot"):
      if params.get_bool(param):
        shutdown = True
        params.put("LastManagerExitReason", f"{param} {datetime.datetime.now()}", block=True)
        cloudlog.warning(f"Shutting down manager - {param} set")

    if shutdown:
      break


def main() -> None:
  manager_init()
  if os.getenv("PREPAREONLY") is not None:
    return

  # SystemExit on sigterm
  signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(1))

  try:
    manager_thread()
  except Exception:
    traceback.print_exc()
    cloudlog.exception("crash")
  finally:
    manager_cleanup()

  params = Params()
  if params.get_bool("DoUninstall"):
    cloudlog.warning("uninstalling")
    HARDWARE.uninstall()
  elif params.get_bool("DoReboot"):
    cloudlog.warning("reboot")
    HARDWARE.reboot()
  elif params.get_bool("DoShutdown"):
    cloudlog.warning("shutdown")
    HARDWARE.shutdown()


if __name__ == "__main__":
  unblock_stdout()

  try:
    main()
  except KeyboardInterrupt:
    print("got CTRL-C, exiting")
  except Exception:
    add_file_handler(cloudlog)
    cloudlog.exception("Manager failed to start")

    try:
      managed_processes['ui'].stop()
    except Exception:
      pass

    # Show last 3 lines of traceback
    error = traceback.format_exc(-3)
    error = "Manager failed to start\n\n" + error
    with TextWindow(error) as t:
      t.wait_for_exit()

    raise

  # manual exit because we are forked
  sys.exit(0)
