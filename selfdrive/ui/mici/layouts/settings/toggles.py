import pyray as rl
from collections.abc import Callable
from cereal import log

from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle, BigMultiToggle, BigToggle
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import NavWidget
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants


class TogglesLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)

    self.now_exp = ui_state.params.get_bool("ExperimentalMode")
    self.state_update_ct = 0

    self._personality_toggle = BigMultiParamToggle("driving personality", "LongitudinalPersonality", ["aggressive", "standard", "relaxed"])
    self._experimental_btn = BigParamControl("experimental mode", "ExperimentalMode")
    is_metric_toggle = BigParamControl("use metric units", "IsMetric")
    ldw_toggle = BigParamControl("lane departure warnings", "IsLdwEnabled")
    always_on_dm_toggle = BigParamControl("always-on driver monitor", "AlwaysOnDM")
    record_front = BigParamControl("record & upload driver camera", "RecordFront", toggle_callback=restart_needed_callback)
    record_mic = BigParamControl("record & upload mic audio", "RecordAudio", toggle_callback=restart_needed_callback)
    enable_openpilot = BigParamControl("enable openpilot", "OpenpilotEnabledToggle", toggle_callback=restart_needed_callback)
    C4UIOnC3X = BigParamControl("use c4 ui in c3x", "C4UIOnC3X", toggle_callback=restart_needed_callback)
    self._accel_method_setting = BigMultiToggle("accel method", ["recommend", "official"], select_callback=self._accel_method_setting_callback, toggle_callback=restart_needed_callback)
    GpsAlwaysSwitch = BigParamControl("always receive GPS signals", "GpsAlwaysSwitch", toggle_callback=restart_needed_callback)
    DisableMaxSpeedModify = BigParamControl("use TSSP acc over 115", "DisableMaxSpeedModify", toggle_callback=restart_needed_callback)
    ForceHybridVehicle = BigParamControl("force hybrid vehicle", "ForceHybridVehicle", toggle_callback=restart_needed_callback)
    IgnoreRerouteHarness = BigParamControl("ignore TSSP bypass harness", "IgnoreRerouteHarness", toggle_callback=restart_needed_callback)
    self._lta_enable_sw_button = BigToggle("return from edge of lane", "" ,toggle_callback=self._lta_enable_sw_button_callback) #ハボタン
    self._dexp_sw_mode_button = BigToggle("dynamic experimental mode", "" ,toggle_callback=self._dexp_sw_mode_button_callback) #dXボタン（できればOnroadHudにも）
    self._start_accel_power_up_disp_enable_button = BigToggle("turbo boost", "" ,toggle_callback=self._start_accel_power_up_disp_enable_button_callback) #ターボブースト
    #PedalMethod(N,A,AA,iP,eP)
    self._accel_ctrl_disable_button = BigToggle("follow lead car", "" ,toggle_callback=self._accel_ctrl_disable_button_callback) #前走車追従（できればOnroadHudにも）
    self._decel_ctrl_disable_button = BigToggle("tight curve slowdown", "" ,toggle_callback=self._decel_ctrl_disable_button_callback) #カーブ減速（できればOnroadHudにも）
    self._long_speeddown_disable_button = BigToggle("chill mode signal detective", "" ,toggle_callback=self._long_speeddown_disable_button_callback) #イチロウロング
    self._mads_button = BigToggle("MADS toggle", "" ,toggle_callback=self._mads_button_callback) #MADS
    self._limitspeed_sw_button = BigMultiToggle("acc speed limit", ["manual", "auto", "record"], select_callback=self._limitspeed_sw_button_callback) #標識レコードボタン（これだけはOnroadに追加したい）
    self._lockon_disp_disable_button = BigToggle("lockon erase", "" ,toggle_callback=self._lockon_disp_disable_button_callback) #ロックオンOFFボタン（減速時にワンペダルに落ちない）
    #●●● ナイトスキャナー

    self._scroller = Scroller([
      self._personality_toggle,
      self._experimental_btn,
      is_metric_toggle,
      ldw_toggle,
      always_on_dm_toggle,
      record_front,
      record_mic,
      enable_openpilot,
      self._accel_method_setting,
      GpsAlwaysSwitch,
      DisableMaxSpeedModify,
      ForceHybridVehicle,
      IgnoreRerouteHarness,
      self._lta_enable_sw_button,
      self._dexp_sw_mode_button,
      self._start_accel_power_up_disp_enable_button,

      self._accel_ctrl_disable_button,
      self._decel_ctrl_disable_button,
      self._long_speeddown_disable_button,
      self._mads_button,
      self._limitspeed_sw_button,
      self._lockon_disp_disable_button,
      C4UIOnC3X,
    ], snap_items=False)

    # Toggle lists
    self._refresh_toggles = (
      ("ExperimentalMode", self._experimental_btn),
      ("IsMetric", is_metric_toggle),
      ("IsLdwEnabled", ldw_toggle),
      ("AlwaysOnDM", always_on_dm_toggle),
      ("RecordFront", record_front),
      ("RecordAudio", record_mic),
      ("OpenpilotEnabledToggle", enable_openpilot),
      ("GpsAlwaysSwitch", GpsAlwaysSwitch),
      ("DisableMaxSpeedModify", DisableMaxSpeedModify),
      ("ForceHybridVehicle", ForceHybridVehicle),
      ("IgnoreRerouteHarness", IgnoreRerouteHarness),
      ("C4UIOnC3X", C4UIOnC3X),
    )

    enable_openpilot.set_enabled(lambda: not ui_state.engaged)
    record_front.set_enabled(False if ui_state.params.get_bool("RecordFrontLock") else (lambda: not ui_state.engaged))
    record_mic.set_enabled(lambda: not ui_state.engaged)

    if ui_state.params.get_bool("ShowDebugInfo"):
      gui_app.set_show_touches(True)
      gui_app.set_show_fps(True)

    ui_state.add_engaged_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()

    if ui_state.sm.updated["selfdriveState"]:
      personality = PERSONALITY_TO_INT[ui_state.sm["selfdriveState"].personality]
      if personality != ui_state.personality and ui_state.started:
        self._personality_toggle.set_value(self._personality_toggle._options[personality])
      ui_state.personality = personality

    self.state_update_ct += 1
    if self.state_update_ct % 10 == 0: #20fpsで0.5秒ごとに
      self._ip_toggles_update() #ここでも呼ぶ

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # CP gating for experimental mode
    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._experimental_btn.set_enabled(True)
        self._personality_toggle.set_enabled(True)
      else:
        # no long for now
        self._experimental_btn.set_enabled(False)
        self._experimental_btn.set_checked(False)
        self._personality_toggle.set_enabled(False)
        ui_state.params.remove("ExperimentalMode")

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

    self._ip_toggles_update() #togglesに遷移した時にしか呼ばれない

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)

  def _ip_toggles_update(self):
    limitspeed_sw = 0
    try:
      with open('/dev/shm/limitspeed_sw.txt','r') as fp:
        limitspeed_sw_str = fp.read()
        if limitspeed_sw_str:
          limitspeed_sw = int(limitspeed_sw_str)
    except Exception as e:
      pass

    if limitspeed_sw == 0:
      self._limitspeed_sw_button.set_value("manual") #直接文字列をセットする
    elif limitspeed_sw == 1:
      self._limitspeed_sw_button.set_value("auto") #直接文字列をセットする
    else: #limitspeed_sw == 2
      self._limitspeed_sw_button.set_value("record") #直接文字列をセットする


    lockon_disp_disable = 0
    try:
      with open('/dev/shm/lockon_disp_disable.txt','r') as fp: # /dev/shmのまま
        lockon_disp_disable_str = fp.read()
        if lockon_disp_disable_str:
          lockon_disp_disable = int(lockon_disp_disable_str)
    except Exception as e:
      pass
    self._lockon_disp_disable_button.set_checked(lockon_disp_disable == 0)

    accel_ctrl_disable = 0
    try:
      with open('/data/accel_ctrl_disable.txt','r') as fp: # /data/から取る
        accel_ctrl_disable_str = fp.read()
        if accel_ctrl_disable_str:
          accel_ctrl_disable = int(accel_ctrl_disable_str)
    except Exception as e:
      pass
    self._accel_ctrl_disable_button.set_checked(accel_ctrl_disable == 0)

    decel_ctrl_disable = 0
    try:
      with open('/data/decel_ctrl_disable.txt','r') as fp: # /data/から取る
        decel_ctrl_disable_str = fp.read()
        if decel_ctrl_disable_str:
          decel_ctrl_disable = int(decel_ctrl_disable_str)
    except Exception as e:
      pass
    self._decel_ctrl_disable_button.set_checked(decel_ctrl_disable == 0)

    steer_always = 0
    try:
      with open('/dev/shm/steer_always.txt','r') as fp: # /dev/shmのまま
        steer_always_str = fp.read()
        if steer_always_str:
          if int(steer_always_str) >= 1:
            steer_always = 2 #なぜ2なのか忘れた。
    except Exception as e:
      pass
    self._mads_button.set_checked(steer_always != 0)

    long_speeddown_disable = 0
    try:
      with open('/data/long_speeddown_disable.txt','r') as fp: # /data/から取る
        long_speeddown_disable_str = fp.read()
        if long_speeddown_disable_str:
          long_speeddown_disable = int(long_speeddown_disable_str)
    except Exception as e:
      pass
    self._long_speeddown_disable_button.set_checked(long_speeddown_disable == 0)

    start_accel_power_up_disp_enable = 0
    try:
      with open('/data/start_accel_power_up_disp_enable.txt','r') as fp: # /data/から取る
        start_accel_power_up_disp_enable_str = fp.read()
        if start_accel_power_up_disp_enable_str:
          start_accel_power_up_disp_enable = int(start_accel_power_up_disp_enable_str)
    except Exception as e:
      pass
    self._start_accel_power_up_disp_enable_button.set_checked(start_accel_power_up_disp_enable != 0)

    if self.now_exp != ui_state.params.get_bool("ExperimentalMode"):
      self._dexp_sw_mode_button_callback(False) #ExperimentalModeを操作したらdX解除する
      self.now_exp = ui_state.params.get_bool("ExperimentalMode")

    dexp_sw_mode = 0
    try:
      with open('/data/dexp_sw_mode.txt','r') as fp: # /data/から取る
        dexp_sw_mode_str = fp.read()
        if dexp_sw_mode_str:
          dexp_sw_mode = int(dexp_sw_mode_str)
    except Exception as e:
      pass
    self._dexp_sw_mode_button.set_checked(dexp_sw_mode != 0)

    lta_enable_sw = 0
    try:
      with open('/data/lta_enable_sw.txt','r') as fp: # /data/から取る
        lta_enable_sw_str = fp.read()
        if lta_enable_sw_str:
          lta_enable_sw = int(lta_enable_sw_str)
    except Exception as e:
      pass
    self._lta_enable_sw_button.set_checked(lta_enable_sw != 0)

    #self._accel_method_settingの設定処理
    if ui_state.params.get_bool("AccelMethodSwitch") == False:
      self._accel_method_setting.set_value("recommend") #直接文字列をセットする
    else:
      self._accel_method_setting.set_value("official") #直接文字列をセットする

  def _accel_method_setting_callback(self,str):
    #self._accel_method_settingの変更処理
    if str == "recommend":
      ui_state.params.put_bool("AccelMethodSwitch",False) #indexを設定
    else:
      ui_state.params.put_bool("AccelMethodSwitch",True) #indexを設定

  def _lta_enable_sw_button_callback(self,onoff):
    lta_enable_sw = int(onoff)
    with open('/dev/shm/lta_enable_sw.txt','w') as fp2:
      fp2.write("%d" % (lta_enable_sw))
    with open('/data/lta_enable_sw.txt','w') as fp3:
      fp3.write("%d" % (lta_enable_sw))

  def _dexp_sw_mode_button_callback(self,onoff):
    dexp_sw_mode = int(onoff)
    with open('/dev/shm/dexp_sw_mode.txt','w') as fp2:
      fp2.write("%d" % (dexp_sw_mode))
    with open('/data/dexp_sw_mode.txt','w') as fp3:
      fp3.write("%d" % (dexp_sw_mode))

  def _start_accel_power_up_disp_enable_button_callback(self,onoff):
    start_accel_power_up_disp_enable = int(onoff)
    with open('/dev/shm/start_accel_power_up_disp_enable.txt','w') as fp2:
      fp2.write("%d" % (start_accel_power_up_disp_enable))
    with open('/data/start_accel_power_up_disp_enable.txt','w') as fp3:
      fp3.write("%d" % (start_accel_power_up_disp_enable))

  def _long_speeddown_disable_button_callback(self,onoff):
    long_speeddown_disable = int(not onoff)
    with open('/dev/shm/long_speeddown_disable.txt','w') as fp2:
      fp2.write("%d" % (long_speeddown_disable))
    with open('/data/long_speeddown_disable.txt','w') as fp3:
      fp3.write("%d" % (long_speeddown_disable))

  def _mads_button_callback(self,onoff):
    mads_steer_always = int(onoff)
    with open('/dev/shm/steer_always.txt','w') as fp2:
      fp2.write("%d" % (mads_steer_always))

  def _accel_ctrl_disable_button_callback(self,onoff):
    accel_ctrl_disable = int(not onoff)
    with open('/dev/shm/accel_ctrl_disable.txt','w') as fp2:
      fp2.write("%d" % (accel_ctrl_disable))
    with open('/data/accel_ctrl_disable.txt','w') as fp3:
      fp3.write("%d" % (accel_ctrl_disable))

  def _decel_ctrl_disable_button_callback(self,onoff):
    decel_ctrl_disable = int(not onoff)
    with open('/dev/shm/decel_ctrl_disable.txt','w') as fp2:
      fp2.write("%d" % (decel_ctrl_disable))
    with open('/data/decel_ctrl_disable.txt','w') as fp3:
      fp3.write("%d" % (decel_ctrl_disable))

  def _lockon_disp_disable_button_callback(self,onoff):
    lockon_disp_disable = int(not onoff)
    with open('/dev/shm/lockon_disp_disable.txt','w') as fp2:
      fp2.write("%d" % (lockon_disp_disable))
    # with open('/data/lockon_disp_disable.txt','w') as fp3:
    #   fp3.write("%d" % (lockon_disp_disable))

  def _limitspeed_sw_button_callback(self,str):
    limitspeed_sw = int(str)
    with open('/dev/shm/limitspeed_sw.txt','w') as fp2:
      fp2.write("%d" % (limitspeed_sw))
    with open('/data/limitspeed_sw.txt','w') as fp3:
      fp3.write("%d" % (limitspeed_sw))
