from cereal import log

from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl, BigMultiParamToggle, BigMultiToggle, BigToggle, BigMultiToggleAA, BigMultiToggleKN, BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigInputDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state

PERSONALITY_TO_INT = log.LongitudinalPersonality.schema.enumerants


class TogglesLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()

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
    self._accel_engaged_button = BigMultiToggleAA("pedal mode    ", ["disengage", "A", "AA", "iP", "eP"], select_callback=self._accel_engaged_button_callback) #PedalMethod(N,A,AA,iP,eP)
    self._accel_ctrl_disable_button = BigToggle("follow lead car", "" ,toggle_callback=self._accel_ctrl_disable_button_callback) #前走車追従（できればOnroadHudにも）
    self._decel_ctrl_disable_button = BigToggle("tight curve slowdown", "" ,toggle_callback=self._decel_ctrl_disable_button_callback) #カーブ減速（できればOnroadHudにも）
    self._long_speeddown_disable_button = BigToggle("chill mode signal detective", "" ,toggle_callback=self._long_speeddown_disable_button_callback) #イチロウロング
    self._mads_button = BigToggle("MADS toggle", "" ,toggle_callback=self._mads_button_callback) #MADS
    self._limitspeed_sw_button = BigMultiToggle("acc speed limit", ["manual", "auto", "record"], select_callback=self._limitspeed_sw_button_callback) #標識レコードボタン（これだけはOnroadに追加したい）
    self._lockon_disp_disable_button = BigToggle("lockon indicator", "" ,toggle_callback=self._lockon_disp_disable_button_callback) #ロックオンOFFボタン（減速時にワンペダルに落ちない）
    self._knight_scanner_bit3_button = BigMultiToggleKN("knight scanner", ["param_0", "param_1", "param_2", "param_3", "param_4", "param_5", "param_6", "param_7"], select_callback=self._knight_scanner_bit3_button_callback) #●●● ナイトスキャナー

    icon_car_weight = gui_app.texture("offroad/icon_car_weight.png",64,64)
    self._vehicle_mass_btn = BigButton("vehicle weight  ", "", icon_car_weight)
    try:
      with open('/data/vehicle_mass.txt','r') as fp:
        vehicle_mass_str = fp.read()
        if vehicle_mass_str:
          self._vehicle_mass_btn.set_value(vehicle_mass_str+" [kg]")
    except Exception as e:
      pass
    self._vehicle_mass_btn.set_click_callback(self._vehicle_mass_btn_callback)

    icon_car_key = gui_app.texture("offroad/icon_car_key.png",64,64)
    self._auto_door_lock_btn = BigButton("auto door lock          ", "", icon_car_key)
    try:
      with open('/data/run_auto_lock.txt','r') as fp:
        auto_door_lock_str = fp.read() #ロックするスピードをテキストで30みたいに書いておく。ファイルが無いか0でオートロック無し。
        if auto_door_lock_str:
          self._auto_door_lock_btn.set_value(auto_door_lock_str+" [km/h]")
    except Exception as e:
      pass
    self._auto_door_lock_btn.set_click_callback(self._auto_door_lock_btn_callback)

    self._scroller.add_widgets([
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
      self._accel_engaged_button,
      self._accel_ctrl_disable_button,
      self._decel_ctrl_disable_button,
      self._long_speeddown_disable_button,
      self._mads_button,
      self._limitspeed_sw_button,
      self._lockon_disp_disable_button,
      self._knight_scanner_bit3_button,
      self._vehicle_mass_btn,
      self._auto_door_lock_btn,
      C4UIOnC3X,
    ])

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
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()

    # CP gating for experimental mode
    if ui_state.CP is not None:
      if ui_state.has_longitudinal_control:
        self._experimental_btn.set_visible(True)
        self._personality_toggle.set_visible(True)
      else:
        # no long for now
        self._experimental_btn.set_visible(False)
        self._experimental_btn.set_checked(False)
        self._personality_toggle.set_visible(False)
        ui_state.params.remove("ExperimentalMode")

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

    self._ip_toggles_update() #togglesに遷移した時にしか呼ばれない

  def _ip_toggles_update(self):
    Knight_scanner = 0
    try:
      with open('/data/knight_scanner_bit3.txt','r') as fp: # /data/から取る
        Knight_scanner_str = fp.read()
        if Knight_scanner_str:
          Knight_scanner = int(Knight_scanner_str)
    except Exception as e:
      pass

    if Knight_scanner == 0:
      self._knight_scanner_bit3_button.set_value("param_0") #直接文字列をセットする
    elif Knight_scanner == 1:
      self._knight_scanner_bit3_button.set_value("param_1") #直接文字列をセットする
    elif Knight_scanner == 2:
      self._knight_scanner_bit3_button.set_value("param_2") #直接文字列をセットする
    elif Knight_scanner == 3:
      self._knight_scanner_bit3_button.set_value("param_3") #直接文字列をセットする
    elif Knight_scanner == 4:
      self._knight_scanner_bit3_button.set_value("param_4") #直接文字列をセットする
    elif Knight_scanner == 5:
      self._knight_scanner_bit3_button.set_value("param_5") #直接文字列をセットする
    elif Knight_scanner == 6:
      self._knight_scanner_bit3_button.set_value("param_6") #直接文字列をセットする
    else: #Knight_scanner == 7
      self._knight_scanner_bit3_button.set_value("param_7") #直接文字列をセットする


    accel_engaged = 0
    try:
      with open('/data/accel_engaged.txt','r') as fp: # /data/から取る
        accel_engaged_str = fp.read()
        if accel_engaged_str:
          accel_engaged = int(accel_engaged_str)
    except Exception as e:
      pass

    if accel_engaged == 0:
      self._accel_engaged_button.set_value("disengage") #直接文字列をセットする
    elif accel_engaged == 1:
      self._accel_engaged_button.set_value("A") #直接文字列をセットする
    elif accel_engaged == 2:
      self._accel_engaged_button.set_value("AA") #直接文字列をセットする
    elif accel_engaged == 3:
      self._accel_engaged_button.set_value("iP") #直接文字列をセットする
    else: #accel_engaged == 4
      self._accel_engaged_button.set_value("eP") #直接文字列をセットする

    limitspeed_sw = 0
    try:
      with open('/data/limitspeed_sw.txt','r') as fp: # /data/から取る
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
      with open('/data/steer_always.txt','r') as fp: # /data/から取る
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
      #self._dexp_sw_mode_button_callback(False) #ExperimentalModeを操作したらdX解除する
      #ここで解除するとボタン操作でない場合でも解除されてしまう。
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
    with open('/data/steer_always.txt','w') as fp3:
      fp3.write("%d" % (mads_steer_always))

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
    if str == "manual":
      limitspeed_sw = 0
    elif str == "auto":
      limitspeed_sw = 1
    else: #str == "record":
      limitspeed_sw = 2
    with open('/dev/shm/limitspeed_sw.txt','w') as fp2:
      fp2.write("%d" % (limitspeed_sw))
    with open('/data/limitspeed_sw.txt','w') as fp3:
      fp3.write("%d" % (limitspeed_sw))

  def _accel_engaged_button_callback(self,str):
    if str == "disengage":
      accel_engaged = 0
    elif str == "A":
      accel_engaged = 1
    elif str == "AA":
      accel_engaged = 2
    elif str == "iP":
      accel_engaged = 3
    else: #str == "eP":
      accel_engaged = 4
    with open('/dev/shm/accel_engaged.txt','w') as fp2:
      fp2.write("%d" % (accel_engaged))
    with open('/data/accel_engaged.txt','w') as fp3:
      fp3.write("%d" % (accel_engaged))

  def _knight_scanner_bit3_button_callback(self,str):
    Knight_scanner = 7
    if str == "param_0":
      Knight_scanner = 0
    elif str == "param_1":
      Knight_scanner = 1
    elif str == "param_2":
      Knight_scanner = 2
    elif str == "param_3":
      Knight_scanner = 3
    elif str == "param_4":
      Knight_scanner = 4
    elif str == "param_5":
      Knight_scanner = 5
    elif str == "param_6":
      Knight_scanner = 6
    else: #str == "param_7":
      Knight_scanner = 7
    with open('/dev/shm/knight_scanner_bit3.txt','w') as fp2:
      fp2.write("%d" % (Knight_scanner))
    with open('/data/knight_scanner_bit3.txt','w') as fp3:
      fp3.write("%d" % (Knight_scanner))

  def _vehicle_mass_btn_callback(self):
    vehicle_mass = self._vehicle_mass_btn.value
    vehicle_mass = vehicle_mass.removesuffix(" [kg]")

    def vehicle_mass_callback(weight: str):
      if weight:
        try:
          with open('/data/vehicle_mass.txt','w') as fp:
            fp.write("%s" % (weight))
        except Exception as e:
          self._vehicle_mass_btn.set_value("")
          return

        if weight == "0" or not weight:
          self._vehicle_mass_btn.set_value("")
        else:
          self._vehicle_mass_btn.set_value(weight+" [kg]")

    dlg = BigInputDialog("Vehicle weight", vehicle_mass, confirm_callback=vehicle_mass_callback)
    gui_app.push_widget(dlg)

  def _auto_door_lock_btn_callback(self):
    lock_speed = self._auto_door_lock_btn.value
    lock_speed = lock_speed.removesuffix(" [km/h]")

    def lock_speed_callback(lock: str):
      if lock:
        try:
          with open('/data/run_auto_lock.txt','w') as fp:
            fp.write("%s" % (lock))
        except Exception as e:
          self._auto_door_lock_btn.set_value("")
          return

        if lock == "0" or not lock:
          self._auto_door_lock_btn.set_value("")
        else:
          self._auto_door_lock_btn.set_value(lock+" [km/h]")

    dlg = BigInputDialog("Auto door lock", lock_speed, confirm_callback=lock_speed_callback)
    gui_app.push_widget(dlg)

