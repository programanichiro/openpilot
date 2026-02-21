import os
import pyray as rl
import time
from cereal import log
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.common.params import Params, ParamKeyFlag, UnknownKeyName
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

y_ofs = 135
th_tmp1 = 62; #ここから黄色
th_tmp2 = 71; #ここから赤
btn_size = 192
# img_size = (btn_size / 4) * 3;
UI_BORDER_SIZE = 30
global_angle_steer00 = 0
long_speeddown_disable00 = 0
g_night_mode = 100
g_lockon_disp_disable = False

@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)  # Added
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK


UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()

    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False

    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_JP: rl.Font = gui_app.font("JP")
    #self._font_uni: rl.Font = gui_app.font("JP2") #動的ロード
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)
    self._ip_button_init()

  def _update_state(self) -> None:
    """Update HUD state based on car state and controls state."""
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False
      self.set_speed = SET_SPEED_NA
      self.speed = 0.0
      return

    controls_state = sm['controlsState']
    car_state = sm['carState']

    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = (
      controls_state.vCruiseDEPRECATED if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    self.ACC_speed = int(round(self.set_speed)) #車体SetSpeed取っておく
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    try:
      with open('/dev/shm/cruise_info.txt','r') as fp:
        cruise_info_str = fp.read()
        if cruise_info_str:
          self.limit_speed_override = False
          self.add_v_by_lead = False
          self.curve_brake = False
          self.turbo_boost = False
          if cruise_info_str.startswith(";"): #先頭セミコロンで制限速度適用
            cruise_info_str = cruise_info_str[1:]
            self.limit_speed_override = True
          if cruise_info_str.startswith(","): #先頭カンマで増速、前走車追従
            cruise_info_str = cruise_info_str[1:]
            self.add_v_by_lead = True
          if cruise_info_str.endswith("."): #末尾ピリオドで減速
            cruise_info_str = cruise_info_str[:-1]
            self.curve_brake = True
          if cruise_info_str.endswith(";"): #末尾セミコロンスタートBoost
            cruise_info_str = cruise_info_str[:-1]
            self.turbo_boost = True
          self.set_speed = int(cruise_info_str)
    except Exception as e:
      pass

    if self.is_cruise_set and not ui_state.is_metric:
      self.set_speed *= KM_TO_MILE

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    self.maxspeed_org = car_state.vCruise #これで元の41〜 , v_cruise; //レバー値の元の値。黄色点滅警告にはマッチしてる気がする。
    self.vc_speed = v_ego
    self._ip_update_state(sm)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height + y_ofs,
      COLORS.HEADER_GRADIENT_START,
      COLORS.HEADER_GRADIENT_END,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_current_speed(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size + y_ofs
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    self._ip_draw(rect)

  def user_interacting(self) -> bool:
    return (self._exp_button.is_pressed
      or self._accel_engaged_button.is_pressed
      or self._dexp_sw_mode_button.is_pressed
      or self._long_speeddown_disable_button.is_pressed
      or self._lta_enable_sw_button.is_pressed
      or self._start_accel_power_up_disp_enable_button.is_pressed
      or self._accel_ctrl_disable_button.is_pressed
      or self._decel_ctrl_disable_button.is_pressed
      or self._knight_scanner_bit3_button.is_pressed
      or self._limitspeed_sw_button.is_pressed
      or self._LongitudinalPersonality_button.is_pressed
      or self._lockon_disp_disable_button.is_pressed
      or self._set_speed_MAX_button.is_pressed
      )

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45 + y_ofs

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    #SetSpeedの色
    rect_color = COLORS.BLACK_TRANSLUCENT
    rect_border_color = COLORS.BORDER_TRANSLUCENT

    if self.limit_speed_override:
      # self.limit_speed_num = int(limitspeed_data[0])
      rect_color = rl.Color(235, 235, 235, 200)
      rect_border_color = rl.Color(235, 235, 235, 200)

    self.yellow_flash_ct += 1
    self.db_rec_mode = False
    if self.limit_speed_override == False:
      self.yellow_flag = False
      if self.Limit_speed_mode == 2:
        rect_color = rl.Color(100, 0, 0, 250)
        if self.set_speed >= 30:
          self.db_rec_mode = True
        self.yellow_flag = True
      elif self.Limit_speed_mode == 1 and self.limit_speed_auto_detect == 1:
        if self.maxspeed_org+12 <= self.set_speed and self.maxspeed_org+5 < self.vc_speed * 3.6:
          if self.yellow_flash_ct % 6 < 3:
            rect_color = rl.Color(255, 255, 0, 255) # 速度がレバーより10km/h以上高いとギクシャクする警告、点滅させる。
            self.yellow_flag = True
    else:
      if self.maxspeed_org+12 > self.set_speed or self.maxspeed_org+5 >= self.vc_speed * 3.6:
        #g_night_modeは保留
        pass #rect_color = rl.Color(235, 235, 235, 200)
      elif self.is_cruise_set:
        if self.yellow_flash_ct % 6 < 3:
          rect_color = rl.Color(255, 255, 0, 255) # 速度がレバーより10km/h以上高いとギクシャクする警告、点滅させる。
        else:
          pass # rect_color = rl.Color(235, 235, 235, 200)

    if self.add_v_by_lead and self.is_cruise_set:
      rect_border_color = rl.Color(0, 0xff, 0, 200) #前走車追従時は緑
    if self.curve_brake and self.is_cruise_set:
      rect_color = COLORS.BLACK_TRANSLUCENT
      rect_border_color = rl.Color(0xff, 0, 0, 200) #減速時は赤
    if self.turbo_boost and self.is_cruise_set:
      rect_border_color = rl.Color(0xff, 0xff, 0, 200) #スタートダッシュ時は黄色

    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, rect_color)

    if self.limit_speed_override == True or (self.Limit_speed_mode == 1 and self.limit_speed_auto_detect == 1):
      # 太い赤枠を内側に描画する。
      ls_w2 = 35
      set_speed_rect2 = rl.Rectangle(x+ls_w2/2, y+ls_w2/2, set_speed_width-ls_w2, UI_CONFIG.set_speed_height-ls_w2)
      speed_limit_border_color = rl.Color(205, 44, 38, (255 if self.limit_speed_override else 180))
      rl.draw_rectangle_rounded_lines_ex(set_speed_rect2, 0.21, 10, ls_w2-17.5, speed_limit_border_color)

    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, rect_border_color)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if ui_state.status == UIStatus.ENGAGED:
        max_color = COLORS.ENGAGED
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.DISENGAGED
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.OVERRIDE

    if self.limit_speed_override:
      max_color = rl.Color(0x24, 0x57, 0xa1 , 255)
      set_speed_color = rl.Color(0x24, 0x57, 0xa1 , 255) #速度標識の数字に合わせる。

    if self.red_signal_scan_flag >= 3:
      set_speed_color = rl.Color(0xff, 0, 0 , 255)

    max_text = tr("MAX")
    if self.limit_speed_override:
      max_text = tr("AUTO")
    if self.db_rec_mode:
      max_text = tr("REC")

    max_text_width = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + 27),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))

    if set_speed_text == "1" and self.accel_engaged == 4: #MAXが1の時
      red_signal_eP_iP_set = False
      try:
        with open('/dev/shm/red_signal_eP_iP_set.txt','r') as fp:
          red_signal_eP_iP_set_txt = fp.read()
          if red_signal_eP_iP_set_txt and int(red_signal_eP_iP_set_txt) == 1:
            red_signal_eP_iP_set = True
      except Exception as e:
        pass
      if red_signal_eP_iP_set == False:
        set_speed_text = "8"
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

    self._set_speed_MAX_button.render(set_speed_rect)

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2 + y_ofs)

    speed_waku = self.status_col
    if self.red_signal_scan_flag >= 1: #== 1
      if (self.speed > 45 and self.clipped_brightness0 >= 90 #昼は45超を信頼度低いとする。
          or self.speed > 65 and self.clipped_brightness0 < 90): #夜は65超を信頼度低いとする。
        speed_waku = rl.Color(171, 171, 0 , 255)
      else:
        speed_waku = rl.Color(0xff, 0, 0 , 255)
      # elif self.red_signal_scan_flag >= 2:
      #   speed_waku = rl.Color(0xff, 0xff, 0 , 255)
    else:
      speed_waku = self.status_col

    # // current speed
    velo_for_trans = self.speed #km/h
    velo_for_trans_limit = 0.05 #0.05km/h以下では速度を半透明にする。
    if velo_for_trans >= velo_for_trans_limit:
      speed_pos_1 = rl.Vector2(speed_pos.x-7,speed_pos.y)
      rl.draw_text_ex(self._font_bold, speed_text, speed_pos_1, FONT_SIZES.current_speed, 0, speed_waku)
      speed_pos_1 = rl.Vector2(speed_pos.x+7,speed_pos.y)
      rl.draw_text_ex(self._font_bold, speed_text, speed_pos_1, FONT_SIZES.current_speed, 0, speed_waku)
      speed_pos_1 = rl.Vector2(speed_pos.x,speed_pos.y-7)
      rl.draw_text_ex(self._font_bold, speed_text, speed_pos_1, FONT_SIZES.current_speed, 0, speed_waku)
      speed_pos_1 = rl.Vector2(speed_pos.x,speed_pos.y+7)
      rl.draw_text_ex(self._font_bold, speed_text, speed_pos_1, FONT_SIZES.current_speed, 0, speed_waku)

    speed_num_color = COLORS.WHITE
    if self.red_signal_scan_flag >= 2 and self.red_signal_scan_flag_txt_ct %6 < 3:
      speed_num_color = rl.Color(0xff, 100, 100 , 255) #赤信号認識中は点滅。
      if self.red_signal_scan_flag_2 == False and str(round(self.set_speed)) != "1":
        self.red_signal_scan_flag_2 = True
        if self.red_signal_scan_flag == 2:
          # soundButton2(1); //pikiriオンを信号認識開始に転用。
          with open('/dev/shm/sound_py_request.txt','w') as fp2:
            fp2.write('%d' % (103)) #pikiri.wav
    else:
      speed_num_color = rl.Color(0xff, 0xff, 0xff , 255)
      if self.red_signal_scan_flag < 2 and self.speed > 24:
        self.red_signal_scan_flag_2 = False #ある程度スピードが上がらないと、このフラグも戻さない。
      if velo_for_trans < velo_for_trans_limit:
        speed_num_color = rl.Color(0xff, 0xff, 0xff , 70)

    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, speed_num_color)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2 + y_ofs)
    longitudinal_control = ui_state.sm["carParams"].openpilotLongitudinalControl
    if longitudinal_control:
      rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, speed_num_color if velo_for_trans < velo_for_trans_limit else COLORS.WHITE_TRANSLUCENT)
    else:
      COLOR_STATUS_WARNING = rl.Color(0xDA, 0x6F, 0x25, 0xf1)
      rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLOR_STATUS_WARNING)

    if self.is_cruise_set:
      ACC_font_size = 40
      ACC_font_size_for_rect = ACC_font_size+15
      ACC_font_x = rect.x + rect.width/2 + unit_text_size.x/2 + 43
      ACC_font_y = rect.y+290 + y_ofs-35
      self._drawTextCenter(self._font_bold , ACC_font_size , ACC_font_x, ACC_font_y+ACC_font_size/2, str(self.ACC_speed) , 100 if velo_for_trans < velo_for_trans_limit else 235 , False , 0x24, 0x57, 0xa1 , 240, 240, 240, 70 if velo_for_trans < velo_for_trans_limit else 230 , bk_yofs=0, bk_corner_r=0.4 , bk_add_w=10 , bk_xofs=-5, bk_add_h=0)

  def _ip_button_init(self):
    def copy_data2devshm(file_name):
      try:
        with open('/data/'+file_name, 'rb') as src, open('/dev/shm/'+file_name, 'wb') as dst:
          dst.write(src.read())
      except Exception as e:
        pass
    copy_data2devshm('accel_engaged.txt')
    copy_data2devshm('dexp_sw_mode.txt')
    copy_data2devshm('long_speeddown_disable.txt')
    copy_data2devshm('lta_enable_sw.txt')
    copy_data2devshm('start_accel_power_up_disp_enable.txt')
    copy_data2devshm('accel_ctrl_disable.txt')
    copy_data2devshm('decel_ctrl_disable.txt')
    copy_data2devshm('knight_scanner_bit3.txt')
    copy_data2devshm('limitspeed_sw.txt')
    copy_data2devshm('steer_always.txt')

    self.global_engageable = False
    self.ktsc_ct_n = 1
    self.ktsc_ct = 0
    n = 15+1 #タイミングの問題で画面外に一つ増やす
    self.ktsc_t = [0.0] * n
    self.dir0 = 1.0
    self.Knight_scanner = 0

    self.road_th_ct_ct = 0
    self.before_road_th_ct = 0
    self.road_info_txt_flag = False
    self.road_info_txt_ct = 0
    self.road_info_txt = ""
    self.kmh = "" #制限速度
    self.road_name = "" #道路名
    self.road_bear = "99999" #道路方位
    self.disp_ichiro_logo = False

    self.logo_trs = 150

    self.clipped_brightness0 = 101
    self.red_signal_scan_flag = 0
    self.red_signal_scan_flag_txt_ct = 0
    self.night_mode_ct = 0
    self.accel_engaged = 0
    self.status_col = rl.Color(0x12, 0x28, 0x39, 0xFF) #UIStatus.DISENGAGEDの色
    self.red_signal_scan_flag_2 = False

    self.tss_type = 0
    self.phv_2019 = False
    self.a0 = 150
    self.a1 = 150
    self.a2 = 150
    self.a3 = 150

    self.vc_accel = 0

    self.osm_frame_ct_ct = -1 #-1 or 100以上でosmへの通信が死んでいる。
    self.osm_per = 0 #2Hzに対してosmの応答率。走行中ならだいたい50パーセントくらいになる。
    self.osm_access_counter_txt = ""
    self.osm_access_counter_ct = 0
    self.before_osm_frame_ct = 0

    self.blue_signal_chk = 0
    self.limit_vc_info = 0

    self.distance_traveled = 0
    self.prev_draw_t = time.monotonic_ns() / 1_000_000
    self.before_distance_traveled = 0
    self.h_manual_dist = 0.001
    self.h_autopilot_dist = 0 #停止時間は1秒を1m換算でカウントする。
    self.brake_light = False
    self.ahr = 0

    self.ACC_speed = 0
    self.handle_calibct = 0
    self.global_angle_steer0 = 0
    self.handle_center = -100
    self.temperature = 0 #温度が取れなくなったので目安。
    self.temp_disp1 = "○"
    self.temp_disp2 = "☆"
    self.temp_disp3 = "°C"
    self.car_bearing = 0
    self.yellow_flag = False
    self.yellow_flash_ct = 0
    self.exp_mode = Params().get("ExperimentalMode")
    self.dexp_sw_mode = False
    self.limit_speed_num = 0
    self.limit_speed_auto_detect = 0
    self.limit_speed_override = False
    self.add_v_by_lead = False
    self.curve_brake = False
    self.turbo_boost = False
    self.db_rec_mode = False
    self.maxspeed_org = 0
    self.vc_speed = 0
    self.ip_update_state_ct = 0
    self.button_style_only = True
    font_sz = 100
    font_wt = "JP" # FontWeight.BOLD #EXTRA_BOLD
    self._accel_engaged_button = Button("A",click_callback=self._press_accel_engaged,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_accel_engaged()

    self._dexp_sw_mode_button = Button("dX",click_callback=self._press_dexp_sw_mode,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_dexp_sw_mode()

    self._long_speeddown_disable_button = Button("iL",click_callback=self._press_long_speeddown_disable,font_size=font_sz,font_weight=font_wt, border_radius=20) #イチロウロング独立ボタン
    self._press_long_speeddown_disable()

    self._lta_enable_sw_button = Button("/ \\",click_callback=self._press_lta_enable_sw,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_lta_enable_sw()

    #日本語フォント対応
    self._start_accel_power_up_disp_enable_button = Button("⇧",click_callback=self._press_start_accel_power_up_disp_enable,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_start_accel_power_up_disp_enable()

    self._accel_ctrl_disable_button = Button("↑",click_callback=self._press_accel_ctrl_disable,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_accel_ctrl_disable()

    self._decel_ctrl_disable_button = Button("↓",click_callback=self._press_decel_ctrl_disable,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_decel_ctrl_disable()

    font_sz = 25
    font_wt = "JP" #FontWeight.BOLD #EXTRA_BOLD
    self._knight_scanner_bit3_button = Button("●●●",click_callback=self._press_knight_scanner_bit3,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_knight_scanner_bit3()

    self._limitspeed_sw_button = Button("○",click_callback=self._press_limitspeed_sw,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_limitspeed_sw()

    font_sz = 25
    self._LongitudinalPersonality_button = Button("⬆︎⬆︎",click_callback=self._press_LongitudinalPersonality,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_LongitudinalPersonality()

    font_sz = 40
    self._lockon_disp_disable_button = Button("■",click_callback=self._press_lockon_disp_disable,font_size=font_sz,font_weight=font_wt, border_radius=20)
    self._press_lockon_disp_disable()

    font_sz = 10 #ACC速度にかぶせる透明ボタン
    self._set_speed_MAX_button = Button("",click_callback=self._press_set_speed_MAX,font_size=font_sz,font_weight=font_wt, border_radius=0.35*200/2)
    self._set_speed_MAX_button.set_button_style(ButtonStyle.HudUnder) #バック透明
    self._press_set_speed_MAX() #_press_accel_engagedより後に呼ぶこと。
    self.button_style_only = False


  def _ip_draw(self, rect: rl.Rectangle):
    #set speed MAXボタン
    #self._set_speed_MAX_button.render() -> _draw_set_speedで描画する。

    #下段ボタン
    ud_btn_w0 = rect.width / 5
    ud_btn_w = ud_btn_w0 * 0.7
    ud_btn_h0 = 160
    ud_btn_h = 160
    self._knight_scanner_bit3_button.render(rl.Rectangle(rect.x +rect.width/2 - ud_btn_w/2 + ud_btn_w0*0, rect.y + rect.height - ud_btn_h0*1, ud_btn_w, ud_btn_h))
    self._limitspeed_sw_button.render(rl.Rectangle(rect.x +rect.width/2 - ud_btn_w/2 + ud_btn_w0*(-1), rect.y + rect.height - ud_btn_h0*1, ud_btn_w, ud_btn_h))
    self._LongitudinalPersonality_button.render(rl.Rectangle(rect.x +rect.width/2 - ud_btn_w/2 + ud_btn_w0*(1), rect.y + rect.height - ud_btn_h0*1, ud_btn_w, ud_btn_h))
    self._lockon_disp_disable_button.render(rl.Rectangle(rect.x +rect.width/2 - ud_btn_w/2 + ud_btn_w0*(2), rect.y + rect.height - ud_btn_h0*1, ud_btn_w, ud_btn_h))

    #左右配置ボタン
    btn_w0 = 200
    btn_w = 150
    btn_h0 = 175
    btn_h = 150
    self._start_accel_power_up_disp_enable_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*2, rect.y + rect.height - btn_h0*3.1, btn_w, btn_h))
    self._accel_engaged_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*2, rect.y + rect.height - btn_h0*2.1, btn_w, btn_h))

    self._accel_ctrl_disable_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*1, rect.y + rect.height - btn_h0*3.6, btn_w, btn_h))
    self._decel_ctrl_disable_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*1, rect.y + rect.height - btn_h0*2.6, btn_w, btn_h))
    self._long_speeddown_disable_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*1, rect.y + rect.height - btn_h0*1.6, btn_w, btn_h))

    self._lta_enable_sw_button.render(rl.Rectangle(rect.x +(btn_w0-btn_w)+ btn_w0*0, rect.y + rect.height - btn_h0*3.3, btn_w, btn_h))
    self._dexp_sw_mode_button.render(rl.Rectangle(rect.x +(btn_w0-btn_w)+ btn_w0*0, rect.y + rect.height - btn_h0*2.3, btn_w, btn_h))

    #速度標識
    if not self.limit_speed_num or self.limit_speed_num == 0:
      traffic_speed = CRUISE_DISABLED_CHAR
    else:
      traffic_speed = str(self.limit_speed_num)

    traffic_speed_r = 120 / 2
    traffic_speed_x = rect.x + 247 -7
    traffic_speed_y = rect.y + rect.height - traffic_speed_r*2 - 50
    traffic_back_color = rl.Color(235, 235, 235, int(0.85*255))
    rl.draw_circle(int(traffic_speed_x+traffic_speed_r),int(traffic_speed_y+traffic_speed_r),float(traffic_speed_r),traffic_back_color)

    arc_w = -22 #内側に描画
    if self.limit_speed_num >= 100:
      arc_w = -15 #枠と数字が被らないように枠を細くする。

    arc_w = arc_w * traffic_speed_r / (150 / 2)
    arc_w_color = rl.Color(205, 44, 38, 255)

    arc_center = rl.Vector2(traffic_speed_x+traffic_speed_r,traffic_speed_y+traffic_speed_r)
    rl.draw_ring(arc_center,float(traffic_speed_r+arc_w),float(traffic_speed_r-4),float(270+self.car_bearing+5), float(270+self.car_bearing-5 + 360),90,arc_w_color)
    #car_bearingがdraw_ring的に逆回り。rl.draw_ringはX軸から時計回りだから、270が上（北とみなす）。Qtと違う。

    f_size = traffic_speed_r * 67 / (150 / 2)
    traffic_speed_size = measure_text_cached(self._font_semi_bold, traffic_speed, int(f_size))
    rl.draw_text_ex(
      self._font_semi_bold,
      traffic_speed,
      rl.Vector2(traffic_speed_x+traffic_speed_r-traffic_speed_size.x/2, traffic_speed_y+traffic_speed_r-traffic_speed_size.y/2-2),
      float(f_size),
      0,
      rl.Color(0x24, 0x57, 0xa1 , 255),
    )

    temp_rc = rl.Rectangle(rect.x+65-27, rect.y+110+6-150+y_ofs, 233+27*2-5, 54)
    status_col = self.status_col

    if self.temperature < th_tmp1: #警告色の変化はサイドバーと違う。もっと早く警告される。
      temp_col = status_col
      letter_col = rl.Color(0xff, 0xff, 0xff , 200)
    elif self.temperature < th_tmp2:
      temp_col = rl.Color(240, 240, 0, 200)
      letter_col = rl.Color(10, 10, 10 , 255)
    else:
      temp_col = rl.Color(240, 0, 0, 200)
      letter_col = rl.Color(0xff, 0xff, 0 , 255)

    rl.draw_rectangle_rounded(temp_rc, 1.0, 10, temp_col)

    self._drawText(font=self._font_semi_bold,font_size=44,x=rect.x+65+120-5+40,y=temp_rc.y+temp_rc.height-1,text=self.temp_disp3,alpha=-1,color_ex=letter_col) #x,yを下段中心にtextを表示する
    self._drawText(font=self._font_JP,font_size=56,x=rect.x+65+55+5-5+30,y=temp_rc.y+temp_rc.height+3,text=self.temp_disp2,alpha=-1,color_ex=letter_col) #x,yを下段中心にtextを表示する
    self._drawText(font=self._font_JP,font_size=47,x=rect.x+65+5+5+20,y=temp_rc.y+temp_rc.height-1,text=self.temp_disp1,alpha=-1,color_ex=letter_col) #x,yを下段中心にtextを表示する

    calib_h = -33 -33 - 30 -10 #表示位置を上に
    rc2 =  rl.Rectangle(rect.x + rect.width - btn_size / 2 - UI_BORDER_SIZE * 2 - 100 + 36, rect.y -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h -36, 200, 36)
    if abs(self.global_angle_steer0-self.handle_center) > 5 and self.handle_center >= -99:
      #ハンドル角度を表示
      #status_col = p.setBrush(bg_colors[status]);
      rc3 = rl.Rectangle(rc2.x+20,rc2.y-30,rc2.width-40,rc2.height+30)
      rl.draw_rectangle_rounded(rc3, 1.0, 10, status_col)

      h_ang_i = int(self.global_angle_steer0-self.handle_center)
      h_ang_i = 99 if h_ang_i > 99 else (-99 if h_ang_i < -99 else h_ang_i)
      if h_ang_i >= 0:
        h_ang = "+"+str(h_ang_i)+"°"
      else:
        h_ang = str(h_ang_i)+"°"

      self._drawText(font=self._font_bold,font_size=60,x=rc3.x+rc3.width/2,y=rc3.y+rc3.height+3,text=h_ang,alpha=200) #x,yを下段中心にtextを表示する
    elif self.handle_center >= -99:
      #ハンドルセンター値を表示
      #status_col = p.setBrush(bg_colors[status]);
      rl.draw_rectangle_rounded(rc2, 1.0, 10, status_col)

      hc_str = f"{self.handle_center:.2f}"

      # p.setFont(InterFont(33, QFont::Bold));
      # drawText(p, surface_rect.right() - btn_size / 2 - UI_BORDER_SIZE * 2 , -20 + btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs + calib_h - 8, QString::number(hc,'f',2) + "deg", 200);
      self._drawText(font=self._font_bold,font_size=33,x=rc2.x+rc2.width/2,y=rc2.y+rc2.height,text=hc_str+"deg",alpha=200) #x,yを下段中心にtextを表示する
    else:
      calib_col = rl.Color(150, 150, 0, 0xf1)
      rl.draw_rectangle_rounded(rc2, 1.0, 10, calib_col)

      if self.handle_calibct == 0:
        self._drawText(font=self._font_semi_bold,font_size=33,x=rc2.x+rc2.width/2,y=rc2.y+rc2.height,text="Calibrating",alpha=200) #x,yを下段中心にtextを表示する
      else:
        self._drawText(font=self._font_bold,font_size=33,x=rc2.x+rc2.width/2,y=rc2.y+rc2.height+1,text=str(self.handle_calibct)+"%",alpha=200) #x,yを下段中心にtextを表示する

    font_size_debug_info = 44
    debug_disp_xpos = rect.x
    rect_h = rect.y+rect.height

    debug_disp_xpos = self._drawTextLeft(self._font_JP , font_size_debug_info , debug_disp_xpos , rect_h+4 , "↓" , 200 , False , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 5 , 0.3 , 11 , 0 , -5) + 11
    cv_str = str(int(self.limit_vc_info))
    debug_disp_xpos = self._drawTextLeft(self._font_semi_bold , font_size_debug_info , debug_disp_xpos , rect_h+4 , cv_str , 140 , False , 0, 0, 0 , 0xdf, 0xdf, 0x00, 200 , 5 , 0.3 , bk_add_w=11-2 , bk_xofs=0 , bk_add_h=-5)+3

    debug_disp_xpos = self._drawTextLeft(self._font_semi_bold , font_size_debug_info , debug_disp_xpos+4 , rect_h+4 , "AP" , 200 , False , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 5 , 0.3 , bk_add_w=4 , bk_xofs=0-1 , bk_add_h=-5) + 4
    ahr_str = str(int(self.ahr)) + "%"
    debug_disp_xpos = self._drawTextLeft(self._font_semi_bold , font_size_debug_info , debug_disp_xpos , rect_h+4 , ahr_str , 140 , False , 0, 0, 0 , 0xdf, 0xdf, 0x00, 200 , 5 , 0.3 , 4 , 0 , -5)

    debug_disp_xpos = self._drawTextLeft(self._font_semi_bold , font_size_debug_info , debug_disp_xpos+6 , rect_h+4 , "Trip" , 200 , False , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 5 , 0.3 , 10 , -3 , -5) + 5
    trip_str = f"{self.distance_traveled / 1000:.1f}" + "km"
    debug_disp_xpos = self._drawTextLeft(self._font_semi_bold , font_size_debug_info , debug_disp_xpos , rect_h+4 , trip_str , 140 , False , 0, 0, 0 , 0xdf, 0xdf, 0x00, 200 , 5 , 0.3 , 8 , -2 , -5) + 3+4

    if abs(self.vc_speed) < 0.1/3.6:
      debug_disp_xpos = self._drawTextLeft(self._font_JP , font_size_debug_info , debug_disp_xpos , rect_h+4 , "●" , 200 , False , 0xdf, 0xdf, 0x00 , 0, 0, 0, 140 , 5 , 0.3 , bk_add_w=11, bk_xofs=0-4 , bk_add_h=-5) + 12-2
      blue_signal_chk_str = str(self.blue_signal_chk)
      debug_disp_xpos = self._drawTextLeft(self._font_semi_bold , font_size_debug_info , debug_disp_xpos , rect_h+4 , blue_signal_chk_str , 140 , False , 0, 0, 0 , 0xdf, 0xdf, 0x00, 200 , 5 , 0.3 , bk_add_w=13-3 , bk_xofs=1-2 ,bk_add_h=-5)

    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    if self.osm_per >= 0:
      h = rect.height * self.osm_per // 100
      wp1 = 10
      if 0 <= self.osm_frame_ct_ct and self.osm_frame_ct_ct < 100:
        osm_bar_color = rl.Color(0, 245, 0, 200) #緑
      else:
        osm_bar_color = rl.Color(245, 0, 0, 200) #赤、通信断絶。

#      rl.draw_rectangle(int(rect.x) , int(rect_h - h) , int(wp1) , int(h) , osm_bar_color) #draw_rectangleはパラメータに整数を要求する。
      rc2 =  rl.Rectangle(int(rect.x), int(rect_h - h) , int(wp1), int(h))
      rl.draw_rectangle_rounded(rc2, 1.0, 5, osm_bar_color)

    #加速減速表示
    car_state = ui_state.sm['carState']
    self.vc_accel += (car_state.aEgo - self.vc_accel) / 5
    hha = 0
    if self.vc_accel > 0:
      hha = 1 - 0.1 / self.vc_accel
      va_color = rl.Color(int(0.09*255), int(0.945*255), int(0.26*255), 200)
    if self.vc_accel < 0:
      hha = 1 + 0.1 / self.vc_accel
      va_color = rl.Color(245, 0, 0, 200)
    if hha < 0:
      hha = 0
    hha = hha * rect.height
    wp = 35
    if self.vc_accel > 0:
      meter = [(rect.x+rect.width - wp + wp/2 , rect.y+rect.height/2),
               (rect.x+rect.width , rect.y+rect.height/2),
               (rect.x+rect.width , rect.y+rect.height/2 - hha/2),
               (rect.x+rect.width - wp/2 - wp/2 * hha / rect.height , rect.y+rect.height/2 - hha/2)]
      rl.draw_triangle_fan(meter,len(meter),va_color)
    elif self.vc_accel < 0:
      meter = [(rect.x+rect.width - wp/2 - wp/2 * hha / rect.height , rect.y+rect.height/2 + hha/2),
               (rect.x+rect.width , rect.y+rect.height/2 + hha/2),
               (rect.x+rect.width , rect.y+rect.height/2),
               (rect.x+rect.width - wp + wp/2 , rect.y+rect.height/2)]
      rl.draw_triangle_fan(meter,len(meter),va_color)

    #タコメーター
    taco_rpm = car_state.engineRpm
    taco_max = 5000
    if taco_rpm > taco_max:
      taco_rpm = taco_max #5000回転表示がMAX。
    #taco_rpm = taco_max/2; //表示テスト
    upper_2w = 200+80
    under_2w = 100+55
    lu = rect.x+rect.width/2-upper_2w
    ur = lu + upper_2w * 2 * taco_rpm / taco_max #rect.x+rect.width/2+upper_2w
    ld = rect.x+rect.width/2-under_2w
    dr = ld + under_2w * 2 * taco_rpm / taco_max #rect.x+rect.width/2+under_2w
    taco_y = rect.y-10
    taco_meter = [(lu,20+taco_y),
                  (ld,50 + 40*3+10+taco_y),
                  (dr,50 + 40*3+10+taco_y),
                  (ur,20+taco_y)]
    #p.setBrush(QColor::fromRgbF(0.8, 0.0, 0.0, 0.65)); //赤
    taco_color = rl.Color(int(0.96*0.7*255), int(0.51*0.7*255), int(0.12*0.7*255),int(0.65*255)) #オレンジ
    rl.draw_triangle_fan(taco_meter, len(taco_meter), taco_color)

    rl.end_blend_mode() #元のブレンドに戻す

    top_label_size=33
    top_label_center_x = rect.x+rect.width/2
    top_label_top_y = rect.y + 50
    self._drawText(font=self._font_bold,font_size=top_label_size,x=top_label_center_x,y=top_label_top_y + 40*0,text="extra cruise speed engagement",alpha=self.a0,brakeLight=self.brake_light) #x,yを下段中心にtextを表示する
    self._drawText(font=self._font_bold,font_size=top_label_size,x=top_label_center_x,y=top_label_top_y + 40*1,text="slow down corner correctly",alpha=self.a1,brakeLight=self.brake_light) #x,yを下段中心にtextを表示する
    self._drawText(font=self._font_bold,font_size=top_label_size,x=top_label_center_x,y=top_label_top_y + 40*2,text="speed limit auto detect",alpha=self.a2,brakeLight=self.brake_light) #x,yを下段中心にtextを表示する
    self._drawText(font=self._font_bold,font_size=top_label_size,x=top_label_center_x,y=top_label_top_y + 40*3,text="auto brake holding",alpha=self.a3,brakeLight=self.brake_light) #x,yを下段中心にtextを表示する

    tssp_47700_flag = ((ui_state.sm["carParams"].flags & 8192) != 0)
    tssp_47700 = (tssp_47700_flag and self.brake_light == False)
    label_red = 255
    label_grn = 255
    label_blu = 255
    if tssp_47700:
      label_blu = 0

    if self.tss_type <= 1:
      next_x = self._drawTextRight(font=self._font_bold,font_size=55, x=rect.x+rect.width-20, y=rect.y+65 , text=" TSSP", alpha=self.logo_trs, brakeLight=self.brake_light, red=label_red, grn=label_grn, blu=label_blu) #47060車はTSSP部分が黄色くなる。
      self._drawTextRight(font=self._font_bold,font_size=55, x=next_x, y=rect.y+65 , text="for toyota", alpha=self.logo_trs, brakeLight=self.brake_light)
    else:
      self._drawTextRight(font=self._font_bold,font_size=55, x=rect.x+rect.width-20, y=rect.y+65 , text=" for toyota TSS2", alpha=self.logo_trs, brakeLight=self.brake_light)

    self._drawTextLeft(font=self._font_bold,font_size=50, x=rect.x+20, y=rect.y+60 , text="ICHIRO Pilot", alpha=self.logo_trs, brakeLight=self.brake_light)

    if self.road_info_txt:
      road_th_ct_ct_limit = 30 #30秒無通信チェック。
      if self.speed < 0.1: #velo_for_trans = self.speed #km/h
        road_th_ct_ct_limit = 180 #停止時は3分まで伸ばす。

      if self.road_name == False or (self.road_name == "--" and self.kmh == "0") or self.road_th_ct_ct > road_th_ct_ct_limit * 20:
        self.road_info_txt_flag = False
      else:
        self.road_info_txt_flag = True
        #デバッグ用road_name = self.road_name + "&" + road_bear
        #road_info_baering = int(self.road_bear) #ドットフォントでも漢字出るか？UNIFONTにしないとダメかな。
        if self.kmh != "0":
          self._font_uni = gui_app.font("JP2", self.road_name) #動的ロード
          next_x = self._drawTextRight(self._font_uni, 44 , rect.x+rect.width-10, rect.y+rect.height+1 , self.road_name, 220, bk_alp=128, bk_corner_r= 0.2, bk_yofs=7,bk_add_h=-10,bk_add_w=0)
          self._drawTextRight(self._font_uni, 44 , rect.x+rect.width-10-1, rect.y+rect.height+1 , self.road_name, 220) #2重描き
          self._drawTextRight(self._font_semi_bold, 33, next_x-4, rect.y+rect.height - 4 , self.kmh , 255 , False , 0x24, 0x57, 0xa1 , 255,255,255,200 , 0 , 0.2 , 2 , -1)
        else:
          if self.road_name != "---":
            self._font_uni = gui_app.font("JP2", self.road_name) #動的ロード
            self._drawTextRight(self._font_uni, 44 , rect.x+rect.width-10, rect.y+rect.height+1 , self.road_name, 220, bk_alp=128, bk_corner_r= 0.2, bk_yofs=7,bk_add_h=-10,bk_add_w=0)
            self._drawTextRight(self._font_uni, 44 , rect.x+rect.width-10-1, rect.y+rect.height+1 , self.road_name, 220) #2重描き
          else:
            self.disp_ichiro_logo = True #速度ゼロの---は表示しない。(road_info_baeringは利用するのでroad_info_txt_flagはtrueとする。)

    self._font_uni = gui_app.font("JP2", "テスト神奈川県茅ヶ崎市道路情報(12345)") #動的ロード
    next_x = self._drawTextRight(self._font_uni, 44 , rect.x+rect.width-10, rect.y+rect.height+1 , "テスト神奈川県茅ヶ崎市道路情報(12345)", 220 , bk_alp=128, bk_corner_r= 0.2, bk_yofs=7,bk_add_h=-10,bk_add_w=0)
    self._drawTextRight(self._font_uni, 44 , rect.x+rect.width-10-1, rect.y+rect.height+1 , "テスト神奈川県茅ヶ崎市道路情報(12345)", 220)
    self._drawTextRight(self._font_semi_bold, 33, next_x-4, rect.y+rect.height - 4 , "120" , 255 , False , 0x24, 0x57, 0xa1 , 255,255,255,200 , 0 , 0.2 , 2 , -1)

    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    self.knightScanner(rect)
    rl.end_blend_mode() #元のブレンドに戻す

  def _ip_update_state(self,sm):
    try:
      with open('/dev/shm/signal_start_prompt_info.txt','r') as fp:
        signal_start_prompt_info_str = fp.read()
        if signal_start_prompt_info_str:
          pr = int(signal_start_prompt_info_str)
          if pr == 1:
            with open('/dev/shm/sound_py_request.txt','w') as fp2:
              fp2.write('%d' % (6)) #prompt.wav
            with open('/dev/shm/signal_start_prompt_info.txt','w') as fp3:
              fp3.write('%d' % (0))
          elif pr == 2:
            with open('/dev/shm/sound_py_request.txt','w') as fp2:
              fp2.write('%d' % (1)) #engage.wav
            with open('/dev/shm/signal_start_prompt_info.txt','w') as fp3:
              fp3.write('%d' % (0))
          elif pr == 3: #デバッグ用。
            with open('/dev/shm/sound_py_request.txt','w') as fp2:
              fp2.write('%d' % (1))
            with open('/dev/shm/signal_start_prompt_info.txt','w') as fp3:
              fp3.write('%d' % (0))
    except Exception as e:
      pass

    if self.tss_type == 0:
      try:
        with open('/data/tss_type_info.txt','r') as fp:
          tss_type_info = fp.read()
          if tss_type_info:
            self.tss_type = int(tss_type_info)
      except Exception as e:
        pass
      if self.tss_type == 2:
        pass #TSS2
      elif self.tss_type == 1:
        self.phv_2019 = Params().get_bool("DisableMaxSpeedModify")

    if ui_state.status == UIStatus.ENGAGED:
      self.status_col = rl.Color(0x16, 0x7F, 0x40, 0xFF)
    elif ui_state.status == UIStatus.DISENGAGED:
      self.status_col = rl.Color(0x12, 0x28, 0x39, 0xFF)
    elif ui_state.status == UIStatus.OVERRIDE:
      self.status_col = rl.Color(0x89, 0x92, 0x8D, 0xFF)
    else:
      self.status_col = rl.Color(0x12, 0x28, 0x39, 0xFF)

    self.ip_update_state_ct += 1
    cur_draw_t = time.monotonic_ns() / 1_000_000  # ナノ秒→ミリ秒 #millis_since_boot();
    dt = cur_draw_t - self.prev_draw_t
    car_state = sm['carState']
    self.distance_traveled += abs(car_state.vEgo) * dt / 1000
    self.prev_draw_t = cur_draw_t

    now_dist = self.distance_traveled - self.before_distance_traveled
    self.before_distance_traveled = self.distance_traveled

    # if ui_state.status == UIStatus.ENGAGED:
    #   status_col = rl.Color(0x16, 0x7F, 0x40, 0xFF)
    # elif ui_state.status == UIStatus.DISENGAGED:
    #   status_col = rl.Color(0x12, 0x28, 0x39, 0xFF)
    # elif ui_state.status == UIStatus.OVERRIDE:
    #   status_col = rl.Color(0x89, 0x92, 0x8D, 0xFF)

    ss = ui_state.sm["selfdriveState"]
    self.global_engageable = (ss.engageable or ss.enabled)

    self.brake_light = False
    all_brake_light = False
    self.logo_trs = 150
    try:
      with open('/dev/shm/brake_light_state.txt','r') as fp3:
        brake_light_state = fp3.read()
        if brake_light_state and int(brake_light_state) != 0:
          if ui_state.status != UIStatus.DISENGAGED:
            self.brake_light = True
            self.logo_trs = 80 #drawText内部で100足される。
          all_brake_light = True #ちらはエンゲージしていなくてもセットされる。
    except Exception as e:
      pass

    if ui_state.status == UIStatus.DISENGAGED or ui_state.status == UIStatus.OVERRIDE:
      self.h_manual_dist += now_dist #手動運転中
      if all_brake_light and self.vc_speed < 0.1/3.6:
        #self.h_manual_dist += 1.0/(1000/dt) #/20; #1秒を1m換算
        self.h_manual_dist += 1.0 * dt / 1000 #/20; #1秒を1m換算

      if (ui_state.status != UIStatus.DISENGAGED) or (all_brake_light and self.vc_speed < 0.1/3.6):
        #//manual_ct ++; //手動運転中 , エンゲージしていれば停車時も含める。特例としてエンゲージしてなくてもブレーキ踏めば含める（人が運転しているから）
        pass

    else:
      self.h_autopilot_dist += now_dist #オートパイロット中
      if self.vc_speed < 0.1/3.6:
        #self.h_autopilot_dist += 1.0/20; #//1秒を1m換算
        self.h_autopilot_dist += 1.0 * dt / 1000 #//1秒を1m換算
      #//autopilot_ct ++; //オートパイロット中（ハンドル、アクセル操作時は含めない , 停車時は自動運転停車として含める）

    # // double atr = ((double)autopilot_ct * 100) / (autopilot_ct + manual_ct); //autopilot time rate
    # // double adr = (autopilot_dist * 100) / (autopilot_dist + manual_dist); //autopilot distance rate
    self.ahr = (self.h_autopilot_dist * 100) / (self.h_autopilot_dist + self.h_manual_dist) #//autopilot hybrid rate

    if(self.ip_update_state_ct % 10 == 8):
      self.button_style_only = True
      self._press_long_speeddown_disable()
      self.button_style_only = False

    if(self.ip_update_state_ct % 10 == 2):
      self.button_style_only = True
      self._press_LongitudinalPersonality()
      self.button_style_only = False

    if(self.ip_update_state_ct % 10 == 5):
      self.button_style_only = True
      self._press_accel_ctrl_disable()
      self.button_style_only = False

    if(self.ip_update_state_ct % 10 == 0):
      if self.dexp_sw_mode == 1:
        self.button_style_only = True
        self._press_dexp_sw_mode() #ExperimentalModeを操作したらdX解除する
        self.button_style_only = False

    gps_ok = False
    gps_idx_i = 0
    try:
      with open('/dev/shm/limitspeed_data.txt','r') as fp2:
        limitspeed_data_str = fp2.read()
        if limitspeed_data_str:
          limitspeed_data = limitspeed_data_str.split(",")
          limitspeed_flag = int(limitspeed_data[2]) #111,999
          if limitspeed_flag != 999:
            self.limit_speed_num = 0
            self.limit_speed_auto_detect = 0
          else:
            self.limit_speed_num = int(limitspeed_data[0])
            self.limit_speed_auto_detect = 1
    except Exception as e:
      self.limit_speed_num = 0
      self.limit_speed_auto_detect = 0
      pass

    try:
      with open('/dev/shm/gps_axs_data.txt','r') as fp3:
        gps_output_str = fp3.read()
        if gps_output_str:
          gps_output = gps_output_str.split(",")
          gps_ok = True
          gps_idx_i = len(gps_output)
    except Exception as e:
      pass

    if gps_idx_i == 6 and gps_ok:
      self.car_bearing = float(gps_output[2]) #bearing
      if self.car_bearing < 0:
        # 0〜360へ変換
        self.car_bearing += 360
        if self.car_bearing >= 360:
          self.car_bearing = 0

    deviceState = sm['deviceState']
    okGps = (gps_idx_i == 6 and gps_ok and int(gps_output[5]))
    okConnect = False
    last_ping = deviceState.lastAthenaPingTime
    if last_ping != 0:
      okConnect = True if time.monotonic_ns() - last_ping < 80_000_000_000 else False
    self.temperature = 0 #温度が取れなくなったので目安。
    ts = deviceState.thermalStatus
    ThermalStatus = log.DeviceState.ThermalStatus
    if ts == ThermalStatus.green:
      self.temperature = 55 #色変化のための参照値
    elif ts == ThermalStatus.yellow:
      self.temperature = 65 #色変化のための参照値
    else:
      self.temperature = 75 #色変化のための参照値

    max_temp = int(deviceState.maxTempC) #表示はこれを使う。
    # //下の方がマシかQString temp_disp = QString(okConnect ? "● " : "○ ") + QString(okGps ? "★ " : "☆ ") + QString::number(temp) + "°C";
    # //QString temp_disp = QString(okConnect ? "⚫︎ " : "⚪︎ ") + QString(okGps ? "★ " : "☆ ") + QString::number(temp) + "°C";
    # //QString temp_disp1 = QString(okConnect ? "⚫︎" : "⚪︎");
    self.temp_disp1 = "●" if okConnect else "○"
    self.temp_disp2 = "★" if okGps else "☆"
    self.temp_disp3 = str(max_temp) + "°C"

    try:
      with open('/dev/shm/steer_ang_info.txt','r') as fp3:
        steer_ang_info = fp3.read()
        if steer_ang_info:
          self.global_angle_steer0 = float(steer_ang_info)
          global global_angle_steer00
          global_angle_steer00 = self.global_angle_steer0
    except Exception as e:
      pass

    try:
      with open('/dev/shm/handle_center_info.txt','r') as fp3:
        handle_center_info = fp3.read()
        if handle_center_info:
          self.handle_center = float(handle_center_info)
        else:
          with open('/data/handle_calibct_info.txt','r') as fp3:
            handle_calibct_info = fp3.read()
            if handle_calibct_info:
              self.handle_calibct = float(handle_calibct_info)
    except Exception as e:
      pass

    if abs(self.vc_speed) < 0.1/3.6:
      try:
        with open('/dev/shm/blue_signal_chk.txt','r') as fp3:
          blue_signal_chk = fp3.read()
          if blue_signal_chk:
            self.blue_signal_chk = int(blue_signal_chk)
      except Exception as e:
        self.blue_signal_chk = 0

    if self.ip_update_state_ct % 2 == 1:
      try:
        with open('/dev/shm/limit_vc_info.txt','r') as fp3:
          limit_vc_info = fp3.read()
          if limit_vc_info:
            self.limit_vc_info = float(limit_vc_info)
      except Exception as e:
        pass

    if self.osm_access_counter_ct % 20 == 0:
      try:
        with open('/dev/shm/osm_access_counter.txt','r') as fp3:
          self.osm_access_counter_txt = fp3.read()
      except Exception as e:
        pass
    self.osm_access_counter_ct += 1

    if self.osm_access_counter_txt:
      osm_access_data = self.osm_access_counter_txt.split(",")
      self.osm_per = int(osm_access_data[0])
      osm_frame_ct2 = int(osm_access_data[1])
      if osm_frame_ct2 == self.before_osm_frame_ct:
        self.osm_frame_ct_ct += 1 #osm_frame_ct2が変化しなければカウントアップし続ける
      else:
        self.osm_frame_ct_ct = 0 #ゼロに戻らなければ、osmへの通信が死んでいる。
      self.before_osm_frame_ct = osm_frame_ct2

    self.a0 = 150
    self.a1 = 150
    self.a2 = 150
    self.a3 = 150
    if self.global_engageable and not(ui_state.status == UIStatus.ENGAGED or ui_state.status == UIStatus.OVERRIDE):
      self.a0 = 50
      self.a1 = 50
      self.a2 = 50
      self.a3 = 50
    elif self.global_engageable and (ui_state.status == UIStatus.ENGAGED or ui_state.status == UIStatus.OVERRIDE):
      self.a0 = 50
      self.a1 = 50
      self.a2 = 50
      self.a3 = 50
      if self.vc_speed < 1/3.6:
        self.a3 = 200
      if self.limit_speed_auto_detect == 1: #インジケーターはACC自動設定時にするか、速度標識表示時にするか検討中
        self.a2 = 200
      if self.curve_brake:
        self.a1 = 200
      if self.is_cruise_set:
        acc_speed = self.set_speed
        if acc_speed > 0 and (acc_speed < (31 if self.tss_type <= 1 else 26.0)) or (acc_speed > 109.0 and self.phv_2019 == False and self.tss_type <= 1):
          self.a0 = 200

    if self.red_signal_scan_flag_txt_ct % 7 == 0:
      try:
        with open('/dev/shm/red_signal_scan_flag.txt','r') as fp:
          red_signal_scan_flag = fp.read()
          if self.accel_engaged >= 3: #self.accel_engaged == mAccelEngagedButton
            if red_signal_scan_flag:
              self.red_signal_scan_flag = int(red_signal_scan_flag)
          else:
            self.red_signal_scan_flag = 0
      except Exception as e:
        pass
    self.red_signal_scan_flag_txt_ct += 1

    if self.red_signal_scan_flag >= 2:
      self.night_mode_ct += 1 #VSCodeがウォルラス演算子(:=)の構文に対応してないので分けている。
    if (self.red_signal_scan_flag >= 2 and (self.night_mode_ct) % 11 == 0) or self.red_signal_scan_flag_txt_ct % (20*5) == 1: #5秒に一回は更新
      if ui_state.started:
        clipped_brightness = ui_state.light_sensor

        # CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
        if clipped_brightness <= 8:
          clipped_brightness = (clipped_brightness / 903.3)
        else:
          clipped_brightness = ((clipped_brightness + 16) / 116) ** 3

        # Scale back to 0% to 100%
        clipped_brightness = max(0.0, min(100.0, 100.0 * clipped_brightness))

        if self.clipped_brightness0 != clipped_brightness:
          self.clipped_brightness0 = clipped_brightness
          with open('/dev/shm/night_time_info.txt','w') as fp:
            fp.write("%d" % int(clipped_brightness))

          global g_night_mode
          g_night_mode = self.clipped_brightness0 < (90 if g_night_mode == 1 else 75) #ばたつかないようにする。80程度でかなり夕方。

    self.road_info_txt_flag = False
    self.road_info_txt_ct += 1
    if self.road_info_txt_ct % 20 == 17:
      try:
        with open('/dev/shm/road_info.txt','r') as fp:
          self.road_info_txt = fp.read()
      except Exception as e:
        pass

      # osm_access_data = self.osm_access_counter_txt.split(",")
      # self.osm_per = int(osm_access_data[0])
      # osm_frame_ct2 = int(osm_access_data[1])

    self.disp_ichiro_logo = False
    if self.road_info_txt:
      road_info_data = self.road_info_txt.split(",")
      if len(road_info_data) >=4:
        road_th_ct = int(road_info_data[0])
        if road_th_ct == self.before_road_th_ct:
          self.road_th_ct_ct += 1 #road_th_ctが変化しなければカウントアップし続ける
        else:
          self.road_th_ct_ct = 0 #30秒以上ゼロに戻らなければ、road_info_txt_flag = falseにして、道路名は出さない。
        self.before_road_th_ct = road_th_ct

        self.kmh = road_info_data[1]
        self.road_name = road_info_data[2]
        self.road_bear = road_info_data[3]

  def _button_push_sound(self,onoff):
    with open('/dev/shm/sound_py_request.txt','w') as fp2:
      if onoff:
        fp2.write('%d' % (102)) #pipo.wav
      else:
        fp2.write('%d' % (101)) #po.wav

  def _press_accel_engaged(self):
    accel_engaged = 0
    try:
      with open('/dev/shm/accel_engaged.txt','r') as fp:
        accel_engaged_str = fp.read()
        if accel_engaged_str:
          accel_engaged = int(accel_engaged_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      accel_engaged = (accel_engaged + 1) % 5
    if accel_engaged == 0:
      self._accel_engaged_button.set_text("A")
      self._accel_engaged_button.set_button_style(ButtonStyle.HudSOff)
    elif accel_engaged == 1:
      self._accel_engaged_button.set_text("A")
    elif accel_engaged == 2:
      self._accel_engaged_button.set_text("AA")
    elif accel_engaged == 3:
      self._accel_engaged_button.set_text("iP")
    elif accel_engaged == 4:
      self._accel_engaged_button.set_text("eP")

    self.accel_engaged = accel_engaged
    if accel_engaged != 0:
      self._accel_engaged_button.set_button_style(ButtonStyle.HudSOn)

    if self.button_style_only:
      return

    self._button_push_sound(accel_engaged)

    with open('/dev/shm/accel_engaged.txt','w') as fp2:
      fp2.write("%d" % (accel_engaged))
    with open('/data/accel_engaged.txt','w') as fp3:
      fp3.write("%d" % (accel_engaged))


  def _press_dexp_sw_mode(self):
    dexp_sw_mode = 0
    try:
      with open('/dev/shm/dexp_sw_mode.txt','r') as fp:
        dexp_sw_mode_str = fp.read()
        if dexp_sw_mode_str:
          dexp_sw_mode = int(dexp_sw_mode_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      dexp_sw_mode = (dexp_sw_mode + 1) % 2
    if dexp_sw_mode == 0:
      self._dexp_sw_mode_button.set_button_style(ButtonStyle.HudSOff)
    else:
      self._dexp_sw_mode_button.set_button_style(ButtonStyle.HudSOn)

    self.dexp_sw_mode = dexp_sw_mode

    if self.button_style_only:
      return

    self._button_push_sound(dexp_sw_mode)

    with open('/dev/shm/dexp_sw_mode.txt','w') as fp2:
      fp2.write("%d" % (dexp_sw_mode))
    with open('/data/dexp_sw_mode.txt','w') as fp3:
      fp3.write("%d" % (dexp_sw_mode))

  def _press_long_speeddown_disable(self):
    long_speeddown_disable = 0
    try:
      with open('/dev/shm/long_speeddown_disable.txt','r') as fp:
        long_speeddown_disable_str = fp.read()
        if long_speeddown_disable_str:
          long_speeddown_disable = int(long_speeddown_disable_str)
    except Exception as e:
      pass

    global long_speeddown_disable00
    long_speeddown_disable00 = long_speeddown_disable

    if self.button_style_only == False:
      long_speeddown_disable = (long_speeddown_disable + 1) % 2
    if long_speeddown_disable == 0:
      self._long_speeddown_disable_button.set_button_style(ButtonStyle.HudSOn)
    else:
      self._long_speeddown_disable_button.set_button_style(ButtonStyle.HudSOff)

    if self.button_style_only:
      return

    self._button_push_sound(1-long_speeddown_disable)

    with open('/dev/shm/long_speeddown_disable.txt','w') as fp2:
      fp2.write("%d" % (long_speeddown_disable))
    with open('/data/long_speeddown_disable.txt','w') as fp3:
      fp3.write("%d" % (long_speeddown_disable))

  def _press_lta_enable_sw(self):
    lta_enable_sw = 0
    try:
      with open('/dev/shm/lta_enable_sw.txt','r') as fp:
        lta_enable_sw_str = fp.read()
        if lta_enable_sw_str:
          lta_enable_sw = int(lta_enable_sw_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      lta_enable_sw = (lta_enable_sw + 1) % 2
    if lta_enable_sw == 0:
      self._lta_enable_sw_button.set_button_style(ButtonStyle.HudSOff)
    else:
      self._lta_enable_sw_button.set_button_style(ButtonStyle.HudSOn)

    if self.button_style_only:
      return

    self._button_push_sound(lta_enable_sw)

    with open('/dev/shm/lta_enable_sw.txt','w') as fp2:
      fp2.write("%d" % (lta_enable_sw))
    with open('/data/lta_enable_sw.txt','w') as fp3:
      fp3.write("%d" % (lta_enable_sw))


  def _press_start_accel_power_up_disp_enable(self):
    start_accel_power_up_disp_enable = 0
    try:
      with open('/dev/shm/start_accel_power_up_disp_enable.txt','r') as fp:
        start_accel_power_up_disp_enable_str = fp.read()
        if start_accel_power_up_disp_enable_str:
          start_accel_power_up_disp_enable = int(start_accel_power_up_disp_enable_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      start_accel_power_up_disp_enable = (start_accel_power_up_disp_enable + 1) % 2
    if start_accel_power_up_disp_enable == 0:
      self._start_accel_power_up_disp_enable_button.set_button_style(ButtonStyle.HudSOff)
    else:
      self._start_accel_power_up_disp_enable_button.set_button_style(ButtonStyle.HudSOn)

    if self.button_style_only:
      return

    self._button_push_sound(start_accel_power_up_disp_enable)

    with open('/dev/shm/start_accel_power_up_disp_enable.txt','w') as fp2:
      fp2.write("%d" % (start_accel_power_up_disp_enable))
    with open('/data/start_accel_power_up_disp_enable.txt','w') as fp3:
      fp3.write("%d" % (start_accel_power_up_disp_enable))


  def _press_accel_ctrl_disable(self):
    accel_ctrl_disable = 0
    try:
      with open('/dev/shm/accel_ctrl_disable.txt','r') as fp:
        accel_ctrl_disable_str = fp.read()
        if accel_ctrl_disable_str:
          accel_ctrl_disable = int(accel_ctrl_disable_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      accel_ctrl_disable = (accel_ctrl_disable + 1) % 2
    if accel_ctrl_disable == 0:
      self._accel_ctrl_disable_button.set_button_style(ButtonStyle.HudSOn)
    else:
      self._accel_ctrl_disable_button.set_button_style(ButtonStyle.HudSOff)

    if self.button_style_only:
      return

    self._button_push_sound(1-accel_ctrl_disable)

    with open('/dev/shm/accel_ctrl_disable.txt','w') as fp2:
      fp2.write("%d" % (accel_ctrl_disable))
    with open('/data/accel_ctrl_disable.txt','w') as fp3:
      fp3.write("%d" % (accel_ctrl_disable))


  def _press_decel_ctrl_disable(self):
    decel_ctrl_disable = 0
    try:
      with open('/dev/shm/decel_ctrl_disable.txt','r') as fp:
        decel_ctrl_disable_str = fp.read()
        if decel_ctrl_disable_str:
          decel_ctrl_disable = int(decel_ctrl_disable_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      decel_ctrl_disable = (decel_ctrl_disable + 1) % 2
    if decel_ctrl_disable == 0:
      self._decel_ctrl_disable_button.set_button_style(ButtonStyle.HudSOn)
    else:
      self._decel_ctrl_disable_button.set_button_style(ButtonStyle.HudSOff)

    if self.button_style_only:
      return

    self._button_push_sound(1-decel_ctrl_disable)

    with open('/dev/shm/decel_ctrl_disable.txt','w') as fp2:
      fp2.write("%d" % (decel_ctrl_disable))
    with open('/data/decel_ctrl_disable.txt','w') as fp3:
      fp3.write("%d" % (decel_ctrl_disable))


  def _press_knight_scanner_bit3(self):
    Knight_scanner = 0
    try:
      with open('/dev/shm/knight_scanner_bit3.txt','r') as fp:
        Knight_scanner_str = fp.read()
        if Knight_scanner_str:
          Knight_scanner = int(Knight_scanner_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      Knight_scanner = (Knight_scanner + 1) % 8
    if Knight_scanner == 0:
      self._knight_scanner_bit3_button.set_text("○ ○ ○")
      self._knight_scanner_bit3_button.set_button_style(ButtonStyle.HudUnder)
    elif Knight_scanner == 1:
      self._knight_scanner_bit3_button.set_text("● ○ ○")
    elif Knight_scanner == 2:
      self._knight_scanner_bit3_button.set_text("○ ● ○")
    elif Knight_scanner == 3:
      self._knight_scanner_bit3_button.set_text("● ● ○")
    elif Knight_scanner == 4:
      self._knight_scanner_bit3_button.set_text("○ ○ ●")
    elif Knight_scanner == 5:
      self._knight_scanner_bit3_button.set_text("● ○ ●")
    elif Knight_scanner == 6:
      self._knight_scanner_bit3_button.set_text("○ ● ●")
    elif Knight_scanner == 7:
      self._knight_scanner_bit3_button.set_text("● ● ●")

    self.Knight_scanner = Knight_scanner

    if Knight_scanner != 0:
      self._knight_scanner_bit3_button.set_button_style(ButtonStyle.HudUnder)

    if self.button_style_only:
      return

    self._button_push_sound(Knight_scanner)

    with open('/dev/shm/knight_scanner_bit3.txt','w') as fp2:
      fp2.write("%d" % (Knight_scanner))
    with open('/data/knight_scanner_bit3.txt','w') as fp3:
      fp3.write("%d" % (Knight_scanner))


  def _press_limitspeed_sw(self):
    limitspeed_sw = 0
    try:
      with open('/dev/shm/limitspeed_sw.txt','r') as fp:
        limitspeed_sw_str = fp.read()
        if limitspeed_sw_str:
          limitspeed_sw = int(limitspeed_sw_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      limitspeed_sw = (limitspeed_sw + 1) % 3
    if limitspeed_sw == 0:
      self._limitspeed_sw_button.set_text("○")
      self._limitspeed_sw_button.set_button_style(ButtonStyle.HudUnder)
    elif limitspeed_sw == 1:
      self._limitspeed_sw_button.set_text("●")
    elif limitspeed_sw == 2:
      self._limitspeed_sw_button.set_text("⬇︎")

    self.Limit_speed_mode = limitspeed_sw

    if limitspeed_sw != 0:
      self._limitspeed_sw_button.set_button_style(ButtonStyle.HudUnder)

    if self.button_style_only:
      return

    self._button_push_sound(limitspeed_sw)

    with open('/dev/shm/limitspeed_sw.txt','w') as fp2:
      fp2.write("%d" % (limitspeed_sw))
    with open('/data/limitspeed_sw.txt','w') as fp3:
      fp3.write("%d" % (limitspeed_sw))


  def _press_LongitudinalPersonality(self):
    psn_str = Params().get("LongitudinalPersonality")
    psn = int(psn_str)

    if self.button_style_only == False:
      psn = (psn -1 + 3) % 3

    if psn == 0:
      self._LongitudinalPersonality_button.set_text("⬆︎⬆︎⬆︎")
    elif psn == 1:
      self._LongitudinalPersonality_button.set_text("⬆︎⬆︎")
    else:
      self._LongitudinalPersonality_button.set_text("⬆︎")

    self._LongitudinalPersonality_button.set_button_style(ButtonStyle.HudUnder)

    if self.button_style_only:
      return

    self._button_push_sound(1)

    Params().put("LongitudinalPersonality", psn)


  def _press_lockon_disp_disable(self):
    lockon_disp_disable = 0
    try:
      with open('/dev/shm/lockon_disp_disable.txt','r') as fp:
        lockon_disp_disable_str = fp.read()
        if lockon_disp_disable_str:
          lockon_disp_disable = int(lockon_disp_disable_str)
    except Exception as e:
      pass

    if self.button_style_only == False:
      lockon_disp_disable = (lockon_disp_disable + 1) % 2
    if lockon_disp_disable == 0:
      self._lockon_disp_disable_button.set_text("■")
      self._lockon_disp_disable_button.set_button_style(ButtonStyle.HudUnder)
    else:
      self._lockon_disp_disable_button.set_text("□")
      self._lockon_disp_disable_button.set_button_style(ButtonStyle.HudUnder)

    global g_lockon_disp_disable
    g_lockon_disp_disable = lockon_disp_disable

    if self.button_style_only:
      return

    self._button_push_sound(1-lockon_disp_disable)

    with open('/dev/shm/lockon_disp_disable.txt','w') as fp2:
      fp2.write("%d" % (lockon_disp_disable))
    # with open('/data/lockon_disp_disable.txt','w') as fp3:
    #   fp3.write("%d" % (lockon_disp_disable))



  def _press_set_speed_MAX(self):
    sm = ui_state.sm
    cs = sm["selfdriveState"]

    accel_engaged = self.accel_engaged

    if accel_engaged >= 3 and cs.enabled: #ワンペダルのみ
      if int(self.set_speed) != 1: #MAXが1ではない時
        if sm["carState"].vEgo < 0.1/3.6: #スピードが出ていない時
          with open('/dev/shm/force_one_pedal.txt','w') as fp:
            fp.write('%d' % (1)) #これがセットされる条件をなるべく絞る。
        else:
          #⚫︎ボタンの代わりに動作する
          self._press_limitspeed_sw() #MAX_touch()
      else:
        #MAX=1でタッチ(↑ボタン効果で",1"も含む)
        vego = sm["carState"].vEgo
        if vego > 3/3.6 and vego <= 30/3.6: #スピードが3〜30km/hのとき
          with open('/dev/shm/force_low_engage.txt','w') as fp:
            fp.write('%d' % (1))
        else:
          #⚫︎ボタンの代わりに動作する
          self._press_limitspeed_sw() #MAX_touch()
    else:
      #⚫︎ボタンの代わりに動作する
      self._press_limitspeed_sw() #MAX_touch


  def _drawText(self,font,font_size,x,y,text,alpha,brakeLight=False,color_ex=rl.Color(0,0,0,0)):
    text_size = measure_text_cached(font, text, font_size)
    if alpha < 0:
      color = color_ex
    elif brakeLight == False:
      color = rl.Color(0xff, 0xff, 0xff, alpha)
    else:
      alpha += 100
      if alpha > 255:
        alpha = 255
      color = rl.Color(0xff, 0, 0, alpha)

    rl.draw_text_ex(font,text,
      rl.Vector2(x-text_size.x/2, y - text_size.y), #yを下段として表示。
      font_size,
      0, #spacing
      color,
    )
    return text_size.x #続けて利用できるように幅を返す。（次の表示を左右の隣に出すために使える）

  def _drawTextLeft(self, font,font_size, x,y,text,alpha=255 ,brakeLight=False ,red=255, grn=255, blu=255 , bk_red=0, bk_grn=0, bk_blu=0, bk_alp=0, bk_yofs=0, bk_corner_r=0, bk_add_w=0, bk_xofs=0, bk_add_h=0):
    text_size = measure_text_cached(font, text, font_size)

    if bk_alp > 0:
      #//バックを塗る。
      bk_color = rl.Color(int(bk_red), int(bk_grn), int(bk_blu), int(bk_alp))
      rc = rl.Rectangle(x+bk_xofs,y-text_size.y+bk_yofs,text_size.x+bk_add_w,text_size.y+bk_add_h)
      rl.draw_rectangle_rounded(rc, bk_corner_r, 10, bk_color)

    if brakeLight == False:
      pen_color = rl.Color(int(red), int(grn), int(blu), int(alpha))
    else:
      alpha += 100
      if alpha > 255:
        alpha = 255
      pen_color = rl.Color(0xff, 0, 0, int(alpha))

    rl.draw_text_ex(
      font,
      text,
      rl.Vector2(x, y-text_size.y),
      font_size,
      0,
      pen_color,
    )

    return x + text_size.x #続けて並べるxposを返す。

  def _drawTextRight(self, font,font_size, x,y,text,alpha=255 ,brakeLight=False ,red=255, grn=255, blu=255 , bk_red=0, bk_grn=0, bk_blu=0, bk_alp=0, bk_yofs=0, bk_corner_r=0, bk_add_w=0, bk_xofs=0, bk_add_h=0):
    text_size = measure_text_cached(font, text, font_size)
    x -= text_size.x #右寄せ

    if bk_alp > 0:
      #//バックを塗る。
      bk_color = rl.Color(int(bk_red), int(bk_grn), int(bk_blu), int(bk_alp))
      rc = rl.Rectangle(x+bk_xofs,y-text_size.y+bk_yofs,text_size.x+bk_add_w,text_size.y+bk_add_h)
      rl.draw_rectangle_rounded(rc, bk_corner_r, 10, bk_color)

    if brakeLight == False:
      pen_color = rl.Color(int(red), int(grn), int(blu), int(alpha))
    else:
      alpha += 100
      if alpha > 255:
        alpha = 255
      pen_color = rl.Color(0xff, 0, 0, int(alpha))

    rl.draw_text_ex(
      font,
      text,
      rl.Vector2(x, y-text_size.y),
      font_size,
      0,
      pen_color,
    )

    return x #続けて並べるxposを返す。

  def _drawTextCenter(self, font,font_size, x,y,text,alpha=255 ,brakeLight=False ,red=255, grn=255 , blu=255, bk_red=0, bk_grn=0, bk_blu=0, bk_alp=0, bk_yofs=0, bk_corner_r=0, bk_add_w=0, bk_xofs=0, bk_add_h=0):
    text_size = measure_text_cached(font, text, font_size)
    x -= text_size.x /2 #中央寄せ

    if bk_alp > 0:
      #//バックを塗る。
      bk_color = rl.Color(int(bk_red), int(bk_grn), int(bk_blu), int(bk_alp))
      rc = rl.Rectangle(x+bk_xofs,y-text_size.y+bk_yofs,text_size.x+bk_add_w,text_size.y+bk_add_h)
      rl.draw_rectangle_rounded(rc, bk_corner_r, 10, bk_color)

    if brakeLight == False:
      pen_color = rl.Color(int(red), int(grn), int(blu), int(alpha))
    else:
      alpha += 100
      if alpha > 255:
        alpha = 255
      pen_color = rl.Color(0xff, 0, 0, int(alpha))

    rl.draw_text_ex(
      font,
      text,
      rl.Vector2(x, y-text_size.y),
      font_size,
      0,
      pen_color,
    )

    return text_size.x #続けて利用できるように幅を返す。（次の表示を左右の隣に出すために使える）

  def knightScanner(self, rect: rl.Rectangle):
    height = rect.height
    width = rect.width
#   extern int global_status; #ui_state.status == UIStatus.ENGAGED
#   extern int Knight_scanner; #self.Knight_scanner

    rect_w = width #rect().width();
    rect_h = height #rect().height();

    n = 15+1 #タイミングの問題で画面外に一つ増やす
    t = self.ktsc_t #static float t[n];
    t[(int)(self.ktsc_ct/self.ktsc_ct_n)] = 1.0
    ww = rect_w / (n-1) #画面外の一つ分を外す。
    hh = rect_h / 15 #ww

    curve_value = self.limit_vc_info
    if curve_value == 0 or self.global_engageable == False:
      dir = self.dir0 * 0.25
      hh = hh / 3
    elif curve_value < 145:
      try:
        with open('/dev/shm/steer_ang_predicate.txt','r') as fp: #md.position.yによる前方カーブ予測が急な時にTrue
          steer_ang_predicate = fp.read()
          if steer_ang_predicate and int(steer_ang_predicate) != 0:
            dir = self.dir0 * 1.0
          else:
            dir = self.dir0 * 0.5
            if self.vc_speed < 0.1/3.6:
              hh = hh / 3
            else:
              hh = hh * 2 / 3
      except Exception as e:
        dir = self.dir0 * 0.5
        if self.vc_speed < 0.1/3.6:
          hh = hh / 3
        else:
          hh = hh * 2 / 3
    else:
      dir = self.dir0 * 0.5
      if self.vc_speed < 0.1/3.6:
        hh = hh / 3
      else:
        hh = hh * 2 / 3

    left_blinker = ui_state.sm["carState"].leftBlinker
    right_blinker = ui_state.sm["carState"].rightBlinker
    lane_change_height = 0 #280; //↓の下の尖りがウインカーの底辺になるように調整。
    if left_blinker or right_blinker:
      if left_blinker == True:
        self.dir0 = -abs(self.dir0)
      elif right_blinker == True:
        self.dir0 = abs(self.dir0)
      dir = self.dir0 * 1.0
      hh = rect_h / 15 #ww
      hh = hh * 2 / 3
# #if 0
#     if((*s->sm)["carState"].getCarState().getVEgo() >= 50/3.6){ //
#       lane_change_height = 270;
#     }
# #elif 0 //メッセージUIの表示手法変更で、下の隙間から見えるので、lane_change_height持ち上げはひとまず取りやめ。
#     auto lp = (*s->sm)["lateralPlan"].getLateralPlan();
#     if( lp.getLaneChangeState() == cereal::LateralPlan::LaneChangeState::PRE_LANE_CHANGE ||
#         lp.getLaneChangeState() == cereal::LateralPlan::LaneChangeState::LANE_CHANGE_STARTING){ //レーンチェンジの表示で判定
#       lane_change_height = 270;
#     } else { //stand_stillでもウインカーを上げる。
#       std::string stand_still_txt = util::read_file("/dev/shm/stand_still.txt");
#       bool stand_still = false;
#       if(stand_still_txt.empty() == false){
#         stand_still = std::stoi(stand_still_txt) ? true : false;
#       }
#       if(stand_still){
#         lane_change_height = 270;
#       }
#     }
# #endif

    h_pos = rect.y + rect_h - hh

    self.ktsc_ct += dir
    if self.ktsc_ct <= 0 or self.ktsc_ct >= n*self.ktsc_ct_n-1:
      if left_blinker or right_blinker:
        if left_blinker == True and self.ktsc_ct < 0:
          self.ktsc_ct = n*self.ktsc_ct_n-1
        elif right_blinker == True and self.ktsc_ct > n*self.ktsc_ct_n-1:
          self.ktsc_ct = 0
      else:
        if self.ktsc_ct < 0 and dir < 0:
          self.ktsc_ct = 0
        if self.ktsc_ct > n*self.ktsc_ct_n-1 and dir > 0:
          self.ktsc_ct = n*self.ktsc_ct_n-1
        self.dir0 = -self.dir0

    #呼び出し元の状態からここは全て加算ブレンドになる。   p.setCompositionMode(QPainter::CompositionMode_Plus);
    for i in range(n - 1): #for(int i=0; i<(n-1); i++){
      if t[i] > 0.01:
        if left_blinker or right_blinker:
          #流れるウインカー
          kt_color = rl.Color(192, 102, 0, int(255 * t[i]))
        elif self.handle_center >= -99:
          kt_color = rl.Color(200, 0, 0, int(255 * t[i]))
        else:
          kt_color = rl.Color(200, 200, 0, int(255 * t[i])) #ハンドルセンターキャリブレーション中は色を緑に。

        if left_blinker or right_blinker:
          rc = rl.Rectangle(rect.x+rect_w * i / (n-1),h_pos - lane_change_height,ww,hh) #drawRectを使う利点は、角を取ったりできそうだ。
          rl.draw_rectangle_rounded(rc, 1.0, 10, kt_color)
        else: #単に上梅のフラットにする。
          if self.Knight_scanner == 0:
            continue

          rc = rl.Rectangle(rect.x+rect_w * i / (n-1),h_pos - lane_change_height,ww,hh) #drawRectを使う利点は、角を取ったりできそうだ。
          rl.draw_rectangle_rounded(rc, 0.5, 5, kt_color)
#         //ポリゴンで表示。
#         float sx_a = rect_w * i / (n-1) - rect_w / 2;
#         sx_a /= (rect_w / 2); // -1〜1
#         float sx_b = rect_w * (i+1) / (n-1) - rect_w / 2;
#         sx_b /= (rect_w / 2); // -1〜1
#         float x0 = rect_w * i / (n-1);
#         float x1 = x0 + ww;
#         float y0 = h_pos;
#         float y1 = y0 + hh;
#         y0 -= ww/6; //少し持ち上げる。
#         float y0_a = y0 + hh/2 * (1 - sx_a*sx_a); //関数の高さ計算に加減速を反省させればビヨビヨするはず。
#         float y0_b = y0 + hh/2 * (1 - sx_b*sx_b);
#         QPointF scaner[] = {{x0,y0_a},{x1,y0_b}, {x1,y1}, {x0,y1}};
#         p.drawPolygon(scaner, std::size(scaner));
#       }
#     }
      t[i] *= 0.9
    pass
