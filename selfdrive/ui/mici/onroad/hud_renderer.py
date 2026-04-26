import pyray as rl
import time
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.common.filter_simple import FirstOrderFilter
from cereal import log

EventName = log.OnroadEvent.EventName

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

SET_SPEED_PERSISTENCE = 2.5  # seconds


@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 36
  set_speed: int = 112


@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)


FONT_SIZES = FontSizes()
COLORS = Colors()


class TurnIntent(Widget):
  FADE_IN_ANGLE = 30  # degrees

  def __init__(self):
    super().__init__()
    self._pre = False
    self._turn_intent_direction: int = 0

    self._turn_intent_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._turn_intent_rotation_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._txt_turn_intent_left: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20)
    self._txt_turn_intent_right: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 20, flip_x=True)

  def _render(self, _):
    if self._turn_intent_alpha_filter.x > 1e-2:
      turn_intent_texture = self._txt_turn_intent_right if self._turn_intent_direction == 1 else self._txt_turn_intent_left
      src_rect = rl.Rectangle(0, 0, turn_intent_texture.width, turn_intent_texture.height)
      dest_rect = rl.Rectangle(self._rect.x + self._rect.width / 2, self._rect.y + self._rect.height / 2,
                               turn_intent_texture.width, turn_intent_texture.height)

      origin = (turn_intent_texture.width / 2, self._rect.height / 2)
      color = rl.Color(255, 255, 255, int(255 * self._turn_intent_alpha_filter.x))
      rl.draw_texture_pro(turn_intent_texture, src_rect, dest_rect, origin, self._turn_intent_rotation_filter.x, color)

  def _update_state(self) -> None:
    sm = ui_state.sm

    left = any(e.name == EventName.preLaneChangeLeft for e in sm['onroadEvents'])
    right = any(e.name == EventName.preLaneChangeRight for e in sm['onroadEvents'])
    if left or right:
      # pre lane change
      if not self._pre:
        self._turn_intent_rotation_filter.x = self.FADE_IN_ANGLE if left else -self.FADE_IN_ANGLE

      self._pre = True
      self._turn_intent_direction = -1 if left else 1
      self._turn_intent_alpha_filter.update(1)
      self._turn_intent_rotation_filter.update(0)
    elif any(e.name == EventName.laneChange for e in sm['onroadEvents']):
      # fade out and rotate away
      self._pre = False
      self._turn_intent_alpha_filter.update(0)

      if self._turn_intent_direction == 0:
        # unknown. missed pre frame?
        self._turn_intent_rotation_filter.update(0)
      else:
        self._turn_intent_rotation_filter.update(self._turn_intent_direction * self.FADE_IN_ANGLE)
    else:
      # didn't complete lane change, just hide
      self._pre = False
      self._turn_intent_direction = 0
      self._turn_intent_alpha_filter.update(0)
      self._turn_intent_rotation_filter.update(0)


class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    """Initialize the HUD renderer."""
    self.is_cruise_set: bool = False
    self.is_cruise_available: bool = True
    self.set_speed: float = SET_SPEED_NA
    self._set_speed_changed_time: float = 0
    self.speed: float = 0.0
    self.v_ego_cluster_seen: bool = False
    self._engaged: bool = False

    self._can_draw_top_icons = True
    self._show_wheel_critical = False

    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_display: rl.Font = gui_app.font(FontWeight.DISPLAY)
    # self._font_uni: rl.Font = gui_app.font("JP2") #動的ロード

    self._turn_intent = TurnIntent()
    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', 50, 50)
    self._txt_wheel_critical: rl.Texture = gui_app.texture('icons_mici/wheel_critical.png', 50, 50)
    self._txt_exclamation_point: rl.Texture = gui_app.texture('icons_mici/exclamation_point.png', 9, 44)

    self._wheel_alpha_filter = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)
    self._wheel_y_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    self._set_speed_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    self._ip_button_init()

  def set_wheel_critical_icon(self, critical: bool):
    """Set the wheel icon to critical or normal state."""
    self._show_wheel_critical = critical

  def set_can_draw_top_icons(self, can_draw_top_icons: bool):
    """Set whether to draw the top part of the HUD."""
    self._can_draw_top_icons = can_draw_top_icons

  def drawing_top_icons(self) -> bool:
    # whether we're drawing any top icons currently
    return bool(self._set_speed_alpha_filter.x > 1e-2)

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
    set_speed = (
      controls_state.deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
    )
    ACC_speed = set_speed

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
          set_speed = int(cruise_info_str)
    except Exception as e:
      pass

    # self.db_rec_mode = False
    # if self.limit_speed_override == False:
    #   # self.yellow_flag = False
    #   if self.Limit_speed_mode == 2:
    #     # rect_color = rl.Color(100, 0, 0, 250)
    #     if self.set_speed >= 30:
    #       self.db_rec_mode = True

    engaged = sm['selfdriveState'].enabled
    if (set_speed != self.set_speed and engaged) or (engaged and not self._engaged):
      self._set_speed_changed_time = rl.get_time()
    self._engaged = engaged
    self.set_speed = set_speed
    self.is_cruise_set = 0 < ACC_speed < SET_SPEED_NA
    self.is_cruise_available = ACC_speed != -1

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)
    self._ip_update_state(sm)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""

    self._torque_bar.render(rect)

    if True: #self.is_cruise_set:
      self._draw_set_speed(rect)

    self._draw_steering_wheel(rect)
    self._ip_draw(rect)

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else self._txt_wheel

    if self._show_wheel_critical:
      self._wheel_alpha_filter.update(255)
      self._wheel_y_filter.update(0)
    else:
      if ui_state.status == UIStatus.DISENGAGED:
        self._wheel_alpha_filter.update(0)
        self._wheel_y_filter.update(wheel_txt.height / 2)
      else:
        self._wheel_alpha_filter.update(255 * 0.9)
        self._wheel_y_filter.update(0)

    # pos
    pos_x = int(rect.x + 21 + wheel_txt.width / 2)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + self._wheel_y_filter.x)
    rotation = -ui_state.sm['carState'].steeringAngleDeg

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(
      pos_x - wheel_txt.width / 2 - turn_intent_margin,
      pos_y - wheel_txt.height / 2 - turn_intent_margin,
      wheel_txt.width + turn_intent_margin * 2,
      wheel_txt.height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height)
    origin = (wheel_txt.width / 2, wheel_txt.height / 2)

    if self.steer_always:
      if self.cruise_available:
        center = rl.Vector2( #dest_rectの左上がセンター（draw_texture_proのoriginの影響を考慮）
            dest_rect.x,
            dest_rect.y
        )
        radius = dest_rect.width / 2 + 5  # 正円なら width/2 でOK
        rl.draw_circle_v(center, radius, rl.Color(0x17, 0x86, 0x44, 224))

    # color and draw
    color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, color)

    if self._show_wheel_critical:
      # Draw exclamation point icon
      EXCLAMATION_POINT_SPACING = 10
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 2
      rl.draw_texture_ex(self._txt_exclamation_point, rl.Vector2(exclamation_pos_x, exclamation_pos_y), 0.0, 1.0, rl.WHITE)

    mads_margin = 10
    mads_rect = rl.Rectangle(
      pos_x - wheel_txt.width / 2 - mads_margin,
      pos_y - wheel_txt.height / 2 - mads_margin,
      wheel_txt.width + mads_margin * 2,
      wheel_txt.height + mads_margin * 2,
    )
    self._mads_button.render(mads_rect)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    alpha = self._set_speed_alpha_filter.update(0 < rl.get_time() - self._set_speed_changed_time < SET_SPEED_PERSISTENCE and
                                                self._can_draw_top_icons and self._engaged)
    if self._engaged: #standstill状態でも表示したい。
      alpha = 1.0
    else:
      alpha = 0.0
    if alpha < 1e-2:
      return

    x = rect.x
    y = rect.y

    # draw drop shadow
    circle_radius = 162 // 2
    x += rect.width - circle_radius*2
    y += rect.height - circle_radius*2 + 7
    rl.draw_circle_gradient(int(x + circle_radius), int(y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    self.yellow_flash_ct += 1
    self.db_rec_mode = False
    if self.limit_speed_override == False:
      self.yellow_flag = False
      if self.Limit_speed_mode == 2:
        set_speed_color = rl.Color(180, 0, 0, 250)
        if self.set_speed >= 30:
          self.db_rec_mode = True
        self.yellow_flag = True
      elif self.Limit_speed_mode == 1 and self.limit_speed_auto_detect == 1:
        if self.maxspeed_org+12 <= self.set_speed and self.maxspeed_org+5 < self.vc_speed * 3.6:
          if self.yellow_flash_ct % 6 < 3:
            set_speed_color = rl.Color(255, 255, 0, 255) # 速度がレバーより10km/h以上高いとギクシャクする警告、点滅させる。
            self.yellow_flag = True
    else:
      if self.maxspeed_org+12 > self.set_speed or self.maxspeed_org+5 >= self.vc_speed * 3.6:
        #g_night_modeは保留
        pass #set_speed_color = rl.Color(235, 235, 235, 200)
      elif self.is_cruise_set:
        if self.yellow_flash_ct % 6 < 3:
          set_speed_color = rl.Color(255, 255, 0, 255) # 速度がレバーより10km/h以上高いとギクシャクする警告、点滅させる。
        else:
          pass # set_speed_color = rl.Color(235, 235, 235, 200)

    if self.add_v_by_lead and self.is_cruise_set:
      max_color = rl.Color(0, 0xff, 0, 200) #前走車追従時は緑
    if self.curve_brake and self.is_cruise_set:
      #set_speed_color = COLORS.black_translucent
      max_color = rl.Color(0xff, 0, 0, 200) #減速時は赤
    if self.turbo_boost and self.is_cruise_set:
      max_color = rl.Color(0xff, 0xff, 0, 200) #スタートダッシュ時は黄色

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))

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

    set_speed_text_size = measure_text_cached(self._font_display, set_speed_text, FONT_SIZES.set_speed)
    if self.Limit_speed_mode == 2:
      rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(x -5 + circle_radius*2 - set_speed_text_size.x, y + 3 - 8 - 3 + 4),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )
    if self.Limit_speed_mode == 2:
      rl.end_blend_mode() #元のブレンドに戻す

    max_text = tr("MAX")
    if self.limit_speed_override:
      max_text = tr("AUTO")
    if self.db_rec_mode:
      max_text = tr("REC")
    max_text_size = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed)
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x - 10 + circle_radius*2 - max_text_size.x, y + FONT_SIZES.set_speed - 7 + 4),
      FONT_SIZES.max_speed,
      0,
      max_color,
    )

    set_speed_rect = rl.Rectangle(x+50, y, circle_radius*2-50+30, circle_radius*2+20)
    self._set_speed_MAX_button.render(set_speed_rect)

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)

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

    self.dt = 50 #フレームタイム
    self.distance_traveled = 0
    self.prev_draw_t = time.monotonic_ns() / 1_000_000

    self.button_style_only = True

    self.limit_speed_override = False
    self.add_v_by_lead = False
    self.curve_brake = False
    self.turbo_boost = False
    self.db_rec_mode = False
    self.Limit_speed_mode = 0

    self.ktsc_ct_n = 1
    self.ktsc_ct = 0
    n = 15+1 #タイミングの問題で画面外に一つ増やす
    self.ktsc_t = [0.0] * n
    self.dir0 = 1.0
    self.Knight_scanner = 0

    self.limit_vc_info = 0
    self.ip_update_state_ct = 0
    self.handle_center = -100
    self.handle_calibct = 0
    self.vc_speed = 0
    self._press_set_speed_MAX_ct = 0

    self._accel_engaged_button = Button("A",click_callback=self._press_accel_engaged,font_size=40,font_weight=FontWeight.BOLD, border_radius=45)
    self._press_accel_engaged()

    font_sz = 50 #ACC速度にかぶせる透明ボタン
    self._set_speed_MAX_button = Button("",click_callback=self._press_set_speed_MAX,font_size=font_sz,font_weight=FontWeight.BOLD, border_radius=0.35*200/2)
    self._set_speed_MAX_button.set_button_style(ButtonStyle.HudUnder) #バック透明
    self._press_set_speed_MAX() #_press_accel_engagedより後に呼ぶこと。

    #ハンドルにかぶせるMADS切り替え用透明ボタン
    self._mads_button = Button("",click_callback=self._press_mads,font_size=font_sz,font_weight=FontWeight.BOLD, border_radius=35)
    self._mads_button.set_button_style(ButtonStyle.HudUnder) #バック透明
    self._press_mads()

    dx_icon = gui_app.texture("icons_mici/onroad/dX_icon_128.png",width=60,height=60)
    self.dx_icon_chill = gui_app.texture("icons_mici/wheel.png",width=60,height=60)
    self.dx_icon_exp = gui_app.texture("icons_mici/experimental_mode.png",width=60,height=60)
    self._dexp_sw_mode_button = Button("",click_callback=self._press_dexp_sw_mode,font_size=font_sz,font_weight=FontWeight.BOLD, border_radius=45, icon=dx_icon)
    self._press_dexp_sw_mode()

    lane_icon = gui_app.texture("icons_mici/onroad/lane_keep_w.png",width=72,height=int(147*72/256))
    self._lta_enable_sw_button = Button("",click_callback=self._press_lta_enable_sw,font_size=font_sz,font_weight=FontWeight.BOLD, border_radius=45, icon=lane_icon)
    self._press_lta_enable_sw()

    arrow_up = gui_app.texture("icons_mici/onroad/arrow_up.png",width=60,height=60)
    self._accel_ctrl_disable_button = Button("",click_callback=self._press_accel_ctrl_disable,font_size=font_sz,font_weight=FontWeight.BOLD, border_radius=45, icon=arrow_up)
    self._press_accel_ctrl_disable()

    arrow_down = gui_app.texture("icons_mici/onroad/arrow_down.png",width=60,height=60)
    self._decel_ctrl_disable_button = Button("",click_callback=self._press_decel_ctrl_disable,font_size=font_sz,font_weight=FontWeight.BOLD, border_radius=45, icon=arrow_down)
    self._press_decel_ctrl_disable()

    arrow_up2 = gui_app.texture("icons_mici/onroad/arrow_up2.png",width=57,height=57)
    self._start_accel_power_up_disp_enable_button = Button("",click_callback=self._press_start_accel_power_up_disp_enable,font_size=font_sz,font_weight=FontWeight.BOLD, border_radius=45, icon=arrow_up2)
    self._press_start_accel_power_up_disp_enable()

    self.button_style_only = False

    self.yellow_flag = False
    self.limit_speed_auto_detect = False
    self.maxspeed_org = 0
    self.limit_speed_num = 0
    self.yellow_flash_ct = 0

    self.steer_always = False
    self.cruise_available = False

    self.osm_frame_ct_ct = -1 #-1 or 100以上でosmへの通信が死んでいる。
    self.osm_per = 0 #2Hzに対してosmの応答率。走行中ならだいたい50パーセントくらいになる。
    self.osm_access_counter_txt = ""
    #self.osm_access_counter_ct = 0
    self.before_osm_frame_ct = 0

    self.road_th_ct_ct = 0
    self.before_road_th_ct = 0
    self.road_info_txt_flag = False
    #self.road_info_txt_ct = 0
    self.road_info_txt = ""
    self.kmh = "" #制限速度
    self.road_name = "" #道路名
    self.road_bear = "99999" #道路方位
    #self.disp_ichiro_logo = False

    self.dexp_sw_mode = 0
    self._disp_button_ct = 0

  def user_interacting(self) -> bool:
    if self._press_set_speed_MAX_ct > 0:
      return True
    return (self._set_speed_MAX_button.is_pressed
            or self._mads_button.is_pressed
            or self._dexp_sw_mode_button.is_pressed
            or self._lta_enable_sw_button.is_pressed
            or self._accel_ctrl_disable_button.is_pressed
            or self._decel_ctrl_disable_button.is_pressed
            or self._start_accel_power_up_disp_enable_button.is_pressed
            or self._accel_engaged_button.is_pressed
            )

  def _ip_update_state(self,sm):
    self.ip_update_state_ct += 1 #c4は60fpsなのでファイルアクセス頻度が3倍になる。self.dtで補正できるが、今のところKnightScannerしかタイミングが早まるのを加味していない。
    car_state = sm['carState']
    self.vc_speed = car_state.vEgo
    self.maxspeed_org = car_state.vCruise #これで元の41〜 , v_cruise; //レバー値の元の値。黄色点滅警告にはマッチしてる気がする。

    if self._press_set_speed_MAX_ct > 0:
      self._press_set_speed_MAX_ct -= 1

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

    limitspeed_sw = 0
    try:
      with open('/dev/shm/limitspeed_sw.txt','r') as fp:
        limitspeed_sw_str = fp.read()
        if limitspeed_sw_str:
          limitspeed_sw = int(limitspeed_sw_str)
    except Exception as e:
      pass

    self.Limit_speed_mode = limitspeed_sw

    if self.ip_update_state_ct % 2 == 1:
      try:
        with open('/dev/shm/limit_vc_info.txt','r') as fp3:
          limit_vc_info = fp3.read()
          if limit_vc_info:
            self.limit_vc_info = float(limit_vc_info)
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

    if self.ip_update_state_ct % 20 == 1:
      Knight_scanner = 0
      try:
        with open('/dev/shm/knight_scanner_bit3.txt','r') as fp:
          Knight_scanner_str = fp.read()
          if Knight_scanner_str:
            Knight_scanner = int(Knight_scanner_str)
      except Exception as e:
        pass
      self.Knight_scanner = Knight_scanner

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

    if self.ip_update_state_ct % 20 == 13:
      try:
        with open('/dev/shm/steer_always.txt','r') as fp:
          steer_always_str = fp.read()
          if steer_always_str and int(steer_always_str) >= 1:
            self.steer_always = True
          else:
            self.steer_always = False
        with open('/dev/shm/cruise_available.txt','r') as fp:
          cruise_available_str = fp.read()
          if cruise_available_str and int(cruise_available_str) >= 1:
            self.cruise_available = True
          else:
            self.cruise_available = False
      except Exception as e:
        pass

    if self.ip_update_state_ct % 20 == 17:
      accel_engaged = 0
      try:
        with open('/dev/shm/accel_engaged.txt','r') as fp:
          accel_engaged_str = fp.read()
          if accel_engaged_str:
            accel_engaged = int(accel_engaged_str)
      except Exception as e:
        pass
      self.accel_engaged = accel_engaged

    if self.ip_update_state_ct % 20 == 7:
      try:
        with open('/dev/shm/osm_access_counter.txt','r') as fp3:
          self.osm_access_counter_txt = fp3.read()
      except Exception as e:
        pass

    if self.osm_access_counter_txt:
      osm_access_data = self.osm_access_counter_txt.split(",")
      self.osm_per = int(osm_access_data[0])
      osm_frame_ct2 = int(osm_access_data[1])
      if osm_frame_ct2 == self.before_osm_frame_ct:
        self.osm_frame_ct_ct += 1 #osm_frame_ct2が変化しなければカウントアップし続ける
      else:
        self.osm_frame_ct_ct = 0 #ゼロに戻らなければ、osmへの通信が死んでいる。
      self.before_osm_frame_ct = osm_frame_ct2

    self.road_info_txt_flag = False
    if self.ip_update_state_ct % 20 == 4:
      try:
        with open('/dev/shm/road_info.txt','r') as fp:
          self.road_info_txt = fp.read()
      except Exception as e:
        pass

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

    if(self.ip_update_state_ct % 10 == 5):
      self.button_style_only = True
      self._press_accel_ctrl_disable()
      self.button_style_only = False

    cur_draw_t = time.monotonic_ns() / 1_000_000  # ナノ秒→ミリ秒 #millis_since_boot();
    self.dt = cur_draw_t - self.prev_draw_t #フレームタイム
    self.distance_traveled += abs(car_state.vEgo) * self.dt / 1000
    self.prev_draw_t = cur_draw_t
    if self.dt == 0:
      self.dt = 1 #0割り算対策


  def _ip_draw(self, rect: rl.Rectangle):

    if gui_app.big_ui():
      right_margin = 120
      font_size = 44
      font_size_km = 33
      y_pos = rect.y+rect.height
      y_pos2 = rect.y+rect.height
    else:
      right_margin = 0
      font_size = 38
      font_size_km = 29
      y_pos = rect.y+font_size+3-2
      y_pos2 = rect.y+font_size+3 #速度用
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
          next_x = self._drawTextRight(self._font_uni, font_size , rect.x+rect.width-right_margin, y_pos+1 , self.road_name, 220, bk_alp=128, bk_corner_r= 0.2, bk_yofs=7,bk_add_h=-10,bk_add_w=0)
          self._drawTextRight(self._font_uni, font_size , rect.x+rect.width-right_margin-1, y_pos+1 , self.road_name, 220) #2重描き
          self._drawTextRight(self._font_semi_bold, font_size_km, next_x-4, y_pos2 - 4 , self.kmh , 255 , False , 0x24, 0x57, 0xa1 , 255,255,255,200 , 0 , 0.2 , 2 , -1)
        else:
          if self.road_name != "---":
            self._font_uni = gui_app.font("JP2", self.road_name) #動的ロード
            self._drawTextRight(self._font_uni, font_size , rect.x+rect.width-right_margin, y_pos+1 , self.road_name, 220, bk_alp=128, bk_corner_r= 0.2, bk_yofs=7,bk_add_h=-10,bk_add_w=0)
            self._drawTextRight(self._font_uni, font_size , rect.x+rect.width-right_margin-1, y_pos+1 , self.road_name, 220) #2重描き
          else:
            self.disp_ichiro_logo = True #速度ゼロの---は表示しない。(road_info_baeringは利用するのでroad_info_txt_flagはtrueとする。)

    # self._font_uni = gui_app.font("JP2", "テスト神奈川県茅ヶ崎市道路情報(12345)") #動的ロード
    # next_x = self._drawTextRight(self._font_uni, font_size , rect.x+rect.width-right_margin, y_pos+1 , "テスト神奈川県茅ヶ崎市道路情報(12345)", 220 , bk_alp=128, bk_corner_r= 0.2, bk_yofs=7,bk_add_h=-10,bk_add_w=0)
    # self._drawTextRight(self._font_uni, font_size , rect.x+rect.width-right_margin-1, y_pos+1 , "テスト神奈川県茅ヶ崎市道路情報(12345)", 220)
    # self._drawTextRight(self._font_semi_bold, font_size_km, next_x-4, y_pos2 - 4 , "120" , 255 , False , 0x24, 0x57, 0xa1 , 255,255,255,200 , 0 , 0.2 , 2 , -1)

    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    self.knightScanner(rect)
    rl.end_blend_mode() #元のブレンドに戻す

    sm = ui_state.sm

    # Get monitoring state
    dm_state = sm["driverMonitoringState"]
    self._is_active = dm_state.activePolicy == log.DriverMonitoringState.MonitoringPolicy.vision
    self._face_detected = dm_state.visionPolicyState.faceDetected

    # with open('/tmp/debug_out_w','w') as fp:
    #   fp.write("_is_active:%d , _face_detected:%d" % (int(self._is_active),int(self._face_detected)))

    if not self._is_active or not self._face_detected:
      self._disp_button_ct = 20 * 5 * 50 / self.dt #20fpsよりリfpsが速いc4対策。

    if self._disp_button_ct > 0:
      self._disp_button_ct -= 1

      dX_rect = rl.Rectangle(
        # 70, 20, 90, 90,
        int(rect.x+70), int(rect.y+rect.height-90-20), 90, 90, #左下の良い感じの位置
      )
      self._dexp_sw_mode_button.render(dX_rect)

      lane_rect = rl.Rectangle(
        int(rect.x+70), int(rect.y+20), 90, 90, #左上のいい感じの位置
      )
      self._lta_enable_sw_button.render(lane_rect)

      accel_ctrl_disable_rect = rl.Rectangle(
        int(rect.x+rect.width-90-20), int(rect.y+20), 90, 90, #右上のいい感じの位置
      )
      self._accel_ctrl_disable_button.render(accel_ctrl_disable_rect)

      #右下はSETSPEEDがあるので、使わない。

      decel_ctrl_disable_rect = rl.Rectangle(
        #int(rect.x+rect.width-90-90-20), int(rect.y+rect.height-90-20), 90, 90, #右下中央寄りのいい感じの位置
        int(rect.x+90+70), int(rect.y+rect.height-90-20), 90, 90, #左下中央寄りのいい感じの位置
      )
      self._decel_ctrl_disable_button.render(decel_ctrl_disable_rect)

      start_accel_power_up_disp_enable_rect = rl.Rectangle(
        int(rect.x+rect.width-90-90-20), int(rect.y+20), 90, 90, #右上中央寄りのいい感じの位置
      )
      self._start_accel_power_up_disp_enable_button.render(start_accel_power_up_disp_enable_rect)

      accel_engaged_rect = rl.Rectangle(
        int(rect.x+rect.width-90-90-20), int(rect.y+rect.height-90-20), 90, 90, #右下中央寄りのいい感じの位置
      )
      self._accel_engaged_button.render(accel_engaged_rect)

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
    if curve_value == 0 or self._engaged == False:
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

    dir *= self.dt / 50 #20fpsよりリfpsが速いc4対策。
    if dir > 1.0: #飛び飛びになるのを防ぐ。
      dir = 1
    elif dir < -1.0:
      dir = -1

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
    k_d = self.dt / 50 #20fpsよりfpsが速いc4対策。
    if k_d > 1.0:
      k_t = 0.9
    else:
      k_t = 0.9 ** k_d
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

      t[i] *= k_t
    pass

  def _button_push_sound(self,onoff):
    self._press_set_speed_MAX_ct = 3 #推したら数フレームはinteractingを継続
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
      self._accel_engaged_button.set_button_style(ButtonStyle.HudBOff)
    elif accel_engaged == 1:
      self._accel_engaged_button.set_text("A")
    elif accel_engaged == 2:
      self._accel_engaged_button.set_text("aa") #AA->aa改行対策
    elif accel_engaged == 3:
      self._accel_engaged_button.set_text("iP")
    elif accel_engaged == 4:
      self._accel_engaged_button.set_text("eP")

    self.accel_engaged = accel_engaged
    if accel_engaged != 0:
      self._accel_engaged_button.set_button_style(ButtonStyle.HudBOn)

    if self.button_style_only:
      return

    self._button_push_sound(accel_engaged)
    if self._disp_button_ct < 20 * 3 * 50 / self.dt:
      self._disp_button_ct = 20 * 3 * 50 / self.dt #ボタン操作したら3秒延長

    with open('/dev/shm/accel_engaged.txt','w') as fp2:
      fp2.write("%d" % (accel_engaged))
    with open('/data/accel_engaged.txt','w') as fp3:
      fp3.write("%d" % (accel_engaged))

  def _press_set_speed_MAX(self):
    sm = ui_state.sm
    cs = sm["selfdriveState"]

    accel_engaged = self.accel_engaged

    if accel_engaged >= 3 and cs.enabled: #ワンペダルのみ
      if int(self.set_speed) != 1: #MAXが1ではない時
        if sm["carState"].vEgo < 0.1/3.6: #スピードが出ていない時
          with open('/dev/shm/force_one_pedal.txt','w') as fp:
            fp.write('%d' % (1)) #これがセットされる条件をなるべく絞る。
          self._button_push_sound(1)
        else:
          #⚫︎ボタンの代わりに動作する
          self._press_limitspeed_sw() #MAX_touch()
      else:
        #MAX=1でタッチ(↑ボタン効果で",1"も含む)
        vego = sm["carState"].vEgo
        if vego > 3/3.6 and vego <= 30/3.6: #スピードが3〜30km/hのとき
          with open('/dev/shm/force_low_engage.txt','w') as fp:
            fp.write('%d' % (1))
          self._button_push_sound(1)
        else:
          #⚫︎ボタンの代わりに動作する
          self._press_limitspeed_sw() #MAX_touch()
    else:
      #⚫︎ボタンの代わりに動作する
      self._press_limitspeed_sw() #MAX_touch

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

    self.Limit_speed_mode = limitspeed_sw

    if self.button_style_only:
      return

    self._button_push_sound(limitspeed_sw)

    with open('/dev/shm/limitspeed_sw.txt','w') as fp2:
      fp2.write("%d" % (limitspeed_sw))
    with open('/data/limitspeed_sw.txt','w') as fp3:
      fp3.write("%d" % (limitspeed_sw))

  def _press_mads(self):
    try:
      with open('/dev/shm/steer_always.txt','r') as fp:
        steer_always = fp.read()
        if steer_always:
          self.steer_always = int(steer_always)
    except Exception as e:
      pass

    if self.button_style_only:
      return

    self.steer_always = not self.steer_always
    self._button_push_sound(self.steer_always)

    with open('/dev/shm/steer_always.txt','w') as fp2:
      fp2.write('%d' % (1 if self.steer_always else 0))
    with open('/data/steer_always.txt','w') as fp3:
      fp3.write('%d' % (1 if self.steer_always else 0))
    return

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
      self._dexp_sw_mode_button.set_button_style(ButtonStyle.HudBOff)
    else:
      self._dexp_sw_mode_button.set_button_style(ButtonStyle.HudBOn)

    sm = ui_state.sm
    self._dexp_sw_mode_button.set_icon(self.dx_icon_exp if sm['selfdriveState'].experimentalMode else self.dx_icon_chill)

    self.dexp_sw_mode = dexp_sw_mode

    if self.button_style_only:
      return

    self._button_push_sound(dexp_sw_mode)
    if self._disp_button_ct < 20 * 3 * 50 / self.dt:
      self._disp_button_ct = 20 * 3 * 50 / self.dt #ボタン操作したら3秒延長

    with open('/dev/shm/dexp_sw_mode.txt','w') as fp2:
      fp2.write("%d" % (dexp_sw_mode))
    with open('/data/dexp_sw_mode.txt','w') as fp3:
      fp3.write("%d" % (dexp_sw_mode))

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
      self._lta_enable_sw_button.set_button_style(ButtonStyle.HudBOff)
    else:
      self._lta_enable_sw_button.set_button_style(ButtonStyle.HudBOn)

    if self.button_style_only:
      return

    self._button_push_sound(lta_enable_sw)
    if self._disp_button_ct < 20 * 3 * 50 / self.dt:
      self._disp_button_ct = 20 * 3 * 50 / self.dt #ボタン操作したら3秒延長

    with open('/dev/shm/lta_enable_sw.txt','w') as fp2:
      fp2.write("%d" % (lta_enable_sw))
    with open('/data/lta_enable_sw.txt','w') as fp3:
      fp3.write("%d" % (lta_enable_sw))


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
      self._accel_ctrl_disable_button.set_button_style(ButtonStyle.HudBOn)
    else:
      self._accel_ctrl_disable_button.set_button_style(ButtonStyle.HudBOff)

    if self.button_style_only:
      return

    self._button_push_sound(1-accel_ctrl_disable)
    if self._disp_button_ct < 20 * 3 * 50 / self.dt:
      self._disp_button_ct = 20 * 3 * 50 / self.dt #ボタン操作したら3秒延長

    with open('/dev/shm/accel_ctrl_disable.txt','w') as fp2:
      fp2.write("%d" % (accel_ctrl_disable))
    with open('/data/accel_ctrl_disable.txt','w') as fp3:
      fp3.write("%d" % (accel_ctrl_disable))

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
      self._start_accel_power_up_disp_enable_button.set_button_style(ButtonStyle.HudBOff)
    else:
      self._start_accel_power_up_disp_enable_button.set_button_style(ButtonStyle.HudBOn)

    if self.button_style_only:
      return

    self._button_push_sound(start_accel_power_up_disp_enable)
    if self._disp_button_ct < 20 * 3 * 50 / self.dt:
      self._disp_button_ct = 20 * 3 * 50 / self.dt #ボタン操作したら3秒延長

    with open('/dev/shm/start_accel_power_up_disp_enable.txt','w') as fp2:
      fp2.write("%d" % (start_accel_power_up_disp_enable))
    with open('/data/start_accel_power_up_disp_enable.txt','w') as fp3:
      fp3.write("%d" % (start_accel_power_up_disp_enable))

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
      self._decel_ctrl_disable_button.set_button_style(ButtonStyle.HudBOn)
    else:
      self._decel_ctrl_disable_button.set_button_style(ButtonStyle.HudBOff)

    if self.button_style_only:
      return

    self._button_push_sound(1-decel_ctrl_disable)
    if self._disp_button_ct < 20 * 3 * 50 / self.dt:
      self._disp_button_ct = 20 * 3 * 50 / self.dt #ボタン操作したら3秒延長

    with open('/dev/shm/decel_ctrl_disable.txt','w') as fp2:
      fp2.write("%d" % (decel_ctrl_disable))
    with open('/data/decel_ctrl_disable.txt','w') as fp3:
      fp3.write("%d" % (decel_ctrl_disable))

