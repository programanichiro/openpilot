import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
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
  white: rl.Color = rl.WHITE
  white_translucent: rl.Color = rl.Color(255, 255, 255, 200)


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

    self._txt_turn_intent_left: rl.Texture = gui_app.texture('icons_mici/turn_intent_left.png', 50, 19)
    self._txt_turn_intent_right: rl.Texture = gui_app.texture('icons_mici/turn_intent_right.png', 50, 19)

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

    self._turn_intent = TurnIntent()
    self._torque_bar = TorqueBar()

    self._txt_wheel: rl.Texture = gui_app.texture('icons_mici/wheel.png', 50, 50)
    self._txt_wheel_critical: rl.Texture = gui_app.texture('icons_mici/wheel_critical.png', 50, 50)
    self._txt_exclamation_point: rl.Texture = gui_app.texture('icons_mici/exclamation_point.png', 44, 44)

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
      controls_state.vCruiseDEPRECATED if v_cruise_cluster == 0.0 else v_cruise_cluster
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

    self.db_rec_mode = False
    if self.limit_speed_override == False:
      # self.yellow_flag = False
      if self.Limit_speed_mode == 2:
        # rect_color = rl.Color(100, 0, 0, 250)
        if self.set_speed >= 30:
          self.db_rec_mode = True

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

    if ui_state.sm['controlsState'].lateralControlState.which() != 'angleState':
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

    # color and draw
    color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, color)

    if self._show_wheel_critical:
      # Draw exclamation point icon
      EXCLAMATION_POINT_SPACING = 10
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 2
      rl.draw_texture(self._txt_exclamation_point, int(exclamation_pos_x), int(exclamation_pos_y), rl.WHITE)

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
    y += rect.height - circle_radius*2
    rl.draw_circle_gradient(int(x + circle_radius), int(y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    set_speed_text_size = measure_text_cached(self._font_display, set_speed_text, FONT_SIZES.set_speed)
    rl.draw_text_ex(
      self._font_display,
      set_speed_text,
      rl.Vector2(x -5 + circle_radius*2 - set_speed_text_size.x, y + 3 - 8 - 3 + 4),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

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

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.white)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.white_translucent)

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

  def _ip_update_state(self,sm):
    self.ip_update_state_ct += 1
    car_state = sm['carState']
    self.vc_speed = car_state.vEgo

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

  def _ip_draw(self, rect: rl.Rectangle):
    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    self.knightScanner(rect)
    rl.end_blend_mode() #元のブレンドに戻す


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

      t[i] *= 0.9
    pass
