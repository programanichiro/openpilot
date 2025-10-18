import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle

# Constants
SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

y_ofs = 150

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
  white: rl.Color = rl.WHITE
  disengaged: rl.Color = rl.Color(145, 155, 149, 255)
  override: rl.Color = rl.Color(145, 155, 149, 255)  # Added
  engaged: rl.Color = rl.Color(128, 216, 166, 255)
  disengaged_bg: rl.Color = rl.Color(0, 0, 0, 153)
  override_bg: rl.Color = rl.Color(145, 155, 149, 204)
  engaged_bg: rl.Color = rl.Color(128, 216, 166, 204)
  grey: rl.Color = rl.Color(166, 166, 166, 255)
  dark_grey: rl.Color = rl.Color(114, 114, 114, 255)
  black_translucent: rl.Color = rl.Color(0, 0, 0, 166)
  white_translucent: rl.Color = rl.Color(255, 255, 255, 200)
  border_translucent: rl.Color = rl.Color(255, 255, 255, 75)
  header_gradient_start: rl.Color = rl.Color(0, 0, 0, 114)
  header_gradient_end: rl.Color = rl.BLANK


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
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_medium: rl.Font = gui_app.font(FontWeight.MEDIUM)

    self._exp_button: ExpButton = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)
    self.ip_button_init()

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
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1

    if self.is_cruise_set and not ui_state.is_metric:
      self.set_speed *= KM_TO_MILE

    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

    self.ip_update_state()

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    # Draw the header background
    rl.draw_rectangle_gradient_v(
      int(rect.x),
      int(rect.y),
      int(rect.width),
      UI_CONFIG.header_height + y_ofs,
      COLORS.header_gradient_start,
      COLORS.header_gradient_end,
    )

    if self.is_cruise_available:
      self._draw_set_speed(rect)

    self._draw_current_speed(rect)

    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size + y_ofs
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))

    btn_w0 = 250
    btn_w = 200
    btn_h0 = 175
    btn_h = 150
    self._start_accel_power_up_disp_enable_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*2, rect.y + rect.height - btn_h0*3, btn_w, btn_h))
    self._accel_engaged_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*2, rect.y + rect.height - btn_h0*2, btn_w, btn_h))

    self._accel_ctrl_disable_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*1, rect.y + rect.height - btn_h0*3.5, btn_w, btn_h))
    self._decel_ctrl_disable_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*1, rect.y + rect.height - btn_h0*2.5, btn_w, btn_h))
    self._long_speeddown_disable_button.render(rl.Rectangle(rect.x + rect.width - btn_w0*1, rect.y + rect.height - btn_h0*1.5, btn_w, btn_h))

    self._lta_enable_sw_button.render(rl.Rectangle(rect.x +(btn_w0-btn_w)+ btn_w0*0, rect.y + rect.height - btn_h0*3.5, btn_w, btn_h))
    self._dexp_sw_mode_button.render(rl.Rectangle(rect.x +(btn_w0-btn_w)+ btn_w0*0, rect.y + rect.height - btn_h0*2.5, btn_w, btn_h))

  def user_interacting(self) -> bool:
    return (self._exp_button.is_pressed
      or self._accel_engaged_button.is_pressed
      or self._dexp_sw_mode_button.is_pressed
      or self._long_speeddown_disable_button.is_pressed
      or self._lta_enable_sw_button.is_pressed
      or self._start_accel_power_up_disp_enable_button.is_pressed
      or self._accel_ctrl_disable_button.is_pressed
      or self._decel_ctrl_disable_button.is_pressed
      )

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Draw the MAX speed indicator box."""
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45 + y_ofs

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.black_translucent)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.border_translucent)

    max_color = COLORS.grey
    set_speed_color = COLORS.dark_grey
    if self.is_cruise_set:
      set_speed_color = COLORS.white
      if ui_state.status == UIStatus.ENGAGED:
        max_color = COLORS.engaged
      elif ui_state.status == UIStatus.DISENGAGED:
        max_color = COLORS.disengaged
      elif ui_state.status == UIStatus.OVERRIDE:
        max_color = COLORS.override

    max_text = "MAX"
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
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    """Draw the current vehicle speed and unit."""
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2 + y_ofs)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.white)

    unit_text = "km/h" if ui_state.is_metric else "mph"
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2 + y_ofs)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.white_translucent)

  def ip_button_init(self):
    try:
      with open('/data/accel_engaged.txt', 'rb') as src, open('/dev/shm/accel_engaged.txt', 'wb') as dst:
        dst.write(src.read())
    except Exception as e:
      pass
    try:
      with open('/data/dexp_sw_mode.txt', 'rb') as src, open('/dev/shm/dexp_sw_mode.txt', 'wb') as dst:
        dst.write(src.read())
    except Exception as e:
      pass
    try:
      with open('/data/long_speeddown_disable.txt', 'rb') as src, open('/dev/shm/long_speeddown_disable.txt', 'wb') as dst:
        dst.write(src.read())
    except Exception as e:
      pass
    try:
      with open('/data/lta_enable_sw.txt', 'rb') as src, open('/dev/shm/lta_enable_sw.txt', 'wb') as dst:
        dst.write(src.read())
    except Exception as e:
      pass
    try:
      with open('/data/start_accel_power_up_disp_enable.txt', 'rb') as src, open('/dev/shm/start_accel_power_up_disp_enable.txt', 'wb') as dst:
        dst.write(src.read())
    except Exception as e:
      pass
    try:
      with open('/data/accel_ctrl_disable.txt', 'rb') as src, open('/dev/shm/accel_ctrl_disable.txt', 'wb') as dst:
        dst.write(src.read())
    except Exception as e:
      pass
    try:
      with open('/data/decel_ctrl_disable.txt', 'rb') as src, open('/dev/shm/decel_ctrl_disable.txt', 'wb') as dst:
        dst.write(src.read())
    except Exception as e:
      pass

    self.ip_update_state_ct = 0
    self.button_style_only = True
    font_sz = 75
    font_wt = FontWeight.BOLD #EXTRA_BOLD
    self._accel_engaged_button = Button("A",click_callback=self._press_accel_engaged,font_size=font_sz,font_weight=font_wt)
    self._press_accel_engaged()

    self._dexp_sw_mode_button = Button("dX",click_callback=self._press_dexp_sw_mode,font_size=font_sz,font_weight=font_wt)
    self._press_dexp_sw_mode()

    self._long_speeddown_disable_button = Button("iL",click_callback=self._press_long_speeddown_disable,font_size=font_sz,font_weight=font_wt) #イチロウロング独立ボタン
    self._press_long_speeddown_disable()

    self._lta_enable_sw_button = Button("/ \\",click_callback=self._press_lta_enable_sw,font_size=font_sz,font_weight=font_wt)
    self._press_lta_enable_sw()

    #日本語フォント対応待ち
    # self._start_accel_power_up_disp_enable_button = Button("⇧",click_callback=self._press_start_accel_power_up_disp_enable,font_size=font_sz,font_weight=font_wt)
    # self._accel_ctrl_disable_button = Button("↑",click_callback=self._press_accel_ctrl_disable,font_size=font_sz,font_weight=font_wt)
    # self._decel_ctrl_disable_button = Button("↓",click_callback=self._press_decel_ctrl_disable,font_size=font_sz,font_weight=font_wt)

    self._start_accel_power_up_disp_enable_button = Button("Bst",click_callback=self._press_start_accel_power_up_disp_enable,font_size=font_sz,font_weight=font_wt)
    self._press_start_accel_power_up_disp_enable()

    self._accel_ctrl_disable_button = Button("Up",click_callback=self._press_accel_ctrl_disable,font_size=font_sz,font_weight=font_wt)
    self._press_accel_ctrl_disable()

    self._decel_ctrl_disable_button = Button("Dn",click_callback=self._press_decel_ctrl_disable,font_size=font_sz,font_weight=font_wt)
    self._press_decel_ctrl_disable()
    self.button_style_only = False

  def ip_update_state(self):
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
              fp2.write('%d' % (1)) #po.wav
            with open('/dev/shm/signal_start_prompt_info.txt','w') as fp3:
              fp3.write('%d' % (0))
    except Exception as e:
      pass

    self.ip_update_state_ct += 1
    if(self.ip_update_state_ct % 10 == 7):
      self.button_style_only = True
      self._press_long_speeddown_disable()
      self.button_style_only = False


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

    if accel_engaged != 0:
      self._accel_engaged_button.set_button_style(ButtonStyle.HudSOn)

    if self.button_style_only:
      return

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

    if self.button_style_only:
      return

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

    if self.button_style_only == False:
      long_speeddown_disable = (long_speeddown_disable + 1) % 2
    if long_speeddown_disable == 0:
      self._long_speeddown_disable_button.set_button_style(ButtonStyle.HudSOn)
    else:
      self._long_speeddown_disable_button.set_button_style(ButtonStyle.HudSOff)

    if self.button_style_only:
      return

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
      self._lta_enable_sw_button.set_button_style(ButtonStyle.HudSOn)
    else:
      self._lta_enable_sw_button.set_button_style(ButtonStyle.HudSOff)

    if self.button_style_only:
      return

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
      self._start_accel_power_up_disp_enable_button.set_button_style(ButtonStyle.HudSOn)
    else:
      self._start_accel_power_up_disp_enable_button.set_button_style(ButtonStyle.HudSOff)

    if self.button_style_only:
      return

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

    with open('/dev/shm/decel_ctrl_disable.txt','w') as fp2:
      fp2.write("%d" % (decel_ctrl_disable))
    with open('/data/decel_ctrl_disable.txt','w') as fp3:
      fp3.write("%d" % (decel_ctrl_disable))
