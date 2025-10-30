import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget


class ExpButton(Widget):
  def __init__(self, button_size: int, icon_size: int):
    super().__init__()
    self._params = Params()
    self._experimental_mode: bool = False
    self._engageable: bool = False

    self.limitspeed_update_ct = 0
    self.steer_always = False
    self.cruise_available = False

    # State hold mechanism
    self._hold_duration = 2.0  # seconds
    self._held_mode: bool | None = None
    self._hold_end_time: float | None = None

    self._white_color: rl.Color = rl.Color(255, 255, 255, 255)
    self._black_bg: rl.Color = rl.Color(0, 0, 0, 166)
    self._mads_on_bg: rl.Color = rl.Color(0x17, 0x86, 0x44, 224)
    self._mads_off_bg: rl.Color = rl.Color(0x17, 0x86, 0x44, 144)
    self._txt_wheel: rl.Texture = gui_app.texture('icons/chffr_wheel.png', icon_size, icon_size)
    self._txt_exp: rl.Texture = gui_app.texture('icons/experimental.png', icon_size, icon_size)
    self._rect = rl.Rectangle(0, 0, button_size, button_size)

  def set_rect(self, rect: rl.Rectangle) -> None:
    self._rect.x, self._rect.y = rect.x, rect.y

  def _update_state(self) -> None:
    selfdrive_state = ui_state.sm["selfdriveState"]
    self._experimental_mode = selfdrive_state.experimentalMode
    self._engageable = selfdrive_state.engageable or selfdrive_state.enabled

    self.limitspeed_update_ct += 1
    if self.limitspeed_update_ct % 20 == 10:
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

  def _handle_mouse_release(self, _):
    super()._handle_mouse_release(_)
    if self._is_toggle_allowed():
      new_mode = not self._experimental_mode
      self._params.put_bool("ExperimentalMode", new_mode)

      # Hold new state temporarily
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    self._white_color.a = 180 if self.is_pressed or not self._engageable else 255

    texture = self._txt_exp if self._held_or_actual_mode() else self._txt_wheel
    bg_color = self._black_bg
    if self.steer_always:
      if self.cruise_available:
        bg_color = self._mads_on_bg
      else:
        bg_color = self._mads_off_bg
    rl.draw_circle(center_x, center_y, self._rect.width / 2, bg_color)
    rl.draw_texture(texture, center_x - texture.width // 2, center_y - texture.height // 2, self._white_color)

  def _held_or_actual_mode(self):
    now = time.monotonic()
    if self._hold_end_time and now < self._hold_end_time:
      return self._held_mode

    if self._hold_end_time and now >= self._hold_end_time:
      self._hold_end_time = self._held_mode = None

    return self._experimental_mode

  def _is_toggle_allowed(self):
    if not self._params.get_bool("ExperimentalModeConfirmed"):
      return False

    car_params = ui_state.sm["carParams"]
    if car_params.alphaLongitudinalAvailable:
      return self._params.get_bool("AlphaLongitudinalEnabled")
    else:
      return car_params.openpilotLongitudinalControl
