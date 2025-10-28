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
    self._press_time_ms = 0

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
    release_time_ms = time.monotonic_ns() / 1_000_000
    if release_time_ms - self._press_time_ms > 1000:
      #1秒以上長押し後に離すとここ。
      self.steer_always = not self.steer_always
      with open('/dev/shm/steer_always.txt','w') as fp:
        fp.write('%d' % (1 if self.steer_always else 0))
      return

    if self._is_toggle_allowed():
      new_mode = not self._experimental_mode
      self._params.put_bool("ExperimentalMode", new_mode)

      with open('/dev/shm/dexp_sw_mode.txt','w') as fp2:
        fp2.write("%d" % (0)) # experimentalModeを操作したらdX解除する

      # Hold new state temporarily
      self._held_mode = new_mode
      self._hold_end_time = time.monotonic() + self._hold_duration

  def _handle_mouse_press(self, _):
    super()._handle_mouse_press(_)
    self._press_time_ms = time.monotonic_ns() / 1_000_000

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
#    rl.draw_texture(texture, center_x - texture.width // 2, center_y - texture.height // 2, self._white_color)
    #画像を中心で回転させて描画する
    src_rect = rl.Rectangle(0, 0, texture.width, texture.height)  # テクスチャ全体を使用
    dest_rect = rl.Rectangle(center_x, center_y, texture.width, texture.height)  # 描画先の中心座標
    origin = rl.Vector2(texture.width / 2, texture.height / 2)  # 回転の中心（画像の中央）

    # Angle度回転（時計回り）
    import openpilot.selfdrive.ui.onroad.hud_renderer as hud #遅延インポート、重くないらしい。
    rl.draw_texture_pro(texture, src_rect, dest_rect, origin, -hud.global_angle_steer00, self._white_color)

    #＝＞　hud.long_speeddown_disable00 #long_speeddown_disable.txtを開かなくても、long_speeddown_disable.txtを使う。

  # const float x_Long_enable = surface_rect.right() - btn_size / 2 - UI_BORDER_SIZE * 2;
  # const float y_Long_enable = btn_size / 2 + int(UI_BORDER_SIZE * 1.5)+y_ofs;
  # std::string long_speeddown_disable_txt = util::read_file("/dev/shm/long_speeddown_disable.txt");
  # Long_enable = true;　＝＞　hud.long_speeddown_disable00
  # if(long_speeddown_disable_txt.empty() == false){
  #   if(std::stoi(long_speeddown_disable_txt) != 0){
  #     Long_enable = false;
  #   }
  # }
  # int long_base_angle0 = 45; //下中央から左右に何度か指定する。
  # if(/*(Long_enable || (*s->sm)["selfdriveState"].getSelfdriveState().getExperimentalMode()) && とにかく表示する*/ global_engageable){
  #   const int arc_w = -8; //内側に描画
  #   QPen pen = QPen(QColor(255, 255, ((*s->sm)["selfdriveState"].getSelfdriveState().getExperimentalMode()) ? 0 : 255, 180), abs(arc_w));
  #   pen.setCapStyle(Qt::FlatCap); //端をフラットに
  #   p.setPen(pen);
  #   const float x = x_Long_enable;
  #   const float y = y_Long_enable;
  #   static float desired_path_x_rate = 0 , desired_path_x_rate0 = 0;
  #   //static unsigned int desired_path_x_rate_ct = 0;
  #   if(true /*|| desired_path_x_rate_ct ++ % 2 == 0*/){
  #     std::string desired_path_x_rate_txt = util::read_file("/dev/shm/desired_path_x_rate.txt");
  #     if(desired_path_x_rate_txt.empty() == false){
  #       desired_path_x_rate0 = std::stof(desired_path_x_rate_txt);
  #       if(desired_path_x_rate0 < 0.0f){
  #         desired_path_x_rate0 = 0.0f; //なんか逆に動く場合がある？
  #       }
  #       if(desired_path_x_rate0 > 1.0f){
  #         desired_path_x_rate0 = 1.0f;
  #       }
  #     }
  #   }
  #   float max_diff = fabs(desired_path_x_rate0-desired_path_x_rate) / 10; // = 0.1; //これ以上一気にメーターが疎かない。可変式
  #   if(max_diff < 0.02)max_diff = 0.02; else if(max_diff > 0.05)max_diff = 0.05;
  #   if(desired_path_x_rate0 > desired_path_x_rate){
  #     if(desired_path_x_rate0 > desired_path_x_rate + max_diff){
  #       desired_path_x_rate += max_diff;
  #     } else {
  #       desired_path_x_rate = desired_path_x_rate0;
  #     }
  #   } else if(desired_path_x_rate0 < desired_path_x_rate){
  #     if(desired_path_x_rate0 < desired_path_x_rate - max_diff){
  #       desired_path_x_rate -= max_diff;
  #     } else {
  #       desired_path_x_rate = desired_path_x_rate0;
  #     }
  #   }
  #   int long_base_angle = long_base_angle0; //下中央から左右に何度か指定する。
  #   p.drawArc(x - btn_size / 2 -arc_w/2, y - btn_size / 2 -arc_w/2, btn_size+arc_w, btn_size+arc_w, (-90-long_base_angle)*16, -(360-long_base_angle*2)*16*desired_path_x_rate);
  # }
  # if(Long_enable){ //エンゲージしてなくても表示する。完全になくなると操作の目標を失うため。(OFFで消えたら仕方がないが) , Experimental Modeでは表示しない。
  #   const float x = x_Long_enable;
  #   const float y = y_Long_enable;
  #   const int long_base_angle = long_base_angle0-3; //下中央から左右に何度か指定する。
  #   //ONOFFの状態をこれで視認できる。
  #   const int arc_w_base = -14; //内側に描画
  #   QPen pen = QPen(QColor(255, 255, ((*s->sm)["selfdriveState"].getSelfdriveState().getExperimentalMode()) ? 0 : 255, 180), abs(arc_w_base));
  #   pen.setCapStyle(Qt::FlatCap); //端をフラットに
  #   p.setPen(pen);
  #   p.drawArc(x - btn_size / 2 -arc_w_base/2, y - btn_size / 2 -arc_w_base/2, btn_size+arc_w_base, btn_size+arc_w_base, (-90-(long_base_angle))*16, ((long_base_angle)*2)*16);
  # }


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

    # Mirror exp mode toggle using persistent car params
    return ui_state.has_longitudinal_control
