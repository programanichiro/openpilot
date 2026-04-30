import math
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params


def draw_circle_gradient(center_x: float, center_y: float, radius: int,
                         top: rl.Color, bottom: rl.Color) -> None:
  # Draw a square with the gradient
  rl.draw_rectangle_gradient_v(int(center_x - radius), int(center_y - radius),
                               radius * 2, radius * 2,
                               top, bottom)

  # Paint over square with a ring
  outer_radius = math.ceil(radius * math.sqrt(2)) + 1
  rl.draw_ring(rl.Vector2(int(center_x), int(center_y)), radius, outer_radius,
               0.0, 360.0,
               20, rl.BLACK)


class ConfidenceBall(Widget):
  def __init__(self, demo: bool = False):
    super().__init__()
    self._demo = demo
    self._confidence_filter = FirstOrderFilter(-0.5, 0.5, 1 / gui_app.target_fps)
    self._LongitudinalPersonality = 0
    self._LongitudinalPersonality_ct = 0
    self._lp1 = gui_app.texture("icons_mici/onroad/acc_dist1_w2.png",width=SIDE_PANEL_WIDTH-5,height=int(256*(SIDE_PANEL_WIDTH-5)/191)) #幅をSIDE_PANEL_WIDTH程度に
    self._lp2 = gui_app.texture("icons_mici/onroad/acc_dist2_w2.png",width=SIDE_PANEL_WIDTH-5,height=int(256*(SIDE_PANEL_WIDTH-5)/191))
    self._lp3 = gui_app.texture("icons_mici/onroad/acc_dist3_w2.png",width=SIDE_PANEL_WIDTH-5,height=int(256*(SIDE_PANEL_WIDTH-5)/191))
    self.brake_light_alpha = 0
    self.vc_accel = 0

    self.ui_freeze_flag = False
    self.button_style_only = True
    font_sz = 10 #acc_distにかぶせる透明ボタン
    font_wt = FontWeight.BOLD
    self._LongitudinalPersonality_button = Button("",click_callback=self._press_LongitudinalPersonality,font_size=font_sz,font_weight=font_wt, border_radius=10)
    self._LongitudinalPersonality_button.set_button_style(ButtonStyle.HudUnder) #バック透明
    self._press_LongitudinalPersonality()
    self.button_style_only = False


  def update_filter(self, value: float):
    self._confidence_filter.update(value)

  def _update_state(self):
    if self._demo:
      return

    # animate status dot in from bottom
    if ui_state.status == UIStatus.DISENGAGED:
      self._confidence_filter.update(-0.5)
    else:
      self._confidence_filter.update((1 - max(ui_state.sm['modelV2'].meta.disengagePredictions.brakeDisengageProbs or [1])) *
                                                        (1 - max(ui_state.sm['modelV2'].meta.disengagePredictions.steerOverrideProbs or [1])))

  def _render(self, _):
    content_rect = rl.Rectangle(
      self.rect.x + self.rect.width - SIDE_PANEL_WIDTH,
      self.rect.y,
      SIDE_PANEL_WIDTH,
      self.rect.height,
    )

    status_dot_radius = 24
    dot_height = (1 - self._confidence_filter.x) * (content_rect.height - 2 * status_dot_radius) + status_dot_radius
    dot_height = self._rect.y + dot_height

    # confidence zones
    if ui_state.status == UIStatus.ENGAGED or self._demo:
      if self._confidence_filter.x > 0.5:
        top_dot_color = rl.Color(0, 255, 204, 255)
        bottom_dot_color = rl.Color(0, 255, 38, 255)
      elif self._confidence_filter.x > 0.2:
        top_dot_color = rl.Color(255, 200, 0, 255)
        bottom_dot_color = rl.Color(255, 115, 0, 255)
      else:
        top_dot_color = rl.Color(255, 0, 21, 255)
        bottom_dot_color = rl.Color(255, 0, 89, 255)

    elif ui_state.status == UIStatus.OVERRIDE:
      top_dot_color = rl.Color(255, 255, 255, 255)
      bottom_dot_color = rl.Color(82, 82, 82, 255)

    else:
      top_dot_color = rl.Color(50, 50, 50, 255)
      bottom_dot_color = rl.Color(13, 13, 13, 255)

    draw_circle_gradient(content_rect.x + content_rect.width - status_dot_radius,
                         dot_height, status_dot_radius,
                         top_dot_color, bottom_dot_color)

    #ここにACC距離アイコン、描けそう
    if self._LongitudinalPersonality_ct % 5 == 0:
      self._LongitudinalPersonality = int(Params().get("LongitudinalPersonality"))

    y_ofs = 10
    if self._LongitudinalPersonality == 0:
      rl.draw_texture(self._lp1,int(content_rect.x+(SIDE_PANEL_WIDTH-self._lp1.width)/2),int(content_rect.y + content_rect.height -self._lp1.height-y_ofs), rl.Color(240,240,240,230))
    elif self._LongitudinalPersonality == 1:
      rl.draw_texture(self._lp2,int(content_rect.x+(SIDE_PANEL_WIDTH-self._lp2.width)/2),int(content_rect.y + content_rect.height -self._lp2.height-y_ofs), rl.Color(240,240,240,230))
    else: #if self._LongitudinalPersonality == 2:
      rl.draw_texture(self._lp3,int(content_rect.x+(SIDE_PANEL_WIDTH-self._lp3.width)/2),int(content_rect.y + content_rect.height -self._lp3.height-y_ofs), rl.Color(240,240,240,230))

    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    brake_flag = False
    try:
      with open('/dev/shm/brake_light_state.txt','r') as fp3:
        brake_light_state = fp3.read()
        if brake_light_state and int(brake_light_state) != 0:
          #エンゲージしていなくてもセットされる。
          brake_flag = True
    except Exception as e:
      pass

    alp_add = 30 if gui_app.big_ui() else 10 #c4は60Hz
    if brake_flag:
      self.brake_light_alpha += alp_add
      if self.brake_light_alpha > 170:
        self.brake_light_alpha = 170
    else:
      self.brake_light_alpha -= alp_add
      if self.brake_light_alpha < 0:
        self.brake_light_alpha = 0
    rl.draw_rectangle_rounded(content_rect,0.5,10,rl.Color(255, 0, 0, self.brake_light_alpha)) #角丸の赤いオーバーレイ

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
    hha = hha * content_rect.height
    wp = 70 * 1/4
    if self.vc_accel > 0:
      meter = [(content_rect.x+content_rect.width - wp + wp/2 , content_rect.y+content_rect.height/2),
               (content_rect.x+content_rect.width , content_rect.y+content_rect.height/2),
               (content_rect.x+content_rect.width , content_rect.y+content_rect.height/2 - hha/2),
               (content_rect.x+content_rect.width - wp/2 - wp/2 * hha / content_rect.height , content_rect.y+content_rect.height/2 - hha/2)]
      rl.draw_triangle_fan(meter,len(meter),va_color)
    elif self.vc_accel < 0:
      meter = [(content_rect.x+content_rect.width - wp/2 - wp/2 * hha / content_rect.height , content_rect.y+content_rect.height/2 + hha/2),
               (content_rect.x+content_rect.width , content_rect.y+content_rect.height/2 + hha/2),
               (content_rect.x+content_rect.width , content_rect.y+content_rect.height/2),
               (content_rect.x+content_rect.width - wp + wp/2 , content_rect.y+content_rect.height/2)]
      rl.draw_triangle_fan(meter,len(meter),va_color)

    rl.end_blend_mode() #元のブレンドに戻す

    self._LongitudinalPersonality_ct += 1

    btn_h = 300 * gui_app._scale
    self._LongitudinalPersonality_button.render(rl.Rectangle(content_rect.x, content_rect.y + content_rect.height - btn_h, content_rect.width, btn_h))

  def ui_freeze(self, freeze):
    self.ui_freeze_flag = freeze

  def _button_push_sound(self,onoff):
    with open('/dev/shm/sound_py_request.txt','w') as fp2:
      if onoff:
        fp2.write('%d' % (102)) #pipo.wav
      else:
        fp2.write('%d' % (101)) #po.wav

  def _press_LongitudinalPersonality(self):
    if self.ui_freeze_flag:
      return

    psn_str = Params().get("LongitudinalPersonality")
    psn = int(psn_str)

    if self.button_style_only == False:
      psn = (psn -1 + 3) % 3

    if self.button_style_only:
      return

    self._button_push_sound(1)

    Params().put("LongitudinalPersonality", psn)
