import numpy as np
import raylib_nb as rl
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH, OFFICIAL_LAYOUT
from openpilot.selfdrive.ui.mici.onroad.long_indicator import LongIndicator
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.mici.onroad.status import status_fade
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.polygon import Gradient, draw_polygon
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params


CIRCLE_PTS = np.stack([np.cos(t := np.linspace(0, 2 * np.pi, 64, endpoint=False)), np.sin(t)], axis=1)
# drop shadow from the design: offset (0, 2), blur 8, black at 50%. Stacked discs at gaussian quantiles approximate the blur
SHADOW_OFFSET, SHADOW_SIGMA, SHADOW_ALPHA = 2, 4, 0.5
SHADOW_QUANTILES = (-1.53, -0.89, -0.49, -0.16, 0.16, 0.49, 0.89, 1.53)
SHADOW_LAYER_COLOR = rl.Color(0, 0, 0, round(255 * (1 - (1 - SHADOW_ALPHA) ** (1 / len(SHADOW_QUANTILES)))))


def draw_circle_gradient(center_x: float, center_y: float, radius: float,
                         top: rl.Color, bottom: rl.Color, alpha: float = 1.0) -> None:
  screen = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
  shadow = rl.Color(0, 0, 0, round(SHADOW_LAYER_COLOR.a * alpha))
  for z in SHADOW_QUANTILES:
    draw_polygon(screen, CIRCLE_PTS * (radius + z * SHADOW_SIGMA) + (center_x, center_y + SHADOW_OFFSET), color=shadow)
  top, bottom = rl.Color(top.r, top.g, top.b, round(top.a * alpha)), rl.Color(bottom.r, bottom.g, bottom.b, round(bottom.a * alpha))
  # gradient y is measured from the bottom of the screen
  gradient = Gradient(start=(0, 1 - (center_y - radius) / gui_app.height), end=(0, 1 - (center_y + radius) / gui_app.height),
                      colors=[bottom, top], stops=[0.0, 1.0])
  draw_polygon(screen, CIRCLE_PTS * radius + (center_x, center_y), gradient=gradient)


# LongIndicator は渡した矩形の横幅の中央に、y=100(リード車) と y=122/136/152(車間バー)へ描く。
# (以前は矩形の x+46 が中心だったが、upstream が矩形の中央に描く形に変えた)
# 緑アイコンのテクスチャは最大 76px あるが、大半がグロー(光彩)で実体のバーは白と同じ幅。
# 見えている部分はサイドパネル(60px)に収まるので、縮小は不要。
LONG_INDICATOR_BOTTOM = 172  # 152 + 39/2 (3本目の緑バーの下端)
LONG_INDICATOR_Y_OFS = -5     # 0でサイドパネルの下端ぴったりに置く->もう少し下げる。
# 公式の配置で、左パネルの LongIndicator(y=86〜158 あたり)にかぶせる車間ボタンの範囲。上の DMoji と下のハンドル・MADS ボタン(y=168〜)を避ける
OFFICIAL_GAP_BUTTON_Y = 76
OFFICIAL_GAP_BUTTON_H = 90
# 公式の配置で、上の y は C4 の画面の高さ(240)のときの値。LongIndicator と車間ボタンはパネルの下端から測って置き、
# C3X で C4 の画面を使うとき(パネルが高い)も下からの余白を C4 と同じにする
OFFICIAL_PANEL_REF_H = 240


class ConfidenceBall(Widget):
  def __init__(self):
    super().__init__()
    self._confidence_filter = FirstOrderFilter(0.0 if OFFICIAL_LAYOUT else -0.5, 0.5, 1 / gui_app.target_fps)
    self._alpha_filter = FirstOrderFilter(1.0, 0.05, 1 / gui_app.target_fps)
    self._hidden = False
    # 公式の LongIndicator をここで描く。タコメーターと場所が被るため AugmentedRoadView から移した。
    # 車間距離の表示を含むので、自前の acc_dist アイコン(_lp1〜3)は廃止した。
    self._long_indicator = LongIndicator()
    # 旧 acc_dist アイコンは常時表示だったので、それに揃える。エンゲージ前や警告中でも消えない。
    self._long_indicator.set_always_visible(True)
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

  def set_hidden(self, hidden: bool):
    self._hidden = hidden

  def _update_state(self):
    # animate status dot in from bottom
    # フォークの配置は右パネルの下端に LongIndicator と車間ボタンを置いているので、エンゲージしていない時は公式の「下端で待機」にせず、従来どおり下に隠す
    if ui_state.status == UIStatus.DISENGAGED:
      self._confidence_filter.update(0.0 if OFFICIAL_LAYOUT else -0.5)
    else:
      self._confidence_filter.update((1 - max(ui_state.sm['modelV2'].meta.disengagePredictions.brakeDisengageProbs or [1])) *
                                                        (1 - max(ui_state.sm['modelV2'].meta.disengagePredictions.steerOverrideProbs or [1])))

  def _render(self, _):
    if OFFICIAL_LAYOUT:
      # 公式の配置: 映像の右端に描く。渡される矩形は映像の範囲
      alpha = self._alpha_filter.update(0.0 if self._hidden else 1.0)
      if alpha < 1e-2:
        return
      status_dot_radius = 16
      margin = 10
      dot_x = self._rect.x + self._rect.width - margin - status_dot_radius
      dot_height = (1 - self._confidence_filter.x) * (self._rect.height - 2 * (status_dot_radius + margin)) + status_dot_radius + margin
      dot_height = self._rect.y + dot_height
      top_dot_color, bottom_dot_color = self._dot_colors()
      draw_circle_gradient(dot_x, dot_height, status_dot_radius, top_dot_color, bottom_dot_color, alpha)
      return

    content_rect = rl.Rectangle(
      self.rect.x + self.rect.width - SIDE_PANEL_WIDTH,
      self.rect.y,
      SIDE_PANEL_WIDTH,
      self.rect.height,
    )

    status_dot_radius = 24
    dot_height = (1 - self._confidence_filter.x) * (content_rect.height - 2 * status_dot_radius) + status_dot_radius
    dot_height = self._rect.y + dot_height

    # ブレーキの赤は一番下に敷く（信頼度ボールと LongIndicator はその上に描く）
    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    self._draw_brake_light(content_rect)
    rl.end_blend_mode() #元のブレンドに戻す

    top_dot_color, bottom_dot_color = self._dot_colors()
    draw_circle_gradient(content_rect.x + content_rect.width - status_dot_radius-5,
                         dot_height, status_dot_radius,
                         top_dot_color, bottom_dot_color)

    # LongIndicator をサイドパネルの下端に寄せて描く。
    self._long_indicator.render(rl.Rectangle(
      content_rect.x,  # サイドパネルの中央に描かれる
      content_rect.y + content_rect.height - LONG_INDICATOR_BOTTOM - LONG_INDICATOR_Y_OFS,
      content_rect.width,
      content_rect.height,
    ))

    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    self._draw_accel_meter(content_rect)
    rl.end_blend_mode() #元のブレンドに戻す

    btn_h = content_rect.width * 1.5 #だいたいこのくらいの高さ
    self._LongitudinalPersonality_button.render(rl.Rectangle(content_rect.x, content_rect.y + content_rect.height - btn_h, content_rect.width, btn_h))

  def render_official_long_indicator(self, panel_rect: rl.Rectangle, should_draw: bool) -> None:
    # 公式の配置: LongIndicator を左パネルに描く（C4 では upstream と同じ位置。下端からの余白で置く）
    self._long_indicator.set_should_draw(should_draw)
    self._long_indicator.render(rl.Rectangle(panel_rect.x, self._official_bottom_based_y(panel_rect), panel_rect.width, panel_rect.height))

  @staticmethod
  def _official_bottom_based_y(panel_rect: rl.Rectangle) -> float:
    # C4 の高さ(240)ならパネルの上端と同じ。パネルが高いほど下にずれる
    return panel_rect.y + panel_rect.height - OFFICIAL_PANEL_REF_H

  def render_official_brake_light(self, panel_rect: rl.Rectangle) -> None:
    # 公式の配置: パネルのアイコンを描く前に呼ぶ。ブレーキの赤は左パネル全体の一番下に敷く
    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    self._draw_brake_light(panel_rect)
    rl.end_blend_mode() #元のブレンドに戻す

  def render_official_overlays(self, panel_rect: rl.Rectangle, camera_rect: rl.Rectangle) -> None:
    # 公式の配置: パネルのアイコンを描いた後に呼ぶ。加減速メーターは映像の右端に描く
    rl.begin_blend_mode(rl.BLEND_ADDITIVE) #加算ブレンド
    self._draw_accel_meter(camera_rect)
    rl.end_blend_mode() #元のブレンドに戻す

    # 車間ボタンは左パネルの LongIndicator にかぶせる
    self._LongitudinalPersonality_button.render(rl.Rectangle(panel_rect.x, self._official_bottom_based_y(panel_rect) + OFFICIAL_GAP_BUTTON_Y,
                                                             panel_rect.width, OFFICIAL_GAP_BUTTON_H))

  def _dot_colors(self) -> tuple[rl.Color, rl.Color]:
    # confidence zones while engaged, top and bottom gradient colors
    if self._confidence_filter.x > 0.5:
      engaged = (rl.Color(0, 255, 187, 255), rl.Color(0, 255, 64, 255))
    elif self._confidence_filter.x > 0.2:
      engaged = (rl.Color(255, 200, 0, 255), rl.Color(255, 115, 0, 255))
    else:
      engaged = (rl.Color(255, 0, 21, 255), rl.Color(255, 0, 89, 255))
    override = (rl.Color(255, 255, 255, 255), rl.Color(82, 82, 82, 255))
    disengaged = (rl.Color(102, 102, 102, 255), rl.Color(26, 26, 26, 255))
    top_dot_color, bottom_dot_color = (status_fade.color({UIStatus.ENGAGED: e, UIStatus.OVERRIDE: o, UIStatus.DISENGAGED: d})
                                       for e, o, d in zip(engaged, override, disengaged, strict=True))

    return top_dot_color, bottom_dot_color

  def _draw_brake_light(self, content_rect: rl.Rectangle) -> None:
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

  def _draw_accel_meter(self, content_rect: rl.Rectangle) -> None:
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
