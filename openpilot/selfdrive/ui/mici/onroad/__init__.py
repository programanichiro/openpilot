import raylib_nb as rl


def _read_official_layout() -> bool:
  # 設定画面の「official onroad layout」が書く。UI の起動時に一度だけ読むので、切り替えたら端末の再起動が必要
  try:
    with open('/data/official_onroad_layout.txt', 'r') as fp:
      return int(fp.read()) == 1
  except Exception:
    return False


# True: upstream の配置（黒いサイドパネルは左。DMoji・LongIndicator・ハンドルをパネルに置く）
# False: フォークの配置（サイドパネルは右。ブレーキの赤表示・加減速メーター・LongIndicator・車間ボタンをパネルに置く）
OFFICIAL_LAYOUT = _read_official_layout()

# upstream は 72。フォークの配置は右パネルの中身を 60px で配置しているので 60 のまま
SIDE_PANEL_WIDTH = 72 if OFFICIAL_LAYOUT else 60
DIMMED_ICON_ALPHA = 0.32  # side panel icons while disengaged


def blend_colors(a: rl.Color, b: rl.Color, f: float) -> rl.Color:
  h0, s0, v0 = (hsv0 := rl.color_to_hsv(a)).x, hsv0.y, hsv0.z
  h1, s1, v1 = (hsv1 := rl.color_to_hsv(b)).x, hsv1.y, hsv1.z
  dh = ((h1 - h0 + 180) % 360) - 180  # shortest hue delta
  color = rl.color_from_hsv((h0 + f * dh) % 360,
                            s0 + f * (s1 - s0),
                            v0 + f * (v1 - v0))
  color.a = round(a.a + f * (b.a - a.a))
  return color
