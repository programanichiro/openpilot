#!/usr/bin/env python3
"""Capture APN auth settings with isolated params using the desktop raylib backend.

SCALE=1 python3 openpilot/selfdrive/ui/tests/diff/shot_apn_auth.py /tmp/apn_auth_shots/mici
BIG=1 SCALE=0.75 python3 openpilot/selfdrive/ui/tests/diff/shot_apn_auth.py /tmp/apn_auth_shots/tizi
Add --lang ja to render translated strings in Japanese.
"""
import argparse
import os
from collections.abc import Callable
from pathlib import Path

BIG = os.getenv("BIG") == "1"
os.environ.setdefault("SCALE", "0.75" if BIG else "1")
os.environ["PRIME_TYPE"] = "0"
os.environ["OFFSCREEN"] = "1"

import pyray as rl
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.ui.tests.diff.replay import setup_state

FPS = 60


def main(output: Path, lang: str | None = None):
  output.mkdir(parents=True, exist_ok=True)
  setup_state()
  if lang is not None:
    from openpilot.system.ui.lib.multilang import multilang
    # Select the language before init_window() loads its fallback font.
    multilang.change_language(lang)
  params = Params()

  def seed(populated: bool):
    for key, value in (("GsmApn", "example.apn"), ("GsmAuth", "chap"), ("GsmUsername", "mio@iij"), ("GsmPassword", "example-secret")):
      if populated:
        params.put(key, value, block=True)
      else:
        params.remove(key)

  seed(True)
  from openpilot.selfdrive.ui.ui_state import ui_state, device
  from openpilot.selfdrive.ui.lib.prime_state import PrimeType
  from openpilot.system.ui.lib.application import gui_app

  rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
  gui_app.init_window("APN authentication screenshots", fps=FPS)
  if gui_app._render_texture is None:
    gui_app._render_texture = rl.load_render_texture(gui_app._scaled_width, gui_app._scaled_height)

  if BIG:
    from openpilot.selfdrive.ui.layouts.settings.settings import SettingsLayout, PanelType
    from openpilot.system.ui.widgets.network import PanelType as NetworkPanel
    root = SettingsLayout()
    gui_app.push_widget(root)
    root.set_current_panel(PanelType.NETWORK)
    network = root._panels[PanelType.NETWORK].instance
    network._set_current_panel(NetworkPanel.ADVANCED)
    panel = network._advanced_panel
  else:
    from openpilot.selfdrive.ui.mici.layouts.settings.network.network_layout import NetworkLayoutMici
    panel = root = NetworkLayoutMici()
    gui_app.push_widget(root)

  def close_dialogs():
    while gui_app.get_active_widget() is not root:
      if BIG and gui_app.get_active_widget() is panel._keyboard:
        panel._keyboard._cancel_button_callback()
      else:
        gui_app.pop_widget()

  def scroll_to(widget):
    scroller = panel._scroller
    if BIG:
      scroller.scroll_panel.set_offset(scroller.scroll_panel.offset + scroller.rect.y - widget.rect.y)
    else:
      scroller._layout()
      scroller.scroll_to(widget.rect.x + widget.rect.width / 2 - (panel.rect.x + panel.rect.width / 2))

  steps: dict[int, Callable] = {}
  captures: dict[int, str] = {}
  next_frame = FPS

  def add(action: Callable, name: str | None = None, close: bool = True):
    nonlocal next_frame

    def run():
      if close:
        close_dialogs()
      action()

    steps[next_frame] = run
    if name:
      captures[next_frame + FPS] = name + ".png"
    next_frame += FPS * 2

  def show_state(populated: bool):
    seed(populated)
    panel.show_event()

  def type_password():
    if BIG:
      panel._keyboard.set_text("new-secret")
    else:
      gui_app.get_active_widget()._keyboard.set_text("new-secret")

  def choose_off():
    # Start at CHAP even in the empty fixture so selecting off isn't a no-op.
    panel._set_apn_auth(2 if BIG else "chap")
    panel._set_apn_auth(0 if BIG else "off")

  for state, populated in (("set", True), ("empty", False)):
    add(lambda populated=populated: show_state(populated))
    if BIG:
      add(lambda: scroll_to(panel._apn_btn), f"{state}_rows")
    else:
      for row in ("apn", "apn_user", "apn_password", "apn_auth"):
        add(lambda row=row: scroll_to(getattr(panel, f"_{row}_btn")), f"{state}_{row}_row")
    if populated:
      add(lambda: scroll_to(panel._apn_delete_btn), "set_delete_credentials_row")
      add(panel._delete_apn_credentials, "set_delete_credentials_confirm")
      if not BIG:
        for index, choice in ((1, "no"), (2, "yes")):
          add(lambda index=index: scroll_confirm_choice(index), f"set_delete_credentials_{choice}", close=False)
    add(panel._edit_apn, f"{state}_apn_keyboard")
    add(panel._edit_apn_user, f"{state}_user_keyboard")
    add(panel._edit_apn_password, f"{state}_password_keyboard")
    add(type_password, f"{state}_password_typed", close=False)
    add(choose_off, f"{state}_off_confirm")
    if not BIG:
      for index, choice in ((1, "no"), (2, "yes")):
        add(lambda index=index: scroll_confirm_choice(index), f"{state}_off_{choice}", close=False)

  def scroll_confirm_choice(index: int):
    dialog = gui_app.get_active_widget()
    dialog._scroller._layout()
    choice = dialog._scroller.items[index]
    dialog._scroller.scroll_to(choice.rect.x + choice.rect.width / 2 - (dialog.rect.x + dialog.rect.width / 2))

  if BIG:
    def invalid_input():
      panel._edit_apn_user()
      panel._keyboard.set_text("invalid user")
      panel._keyboard._key_callback("->")

    add(invalid_input, "invalid_credential_alert")
  else:
    add(panel._apn_user_btn._long_press_callback, "credential_rule")

  frame = 0
  written: list[str] = []
  original_end_drawing = rl.end_drawing

  def end_drawing():
    original_end_drawing()
    if frame in captures:
      image = rl.load_image_from_texture(gui_app._render_texture.texture)
      try:
        rl.image_flip_vertical(image)
        name = captures[frame]
        if not rl.export_image(image, str(output / name)):
          raise RuntimeError(f"Could not export {name}")
        written.append(name)
      finally:
        rl.unload_image(image)

  rl.end_drawing = end_drawing
  rl.get_frame_time = lambda: 1.0 / FPS
  rl.get_time = lambda: frame / FPS
  device.set_override_interactive_timeout(99999)
  try:
    for _ in gui_app.render():
      ui_state.update()
      ui_state.prime_state.set_type(PrimeType.NONE)
      if frame in steps:
        steps[frame]()
      frame += 1
      if frame > max(captures) + 1:
        break
  finally:
    rl.end_drawing = original_end_drawing
    gui_app.close()
  if len(written) != len(captures):
    raise RuntimeError(f"Captured {len(written)} of {len(captures)} screenshots")
  print(f"Wrote {len(written)} screenshots to {output}")


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("output", type=Path)
  parser.add_argument("--lang", help="Language code for translated strings (e.g. ja)")
  args = parser.parse_args()
  with OpenpilotPrefix():
    main(args.output, args.lang)
