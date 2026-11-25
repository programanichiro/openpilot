#!/usr/bin/env python3
"""Check Japanese APN-auth translations against the bundled CJK font subset."""
from pathlib import Path

from fontTools.ttLib import TTFont

from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.ui.translations.potools import parse_po

APN_AUTH_MSGIDS = (
  "Enter APN user name",
  "Enter APN password",
  "APN User Name",
  "APN Password",
  "APN Authentication",
  "Off",
  "PAP",
  "CHAP",
  "Also delete the saved APN user name and password?",
  "Yes",
  "No",
  "Use 1-64 ASCII letters, digits or symbols, without spaces, double quotes, backslashes or commas.",
  "Delete APN Credentials",
  "DELETE",
  "Delete the saved APN user name and password?",
)


def main() -> int:
  font_path = Path(BASEDIR) / "openpilot/selfdrive/assets/fonts/NotoSansCJKjp-Regular.otf"
  po_path = Path(BASEDIR) / "openpilot/selfdrive/ui/translations/app_ja.po"
  _, entries = parse_po(po_path)
  translations = {entry.msgid: entry.msgstr for entry in entries}
  with TTFont(font_path) as font:
    cmap = font.getBestCmap() or {}

  failed = False
  for msgid in APN_AUTH_MSGIDS:
    if not translations.get(msgid):
      print(f"Missing Japanese translation: {msgid}")
      failed = True
      continue
    missing = sorted({char for char in translations[msgid] if ord(char) > 127 and ord(char) not in cmap}, key=ord)
    if missing:
      glyphs = ", ".join(f"{char} (U+{ord(char):04X})" for char in missing)
      print(f"{msgid}: missing {glyphs}")
      failed = True

  if failed:
    return 1
  print(f"PASS: all non-ASCII glyphs in {len(APN_AUTH_MSGIDS)} Japanese APN-auth translations exist in {font_path.name}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
