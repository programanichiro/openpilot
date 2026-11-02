import json
import threading
from pathlib import Path

from openpilot.common.params import Params
from openpilot.selfdrive.modeld.helpers import chestnut_compiled, chestnut_present

from .catalog import Bundle, bundles_from_manifest, download_bundle, fetch_manifest, model_root

STOCK_LABEL = "stock"
META_NAME = ".selected.json"


def model_source() -> str:
  return "chestnut" if chestnut_compiled() and chestnut_present() else "qcom"


def param_key(source: str) -> str:
  return "ModelSwapBigPkl" if source == "chestnut" else "ModelSwapPkl"


class ModelSwapSelector:
  def __init__(self):
    self._params = Params()
    self._source = model_source()
    self._bundles: list[Bundle] | None = None
    self._loading = False
    self._busy = False
    self._error: str | None = None  # モデルの選択（ダウンロード）の失敗
    self._list_error: str | None = None  # モデル一覧の取得の失敗。起動直後の通信不可などで起き、選択には影響しない
    self._meta = self._read_meta()

  @property
  def source(self) -> str:
    return self._source

  @property
  def bundles(self) -> list[Bundle] | None:
    return self._bundles

  @property
  def loading(self) -> bool:
    return self._loading

  @property
  def busy(self) -> bool:
    return self._busy

  @property
  def error(self) -> str | None:
    return self._error

  @property
  def list_error(self) -> str | None:
    return self._list_error

  def _read_meta(self) -> dict:
    try:
      return json.loads((model_root() / META_NAME).read_text())
    except Exception:
      return {}

  def _write_meta(self, path: str, label: str) -> None:
    self._meta = {"path": path, "label": label}
    try:
      meta_path = model_root() / META_NAME
      meta_path.parent.mkdir(parents=True, exist_ok=True)
      meta_path.write_text(json.dumps(self._meta))
    except Exception:
      pass

  def _clear_meta(self) -> None:
    self._meta = {}
    try:
      (model_root() / META_NAME).unlink(missing_ok=True)
    except Exception:
      pass

  def _param_path(self) -> str | None:
    value = self._params.get(param_key(self._source))
    if isinstance(value, bytes):
      value = value.decode()
    return value or None

  def has_selection(self) -> bool:
    return self._param_path() is not None

  def current_bundle(self) -> Bundle | None:
    path = self._param_path()
    if not path:
      return None
    name = Path(path).name
    return next((bundle for bundle in self._bundles or [] if bundle.artifact.file_name == name), None)

  def has_saved_label(self) -> bool:
    path = self._param_path()
    return bool(path) and self._meta.get("path") == path and bool(self._meta.get("label"))

  def current_label(self) -> str:
    path = self._param_path()
    if not path:
      return STOCK_LABEL
    if self._meta.get("path") == path and self._meta.get("label"):
      return self._meta["label"]
    if bundle := self.current_bundle():
      return bundle.short_name
    return "selected"

  def reload(self) -> None:
    if self._loading or self._bundles is not None:
      return
    self._loading = True
    self._list_error = None

    def run() -> None:
      try:
        bundles = bundles_from_manifest(fetch_manifest(self._source))
        self._bundles = [bundle for bundle in bundles if bundle.is_big == (self._source == "chestnut")]
        if bundle := self.current_bundle():
          self._write_meta(self._param_path() or "", bundle.short_name)
      except Exception as error:
        self._list_error = str(error)
      finally:
        self._loading = False

    threading.Thread(target=run, daemon=True).start()

  def select(self, bundle: Bundle | None) -> bool:
    if self._busy:
      return False
    self._busy = True
    self._error = None
    key = param_key(self._source)
    other_key = "ModelSwapPkl" if key == "ModelSwapBigPkl" else "ModelSwapBigPkl"

    def run() -> None:
      try:
        if bundle is None:
          self._params.remove(key)
          self._params.remove(other_key)
          self._clear_meta()
        else:
          paths = download_bundle(bundle)
          path = str(paths[0])
          self._params.put(key, path, block=True)
          self._params.remove(other_key)
          self._write_meta(path, bundle.short_name)
      except Exception as error:
        self._error = str(error)
      finally:
        self._busy = False

    threading.Thread(target=run, daemon=True).start()
    return True
