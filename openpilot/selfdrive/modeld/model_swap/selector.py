import threading
from pathlib import Path

from openpilot.common.params import Params
from openpilot.selfdrive.modeld.helpers import chestnut_compiled, chestnut_present

from .catalog import Bundle, bundles_from_manifest, download_bundle, fetch_manifest

STOCK_LABEL = "stock"


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
    self._error: str | None = None

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

  def _param_path(self) -> str | None:
    value = self._params.get(param_key(self._source))
    if isinstance(value, bytes):
      value = value.decode()
    return value or None

  def current_bundle(self) -> Bundle | None:
    path = self._param_path()
    if not path:
      return None
    name = Path(path).name
    return next((bundle for bundle in self._bundles or [] if bundle.artifact.file_name == name), None)

  def current_label(self) -> str:
    if bundle := self.current_bundle():
      return bundle.short_name
    path = self._param_path()
    return Path(path).stem if path else STOCK_LABEL

  def reload(self) -> None:
    if self._loading or self._bundles is not None:
      return
    self._loading = True
    self._error = None

    def run() -> None:
      try:
        bundles = bundles_from_manifest(fetch_manifest(self._source))
        self._bundles = [bundle for bundle in bundles if bundle.is_big == (self._source == "chestnut")]
      except Exception as error:
        self._error = str(error)
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
        else:
          paths = download_bundle(bundle)
          self._params.put(key, str(paths[0]), block=True)
          self._params.remove(other_key)
      except Exception as error:
        self._error = str(error)
      finally:
        self._busy = False

    threading.Thread(target=run, daemon=True).start()
    return True
