import hashlib
import json
import os
import re
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

MANIFESTS = {
  "qcom": "https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/refs/heads/gh-pages/docs/driving_models_v25.json",
  "chestnut": "https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/refs/heads/gh-pages/docs/driving_models_chestnut_v28.json",
}

PIECE_SIZE = 8 * 1024 * 1024
MAX_CONCURRENT_PIECES = 12
DOWNLOAD_ATTEMPTS = 4
TIMEOUT = 60
HEADERS = {"Accept-Encoding": "identity", "User-Agent": "openpilot-model-swap/1"}


@dataclass(frozen=True)
class Artifact:
  file_name: str
  url: str
  sha256: str


@dataclass(frozen=True)
class Bundle:
  ref: str
  short_name: str
  display_name: str
  generation: int
  is_20hz: bool
  is_big: bool
  runner: str
  overrides: dict[str, str]
  artifacts: tuple[Artifact, ...]

  @property
  def artifact(self) -> Artifact:
    if not self.artifacts:
      raise ValueError(f"{self.short_name} has no artifacts")
    return self.artifacts[0]


def model_root() -> Path:
  if root := os.environ.get("MODEL_SWAP_DIR"):
    return Path(root)
  if os.path.isdir("/data/media/0"):
    return Path("/data/media/0/models")
  return Path.home() / ".cache" / "openpilot-models"


def _open(url: str, headers: dict[str, str] | None = None, timeout: int = TIMEOUT):
  request = urllib.request.Request(url, headers={**HEADERS, **(headers or {})})
  return urllib.request.urlopen(request, timeout=timeout)


def fetch_manifest(source: str = "qcom", timeout: int = TIMEOUT) -> dict:
  if source not in MANIFESTS:
    raise ValueError(f"unknown source {source!r}, expected one of {tuple(MANIFESTS)}")
  with _open(MANIFESTS[source], timeout=timeout) as response:
    return json.load(response)


def _parse_artifacts(model: dict) -> tuple[Artifact, ...]:
  artifact = model.get("artifact") or {}
  uri = artifact.get("download_uri") or {}
  if not (file_name := artifact.get("file_name")) or not (url := uri.get("url")) or not (sha256 := uri.get("sha256")):
    return ()
  return (Artifact(file_name=file_name, url=url, sha256=sha256.lower()),)


def bundles_from_manifest(manifest: dict) -> list[Bundle]:
  bundles = []
  for raw in manifest.get("bundles", []):
    artifacts = tuple(artifact for model in raw.get("models", []) for artifact in _parse_artifacts(model))
    if not artifacts:
      continue
    bundles.append(Bundle(
      ref=raw.get("ref", ""),
      short_name=raw.get("short_name", ""),
      display_name=raw.get("display_name", ""),
      generation=int(raw.get("generation", 0)),
      is_20hz=bool(raw.get("is_20hz", False)),
      is_big=bool(raw.get("is_big", False)),
      runner=raw.get("runner", ""),
      overrides=dict(raw.get("overrides", {})),
      artifacts=artifacts,
    ))
  return bundles


def select_bundle(bundles: list[Bundle], ref: str | None = None, short_name: str | None = None, index: int | None = None) -> Bundle:
  if ref:
    matches = [bundle for bundle in bundles if bundle.ref == ref]
  elif short_name:
    matches = [bundle for bundle in bundles if bundle.short_name.lower() == short_name.lower()]
  elif index is not None:
    try:
      matches = [bundles[index]]
    except IndexError:
      matches = []
  else:
    matches = bundles[:1]
  if not matches:
    raise LookupError("no matching bundle")
  if len(matches) > 1:
    raise LookupError(f"ambiguous bundle: {[bundle.short_name for bundle in matches]}")
  return matches[0]


def sha256_file(path: str | Path) -> str:
  digest = hashlib.sha256()
  with open(path, "rb") as file:
    for chunk in iter(lambda: file.read(1024 * 1024), b""):
      digest.update(chunk)
  return digest.hexdigest().lower()


def _content_range_total(response) -> int | None:
  if response.status != 206:
    return None
  match = re.fullmatch(r"bytes \d+-\d+/(\d+)", response.headers.get("Content-Range", ""))
  return int(match.group(1)) if match else None


def _probe_size(url: str) -> tuple[int, bool]:
  with _open(url, {"Range": "bytes=0-0"}) as response:
    total = _content_range_total(response)
    if total is not None:
      return total, True
    return int(response.headers.get("Content-Length", 0)), False


def _fetch_piece(url: str, path: Path, start: int, end: int) -> None:
  last_error: Exception | None = None
  for attempt in range(DOWNLOAD_ATTEMPTS):
    try:
      with _open(url, {"Range": f"bytes={start}-{end - 1}"}) as response:
        if response.status != 206:
          raise OSError(f"server ignored byte range for {path.name}")
        data = response.read()
      if len(data) != end - start:
        raise OSError(f"short read for {path.name}: {len(data)} of {end - start}")
      with open(path, "r+b") as file:
        file.seek(start)
        file.write(data)
      return
    except Exception as error:
      last_error = error
      if attempt + 1 < DOWNLOAD_ATTEMPTS:
        time.sleep(1 + attempt)
  raise OSError(f"failed to download {path.name} bytes {start}-{end - 1}") from last_error


def _download_ranged(url: str, path: Path, total: int) -> None:
  with open(path, "wb") as file:
    file.truncate(total)
  pieces = [(start, min(start + PIECE_SIZE, total)) for start in range(0, total, PIECE_SIZE)]
  with ThreadPoolExecutor(max_workers=MAX_CONCURRENT_PIECES) as pool:
    futures = [pool.submit(_fetch_piece, url, path, start, end) for start, end in pieces]
    for future in as_completed(futures):
      future.result()


def _download_stream(url: str, path: Path) -> None:
  with _open(url) as response, open(path, "wb") as file:
    for chunk in iter(lambda: response.read(1024 * 1024), b""):
      file.write(chunk)


def download_file(url: str, destination: str | Path, sha256: str | None = None) -> Path:
  destination = Path(destination)
  destination.parent.mkdir(parents=True, exist_ok=True)
  if destination.is_file() and sha256 and sha256_file(destination) == sha256.lower():
    return destination

  temp = destination.with_name(f".{destination.name}.{os.getpid()}.part")
  try:
    total, ranged = _probe_size(url)
    if ranged and total > 0:
      _download_ranged(url, temp, total)
    else:
      _download_stream(url, temp)
    if sha256 and sha256_file(temp) != sha256.lower():
      raise ValueError(f"sha256 mismatch for {destination.name}")
    os.replace(temp, destination)
  finally:
    temp.unlink(missing_ok=True)
  return destination


def download_bundle(bundle: Bundle, root: str | Path | None = None) -> list[Path]:
  root = Path(root) if root is not None else model_root()
  downloaded = []
  for artifact in bundle.artifacts:
    downloaded.append(download_file(artifact.url, root / Path(artifact.file_name).name, artifact.sha256))
  return downloaded
