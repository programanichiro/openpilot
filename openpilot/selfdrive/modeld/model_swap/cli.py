import argparse
import json
from pathlib import Path

from .catalog import MANIFESTS, download_bundle, fetch_manifest, model_root, select_bundle
from .catalog import bundles_from_manifest
from .plain_artifact import read_parts


def _bundles(args):
  return bundles_from_manifest(fetch_manifest(args.source))


def cmd_list(args) -> None:
  for bundle in _bundles(args):
    if args.filter and args.filter.lower() not in f"{bundle.short_name} {bundle.display_name}".lower():
      continue
    fields = (f"{bundle.short_name:8}", f"{bundle.ref[:12]:12}", f"gen={bundle.generation:<2}",
              f"20hz={str(bundle.is_20hz):5}", f"big={str(bundle.is_big):5}", bundle.artifact.file_name)
    print(" ".join(fields))


def cmd_download(args) -> None:
  bundle = select_bundle(_bundles(args), ref=args.ref, short_name=args.short_name, index=args.index)
  paths = download_bundle(bundle, args.root)
  for path in paths:
    print(path)
  if args.activate:
    from openpilot.common.params import Params
    key = "ModelSwapBigPkl" if bundle.is_big else "ModelSwapPkl"
    Params().put(key, str(paths[0]), block=True)
    print(f"{key}={paths[0]}")


def _part_summary(name, header) -> dict:
  return {
    "name": name,
    "inputs": [spec.get("slot") for spec in header.get("inputs", [])],
    "buffers": len(header.get("buffers", [])),
    "calls": len(header.get("calls", [])),
    "input_names": header.get("input_names", []),
    "return_kind": (header.get("returns") or {}).get("kind"),
    "has_check": bool(header.get("check")),
  }


def cmd_inspect(args) -> None:
  path = Path(args.path)
  parts, extras = read_parts(path)
  metadata = extras.get("metadata", {})
  print(json.dumps({
    "path": str(path),
    "size": path.stat().st_size,
    "legacy": isinstance(metadata, dict) and "model" in metadata,
    "native": "input_specs" in extras,
    "warp_dev": metadata.get("warp_dev") if isinstance(metadata, dict) else None,
    "parts": [_part_summary(str(name), header) for name, (header, _) in parts.items()],
    "extras": sorted(extras),
  }, indent=2))


def build_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(prog="model_swap")
  sub = parser.add_subparsers(dest="command", required=True)

  list_parser = sub.add_parser("list")
  list_parser.add_argument("--source", choices=tuple(MANIFESTS), default="qcom")
  list_parser.add_argument("--filter")
  list_parser.set_defaults(func=cmd_list)

  download_parser = sub.add_parser("download")
  download_parser.add_argument("--source", choices=tuple(MANIFESTS), default="qcom")
  download_parser.add_argument("--ref")
  download_parser.add_argument("--short-name")
  download_parser.add_argument("--index", type=int)
  download_parser.add_argument("--root", type=Path, default=model_root())
  download_parser.add_argument("--activate", action="store_true")
  download_parser.set_defaults(func=cmd_download)

  inspect_parser = sub.add_parser("inspect")
  inspect_parser.add_argument("path", type=Path)
  inspect_parser.set_defaults(func=cmd_inspect)
  return parser


def main() -> None:
  args = build_parser().parse_args()
  args.func(args)


if __name__ == "__main__":
  main()
