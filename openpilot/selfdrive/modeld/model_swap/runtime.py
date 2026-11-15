import math

import numpy as np
from tinygrad import Tensor

from openpilot.common.hardware import COMMA_HARDWARE
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

from . import stock_dependencies as stock
from .plain_artifact import load_model

POLICY_INPUTS = ["img_q", "big_img_q", "feat_q", "desire_q", "packed_npy_inputs"]


def _detect_desire_key(shapes: dict) -> str | None:
  return next((key for key in shapes if key.startswith("desire")), None)


def _detect_vision_keys(shapes: dict) -> tuple[str | None, str | None]:
  img_keys = sorted(key for key in shapes if "img" in key)
  return (next((key for key in img_keys if "big" not in key), None),
          next((key for key in img_keys if "big" in key), None))


def derive_frame_skip(input_shapes: dict) -> int:
  features_buffer = input_shapes.get("features_buffer")
  return 1 if not features_buffer or features_buffer[1] >= 99 else 4


def _host_view(tensor: Tensor) -> np.ndarray:
  return np.frombuffer(tensor._buffer()._host_mv(), dtype=np.float32).reshape(tensor.shape)


def _policy_npy_shapes(input_shapes: dict) -> tuple[dict, list[int]]:
  desire_key = _detect_desire_key(input_shapes)
  shapes = {"desire": (input_shapes[desire_key][2],)} if desire_key else {}
  for key, shape in input_shapes.items():
    if key not in (desire_key, "features_buffer") and "img" not in key:
      shapes[key] = tuple(shape)
  if "features_buffer" in input_shapes:
    features_buffer = input_shapes["features_buffer"]
    shapes["prev_feat"] = (features_buffer[0], math.prod(features_buffer[2:]))
  return shapes, [int(np.prod(shape)) for shape in shapes.values()]


def _make_supercombo_queues(input_shapes: dict, frame_skip: int, device: str) -> tuple[dict, dict]:
  road_key, _ = _detect_vision_keys(input_shapes)
  if road_key is None:
    raise ValueError("vision road input missing")
  image_shape = input_shapes[road_key]
  n_frames = image_shape[1] // 6
  image_queue_shape = (frame_skip * (n_frames - 1) + 1, 6, image_shape[2], image_shape[3])

  desire_key = _detect_desire_key(input_shapes)
  if desire_key is None:
    raise ValueError("desire input missing")
  desire_shape = input_shapes[desire_key]
  features_buffer = input_shapes.get("features_buffer")

  npy_arrays = {"tfm": np.zeros((3, 3), dtype=np.float32), "big_tfm": np.zeros((3, 3), dtype=np.float32)}
  shapes, sizes = _policy_npy_shapes(input_shapes)
  packed_npy_inputs = np.zeros(sum(sizes), dtype=np.float32)
  split_indices = np.cumsum(sizes[:-1]) if len(sizes) > 1 else []
  views = np.split(packed_npy_inputs, split_indices) if sizes else []
  for (key, shape), view in zip(shapes.items(), views, strict=True):
    npy_arrays[key] = view.reshape(shape)

  queues = {
    "img_q": Tensor(np.zeros(image_queue_shape, dtype=np.uint8), device=device).contiguous().realize(),
    "big_img_q": Tensor(np.zeros(image_queue_shape, dtype=np.uint8), device=device).contiguous().realize(),
    "desire_q": Tensor(np.zeros((frame_skip * desire_shape[1], desire_shape[0], desire_shape[2]),
                                dtype=np.float32), device=device).contiguous().realize(),
    "packed_npy_inputs": Tensor(packed_npy_inputs, device=device).realize(),
  }
  if features_buffer:
    feat_dim = math.prod(features_buffer[2:])
    queues["feat_q"] = Tensor(np.zeros((frame_skip * features_buffer[1], features_buffer[0], feat_dim),
                                       dtype=np.float32), device=device).contiguous().realize()

  for key in ("tfm", "big_tfm"):
    queues[key] = Tensor(npy_arrays[key], device=device).realize()

  packed = _host_view(queues["packed_npy_inputs"])
  npy_arrays |= {key: view.reshape(shape) for (key, shape), view in
                 zip(shapes.items(), np.split(packed, split_indices) if sizes else [], strict=True)}
  return queues, npy_arrays | {key: _host_view(queues[key]) for key in ("tfm", "big_tfm")}


class LegacyModelAdapter:
  def __init__(self, jits: dict, cam_w: int, cam_h: int, device: str, chestnut: bool):
    self.jits = jits
    self.cam_w, self.cam_h = cam_w, cam_h
    self.DEV = device
    self.QUEUE_DEV = device
    self.WARP_DEV = jits["metadata"].get("warp_dev", device) if COMMA_HARDWARE else device
    self.chestnut = chestnut
    self.nv12_info = get_nv12_info(cam_w, cam_h)
    self.frame_copy_size = self.nv12_info[0] * (self.nv12_info[1] + self.nv12_info[2])

    metadata = jits["metadata"]
    if "model" not in metadata:
      raise ValueError("only legacy supercombo artifacts are supported by this adapter")
    model_metadata = metadata["model"]
    self.input_shapes = model_metadata["input_shapes"]
    self.vision_output_slices = model_metadata["output_slices"]
    self._vision_input_names = [key for key in self.input_shapes if "img" in key]
    self.frame_skip = derive_frame_skip(self.input_shapes)
    self.fused_policy = not chestnut and "run_policy" not in jits

    if chestnut or self.fused_policy:
      self.WARP_DEV = self.DEV
      self.input_queues, self.numpy_inputs, self.frame_slots = stock.make_input_queues(
        self.input_shapes, self.frame_skip, device=self.QUEUE_DEV, frame_copy_size=self.frame_copy_size)
    else:
      self.input_queues, self.numpy_inputs = _make_supercombo_queues(self.input_shapes, self.frame_skip, self.QUEUE_DEV)

    self.run_policy = jits["run_policy"] if "run_policy" in jits else jits[(cam_w, cam_h)]
    self.run_warp = None if self.fused_policy else self._load_warp()
    self._desire_key = next((key for key in self.numpy_inputs if key.startswith("desire")), "desire")
    self._road_key = next((key for key in self._vision_input_names if "big" not in key), "img")
    self._wide_key = next((key for key in self._vision_input_names if "big" in key), "big_img")
    self.frame_buf_params = dict.fromkeys(self._vision_input_names, self.nv12_info)
    self.full_frames: dict[str, Tensor] = {}
    self._blob_cache: dict[tuple[str, int], Tensor] = {}

    if not (chestnut or self.fused_policy):
      frame_size = self.frame_buf_params[self._road_key][3]
      frame = Tensor(np.zeros(frame_size, dtype=np.uint8), device=self.WARP_DEV).contiguous().realize()
      big_frame = Tensor(np.zeros(frame_size, dtype=np.uint8), device=self.WARP_DEV).contiguous().realize()
      self.run_warp(**{key: self.input_queues[key] for key in ("tfm", "big_tfm")}, frame=frame, big_frame=big_frame)

  def _load_warp(self):
    if (self.cam_w, self.cam_h) in self.jits:
      self.warp_frame_size = self.nv12_info[3]
      return self.jits[(self.cam_w, self.cam_h)]
    raise ValueError(f"warp for {self.cam_w}x{self.cam_h} is not embedded in artifact")

  @property
  def vision_input_names(self) -> list[str]:
    return self._vision_input_names

  def get_dummy_inputs(self) -> tuple[dict, dict, dict]:
    frame_size = self.warp_frame_size if hasattr(self, "warp_frame_size") else self.frame_copy_size
    frames = {key: np.zeros(frame_size, dtype=np.uint8) for key in self._vision_input_names}
    transforms = {key: np.eye(3, dtype=np.float32) for key in (self._road_key, self._wide_key)}
    inputs = {key: np.zeros(value.shape, dtype=value.dtype)
              for key, value in self.numpy_inputs.items() if key not in ("tfm", "big_tfm", "prev_feat")}
    return frames, transforms, inputs

  def copy_frames(self, bufs) -> None:
    if self.chestnut or self.fused_policy:
      for key in self._vision_input_names:
        if key in bufs:
          data = bufs[key].data if hasattr(bufs[key], "data") else bufs[key]
          np.copyto(self.frame_slots[key], np.frombuffer(data, dtype=np.uint8, count=self.frame_copy_size))
      return
    for key in bufs:
      data = bufs[key].data if hasattr(bufs[key], "data") else bufs[key]
      pointer = np.frombuffer(data, dtype=np.uint8).ctypes.data
      frame_size = self.frame_buf_params[key][3]
      cache_key = (key, pointer)
      if cache_key not in self._blob_cache:
        self._blob_cache[cache_key] = Tensor.from_blob(pointer, (frame_size,), dtype="uint8", device=self.WARP_DEV)
      self.full_frames[key] = self._blob_cache[cache_key]

  def reset_warmup_buffers(self) -> None:
    for value in self.numpy_inputs.values():
      value[:] = 0
    if not (self.chestnut or self.fused_policy):
      self.full_frames.clear()
      self._blob_cache.clear()

  def run(self) -> Tensor:
    if self.chestnut or self.fused_policy:
      return self.run_policy(**{key: self.input_queues[key] for key in POLICY_INPUTS if key in self.input_queues})
    warped = self.run_warp(**{key: self.input_queues[key] for key in ("tfm", "big_tfm")},
                           frame=self.full_frames[self._road_key], big_frame=self.full_frames[self._wide_key])
    return self.run_policy(**{key: self.input_queues[key] for key in POLICY_INPUTS if key in self.input_queues},
                           warped=warped)


class ModelSwapState:
  def __init__(self, cam_w: int, cam_h: int, pkl_path: str, chestnut: bool = False, verify: bool = False):
    if not COMMA_HARDWARE:
      raise RuntimeError("selected model artifacts require comma hardware")
    self.pkl_path = pkl_path
    self.chestnut = chestnut
    self.DEV = "AMD" if chestnut else "QCOM"
    device_map = {"QCOM": self.DEV, "AMD": self.DEV, "NPU": self.DEV}
    self.jits = load_model(pkl_path, device_map=device_map, verify=verify)
    self.adapter = LegacyModelAdapter(self.jits, cam_w, cam_h, self.DEV, chestnut)
    self.parser = Parser()
    self.vision_output_slices = self.adapter.vision_output_slices
    self.frame_copy_size = self.adapter.frame_copy_size
    self.prev_desire = np.zeros(ModelConstants.DESIRE_LEN, dtype=np.float32)

  @property
  def vision_input_names(self) -> list[str]:
    return self.adapter.vision_input_names

  def warmup(self) -> None:
    frames, transforms, inputs = self.adapter.get_dummy_inputs()
    inputs["desire_pulse"] = inputs[self.adapter._desire_key]
    self.run(frames, transforms, inputs)
    self.adapter.reset_warmup_buffers()
    self.prev_desire[:] = 0

  def run(self, bufs, transforms: dict[str, np.ndarray], inputs: dict[str, np.ndarray],
          after_enqueue=None) -> dict[str, np.ndarray]:
    self.adapter.copy_frames(bufs)

    desire = inputs.get("desire_pulse")
    if desire is None:
      raise ValueError("desire_pulse input missing")
    self.adapter.numpy_inputs[self.adapter._desire_key][:] = np.where(desire - self.prev_desire > .99, desire, 0)
    self.prev_desire[:] = desire
    for key in ("traffic_convention", "action_t"):
      if key in self.adapter.numpy_inputs and key in inputs:
        self.adapter.numpy_inputs[key][:] = inputs[key]
    self.adapter.numpy_inputs["tfm"][:, :] = transforms[self.adapter._road_key].reshape(3, 3)
    self.adapter.numpy_inputs["big_tfm"][:, :] = transforms[self.adapter._wide_key].reshape(3, 3)

    raw_output = self.adapter.run()
    if after_enqueue is not None:
      after_enqueue()
    model_output = raw_output.numpy().flatten()
    sliced = {key: model_output[np.newaxis, value] for key, value in self.vision_output_slices.items()}
    return self.parser.parse_outputs(sliced)
