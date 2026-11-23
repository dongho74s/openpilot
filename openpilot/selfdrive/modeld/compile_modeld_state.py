#!/usr/bin/env python3
"""Compile a driving model that keeps its own history queues (upstream "modeld: move history queues into ONNX",
commaai/openpilot #38916): inputs new_img (2, 6, H, W), desire (8,), traffic_convention, action_t and state_*,
outputs `outputs` (1, N) and next_state_* of the same shapes as the state_* inputs.

The host-side queues compile_modeld.py keeps for the older contract (img_q, feat_q, desire_q, prev_feat) live inside
such a model, so the policy JIT here only feeds the newest warped frame of both cameras and writes next_state_* back
into the state buffers. The warp is compile_modeld's. Only internal_model_manager runs this for a model the user
selected; the stock model and the SConscript build keep using compile_modeld.py, so editing this file does not
rebuild the stock model.
"""
import argparse
import math
import os
from functools import partial

import numpy as np

# patches tinygrad (firmware fetch, Buffer pickling) on import, as for the stock compile
from openpilot.selfdrive.modeld.compile_modeld import (NV12Frame, WARP_INPUTS, WARP_DEV, compile_jit, make_random_images,
                                                       make_warp, read_file_chunked_to_disk, _parse_size)
from openpilot.selfdrive.modeld.helpers import dump_oob, validate_model_file

from tinygrad.tensor import Tensor
from tinygrad.device import Device
from tinygrad.engine.jit import TinyJit

STATE_PREFIX = 'state_'
NEXT_PREFIX = 'next_'
STATE_NPY_INPUTS = ('desire', 'traffic_convention', 'action_t')
# ONNX elem types the state inputs use (uint8 frames, float32 desire/features)
ELEM_DTYPES = {1: np.float32, 2: np.uint8, 10: np.float16}


def is_state_model(input_shapes) -> bool:
  return 'new_img' in input_shapes


def state_names(input_shapes) -> list[str]:
  return sorted(k for k in input_shapes if k.startswith(STATE_PREFIX))


def policy_inputs(input_shapes) -> list[str]:
  return [*state_names(input_shapes), 'packed_npy_inputs']


def get_state_npy_shapes(input_shapes):
  shapes = {k: tuple(input_shapes[k]) for k in STATE_NPY_INPUTS}
  return shapes, [math.prod(s) for s in shapes.values()]


def make_state_warp_queues(input_shapes, device=None):
  npy = {'tfm': np.zeros((3, 3), dtype=np.float32), 'big_tfm': np.zeros((3, 3), dtype=np.float32)}
  return {k: Tensor(v, device='NPY').realize() for k, v in npy.items()}, npy


def make_state_input_queues(input_shapes, state_dtypes, device):
  """State buffers (zeroed, on the policy device) and the packed host inputs with their numpy views."""
  input_queues, npy = make_state_warp_queues(input_shapes)
  shapes, sizes = get_state_npy_shapes(input_shapes)
  packed_npy_inputs = np.zeros(sum(sizes), dtype=np.float32)
  npy.update({k: v.reshape(s) for (k, s), v in zip(shapes.items(), np.split(packed_npy_inputs, np.cumsum(sizes[:-1])), strict=True)})
  for name in state_names(input_shapes):
    dtype = np.dtype(state_dtypes[name])
    input_queues[name] = Tensor(np.zeros(tuple(input_shapes[name]), dtype=dtype), device=device).contiguous().realize()
  input_queues['packed_npy_inputs'] = Tensor(packed_npy_inputs, device='NPY').realize()
  return input_queues, npy


def make_run_state_policy(model_runner, model_metadata, input_dtypes):
  input_shapes = model_metadata['input_shapes']
  npy_shapes, npy_sizes = get_state_npy_shapes(input_shapes)
  names = state_names(input_shapes)
  new_img_dtype = np.dtype(input_dtypes['new_img']).name  # tinygrad casts take a dtype name, not a numpy dtype

  def run_policy(warped, packed_npy_inputs, **states):
    packed_npy_inputs = packed_npy_inputs.to(Device.DEFAULT)
    warped = warped.to(Device.DEFAULT)
    Tensor.realize(packed_npy_inputs, warped)
    desire, traffic_convention, action_t = (t.reshape(s) for t, s in zip(packed_npy_inputs.split(npy_sizes), npy_shapes.values(), strict=True))
    inputs = {
      'new_img': warped.reshape(input_shapes['new_img']).cast(new_img_dtype),
      'desire': desire,
      'traffic_convention': traffic_convention,
      'action_t': action_t,
      **{name: states[name] for name in names},
    }
    outs = model_runner(inputs)
    out = outs['outputs'].cast('float32')
    # The next frame reads the history this frame produced. First compute the output and every next state into
    # buffers of their own while the old state is still intact, then copy them over the state buffers, and realize
    # those copies: an assign nothing returned or realized is dropped from the captured JIT.
    nexts = [outs[NEXT_PREFIX + name].cast(states[name].dtype).contiguous() for name in names]
    Tensor.realize(out, *nexts)
    Tensor.realize(*[states[name].assign(nxt) for name, nxt in zip(names, nexts, strict=True)])
    return out,
  return run_policy


def onnx_input_dtypes(model_path) -> dict[str, np.dtype]:
  from tinygrad.nn.onnx import OnnxPBParser
  graph = OnnxPBParser(model_path).parse()["graph"]
  out = {}
  for value in graph["input"]:
    elem = value["type"]["tensor_type"]["elem_type"]
    if elem not in ELEM_DTYPES:
      raise ValueError(f"input {value['name']}: unsupported ONNX element type {elem}")
    out[value["name"]] = np.dtype(ELEM_DTYPES[elem])
  return out


def main():
  from tinygrad.nn.onnx import OnnxRunner
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
  p = argparse.ArgumentParser()
  p.add_argument('--model-size', type=_parse_size, required=True, help='model input WxH')
  p.add_argument('--camera-resolutions', type=_parse_size, nargs='+', required=True, help='camera resolutions WxH (one or more)')
  p.add_argument('--onnx', required=True)
  p.add_argument('--output', required=True)
  p.add_argument('--frame-skip', type=int, required=True, help='only for the warp, which takes the same argument as the stock one')
  args = p.parse_args()

  model_path = read_file_chunked_to_disk(args.onnx)
  model_w, model_h = args.model_size
  metadata = make_metadata_dict(model_path)
  input_shapes = metadata['input_shapes']
  if not is_state_model(input_shapes):
    raise SystemExit("not a state-queue model (no new_img input); use compile_modeld.py")
  dtypes = onnx_input_dtypes(model_path)
  state_dtypes = {name: dtypes[name] for name in state_names(input_shapes)}
  out = {'metadata': metadata, 'state_dtypes': {k: v.str for k, v in state_dtypes.items()}}

  model_runner = OnnxRunner(model_path)
  run_policy_jit = TinyJit(make_run_state_policy(model_runner, metadata, dtypes), prune=True)
  make_policy_queues = partial(make_state_input_queues, input_shapes, state_dtypes)
  new_img = tuple(input_shapes['new_img'])
  make_random_model_inputs = partial(make_random_images, keys=['warped'], shape=new_img, device=WARP_DEV)
  out['run_policy'] = compile_jit(run_policy_jit, make_random_model_inputs, policy_inputs(input_shapes), make_policy_queues)

  for cam_w, cam_h in args.camera_resolutions:
    nv12 = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
    make_random_warp_inputs = partial(make_random_images, keys=['frame', 'big_frame'], shape=nv12.size, device=WARP_DEV)
    warp = TinyJit(make_warp(nv12, model_w, model_h, args.frame_skip), prune=True)
    out[(cam_w, cam_h)] = compile_jit(warp, make_random_warp_inputs, WARP_INPUTS, partial(make_state_warp_queues, input_shapes))

  with open(args.output, "wb") as f:
    dump_oob(out, f)
  validate_model_file(args.output)
  print(f"Saved JITs to {args.output} ({os.path.getsize(args.output) / 1e6:.2f} MB)")


if __name__ == "__main__":
  main()
