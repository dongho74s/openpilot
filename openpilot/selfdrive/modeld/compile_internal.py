#!/usr/bin/env python3
"""Compile a driving model the user selected in the carrot web (internal_model_manager runs this on the device).

  compile_internal.py queue|state <compile_modeld.py / compile_modeld_state.py arguments>

Runs compile_modeld.py (the stock queue layout) or compile_modeld_state.py (history queues inside the ONNX) with
one change: on QCOM a kernel whose compiled program needs more than PVTMEM_LIMIT bytes of private memory per thread is
compiled again without optimizations (NOOPT), and the smaller one is kept. Why: in Polaris Vega/Vega 2 and CH Driving
the desire stage gives a fully upcast kernel (E_9_25_*) that spills 3232 bytes per thread; the first such program to
run never finishes on the comma's GPU ("Wait timeout"), while the stock model's kernels need at most 60 bytes. The
program is checked before it becomes a runtime, so a rejected one never reaches the GPU.

Kept out of compile_modeld.py on purpose: the SConscript builds the stock model with that file, and the stock kernels
are all far below the limit anyway.
"""
import os
import runpy
import struct
import sys

# no BEAM search: it would run candidate kernels before keep_kernels_small sees them
os.environ["BEAM"] = os.environ["JITBEAM"] = "0"

MODELD_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = {"queue": "compile_modeld.py", "state": "compile_modeld_state.py"}
PVTMEM_LIMIT = 256  # bytes per thread; the stock model's kernels use at most 60
LARGE_CONST_GATHER = 32  # constant indices beyond this are gathered with a one-hot matmul


def qcom_pvtmem(lib: bytes) -> int | None:
  """Private memory per thread of a QCOM (Adreno blob) program, read like ops_qcom.QCOMProgram._parse_lib."""
  try:
    image_desc_off = struct.unpack_from("<I", lib, 0x110)[0]
    return struct.unpack_from("<I", lib, image_desc_off + 0xc8)[0]
  except struct.error:
    return None


def keep_kernels_small(limit: int = PVTMEM_LIMIT, log=print) -> None:
  import tinygrad.engine.realize as realize
  from tinygrad.helpers import Context
  orig = realize.to_program

  def to_program(ast, renderer):
    prg = orig(ast, renderer)
    if renderer.target.device.split(":")[0] != "QCOM" or type(renderer).__name__ != "QCOMCLRenderer" or len(prg.src) < 5:
      return prg
    pvt = qcom_pvtmem(prg.src[4].arg)
    if pvt is None or pvt <= limit:
      return prg
    with Context(NOOPT=1):
      small = orig(ast, renderer)
    small_pvt = qcom_pvtmem(small.src[4].arg)
    keep = small if small_pvt is not None and small_pvt < pvt else prg
    log(f"compile_internal: {prg.arg.name} needs {pvt} B private memory per thread; without optimizations {small_pvt} B "
        f"-> using {'the unoptimized' if keep is small else 'the optimized'} kernel", flush=True)
    return keep

  realize.to_program = to_program


def gather_large_constant_indices_by_matmul(limit: int = LARGE_CONST_GATHER) -> None:
  """ONNX Gather with many constant indices as a one-hot matmul.

  tinygrad gathers constant indices as a concatenation of one-element slices, which fuses into one kernel with a
  gated load per index. Polaris Vega/Vega 2 and CH Driving select their desire-history windows with 225 or 300
  constant indices: that kernel compiles to a 184 KB shader that the comma's GPU never finishes ("Wait timeout"). The
  matmul with a constant 0/1 matrix gives the same values (each output is 1 * x + zeros) as a small reduce kernel.
  Other gathers (tensor indices, few constants, non-float data, indices out of range) keep tinygrad's implementation.
  Unlike a copy, the matmul turns a NaN or Inf anywhere in the gathered axis into NaN and may round float32 data to
  half under IMAGE=1 FLOAT16=1; the gathers it replaces in the catalog models (Codex-checked) all read the 0/1
  desire history."""
  import numpy as np
  import tinygrad.nn.onnx as onnx
  from tinygrad.dtype import dtypes
  from tinygrad.tensor import Tensor
  orig = onnx.onnx_ops["Gather"]

  def Gather(x, indices, axis=0):
    if isinstance(indices, (Tensor, int)) or len(indices) <= limit or not dtypes.is_float(x.dtype):
      return orig(x, indices, axis)
    axis = x._resolve_dim(axis)
    n = x.shape[axis]
    if not all(isinstance(i, int) and -n <= i < n for i in indices):
      return orig(x, indices, axis)  # out of range: tinygrad's own gather rejects it
    index = [n + i if i < 0 else i for i in indices]
    onehot = Tensor(np.eye(n, dtype=np.float32)[index].T.copy()).cast(x.dtype)  # (n, k)
    rest = [d for d in range(x.ndim) if d != axis]
    y = x.permute(*rest, axis) @ onehot                                        # (..., k)
    back = list(range(len(rest)))
    back.insert(axis, len(rest))
    return y.permute(*back).cast(x.dtype)  # the image matmul may compute in another dtype

  onnx.onnx_ops["Gather"] = Gather


def main(argv: list[str]) -> None:
  if len(argv) < 2 or argv[1] not in SCRIPTS:
    raise SystemExit(f"usage: {argv[0]} {'|'.join(SCRIPTS)} <compile arguments>")
  script = os.path.join(MODELD_DIR, SCRIPTS[argv[1]])
  sys.argv = [script, *argv[2:]]
  # import the compiler module first: it patches tinygrad (firmware fetch, Buffer pickling) on import
  import openpilot.selfdrive.modeld.compile_modeld  # noqa: F401
  keep_kernels_small()
  gather_large_constant_indices_by_matmul()
  runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
  main(sys.argv)
