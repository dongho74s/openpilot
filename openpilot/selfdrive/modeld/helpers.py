import io
import json
import os
import pickle
import shutil
import struct
import tempfile
import time
from collections.abc import Collection
from pathlib import Path
from typing import TypeVar

from openpilot.common.file_chunker import get_manifest_path
from openpilot.selfdrive.modeld.big_model import MAX_MODEL_SIZE, selected_manifest

MODELS_DIR = Path(__file__).resolve().parent / 'models'
TG_INPUT_DEVICES_PATH = MODELS_DIR / 'tg_input_devices.json'
MAX_OOB_METADATA_SIZE = 64 << 20
MAX_OOB_BUFFER_SIZE = MAX_MODEL_SIZE
USBGPU_USB_IDS = ((0xADD1, 0x0001), (0x3801, 0x0001))
USBGPU_MIN_SPEED_MBPS = 5000
USBGPU_TRANSIENT_INIT_TEXT = (
  "pcie link not up",
  "pcie power off failed",
  "pcie power on failed",
  "usb bridge reset failed",
  "read(0xb450",
  "f0 out failed: -1",
  "libusb_open: no such device",
  "amd:0 does not exist",
  "bulk out 0x02 failed: input/output error",
)
VisionStreamT = TypeVar("VisionStreamT")


def select_vision_streams(available_streams: Collection[VisionStreamT], road_stream: VisionStreamT,
                          wide_stream: VisionStreamT, use_wide_camera: bool) -> tuple[VisionStreamT | None, bool]:
  """Select modeld camera inputs without waiting on a disabled wide camera."""
  if road_stream in available_streams:
    return road_stream, use_wide_camera and wide_stream in available_streams
  if use_wide_camera and wide_stream in available_streams:
    return wide_stream, False
  return None, False


def _default_tg_input_devices(process_name: str, usbgpu: bool):
  backend = os.getenv("DEV", "QCOM")
  defaults = {
    'openpilot.selfdrive.modeld.modeld': {
      'default': {'WARP_DEV': backend, 'QUEUE_DEV': backend},
      'usbgpu': {'WARP_DEV': backend, 'QUEUE_DEV': 'AMD'},
    },
    'openpilot.selfdrive.modeld.dmonitoringmodeld': {
      'default': {'DEV': backend},
    },
  }
  return defaults[process_name]['default' if not usbgpu else 'usbgpu']


def get_tg_input_devices(process_name: str, usbgpu: bool):
  try:
    with open(TG_INPUT_DEVICES_PATH) as f:
      return json.load(f)[process_name]['default' if not usbgpu else 'usbgpu']
  except FileNotFoundError:
    return _default_tg_input_devices(process_name, usbgpu)

def modeld_pkl_path(usbgpu: bool, model_sha256: str | None = None):
  if not usbgpu:
    return MODELS_DIR / 'driving_tinygrad.pkl'
  if model_sha256 is None:
    model = selected_manifest()
    model_sha256 = model.sha256 if model is not None else 'unavailable'
  return MODELS_DIR / f'big_driving_{model_sha256[:16]}_tinygrad.pkl'

def dump_oob(obj, f):
  with tempfile.TemporaryFile(dir=".") as tmp:
    def buffer_callback(pb: pickle.PickleBuffer):
      m = pb.raw()
      tmp.write(struct.pack('<q', m.nbytes))
      tmp.write(m)
      pb.release() # keep peak ram at ~1 buffer
    stream = io.BytesIO()
    pickle.Pickler(stream, protocol=5, buffer_callback=buffer_callback).dump(obj)
    opcodes = stream.getvalue()
    f.write(struct.pack('<q', len(opcodes)))
    f.write(opcodes)
    tmp.seek(0)
    shutil.copyfileobj(tmp, f)

class _ModelUnpickler(pickle.Unpickler):
  """Unpickle a compiled model with buffer identities of this process.

  tinygrad dedupes UOps by value (UOpMetaClass.ucache), and a buffer UOp is identified by its UNIQUE number, device and
  size. Every model file numbers its buffers from 0 in the process that compiled it, so a second file loaded in the
  same process (a live model switch, Jetlink's prepared warp) received the first file's buffer for each number and
  size both use, and its own data was dropped: the model ran with part of the other model's weights. Each UNIQUE of a
  file gets a fresh number from this process' counter instead (the same file-local number always maps to the same new
  one), so its buffers never collide with buffers already alive here or created later."""
  def __init__(self, file, *, buffers):
    super().__init__(file, buffers=buffers)
    self._uniques: dict[int, int] = {}

  def find_class(self, module, name):
    found = super().find_class(module, name)
    if module != "tinygrad.uop.ops" or name != "UOp":
      return found
    from tinygrad.uop.ops import Ops, UOp
    uniques = self._uniques

    def make_uop(op, *args):  # UOp.__reduce__: (op, dtype, src, arg, tag, metadata[, buffer])
      if op is Ops.UNIQUE and len(args) >= 3 and isinstance(args[2], int):
        args = (args[0], args[1], uniques.setdefault(args[2], next(UOp.unique_num)), *args[3:])
      return UOp(op, *args)
    return make_uop


def load_oob(f):
  header = f.read(8)
  if len(header) != 8:
    raise ValueError("incomplete model metadata header")
  size, = struct.unpack('<q', header)
  if not 0 < size <= MAX_OOB_METADATA_SIZE:
    raise ValueError("invalid model metadata size")
  opcodes = f.read(size)
  if len(opcodes) != size:
    raise ValueError("incomplete model metadata")

  # Keep buffer I/O failures distinct from malformed pickle metadata.
  class IncompleteModelBuffer(EOFError):
    pass

  def buffers():
    prev = None
    while (h := f.read(8)):
      if len(h) != 8:
        raise IncompleteModelBuffer("incomplete model buffer header")
      size, = struct.unpack('<q', h)
      if not 0 <= size <= MAX_OOB_BUFFER_SIZE:
        raise ValueError("invalid model buffer size")
      if prev is not None:
        prev.release()
      buf = bytearray(size)
      if f.readinto(buf) != len(buf):
        raise IncompleteModelBuffer("incomplete model buffer")
      prev = pickle.PickleBuffer(buf)
      yield prev
  try:
    return _ModelUnpickler(io.BytesIO(opcodes), buffers=buffers()).load()
  except IncompleteModelBuffer:
    raise
  except (pickle.UnpicklingError, EOFError, OverflowError) as e:
    raise ValueError("invalid model metadata") from e


def validate_model_file(path: str | Path) -> None:
  with open(path, "rb") as f:
    load_oob(f)
    if f.read(1):
      raise ValueError("unexpected model buffer data")


def usb_device_present(usb_ids: Collection[tuple[int, int]], min_speed_mbps: int = 0) -> bool:
  for d in Path("/sys/bus/usb/devices").glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      speed_mbps = float((d / "speed").read_text()) if min_speed_mbps > 0 else 0
      if usb_id in usb_ids and speed_mbps >= min_speed_mbps:
        return True
    except Exception:
      pass
  return False


def usbgpu_present() -> bool:
  # The custom bridge can remain enumerated over USB 2.0 while its SuperSpeed
  # link is unavailable. That state cannot support PCIe model loading.
  return usb_device_present(USBGPU_USB_IDS, USBGPU_MIN_SPEED_MBPS)


def wait_for_usbgpu_present(timeout: float, poll_interval: float = 0.1) -> bool:
  """Wait briefly for a remembered eGPU bridge to enumerate at SuperSpeed."""
  if usbgpu_present():
    return True

  deadline = time.monotonic() + max(0.0, timeout)
  poll_interval = max(0.01, poll_interval)
  while (remaining := deadline - time.monotonic()) > 0.0:
    time.sleep(min(poll_interval, remaining))
    if usbgpu_present():
      return True
  return False


def refresh_usbgpu_device_cache() -> None:
  """Discard libusb device pointers that may go stale during USB re-enumeration."""
  try:
    from tinygrad.runtime.support.usb import USB3
    USB3.list_devices.__func__.cache_clear()
  except (AttributeError, ImportError):
    pass


def usbgpu_pcie_not_ready(error: BaseException | str) -> bool:
  """Recognize the transient state between USB enumeration and PCIe link-up."""
  if isinstance(error, str):
    message = error.lower()
    return any(marker in message for marker in USBGPU_TRANSIENT_INIT_TEXT)

  pending: list[BaseException] = [error]
  seen: set[int] = set()
  while pending:
    current = pending.pop()
    if id(current) in seen:
      continue
    seen.add(id(current))
    message = str(current).lower()
    if any(marker in message for marker in USBGPU_TRANSIENT_INIT_TEXT):
      return True

    # tinygrad tries KFD, native PCIe, and USB PCIe backends together. When all
    # fail it reports an ExceptionGroup, so the transient USB link error is one
    # of the child exceptions rather than the direct cause/context.
    children = getattr(current, "exceptions", ())
    pending.extend(child for child in children if isinstance(child, BaseException))
    if current.__cause__ is not None:
      pending.append(current.__cause__)
    if current.__context__ is not None:
      pending.append(current.__context__)
  return False


def active_usbgpu_compiled_path() -> Path | None:
  model = selected_manifest()
  if model is None:
    return None
  from openpilot.selfdrive.modeld.precompiled_model import installed
  if (precompiled := installed(model)) is not None:
    return precompiled
  if model.precompiled_only:
    return None
  path = modeld_pkl_path(usbgpu=True, model_sha256=model.sha256)
  return path if Path(get_manifest_path(path)).is_file() else None


def usbgpu_compile_pending() -> bool:
  model = selected_manifest()
  if model is None:
    return False
  from openpilot.selfdrive.modeld.precompiled_model import installed
  if installed(model) is not None:
    return False
  path = modeld_pkl_path(usbgpu=True, model_sha256=model.sha256)
  return not Path(get_manifest_path(path)).is_file()


def usbgpu_compiled_path() -> Path | None:
  return active_usbgpu_compiled_path()


def usbgpu_compiled() -> bool:
  return usbgpu_compiled_path() is not None
