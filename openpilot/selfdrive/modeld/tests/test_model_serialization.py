import io
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.modeld.helpers import dump_oob, load_oob, validate_model_file


def model_bytes():
  stream = io.BytesIO()
  dump_oob({'weights': np.arange(8, dtype=np.uint8), 'bias': np.arange(4, dtype=np.float32)}, stream)
  return stream.getvalue()


def test_model_buffers_round_trip():
  result = load_oob(io.BytesIO(model_bytes()))
  np.testing.assert_array_equal(result['weights'], np.arange(8, dtype=np.uint8))
  np.testing.assert_array_equal(result['bias'], np.arange(4, dtype=np.float32))


@pytest.mark.parametrize('missing', [1, 3, 16])
def test_truncated_model_buffer_is_rejected(missing):
  with pytest.raises(EOFError, match='incomplete model buffer'):
    load_oob(io.BytesIO(model_bytes()[:-missing]))


@pytest.mark.parametrize('suffix', [b'\x00', b'extra', b'\x00' * 8])
def test_compiled_model_validation_rejects_trailing_data(tmp_path: Path, suffix):
  path = tmp_path / 'model.pkl'
  path.write_bytes(model_bytes() + suffix)
  with pytest.raises(ValueError, match='unexpected model buffer data'):
    validate_model_file(path)


def test_compiled_model_validation(tmp_path: Path):
  path = tmp_path / 'model.pkl'
  path.write_bytes(model_bytes())
  validate_model_file(path)
  path.write_bytes(model_bytes()[:-1])
  with pytest.raises(EOFError, match='incomplete model buffer'):
    validate_model_file(path)


COMPILE_TINY_JIT = r'''
import sys
import numpy as np
from tinygrad import Tensor, TinyJit
from openpilot.selfdrive.modeld.helpers import dump_oob
weight = Tensor(np.full(16, float(sys.argv[1]), dtype=np.float32), device="CPU").realize()

@TinyJit
def run(x):
  return (x * weight + 1).realize()

for _ in range(3):
  run(Tensor(np.arange(16, dtype=np.float32), device="CPU").realize())
with open(sys.argv[2], "wb") as f:
  dump_oob(run, f)
'''


def test_models_loaded_in_one_process_keep_their_own_buffers(tmp_path: Path):
  """Two model files compiled in separate processes number their buffers alike; loading the second one in the same
  process (a live model switch) must not hand it the first one's buffers."""
  import os
  import subprocess
  import sys
  from tinygrad import Tensor
  env = {**os.environ, "DEV": "CPU", "PYTHONPATH": os.pathsep.join(p for p in sys.path if p)}
  paths = []
  for weight in ("2", "3"):
    path = tmp_path / f"model_{weight}.pkl"
    subprocess.run([sys.executable, "-c", COMPILE_TINY_JIT, weight, str(path)], check=True, env=env, cwd=tmp_path)
    paths.append(path)
  with open(paths[0], "rb") as f:
    first = load_oob(f)
  with open(paths[1], "rb") as f:
    second = load_oob(f)
  x = np.arange(16, dtype=np.float32)
  for _ in range(2):
    np.testing.assert_allclose(second(Tensor(x, device="CPU").realize()).numpy(), x * 3 + 1)
    np.testing.assert_allclose(first(Tensor(x, device="CPU").realize()).numpy(), x * 2 + 1)
