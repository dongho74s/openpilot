"""Selectable internal driving model: catalog, contract check, modeld selection/fallback and the parked manager."""
import json
import os
from pathlib import Path

import pytest

from openpilot.selfdrive.modeld import internal_model as im
from openpilot.selfdrive.modeld import internal_model_manager as imm

RDF = im.find("RDF")
CAM = (1344, 760)


@pytest.fixture(autouse=True)
def model_dirs(monkeypatch, tmp_path):
  monkeypatch.setenv("CARROT_INTERNAL_MODEL_DIR", str(tmp_path / "internal"))
  models_dir = tmp_path / "models"
  models_dir.mkdir()
  (models_dir / ".build_stamp").write_text("stamp-1")
  monkeypatch.setattr(im, "MODELS_DIR", models_dir)
  return tmp_path


def stock_metadata(**changes):
  widths = {**im.REQUIRED_HEADS, **im.OPTIONAL_HEADS}
  slices, start = {}, 0
  for name, width in widths.items():
    slices[name] = slice(start, start + width)
    start += width
  slices["pad"] = slice(-2, None)
  meta = {"input_shapes": dict(im.REQUIRED_INPUTS), "output_shapes": {"outputs": (1, start + 2)}, "output_slices": slices}
  meta.update(changes)
  return meta


def compile_ok(model, stamp="stamp-1", cameras=(CAM,)):
  im.model_dir(model).mkdir(parents=True, exist_ok=True)
  im.pkl_path(model).write_bytes(b"x" * 10)
  im.write_json(im.compiled_path(model), {"sha256": model.sha256, "stamp": stamp, "cameras": [list(c) for c in cameras], "pkl_size": 10})


def test_catalog_has_one_stock_model_and_pinned_https_downloads():
  assert im.stock_model().id == im.STOCK_ID and sum(m.stock for m in im.CATALOG) == 1
  assert len({m.id for m in im.CATALOG}) == len(im.CATALOG)
  for m in im.CATALOG:
    assert len(m.sha256) == 64 and int(m.sha256, 16) >= 0 and m.size > 0
    assert m.stock or m.url.startswith("https://")


def test_contract_accepts_stock_layout_with_or_without_action():
  assert im.contract_errors(stock_metadata()) == []
  meta = stock_metadata()
  del meta["output_slices"]["action"]
  assert im.contract_errors(meta) == []


@pytest.mark.parametrize("change,expected", [
  ({"input_shapes": {**im.REQUIRED_INPUTS, "features_buffer": (1, 28, 512)}}, "input features_buffer shape"),
  # a 32-frame history needs the matching 33-entry desire_pulse
  ({"input_shapes": {**im.REQUIRED_INPUTS, "features_buffer": (1, 32, 512)}}, "input desire_pulse shape (1, 25, 8) != (1, 33, 8)"),
  ({"input_shapes": {**im.REQUIRED_INPUTS, "lateral_control_params": (1, 2)}}, "unexpected input lateral_control_params"),
  ({"output_shapes": {"a": (1, 10), "b": (1, 10)}}, "expected one output"),
])
def test_contract_rejects_other_inputs_and_outputs(change, expected):
  assert any(expected in e for e in im.contract_errors(stock_metadata(**change)))


def state_metadata(history=32, **changes):
  meta = stock_metadata()
  inputs = im.state_history_inputs(history)
  meta["input_shapes"] = dict(inputs)
  meta["output_shapes"] = {**meta["output_shapes"], **{f"next_{k}": v for k, v in inputs.items() if k.startswith("state_")}}
  meta.update(changes)
  return meta


@pytest.mark.parametrize("history", im.QUEUE_HISTORIES)
def test_contract_accepts_the_queue_layout_with_a_24_or_32_frame_history(history):
  meta = stock_metadata(input_shapes=im.queue_history_inputs(history))
  assert im.contract_kind(meta) == "queue" and im.contract_errors(meta) == []


@pytest.mark.parametrize("history", im.QUEUE_HISTORIES)
def test_contract_accepts_the_state_queue_layout(history):
  meta = state_metadata(history)
  assert im.contract_kind(meta) == "state" and im.contract_errors(meta) == []
  assert meta["input_shapes"]["state_feat_q"] == (4 * history, 1, 512)
  assert meta["input_shapes"]["state_desire_q"] == (4 * (history + 1), 1, 8)


def test_contract_rejects_broken_state_layouts():
  meta = state_metadata()
  del meta["output_shapes"]["next_state_feat_q"]
  assert "missing output next_state_feat_q" in im.contract_errors(meta)
  meta = state_metadata()
  meta["output_shapes"]["next_state_desire_q"] = (100, 1, 8)
  assert any("output next_state_desire_q shape" in e for e in im.contract_errors(meta))
  meta = state_metadata()
  meta["input_shapes"]["state_img_q"] = (2, 9, 6, 128, 256)
  assert any("input state_img_q shape" in e for e in im.contract_errors(meta))
  meta = state_metadata()
  meta["input_shapes"]["state_feat_q"] = (112, 1, 512)   # 28 frames: not a history modeld knows
  assert any("input state_feat_q shape" in e for e in im.contract_errors(meta))
  meta = state_metadata()
  meta["output_shapes"]["debug"] = (1, 3)
  meta["input_shapes"]["lateral_control_params"] = (1, 2)
  errors = im.contract_errors(meta)
  assert "unexpected output debug" in errors and "unexpected input lateral_control_params" in errors
  meta = state_metadata()
  meta["output_slices"]["plan"] = slice(0, 900)
  assert any("head plan width 900" in e for e in im.contract_errors(meta))


def test_contract_rejects_wrong_head_width_missing_head_and_unknown_head():
  meta = stock_metadata()
  meta["output_slices"]["meta"] = slice(0, 53)
  del meta["output_slices"]["plan"]
  meta["output_slices"]["mystery"] = slice(0, 1)
  errors = im.contract_errors(meta)
  assert any("head meta width 53" in e for e in errors)
  assert any("missing head plan" in e for e in errors)
  assert any("unexpected head mystery" in e for e in errors)


def test_stock_request_or_garbage_uses_stock():
  assert im.select_for_modeld(*CAM) is None
  im.request_path().write_text("{not json")
  assert im.select_for_modeld(*CAM) is None
  im.write_json(im.request_path(), {"id": "NOPE"})
  assert im.select_for_modeld(*CAM) is None
  assert im.read_json(im.runtime_path())["active_id"] == im.STOCK_ID


@pytest.mark.parametrize("prepare,reason", [
  (lambda: None, "not_compiled"),
  (lambda: compile_ok(RDF, stamp="old"), "stale_build"),
  (lambda: compile_ok(RDF, cameras=((1928, 1208),)), "camera"),
  (lambda: (compile_ok(RDF), im.pkl_path(RDF).write_bytes(b"short")), "missing"),
])
def test_unusable_artifact_falls_back_with_a_reason(prepare, reason):
  im.write_request(RDF)
  prepare()
  assert im.select_for_modeld(*CAM) is None
  runtime = im.read_json(im.runtime_path())
  assert runtime["active_id"] == im.STOCK_ID and runtime["fallback_reason"] == reason


def test_missing_build_stamp_never_uses_a_custom_model():
  im.write_request(RDF)
  compile_ok(RDF)
  (im.MODELS_DIR / ".build_stamp").unlink()
  assert im.select_for_modeld(*CAM) is None


def start(internal_runs=True, loaded=True):
  """One modeld start that stops (or crashes) without reaching healthy."""
  selected = im.select_for_modeld(*CAM)
  if selected is not None and loaded:
    im.mark_loaded(selected)
    im.HealthTracker(selected).attempt(internal_runs)
  return selected


def test_ready_model_is_selected_and_tracked_from_its_first_inference_until_healthy():
  im.write_request(RDF)
  compile_ok(RDF)
  selected = im.select_for_modeld(*CAM)
  assert selected is not None and selected.pkl_path == im.pkl_path(RDF)
  assert im.read_json(im.runtime_path())["loading"] is True
  im.mark_loaded(selected)
  runtime = im.read_json(im.runtime_path())
  assert runtime["loading"] is False and runtime["pending"] is False and runtime["used"] is False
  health = im.HealthTracker(selected)
  health.attempt()
  assert im.read_json(im.runtime_path())["pending"] is True and im.read_json(im.runtime_path())["used"] is True
  for _ in range(im.HEALTHY_PUBLISHES):
    health.publish()
  runtime = im.read_json(im.runtime_path())
  assert runtime["pending"] is False and runtime["unhealthy_starts"] == 0 and runtime["active_id"] == "RDF"


def test_two_unhealthy_starts_block_the_model_until_selected_again():
  im.write_request(RDF)
  compile_ok(RDF)
  assert start() is not None        # start 1, crashes before healthy
  assert start() is not None        # start 2 (1 unhealthy so far), crashes again
  assert im.select_for_modeld(*CAM) is None       # 2 unhealthy starts: stock
  assert im.read_json(im.runtime_path())["fallback_reason"] == "crashed"
  assert im.select_for_modeld(*CAM) is None       # stays blocked
  im.reset_runtime_block(RDF)
  assert im.select_for_modeld(*CAM) is not None


def test_native_crash_while_loading_counts_as_unhealthy():
  im.write_request(RDF)
  compile_ok(RDF)
  assert start(loaded=False) is not None
  assert start(loaded=False) is not None
  assert im.select_for_modeld(*CAM) is None
  assert im.read_json(im.runtime_path())["fallback_reason"] == "crashed"


def test_starts_where_only_an_external_model_ran_never_block_the_selected_model():
  im.write_request(RDF)
  compile_ok(RDF)
  for _ in range(4):
    assert start(internal_runs=False) is not None
  runtime = im.read_json(im.runtime_path())
  assert runtime["unhealthy_starts"] == 0 and runtime["used"] is False


def test_load_failure_blocks_immediately():
  im.write_request(RDF)
  compile_ok(RDF)
  selected = im.select_for_modeld(*CAM)
  im.record_load_failure(selected)
  assert im.select_for_modeld(*CAM) is None
  assert im.read_json(im.runtime_path())["fallback_reason"] == "load_failed"


def test_select_for_modeld_never_raises(monkeypatch):
  monkeypatch.setattr(im, "requested_model", lambda: 1 / 0)
  assert im.select_for_modeld(*CAM) is None


# --- parked manager ------------------------------------------------------------------------------------------
class FakeParams:
  def __init__(self, onroad=False):
    self.onroad = onroad

  def get_bool(self, key):
    return key == "IsOnroad" and self.onroad


def manager(onroad=False):
  return imm.Manager(FakeParams(onroad), cameras=[CAM])


def test_manager_pipeline_downloads_validates_compiles_then_reports_ready(monkeypatch):
  im.write_request(RDF)
  calls = []
  m = manager()
  monkeypatch.setattr(m, "download", lambda model, token: calls.append("download") or im.onnx_path(model))
  monkeypatch.setattr(m, "validate", lambda model, token, path: calls.append("validate"))
  monkeypatch.setattr(m, "compile", lambda model, token, path, stamp: (calls.append(("compile", stamp)), compile_ok(model)))
  m.step()
  assert calls == ["download", "validate", ("compile", "stamp-1")]
  assert im.read_json(im.status_path())["phase"] == "ready"
  m.step()   # already compiled for this build: nothing to do
  assert len(calls) == 3


def test_manager_contract_failure_is_not_retried_until_selected_again(monkeypatch):
  im.write_request(RDF, now=1)
  m = manager()
  calls = []
  monkeypatch.setattr(m, "download", lambda model, token: im.onnx_path(model))
  def bad(model, token, path):
    calls.append(1)
    raise imm.StepError("contract", "missing head plan")
  monkeypatch.setattr(m, "validate", bad)
  m.step()
  m.step()
  status = im.read_json(im.status_path())
  assert status["phase"] == "error" and status["error_code"] == "contract" and calls == [1]
  im.write_request(RDF, now=2)   # a new selection retries
  m.step()
  assert calls == [1, 1]


def test_manager_stops_work_when_the_ignition_turns_on(monkeypatch):
  im.write_request(RDF)
  m = manager(onroad=True)
  monkeypatch.setattr(m, "download", lambda *a: pytest.fail("must not download onroad"))
  m.step()
  status = im.read_json(im.status_path())
  assert status["phase"] == "error" and status["error_code"] == "waiting_offroad"


def test_manager_recompiles_after_a_build_change(monkeypatch):
  im.write_request(RDF)
  compile_ok(RDF, stamp="stamp-0")
  m = manager()
  monkeypatch.setattr(m, "download", lambda model, token: im.onnx_path(model))
  monkeypatch.setattr(m, "validate", lambda *a: None)
  compiled = []
  monkeypatch.setattr(m, "compile", lambda model, token, path, stamp: (compiled.append(stamp), compile_ok(model, stamp=stamp)))
  m.step()
  assert compiled == ["stamp-1"] and im.artifact_problem(RDF, *CAM) is None


def test_manager_writes_stock_status_for_stock_selection():
  im.write_request(im.stock_model())
  manager().step()
  assert im.read_json(im.status_path())["phase"] == "stock"


def test_real_download_checks_the_pinned_size_and_hash(monkeypatch):
  im.write_request(RDF)
  m = manager()

  def fake_download(manifest, cache_dir, progress_callback=None, phase_callback=None):
    assert manifest.sha256 == RDF.sha256 and manifest.size == RDF.size and manifest.url == RDF.url
    raise OSError(f"model sha256 mismatch: expected {RDF.sha256}, got {'0' * 64}")

  import openpilot.selfdrive.modeld.big_model as big_model
  monkeypatch.setattr(big_model, "_download_model", fake_download)
  with pytest.raises(imm.StepError) as e:
    m.download(RDF, 1)
  assert e.value.code == "sha"


def test_compile_command_uses_the_stock_build_flags(monkeypatch, tmp_path):
  m = manager()
  seen = {}

  class FakeProc:
    returncode = 0
    def __init__(self, cmd, **kw):
      seen.update(cmd=cmd, env=kw["env"])
      Path(cmd[cmd.index("--output") + 1]).write_bytes(b"compiled-pkl")
    def poll(self):
      return 0
  monkeypatch.setattr(imm.subprocess, "Popen", FakeProc)
  monkeypatch.setattr(imm.Path, "is_file", lambda self: str(self) == "/TICI" or Path.exists(self))
  monkeypatch.setattr(imm, "compile_core_available", lambda: False)   # parked comma: cores 4-7 offline
  import openpilot.selfdrive.modeld.helpers as helpers
  monkeypatch.setattr(helpers, "validate_model_file", lambda path: pytest.fail("must not load the model (GPU) in the manager"))
  monkeypatch.setattr(imm, "read_metadata", lambda path: stock_metadata())
  im.model_dir(RDF).mkdir(parents=True)
  m.compile(RDF, 1, im.onnx_path(RDF), "stamp-1")
  cmd = seen["cmd"]
  assert cmd[0] != "taskset" and Path(cmd[1]).name == "compile_internal.py" and cmd[2] == "queue"
  assert cmd[cmd.index("--camera-resolutions") + 1] == "1344x760" and cmd[cmd.index("--frame-skip") + 1] == "4"
  assert cmd[cmd.index("--model-size") + 1] == "512x256"
  assert all(seen["env"][k] == v for k, v in imm.QCOM_FLAGS.items())
  compiled = json.loads(im.compiled_path(RDF).read_text())
  assert compiled["stamp"] == "stamp-1" and compiled["cameras"] == [list(CAM)] and im.pkl_path(RDF).read_bytes() == b"compiled-pkl"


def test_a_state_queue_model_is_compiled_with_its_own_compiler(monkeypatch):
  m = manager()
  seen = {}

  class FakeProc:
    returncode = 0
    def __init__(self, cmd, **kw):
      seen.update(cmd=cmd)
      Path(cmd[cmd.index("--output") + 1]).write_bytes(b"compiled-pkl")
    def poll(self):
      return 0
  monkeypatch.setattr(imm.subprocess, "Popen", FakeProc)
  monkeypatch.setattr(imm, "compile_core_available", lambda: False)
  monkeypatch.setattr(imm, "read_metadata", lambda path: state_metadata())
  model = im.find("CHDRIVING")
  im.model_dir(model).mkdir(parents=True)
  m.compile(model, 1, im.onnx_path(model), "stamp-1")
  assert Path(seen["cmd"][1]).name == "compile_internal.py" and seen["cmd"][2] == "state"
  assert all((imm.MODELD_DIR / f).is_file() for f in ("compile_internal.py", "compile_modeld_state.py", "compile_modeld.py"))


def test_compile_internal_keeps_kernels_with_large_private_memory_out(monkeypatch):
  import struct
  import tinygrad.engine.realize as realize
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.modeld import compile_internal as ci

  def lib(pvt):
    b = bytearray(0x400)
    struct.pack_into("<I", b, 0x110, 0x200)
    struct.pack_into("<I", b, 0x200 + 0xc8, pvt)
    return bytes(b)
  assert ci.qcom_pvtmem(lib(3232)) == 3232 and ci.qcom_pvtmem(b"short") is None

  from tinygrad.helpers import NOOPT
  calls = []
  def orig(ast, renderer):
    noopt = bool(NOOPT.value)
    calls.append((ast, noopt))
    pvt = {"big": 3232 if not noopt else 120, "small": 60, "worse": 400}[ast]
    return NS(src=(None, None, None, None, NS(arg=lib(pvt))), arg=NS(name=ast))
  monkeypatch.setattr(realize, "to_program", orig)
  qcom = type("QCOMCLRenderer", (), {})()
  qcom.target = NS(device="QCOM")
  logs = []
  ci.keep_kernels_small(log=lambda *a, **k: logs.append(a[0]))
  assert ci.qcom_pvtmem(realize.to_program("small", qcom).src[4].arg) == 60          # small kernels: compiled once
  assert ci.qcom_pvtmem(realize.to_program("big", qcom).src[4].arg) == 120           # recompiled without optimizations
  assert calls[-2:] == [("big", False), ("big", True)] and "needs 3232 B" in logs[-1]
  cpu = type("ClangRenderer", (), {})()
  cpu.target = NS(device="CPU")
  assert ci.qcom_pvtmem(realize.to_program("big", cpu).src[4].arg) == 3232           # other devices untouched


def test_unreadable_metadata_at_compile_is_a_contract_error(monkeypatch):
  def broken(path):
    raise ValueError("truncated")
  monkeypatch.setattr(imm, "read_metadata", broken)
  im.model_dir(RDF).mkdir(parents=True)
  with pytest.raises(imm.StepError) as e:
    manager().compile(RDF, 1, im.onnx_path(RDF), "stamp-1")
  assert e.value.code == "contract"


@pytest.mark.parametrize("model_id,env", [("CHDRIVING", "CARROT_TEST_CH_ONNX"), ("POLARISVEGA", "CARROT_TEST_POLARIS_ONNX"),
                                          ("POLARISVEGA2", "CARROT_TEST_POLARIS2_ONNX")])
def test_new_catalog_models_match_a_contract_when_available(model_id, env):
  path = Path(os.getenv(env, "/nonexistent"))
  if not path.is_file():
    pytest.skip(f"{env} not set")
  from openpilot.selfdrive.modeld.internal_model_onnx import read_metadata
  meta = read_metadata(path)
  model = im.find(model_id)
  assert im.contract_errors(meta) == [] and meta["model_checkpoint"] == model.checkpoint
  assert path.stat().st_size == model.size


def test_rdf_onnx_metadata_matches_the_contract_when_available():
  path = Path(os.getenv("CARROT_TEST_RDF_ONNX", "/data/tmp/model-sel/rdf.onnx"))
  if not path.is_file():
    pytest.skip("RDF ONNX not downloaded here")
  from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
  assert im.contract_errors(make_metadata_dict(str(path))) == []


def test_runtime_failure_after_healthy_blocks_the_model_with_its_reason():
  im.write_request(RDF)
  compile_ok(RDF)
  selected = im.select_for_modeld(*CAM)
  health = im.HealthTracker(selected)
  for _ in range(im.HEALTHY_PUBLISHES):
    health.publish()
  im.record_runtime_failure(selected)   # modeld is about to stop on an inference error
  assert im.select_for_modeld(*CAM) is None
  assert im.read_json(im.runtime_path())["fallback_reason"] == "runtime_failed"
  assert im.select_for_modeld(*CAM) is None   # still blocked, same reason
  assert im.read_json(im.runtime_path())["fallback_reason"] == "runtime_failed"


def test_outputs_of_an_external_model_do_not_make_the_selected_model_healthy():
  im.write_request(RDF)
  compile_ok(RDF)
  im.write_json(im.runtime_path(), {"sha256": RDF.sha256, "unhealthy_starts": 1})   # one earlier bad start
  selected = im.select_for_modeld(*CAM)
  im.mark_loaded(selected)
  health = im.HealthTracker(selected)
  health.attempt()                      # it ran once, then an external model took over
  for _ in range(im.HEALTHY_PUBLISHES * 2):
    health.publish(produced_by_internal=False)
  runtime = im.read_json(im.runtime_path())
  assert runtime["unhealthy_starts"] == 1 and health.selected is selected   # not reset to healthy


class FakeChild:
  def __init__(self, polls_before_exit=1000, returncode=0, on_poll=None):
    self.polls, self.returncode, self.on_poll, self.killed = polls_before_exit, None, on_poll, False
    self._code = returncode

  def poll(self):
    if self.on_poll:
      self.on_poll(self)
    if self.killed:
      self.returncode = -9
    elif self.polls <= 0:
      self.returncode = self._code
    self.polls -= 1
    return self.returncode

  def kill(self):
    self.killed = True

  def wait(self, timeout=None):
    return self.returncode


def run_compile(monkeypatch, m, child):
  monkeypatch.setattr(imm.subprocess, "Popen", lambda *a, **k: child)
  monkeypatch.setattr(imm.time, "sleep", lambda s: None)
  im.model_dir(RDF).mkdir(parents=True, exist_ok=True)
  return m.compile(RDF, 1, im.onnx_path(RDF), "stamp-1")


def test_stop_signal_during_compile_kills_the_child_and_leaves_a_resumable_status(monkeypatch):
  monkeypatch.setattr(imm, "read_metadata", lambda path: stock_metadata())   # the compiler is faked, the ONNX absent
  im.write_request(RDF)
  m = manager()
  def signal_arrives(child):
    if child.polls == 998 and not m.stopping:
      m.stopping = True      # what the SIGTERM handler does
      m.kill_child()
  child = FakeChild(on_poll=signal_arrives)
  m.status(RDF, 1, "compiling")
  with pytest.raises(imm.Stop):
    run_compile(monkeypatch, m, child)
  assert child.killed and m.child is None
  assert not im.compiled_path(RDF).exists() and not im.pkl_path(RDF).exists()
  m.mark_paused()
  status = im.read_json(im.status_path())
  assert status["phase"] == "error" and status["error_code"] == "waiting_offroad"


def test_ignition_on_as_the_compiler_exits_does_not_finish_the_artifact(monkeypatch):
  monkeypatch.setattr(imm, "read_metadata", lambda path: stock_metadata())   # the compiler is faked, the ONNX absent
  im.write_request(RDF)
  params = FakeParams()
  m = imm.Manager(params, cameras=[CAM])
  out = im.pkl_path(RDF).with_name(im.PKL_NAME + ".tmp")
  def exits_while_ignition_turns_on(child):
    if child.polls == 0:
      out.write_bytes(b"compiled-pkl")
      params.onroad = True
  with pytest.raises(imm.StepError) as e:
    run_compile(monkeypatch, m, FakeChild(polls_before_exit=0, on_poll=exits_while_ignition_turns_on))
  assert e.value.code == "waiting_offroad"
  assert not out.exists() and not im.pkl_path(RDF).exists() and not im.compiled_path(RDF).exists()


def test_a_handover_to_an_external_model_then_a_normal_stop_is_not_a_failure():
  im.write_request(RDF)
  compile_ok(RDF)
  for _ in range(4):   # each start: 3 internal outputs, then Jetlink takes over, then the manager stops modeld
    selected = im.select_for_modeld(*CAM)
    assert selected is not None
    im.mark_loaded(selected)
    health = im.HealthTracker(selected)
    for _ in range(3):
      health.attempt(True)
      health.publish(True)
    health.attempt(False)
    health.publish(False)
    assert im.read_json(im.runtime_path())["used"] is False   # the card does not claim it is running
    im.mark_clean_stop()
  assert im.read_json(im.runtime_path())["unhealthy_starts"] == 0


def test_a_crash_when_the_internal_model_takes_back_over_is_counted():
  im.write_request(RDF)
  compile_ok(RDF)
  for _ in range(2):   # internal, Jetlink, then the internal model crashes modeld natively as it takes over again
    selected = im.select_for_modeld(*CAM)
    assert selected is not None
    im.mark_loaded(selected)
    health = im.HealthTracker(selected)
    health.attempt(True)
    health.publish(True)
    health.attempt(False)
    health.publish(False)
    health.attempt(True)   # JoiningModel may still look active here; pending was never cleared anyway
  assert im.select_for_modeld(*CAM) is None
  assert im.read_json(im.runtime_path())["fallback_reason"] == "crashed"


def test_a_normal_stop_before_healthy_is_not_counted():
  im.write_request(RDF)
  compile_ok(RDF)
  for _ in range(3):   # short drives
    selected = im.select_for_modeld(*CAM)
    im.mark_loaded(selected)
    health = im.HealthTracker(selected)
    for _ in range(10):
      health.attempt(True)
      health.publish(True)
    im.mark_clean_stop()
  assert im.select_for_modeld(*CAM) is not None
  assert im.read_json(im.runtime_path())["unhealthy_starts"] == 0


def test_in_use_follows_the_producing_model_after_healthy():
  im.write_request(RDF)
  compile_ok(RDF)
  selected = im.select_for_modeld(*CAM)
  im.mark_loaded(selected)
  health = im.HealthTracker(selected)
  for _ in range(im.HEALTHY_PUBLISHES):
    health.attempt(True)
    health.publish(True)
  assert im.read_json(im.runtime_path())["used"] is True and im.read_json(im.runtime_path())["pending"] is False
  health.publish(False)
  assert im.read_json(im.runtime_path())["used"] is False
  health.publish(True)
  runtime = im.read_json(im.runtime_path())
  assert runtime["used"] is True and runtime["unhealthy_starts"] == 0


def test_modeld_marks_the_managers_sigint_as_a_clean_stop_before_loading_the_model():
  import ast
  tree = ast.parse((Path(im.__file__).parent / "modeld.py").read_text())
  function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "main")
  body = ast.unparse(function)
  assert body.index("install_clean_stop_handler()") < body.index("create_internal_model(vipc_client_main")
  block = next(n for n in ast.walk(function) if isinstance(n, ast.If) and "install_clean_stop_handler" in ast.unparse(n.body))
  installer = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "install_clean_stop_handler")
  handlers, stops = {}, []
  def default_int_handler(signum, frame):
    raise KeyboardInterrupt
  fake_signal = type("S", (), {"SIGINT": 2, "signal": staticmethod(lambda sig, fn: handlers.__setitem__(sig, fn)),
                               "default_int_handler": staticmethod(default_int_handler)})
  for stock, installed in ((True, False), (False, True)):
    handlers.clear()
    internal = type("I", (), {"mark_clean_stop": staticmethod(lambda: stops.append(1)),
                              "requested_model": staticmethod(lambda stock=stock: type("M", (), {"stock": stock}))})
    scope = {"signal": fake_signal, "internal_model": internal}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[installer, block], type_ignores=[])), "<sigint>", "exec"), scope)
    assert (2 in handlers) == installed   # the stock model keeps the default SIGINT behaviour
  with pytest.raises(KeyboardInterrupt):
    handlers[2](2, None)
  assert stops == [1]


def test_a_normal_stop_while_loading_is_not_counted():
  im.write_request(RDF)
  compile_ok(RDF)
  for _ in range(3):
    assert im.select_for_modeld(*CAM) is not None   # loading=True
    im.mark_clean_stop()                            # the manager stopped modeld during the load
  assert im.select_for_modeld(*CAM) is not None
  assert im.read_json(im.runtime_path())["unhealthy_starts"] == 0


def test_egpu_standby_warmup_failure_blocks_the_selected_model(monkeypatch):
  import ast
  import sys
  import types
  im.write_request(RDF)
  compile_ok(RDF)
  selected = im.select_for_modeld(*CAM)
  im.mark_loaded(selected)
  tree = ast.parse((Path(im.__file__).parent / "modeld.py").read_text())
  block = next(n for n in ast.walk(tree) if isinstance(n, ast.If) and isinstance(n.test, ast.Name)
               and n.test.id == "usbgpu_model_loaded" and any(isinstance(b, ast.ImportFrom) and b.module and
               b.module.endswith("internal_model_warmup") for b in n.body))
  def warm(model, client):
    assert im.read_json(im.runtime_path())["loading"] is True   # a native crash here would count
    raise RuntimeError("selected internal model output is not finite")
  monkeypatch.setitem(sys.modules, "openpilot.selfdrive.modeld.internal_model_warmup",
                      types.SimpleNamespace(warm_internal_model_from_camera=warm))
  scope = {"usbgpu_model_loaded": True, "internal_selected": selected, "internal_model": im,
           "small_model": object(), "vipc_client_main": object()}
  with pytest.raises(RuntimeError):
    exec(compile(ast.fix_missing_locations(ast.Module(body=[block], type_ignores=[])), "<warmup>", "exec"), scope)
  assert im.select_for_modeld(*CAM) is None
  assert im.read_json(im.runtime_path())["fallback_reason"] == "runtime_failed"


def test_compile_core_needs_the_core_online_and_allowed(monkeypatch, tmp_path):
  states = {"online": "0\n", "affinity": {0, 1, 2, 3, 7}}
  real_read_text = Path.read_text
  monkeypatch.setattr(imm.Path, "read_text", lambda self, *a, **k: states["online"] if "cpu7/online" in str(self) else real_read_text(self, *a, **k))
  monkeypatch.setattr(imm.os, "sched_getaffinity", lambda pid: states["affinity"], raising=False)
  real_is_file = Path.is_file
  monkeypatch.setattr(imm.Path, "is_file", lambda self: True if str(self) == "/usr/bin/taskset" else real_is_file(self))
  assert imm.compile_core_available() is False          # offline while parked
  states["online"] = "1\n"
  assert imm.compile_core_available() is True
  states["affinity"] = {0, 1, 2, 3, 4, 5}
  assert imm.compile_core_available() is False          # not allowed for the manager's processes


# --- weight patch models (WMI V11) -------------------------------------------------------------------------------
WMIV11 = im.find("WMIV11")


def test_patch_models_are_pinned_and_their_copies_stay_inside_both_files():
  for m in im.CATALOG:
    if m.patch is None:
      continue
    p = m.patch
    assert p.template_sha256 == im.stock_model().sha256 and p.template_size == im.stock_model().size
    assert len(p.part_sha256) == 64 and p.part_size > 0 and m.url.startswith("https://") and not m.stock
    spans = sorted((dst, dst + n) for dst, _src, n in p.copies)
    assert all(n > 0 and dst + n <= p.template_size and src + n <= p.part_size for dst, src, n in p.copies)
    assert all(a_end <= b_start for (_a, a_end), (b_start, _b) in zip(spans, spans[1:], strict=False))


def _patch_model(tmp_path, template: bytes, part: bytes, copies, result: bytes | None = None):
  import dataclasses
  import hashlib
  out = bytearray(template)
  for dst, src, n in copies:
    out[dst:dst + n] = part[src:src + n]
  result = bytes(out) if result is None else result
  stock_onnx = tmp_path / "models" / im.ONNX_NAME
  stock_onnx.write_bytes(template)
  sha = lambda b: hashlib.sha256(b).hexdigest()
  patch = im.WeightPatch(template_sha256=sha(template), template_size=len(template), part_sha256=sha(part),
                         part_size=len(part), copies=tuple(copies))
  return dataclasses.replace(WMIV11, size=len(result), sha256=sha(result), patch=patch), bytes(out)


def test_build_patched_applies_the_copies_and_checks_every_hash(tmp_path):
  model, expected = _patch_model(tmp_path, b"T" * 64, b"0123456789", [(4, 0, 3), (40, 7, 3)])
  part = tmp_path / "part.onnx"
  part.write_bytes(b"0123456789")
  target = tmp_path / "out.onnx"
  imm.build_patched(model, part, target)
  assert target.read_bytes() == expected == b"TTTT012" + b"T" * 33 + b"789" + b"T" * 21


@pytest.mark.parametrize("broken,code", [("template", "template"), ("part", "sha"), ("result", "sha"), ("range", "contract")])
def test_build_patched_rejects_a_changed_stock_model_part_or_result(tmp_path, broken, code):
  copies = [(4, 0, 3)] if broken != "range" else [(62, 0, 3)]
  model, _ = _patch_model(tmp_path, b"T" * 64, b"0123456789", copies,
                          result=b"x" * 64 if broken == "result" else None)
  if broken == "template":
    (tmp_path / "models" / im.ONNX_NAME).write_bytes(b"U" * 64)   # e.g. an update changed the stock model
  part = tmp_path / "part.onnx"
  part.write_bytes(b"0123456789" if broken != "part" else b"9123456789")
  target = tmp_path / "out.onnx"
  with pytest.raises(imm.StepError) as e:
    imm.build_patched(model, part, target)
  assert e.value.code == code and not target.exists() and not target.with_name(target.name + ".tmp").exists()


def test_download_of_a_patch_model_fetches_the_part_then_builds_and_verifies_the_onnx(monkeypatch, tmp_path):
  model, expected = _patch_model(tmp_path, b"T" * 64, b"0123456789", [(4, 0, 3)])
  monkeypatch.setattr(im, "CATALOG", (*[m for m in im.CATALOG if m.id != model.id], model))
  seen = []

  def fake_download(manifest, cache_dir, progress_callback=None, phase_callback=None):
    seen.append(manifest)
    cache_dir.mkdir(parents=True, exist_ok=True)
    path = cache_dir / manifest.cache_filename
    path.write_bytes(b"0123456789")
    return path

  import openpilot.selfdrive.modeld.big_model as big_model
  monkeypatch.setattr(big_model, "_download_model", fake_download)
  m = manager()
  path = m.download(model, 1)
  assert seen[0].sha256 == model.patch.part_sha256 and seen[0].size == model.patch.part_size and seen[0].url == model.url
  assert path == im.onnx_path(model) and path.read_bytes() == expected
  assert not (im.model_dir(model) / seen[0].cache_filename).exists()   # the part is not kept
  m.download(model, 1)   # the kept ONNX still has the pinned hash: no second download
  assert len(seen) == 1


def test_template_mismatch_is_not_retried_until_selected_again(monkeypatch, tmp_path):
  model, _ = _patch_model(tmp_path, b"T" * 64, b"0123456789", [(4, 0, 3)])
  monkeypatch.setattr(im, "CATALOG", (*[m for m in im.CATALOG if m.id != model.id], model))
  im.write_request(model, now=1)
  m = manager()
  calls = []
  def download(model, token):
    calls.append(1)
    raise imm.StepError("template", "the stock model differs")
  monkeypatch.setattr(m, "download", download)
  m.step()
  m.step()
  status = im.read_json(im.status_path())
  assert status["phase"] == "error" and status["error_code"] == "template" and calls == [1]


def test_wmiv11_patch_of_the_real_stock_model_gives_the_pinned_onnx_when_available(tmp_path):
  stock = Path(os.getenv("CARROT_TEST_STOCK_ONNX", str(Path(im.__file__).parent / "models" / im.ONNX_NAME)))
  part = Path(os.getenv("CARROT_TEST_WMIV9_POLICY", "/data/tmp/model-sel/wmiv9_policy.onnx"))
  if not stock.is_file() or stock.stat().st_size != im.STOCK_SIZE or not part.is_file():
    pytest.skip("stock ONNX or WMI V9 policy not here")
  (tmp_path / "models" / im.ONNX_NAME).write_bytes(stock.read_bytes())
  target = tmp_path / "wmiv11.onnx"
  imm.build_patched(WMIV11, part, target)
  from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
  meta = make_metadata_dict(str(target))
  assert im.contract_errors(meta) == [] and meta["model_checkpoint"] == WMIV11.checkpoint


def test_a_cached_onnx_with_the_right_size_but_wrong_hash_is_rebuilt_not_compiled(monkeypatch, tmp_path):
  model, expected = _patch_model(tmp_path, b"T" * 64, b"0123456789", [(4, 0, 3)])
  monkeypatch.setattr(im, "CATALOG", (*[m for m in im.CATALOG if m.id != model.id], model))
  downloads = []

  def fake_download(manifest, cache_dir, progress_callback=None, phase_callback=None):
    downloads.append(manifest)
    cache_dir.mkdir(parents=True, exist_ok=True)
    path = cache_dir / manifest.cache_filename
    path.write_bytes(b"0123456789")
    return path

  import openpilot.selfdrive.modeld.big_model as big_model
  monkeypatch.setattr(big_model, "_download_model", fake_download)
  m = manager()
  target = m.download(model, 1)
  data = bytearray(target.read_bytes())
  data[5] ^= 1   # same size, one weight byte changed
  target.write_bytes(bytes(data))
  assert m.download(model, 1).read_bytes() == expected and len(downloads) == 2


# --- metadata without tinygrad -------------------------------------------------------------------------------------
def _vint(v: int) -> bytes:
  out = bytearray()
  while True:
    out.append((v & 0x7f) | (0x80 if v > 0x7f else 0))
    v >>= 7
    if not v:
      return bytes(out)


def _ld(fid: int, payload: bytes) -> bytes:
  return _vint(fid << 3 | 2) + _vint(len(payload)) + payload


def _vi(fid: int, value: int) -> bytes:
  return _vint(fid << 3) + _vint(value)


def tiny_onnx(path, inputs, outputs, slices, checkpoint="ck/1"):
  import codecs
  import pickle

  def value_info(name, shape):
    dims = b"".join(_ld(1, _vi(1, d) if isinstance(d, int) else _ld(2, d.encode())) for d in shape)
    return _ld(1, name.encode()) + _ld(2, _ld(1, _vi(1, 10) + _ld(2, dims)))

  graph = _ld(1, _ld(4, b"Relu")) + _ld(5, _ld(8, b"w") + _ld(9, b"\x00" * 8))   # a node and an initializer to skip
  graph += b"".join(_ld(11, value_info(n, s)) for n, s in inputs.items())
  graph += b"".join(_ld(12, value_info(n, s)) for n, s in outputs.items())
  meta = lambda k, v: _ld(14, _ld(1, k.encode()) + _ld(2, v.encode()))
  path.write_bytes(_vi(1, 8) + _ld(7, graph) + meta("model_checkpoint", checkpoint)
                   + meta("output_slices", codecs.encode(pickle.dumps(slices), "base64").decode()))


def test_read_metadata_reads_inputs_outputs_and_metadata_without_initializers(tmp_path):
  from openpilot.selfdrive.modeld.internal_model_onnx import read_metadata
  meta = stock_metadata()
  path = tmp_path / "m.onnx"
  tiny_onnx(path, {**meta["input_shapes"], "dyn": ("batch", 3)}, meta["output_shapes"], meta["output_slices"])
  got = read_metadata(path)
  assert got["input_shapes"] == {**meta["input_shapes"], "dyn": (0, 3)} and got["output_shapes"] == meta["output_shapes"]
  assert got["output_slices"] == meta["output_slices"] and got["model_checkpoint"] == "ck/1"


def test_validate_never_uses_the_tinygrad_metadata_parser(monkeypatch, tmp_path):
  import openpilot.selfdrive.modeld.get_model_metadata as gmm
  monkeypatch.setattr(gmm, "make_metadata_dict", lambda *a: pytest.fail("tinygrad parser touches the default device"))
  meta = stock_metadata()
  path = tmp_path / "m.onnx"
  tiny_onnx(path, meta["input_shapes"], meta["output_shapes"], meta["output_slices"])
  manager().validate(RDF, 1, path)
  tiny_onnx(path, {**meta["input_shapes"], "features_buffer": (1, 32, 512)}, meta["output_shapes"], meta["output_slices"])
  with pytest.raises(imm.StepError) as e:
    manager().validate(RDF, 1, path)
  assert e.value.code == "contract"


@pytest.mark.parametrize("env", ["CARROT_TEST_STOCK_ONNX", "CARROT_TEST_RDF_ONNX"])
def test_read_metadata_matches_get_model_metadata_on_real_models_when_available(env):
  path = Path(os.getenv(env, "/nonexistent"))
  if not path.is_file():
    pytest.skip(f"{env} not set")
  from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
  from openpilot.selfdrive.modeld.internal_model_onnx import read_metadata
  assert read_metadata(path) == make_metadata_dict(str(path))


@pytest.mark.parametrize("shape,axis,n_idx", [((1, 33, 8), 1, 300), ((33, 8), 0, 225), ((2, 5, 40), -1, 64), ((3, 50), 1, 10)])
def test_large_constant_gathers_become_a_matmul_with_the_same_values(monkeypatch, shape, axis, n_idx):
  import numpy as np
  import tinygrad.nn.onnx as onnx
  from tinygrad.tensor import Tensor
  from openpilot.selfdrive.modeld import compile_internal as ci
  monkeypatch.setitem(onnx.onnx_ops, "Gather", onnx.onnx_ops["Gather"])   # restored after the test
  orig = onnx.onnx_ops["Gather"]
  rng = np.random.default_rng(1)
  x = rng.standard_normal(shape).astype(np.float16)
  n = shape[axis]
  idx = [int(i) for i in rng.integers(-n, n, n_idx)]
  want = orig(Tensor(x), idx, axis).numpy()
  ci.gather_large_constant_indices_by_matmul()
  got = onnx.onnx_ops["Gather"](Tensor(x), idx, axis).numpy()
  assert got.shape == want.shape and got.dtype == want.dtype and np.array_equal(got, want)
  with pytest.raises(Exception):
    onnx.onnx_ops["Gather"](Tensor(x), [-2 * n] * 40, axis).numpy()   # out of range: rejected, as before
  ints, int_idx = Tensor(np.arange(12, dtype=np.int32)), list(range(-5, 7)) * 4   # integer data: tinygrad's own gather
  assert np.array_equal(onnx.onnx_ops["Gather"](ints, int_idx, 0).numpy(), orig(ints, int_idx, 0).numpy())


def live_switch_class(create, warm, records, monkeypatch):
  """modeld.LiveSwitch with fakes for what modeld.py imports natively (VisionIPC, tinygrad, cereal)."""
  import ast
  import sys
  import time as time_module
  from types import SimpleNamespace as NS
  tree = ast.parse((Path(im.__file__).parent / "modeld.py").read_text())
  nodes = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in ("LiveSwitch", "install_clean_stop_handler")]
  monkeypatch.setitem(sys.modules, "openpilot.selfdrive.modeld.internal_model_warmup", NS(warm_internal_model=warm))
  internal = NS(**{k: getattr(im, k) for k in ("request_token", "live_switch_ready", "requested_model", "stock_model",
                                                "read_json", "request_path", "find")})
  internal.STOCK_ID = im.STOCK_ID
  internal.record_runtime_failure = lambda selected: records.append(("runtime_failed", selected.model.id))
  internal.mark_loading = lambda selected: records.append(("loading", selected.model.id))
  internal.mark_loaded = lambda selected: records.append(("loaded", selected.model.id))
  internal.mark_clean_stop = lambda: None
  stock_state = getattr(create, "stock_state", None)
  scope = {"internal_model": internal, "create_internal_model": create, "time": time_module,
           "ModelState": lambda *a, **k: stock_state or FakeState("explicit-stock"),
           "log": NS(OnroadEvent=NS(EventName=NS(internalModelSwitching="internalModelSwitching"))),
           "cloudlog": NS(warning=lambda *a: None, exception=lambda *a: None),
           "signal": NS(SIGINT=2, signal=lambda *a: records.append("sigint"), default_int_handler=None)}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), "<live_switch>", "exec"), scope)
  return scope["LiveSwitch"]


class FakeSwitchParams:
  def __init__(self, **values):
    self.values, self.writes = {"InternalModelLiveSwitch": True, **values}, []
  def get_bool(self, key):
    return bool(self.values.get(key, False))
  def put_bool(self, key, value):
    self.values[key] = value
    self.writes.append(value)
  def put_bool_nonblocking(self, key, value):
    raise AssertionError("LiveSwitch must order its flag writes (blocking puts only)")


class FakeState:
  def __init__(self, name, steps=96):
    self.name, self.history_steps, self.vision_input_names = name, steps, ["img", "big_img"]


def fake_sm(t_ns, event=True, enabled=False, lat=False, long=False, events_ns=None, state_ns=None, control_ns=None,
            valid=True, alive=True):
  from types import SimpleNamespace as NS
  return {
    "logMonoTime": {"onroadEvents": events_ns or t_ns, "selfdriveState": state_ns or t_ns + 1, "carControl": control_ns or t_ns + 2},
    "onroadEvents": [NS(name="internalModelSwitching")] if event else [NS(name="other")],
    "selfdriveState": NS(enabled=enabled, active=enabled), "carControl": NS(latActive=lat, longActive=long, enabled=enabled),
    "valid": {"selfdriveState": valid, "carControl": valid}, "alive": {"selfdriveState": alive, "carControl": alive},
  }


class SM(dict):
  @property
  def logMonoTime(self):
    return self["logMonoTime"]

  @property
  def valid(self):
    return self["valid"]

  @property
  def alive(self):
    return self["alive"]


def requested_switch(monkeypatch, create=None, warm=None, token_model=RDF):
  records, params = [], FakeSwitchParams()
  LiveSwitch = live_switch_class(create or (lambda *a, **k: (FakeState("x"), None)), warm or (lambda m, b: None), records, monkeypatch)
  compile_ok(RDF)
  sw = LiveSwitch(params, None, im.request_token())
  im.write_request(token_model, now=7)
  assert sw.wanted(20, False, *CAM)
  sw.request()
  return sw, params, records, LiveSwitch


def test_live_switch_requests_only_a_new_ready_selection_while_nothing_is_engaged(monkeypatch):
  records, params = [], FakeSwitchParams()
  LiveSwitch = live_switch_class(lambda *a, **k: (FakeState("x"), None), lambda m, b: None, records, monkeypatch)
  sw = LiveSwitch(params, None, im.request_token())
  assert params.writes == [False]
  assert not sw.wanted(20, False, *CAM)                 # nothing selected since modeld started
  im.write_request(RDF, now=1)
  assert not sw.wanted(20, False, *CAM)                 # RDF is not compiled: wait for the parked manager
  compile_ok(RDF)
  assert not sw.wanted(21, False, *CAM)                 # checked once a second
  assert not sw.wanted(40, True, *CAM)                  # never while engaged or lateral/longitudinal is active
  assert sw.wanted(40, False, *CAM) and sw.phase == LiveSwitch.IDLE
  sw.request()
  assert sw.phase == LiveSwitch.REQUESTED and params.values["InternalModelSwitching"] is True
  assert not sw.wanted(60, False, *CAM)                 # one request at a time


def test_a_selection_made_while_modeld_loads_is_still_switched_to(monkeypatch):
  records, params = [], FakeSwitchParams()
  LiveSwitch = live_switch_class(lambda *a, **k: (FakeState("x"), None), lambda m, b: None, records, monkeypatch)
  compile_ok(RDF)
  im.write_request(RDF, now=1)
  start_token = im.request_token()                       # modeld reads it, then loads RDF...
  im.write_request(im.stock_model(), now=2)              # ...while the user picks stock
  sw = LiveSwitch(params, im.Selected(RDF, im.pkl_path(RDF)), start_token)
  assert sw.wanted(20, False, *CAM)


def test_live_switch_ignores_the_running_model_selected_again(monkeypatch):
  records, params = [], FakeSwitchParams()
  LiveSwitch = live_switch_class(lambda *a, **k: (FakeState("x"), None), lambda m, b: None, records, monkeypatch)
  compile_ok(RDF)
  sw = LiveSwitch(params, im.Selected(RDF, im.pkl_path(RDF)), im.request_token())
  im.write_request(RDF, now=5)
  assert not sw.wanted(20, False, *CAM)
  im.write_request(im.stock_model(), now=6)
  assert sw.wanted(20, False, *CAM)                     # back to stock: always ready


def test_the_load_waits_for_the_block_to_be_confirmed_after_the_request(monkeypatch):
  sw, params, _, LiveSwitch = requested_switch(monkeypatch)
  t = sw.requested_at_ns
  later = t + int(0.3e9)
  assert sw.acknowledged(SM(fake_sm(t - 10)), False) == "wait"                       # messages from before the request
  assert sw.acknowledged(SM(fake_sm(later, event=False)), False) == "wait"           # selfdrived has not raised it yet
  assert sw.acknowledged(SM(fake_sm(later, enabled=True)), False) == "wait"          # state still engaged
  assert sw.acknowledged(SM(fake_sm(later, lat=True)), False) == "wait"              # AlwaysLateral still steering
  assert sw.acknowledged(SM(fake_sm(t + 10, control_ns=t + int(0.1e9))), False) == "wait"  # controlsd may not have re-read
  assert sw.acknowledged(SM(fake_sm(later, state_ns=later - 5)), False) == "wait"    # state older than the event
  assert sw.acknowledged(SM(fake_sm(later, control_ns=later)), False) == "wait"      # control not after the idle state
  assert sw.acknowledged(SM(fake_sm(later, valid=False)), False) == "wait"           # invalid confirmation
  assert sw.acknowledged(SM(fake_sm(later, alive=False)), False) == "wait"           # stale confirmation
  assert sw.acknowledged(SM(fake_sm(later)), False) == "go"
  assert sw.acknowledged(SM(fake_sm(later)), True) == "engaged"
  import time as _t
  old = _t.monotonic_ns() - int(1e9)
  sw.requested_at_ns = old - int(1e9)
  assert sw.acknowledged(SM(fake_sm(old, control_ns=old + int(0.3e9))), False) == "wait"  # older than 0.3 s now


def test_a_selection_changed_while_waiting_withdraws_the_request(monkeypatch):
  sw, params, _, LiveSwitch = requested_switch(monkeypatch)
  wmi = im.find("WMIV11")
  im.write_request(wmi, now=8)                          # not compiled
  assert sw.acknowledged(SM(fake_sm(sw.requested_at_ns + int(0.3e9))), False) == "changed"
  sw.withdraw()
  assert params.values["InternalModelSwitching"] is False
  assert not sw.wanted(40, False, *CAM)                 # checked from the start: not ready, so no request
  compile_ok(wmi)
  assert sw.wanted(60, False, *CAM)


def test_no_live_switch_while_an_external_model_drives_and_jetlink_is_held_meanwhile(monkeypatch):
  records, params = [], FakeSwitchParams()
  LiveSwitch = live_switch_class(lambda *a, **k: (FakeState("x"), None), lambda m, b: None, records, monkeypatch)
  compile_ok(RDF)
  sw = LiveSwitch(params, None, im.request_token())
  im.write_request(RDF, now=3)
  assert not sw.wanted(20, False, *CAM, internal_only=False)
  joining = type("J", (), {"join_allowed": True})()
  sw.hold_jetlink(joining)
  assert joining.join_allowed is True                   # idle: Jetlink decides as before
  assert sw.wanted(20, False, *CAM, internal_only=True)
  sw.request()
  sw.hold_jetlink(joining)
  assert joining.join_allowed is False


def test_an_unconfirmed_or_interrupted_request_is_withdrawn_and_retried(monkeypatch):
  sw, params, _, LiveSwitch = requested_switch(monkeypatch)
  assert sw.acknowledged(SM(fake_sm(sw.requested_at_ns)), True) == "engaged"
  sw.withdraw()
  assert params.values["InternalModelSwitching"] is False and sw.phase == LiveSwitch.IDLE
  assert sw.wanted(40, False, *CAM)                     # still pending: requested again once disengaged
  sw.request()
  sw.requested_at_ns -= int(4e9)
  assert sw.acknowledged(SM(fake_sm(sw.requested_at_ns)), False) == "timeout"
  sw.withdraw(timed_out=True)
  assert not sw.wanted(60, False, *CAM)                 # paused after an unconfirmed block
  sw.retry_at_ns = 0
  assert sw.wanted(60, False, *CAM)


def test_the_block_lasts_until_the_new_model_itself_produced_a_full_history(monkeypatch):
  selected = im.Selected(RDF, im.pkl_path(RDF))
  new = FakeState("new", steps=128)
  sw, params, records, LiveSwitch = requested_switch(monkeypatch, create=lambda *a, **k: (new, selected))
  old = FakeState("old")
  model, small, sel = sw.switch(old, old, *CAM, False, {"img": 1, "big_img": 2})
  assert (model, small, sel) == (new, new, selected)
  assert records[-2:] == [("loading", "RDF"), ("loaded", "RDF")] and "sigint" in records
  assert sw.phase == LiveSwitch.WARMING and sw.steps_needed == 128 + LiveSwitch.EXTRA_STEPS
  for _ in range(sw.steps_needed - 1):
    sw.produced(True, True)
  sw.produced(True, False)                              # prepare_only: no output, no history step counted
  assert params.values["InternalModelSwitching"] is True
  sw.produced(True, True)
  assert params.values["InternalModelSwitching"] is False and sw.phase == LiveSwitch.IDLE
  assert params.writes == [False, True, False]
  assert not sw.wanted(80, False, *CAM)                 # the switched-to selection is consumed


def test_outputs_the_new_model_did_not_produce_never_end_the_block(monkeypatch):
  sw, params, _, LiveSwitch = requested_switch(monkeypatch)
  wrapper, old = type("J", (), {})(), FakeState("old")
  wrapper.small = old
  model, small, _ = sw.switch(wrapper, old, *CAM, False, {"img": 1, "big_img": 2})
  assert model is wrapper and wrapper.small is small    # the Jetlink wrapper runs it
  for _ in range(1000):
    sw.produced(False, True)                            # an external result, e.g. a Jetlink fallback frame
    sw.produced(True, False)                            # prepare_only: nothing published
  assert params.values["InternalModelSwitching"] is True and sw.phase == LiveSwitch.WARMING and sw.steps == 0


def test_live_switch_falls_back_to_stock_when_the_new_model_fails_its_warmup(monkeypatch):
  import gc
  import weakref
  selected = im.Selected(RDF, im.pkl_path(RDF))
  failed = FakeState("rdf")
  ref = weakref.ref(failed)
  alive_at_fallback = []
  stock = FakeState("stock")
  def create(*a, **k):
    return failed_holder.pop(), selected
  failed_holder = [failed]
  del failed
  def warm(model, bufs):
    if model.name == "rdf":
      im.write_request(im.find("WMIV11"), now=9)        # changed while warming: the fallback is still the stock model
      raise RuntimeError("NaN")
    gc.collect()
    alive_at_fallback.append(ref() is not None)
  create.stock_state = stock
  sw, params, records, LiveSwitch = requested_switch(monkeypatch, create=create, warm=warm)
  old = FakeState("old")
  model, small, sel = sw.switch(old, old, *CAM, False, {"img": 1, "big_img": 2})
  assert model is stock and sel is None and ("runtime_failed", "RDF") in records
  assert alive_at_fallback == [False]                   # the failed model was released before the fallback loaded
  assert sw.running_sha == im.stock_model().sha256 and sw.phase == LiveSwitch.WARMING



def test_a_modeld_restart_during_a_switch_keeps_the_block_until_the_new_history_is_full(monkeypatch):
  records = []
  LiveSwitch = live_switch_class(lambda *a, **k: (FakeState("x"), None), lambda m, b: None, records, monkeypatch)
  params = FakeSwitchParams(InternalModelSwitching=True)      # left set by the modeld that stopped mid-switch
  sw = LiveSwitch(params, None, im.request_token(), history_steps=128)
  assert params.writes == [] and sw.phase == LiveSwitch.WARMING and sw.steps_needed == 128 + LiveSwitch.EXTRA_STEPS
  joining = type("J", (), {"join_allowed": True})()
  sw.hold_jetlink(joining)
  assert joining.join_allowed is False
  for _ in range(sw.steps_needed - 1):
    sw.produced(True, True)
  assert params.values["InternalModelSwitching"] is True
  sw.produced(True, True)
  assert params.values["InternalModelSwitching"] is False and sw.phase == LiveSwitch.IDLE
  normal = FakeSwitchParams()                                  # a normal start: cleared, idle
  assert LiveSwitch(normal, None, im.request_token(), history_steps=96).phase == LiveSwitch.IDLE and normal.writes == [False]
  stuck = FakeSwitchParams(InternalModelSwitching=True)        # no internal model to fill a history: cleared as before
  assert LiveSwitch(stuck, None, im.request_token(), history_steps=0).phase == LiveSwitch.IDLE and stuck.writes == [False]


def test_a_selection_changed_after_the_confirmation_loads_nothing(monkeypatch):
  made = []
  sw, params, _, LiveSwitch = requested_switch(monkeypatch, create=lambda *a, **k: made.append(1) or (FakeState("x"), None))
  im.write_request(im.find("WMIV11"), now=11)
  old = FakeState("old")
  assert sw.switch(old, old, *CAM, False, {"img": 1, "big_img": 2}) is None and made == []
  sw.withdraw()
  assert params.values["InternalModelSwitching"] is False


def test_live_switch_is_off_unless_the_carrot_web_setting_is_on(monkeypatch):
  records, params = [], FakeSwitchParams(InternalModelLiveSwitch=False)
  LiveSwitch = live_switch_class(lambda *a, **k: (FakeState("x"), None), lambda m, b: None, records, monkeypatch)
  compile_ok(RDF)
  sw = LiveSwitch(params, None, im.request_token())
  im.write_request(RDF, now=12)
  assert not sw.wanted(20, False, *CAM) and sw.phase == LiveSwitch.IDLE and params.writes == [False]
  params.values["InternalModelLiveSwitch"] = True        # turned on later: the pending selection is switched to
  assert sw.wanted(40, False, *CAM)


def test_the_switch_loads_the_model_of_the_confirmed_request(monkeypatch):
  seen = []
  selected = im.Selected(RDF, im.pkl_path(RDF))
  def create(cam_w, cam_h, usbgpu, requested=None):
    seen.append(requested)
    im.write_request(im.stock_model(), now=13)            # a later selection never replaces the confirmed one
    return FakeState("new"), selected
  sw, params, _, LiveSwitch = requested_switch(monkeypatch, create=create)
  old = FakeState("old")
  model, small, sel = sw.switch(old, old, *CAM, False, {"img": 1, "big_img": 2})
  assert seen == [RDF] and sel is selected
  assert sw.wanted(80, False, *CAM) is False              # still warming
  assert sw.token != im.request_token()                   # the stock selection made meanwhile stays pending


def test_a_request_is_withdrawn_even_while_camera_frames_are_missing():
  import ast
  src = (Path(im.__file__).parent / "modeld.py").read_text()
  fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == "main")
  branch = next(n for n in ast.walk(fn) if isinstance(n, ast.If) and ast.unparse(n.test) == "frames is None")
  body = ast.unparse(branch)
  assert "live_switch.phase == LiveSwitch.REQUESTED" in body and "sm.update(0)" in body
  assert "live_switch.acknowledged(sm" in body and "live_switch.withdraw(timed_out=ack == 'timeout')" in body
  assert body.index("live_switch.withdraw") < body.index("continue")


def test_select_for_modeld_uses_a_request_already_read():
  compile_ok(RDF)
  im.write_request(im.stock_model(), now=14)
  selected = im.select_for_modeld(*CAM, requested=RDF)
  assert selected is not None and selected.model is RDF
  assert im.select_for_modeld(*CAM) is None                # without it: request.json (stock)


def test_turning_the_setting_off_before_the_load_withdraws_the_request(monkeypatch):
  made = []
  sw, params, _, LiveSwitch = requested_switch(monkeypatch, create=lambda *a, **k: made.append(1) or (FakeState("x"), None))
  params.values["InternalModelLiveSwitch"] = False
  assert sw.acknowledged(SM(fake_sm(sw.requested_at_ns + int(0.3e9))), False) == "off"
  old = FakeState("old")
  assert sw.switch(old, old, *CAM, False, {"img": 1, "big_img": 2}) is None and made == []   # also right before loading
  sw.withdraw()
  assert params.values["InternalModelSwitching"] is False and sw.phase == LiveSwitch.IDLE
  assert not sw.wanted(40, False, *CAM)                                 # stays pending for the next start


def test_turning_the_setting_off_while_warming_keeps_the_block(monkeypatch):
  sw, params, _, LiveSwitch = requested_switch(monkeypatch)
  old = FakeState("old")
  sw.switch(old, old, *CAM, False, {"img": 1, "big_img": 2})
  params.values["InternalModelLiveSwitch"] = False
  for _ in range(sw.steps_needed - 1):
    sw.produced(True, True)
  assert params.values["InternalModelSwitching"] is True and sw.phase == LiveSwitch.WARMING
  sw.produced(True, True)
  assert params.values["InternalModelSwitching"] is False


def test_main_loop_withdraws_on_off_and_keeps_a_calibration_received_without_frames():
  import ast
  src = (Path(im.__file__).parent / "modeld.py").read_text()
  fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == "main")
  body = ast.unparse(fn)
  assert body.count("if ack in ('engaged', 'changed', 'off', 'timeout'):") == 2
  branch = ast.unparse(next(n for n in ast.walk(fn) if isinstance(n, ast.If) and ast.unparse(n.test) == "frames is None"))
  assert "calib_pending = calib_pending or sm.updated['liveCalibration']" in branch
  assert branch.index("sm.update(0)") < branch.index("calib_pending = calib_pending")
  assert "calib_updated, calib_pending = (sm.updated['liveCalibration'] or calib_pending, False)" in body
  assert "if calib_updated and sm.seen['roadCameraState'] and sm.seen['deviceState']:" in body

