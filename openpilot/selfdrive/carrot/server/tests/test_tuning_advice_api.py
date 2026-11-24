import asyncio
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.tuning_advice import runner

# loaded on its own so the test does not import every other feature (and msgq) through features/__init__.py
_spec = importlib.util.spec_from_file_location("openpilot.selfdrive.carrot.server.features.tuning_advice_test",
                                               Path(__file__).parents[1] / "features/tuning_advice.py")
F = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(F)


@pytest.fixture
def feature(monkeypatch, tmp_path):
  state, logs = tmp_path / "state", tmp_path / "realdata"
  state.mkdir()
  logs.mkdir()
  monkeypatch.setattr(F, "CARROT_STATE_DIR", str(state))
  monkeypatch.setattr(F, "REALDATA", str(logs))
  monkeypatch.setattr(F, "get_param_values", lambda names, defaults: {"LateralAccelLimit": 35})
  flags = {"IsOnroad": False}
  params = SimpleNamespace(get_bool=lambda key: flags.get(key, False), get=lambda key: flags.get(key, False))
  monkeypatch.setattr(F, "_params", lambda: params)
  job = F.Job()
  starts = []
  monkeypatch.setattr(job, "start", lambda: starts.append(1) or True)
  monkeypatch.setattr(job, "stop", lambda: starts.append("stop"))
  monkeypatch.setattr(F, "JOB", job)
  return SimpleNamespace(state=state, logs=logs, flags=flags, params=params, starts=starts, job=job)


def route(f, name, segments=1):
  for n in range(segments):
    (f.logs / f"{name}--{n}").mkdir()


def write_result(f, newest, version=runner.VERSION):
  (f.state / runner.RESULT_NAME).write_text(json.dumps({"version": version, "newest_route": newest, "route": newest}))


def call(handler):
  response = asyncio.run(handler(None))
  return response.status, json.loads(response.body)


def test_a_new_drive_needs_an_analysis(feature):
  route(feature, "00000251--ec2aec2b03")
  route(feature, "00000252--3d51466b37", 3)
  status, p = call(F.api_status)
  assert status == 200 and p["newest_route"] == "00000252--3d51466b37"
  assert p["result"] is None and p["needs_run"] and p["can_run"] and p["current"] == {"LateralAccelLimit": 35}
  write_result(feature, "00000252--3d51466b37")
  assert not call(F.api_status)[1]["needs_run"]
  write_result(feature, "00000252--3d51466b37", version=runner.VERSION + 1)   # an older/newer format is ignored
  assert call(F.api_status)[1]["result"] is None


def test_the_button_runs_with_the_ignition_on_or_off(feature):
  feature.flags["IsOnroad"] = True
  assert call(F.api_run)[0] == 200
  feature.flags["IsOnroad"] = False
  assert call(F.api_run)[0] == 200
  assert feature.starts == [1, 1]


def test_while_driving_the_target_is_the_drive_before(feature):
  route(feature, "00000252--3d51466b37")
  route(feature, "00000253--121bd0b631")          # being recorded
  write_result(feature, "00000252--3d51466b37")
  feature.flags["IsOnroad"] = True
  p = call(F.api_status)[1]
  assert p["newest_route"] == "00000252--3d51466b37" and not p["needs_run"]
  feature.flags["IsOnroad"] = False
  p = call(F.api_status)[1]
  assert p["newest_route"] == "00000253--121bd0b631" and p["needs_run"]


def test_automatic_run_once_per_drive_after_a_parked_minute(feature):
  route(feature, "00000252--3d51466b37")
  F.tick(feature.params, 1000.0)
  F.tick(feature.params, 1000.0 + F.PARKED_BEFORE_START - 1)
  assert feature.starts == []
  F.tick(feature.params, 1000.0 + F.PARKED_BEFORE_START)
  assert feature.starts == [1]
  F.tick(feature.params, 2000.0)                 # same newest route, nothing written yet: not again
  assert feature.starts == [1]
  route(feature, "00000253--121bd0b631")         # the next drive
  F.tick(feature.params, 3000.0)
  assert feature.starts == [1, 1]


def test_the_ignition_does_not_stop_a_run(feature, monkeypatch):
  monkeypatch.setattr(feature.job, "running", lambda: True)
  elapsed = [0.0]
  monkeypatch.setattr(feature.job, "elapsed", lambda: elapsed[0])
  feature.flags["IsOnroad"] = True
  F.tick(feature.params, 10.0)
  feature.flags["IsOnroad"] = False
  F.tick(feature.params, 20.0)
  assert feature.starts == []
  elapsed[0] = F.RUN_TIMEOUT + 1
  F.tick(feature.params, 30.0)                     # only a run that hangs for an hour is stopped
  assert feature.starts == ["stop"]


def test_without_permanent_power_the_next_drive_analyses_the_one_before(feature):
  route(feature, "00000252--3d51466b37")
  route(feature, "00000253--121bd0b631")          # booted with the ignition: straight onroad
  feature.flags["IsOnroad"] = True
  F.tick(feature.params, 0.0)
  assert feature.starts == []
  F.tick(feature.params, F.ONROAD_BEFORE_START)
  assert feature.starts == [1]
  write_result(feature, "00000252--3d51466b37")   # done: nothing more during this drive
  F.tick(feature.params, F.ONROAD_BEFORE_START + 1000)
  assert feature.starts == [1]


def test_a_runner_left_from_before_a_server_restart_still_counts(feature, monkeypatch):
  import os
  monkeypatch.setattr(runner, "is_runner", lambda pid: True)
  fd = runner.acquire_lock(str(feature.state))     # held by "another" runner
  try:
    job = F.Job()
    assert job.running() and job.other_pid() == os.getpid()
    assert job.start() is False                    # no second analysis
  finally:
    os.close(fd)
  assert not F.Job().running()


def test_a_put_off_run_is_tried_again_after_the_ignition_changes(feature):
  route(feature, "00000252--3d51466b37")
  route(feature, "00000253--121bd0b631")
  feature.flags["IsOnroad"] = True
  F.tick(feature.params, 0.0)
  F.tick(feature.params, F.ONROAD_BEFORE_START)
  assert feature.starts == [1]                     # say it was put off (memory): no result
  F.tick(feature.params, F.ONROAD_BEFORE_START + 100)
  assert feature.starts == [1]
  feature.flags["IsOnroad"] = False                # parked: the newest drive is 253 now
  F.tick(feature.params, 1000.0)
  F.tick(feature.params, 1000.0 + F.PARKED_BEFORE_START)
  assert feature.starts == [1, 1]


class ApplyRequest:
  def __init__(self, body):
    self.body = body

  async def json(self):
    return self.body


def apply(body):
  response = asyncio.run(F.api_apply(ApplyRequest(body)))
  return response.status, json.loads(response.body)


@pytest.fixture
def advice(feature, monkeypatch):
  import openpilot.selfdrive.carrot.server.services.param_changes as pc
  import openpilot.selfdrive.carrot.server.services.params as sp
  import openpilot.selfdrive.carrot.server.services.settings as ss
  writes, records = [], []
  monkeypatch.setattr(sp, "set_param_value", lambda name, value, setting=None: writes.append((name, value)))
  monkeypatch.setattr(pc, "append_param_change", lambda *a, **k: records.append((a, k)))
  monkeypatch.setattr(ss, "get_settings_cached", lambda: (None, None, {}, None))
  route(feature, "00000252--3d51466b37")
  (feature.state / runner.RESULT_NAME).write_text(json.dumps({
    "version": runner.VERSION, "newest_route": "00000252--3d51466b37", "route": "00000252--3d51466b37",
    "recommendation": {"status": "change", "param": "LateralAccelLimit", "from": 35, "to": 40}}))
  return SimpleNamespace(writes=writes, records=records)


def test_apply_writes_the_recommendation_when_parked(feature, advice):
  status, p = apply({"name": "LateralAccelLimit", "value": 40})
  assert status == 200 and advice.writes == [("LateralAccelLimit", 40)]
  assert advice.records[0][0][:3] == ("LateralAccelLimit", 35, 40) and advice.records[0][1]["source"] == "tuning_advice"


def test_apply_is_refused_with_the_ignition_on(feature, advice):
  feature.flags["IsOnroad"] = True
  assert apply({"name": "LateralAccelLimit", "value": 40})[0] == 409
  feature.flags["IsOnroad"] = False
  feature.flags["IsEngaged"] = True
  assert apply({"name": "LateralAccelLimit", "value": 40})[0] == 409
  assert advice.writes == []


def test_apply_only_the_current_recommendation_for_the_newest_drive(feature, advice):
  assert apply({"name": "LateralTorqueKpV", "value": 170})[0] == 409
  assert apply({"name": "LateralAccelLimit", "value": 45})[0] == 409
  route(feature, "00000253--121bd0b631")          # a newer drive, not analysed yet
  assert apply({"name": "LateralAccelLimit", "value": 40})[0] == 409
  assert advice.writes == []


def test_a_holder_that_is_not_a_verified_runner_is_never_signalled(feature, monkeypatch):
  import os
  monkeypatch.setattr(runner, "is_runner", lambda pid: False)   # e.g. the file still has an old, reused pid
  sent = []
  monkeypatch.setattr(F.os, "killpg", lambda pid, sig: sent.append(pid))
  monkeypatch.setattr(F.os, "kill", lambda pid, sig: sent.append(pid))
  fd = runner.acquire_lock(str(feature.state))
  try:
    job = F.Job()
    assert job.running() and job.other_pid() is None
    job.stop()
    assert sent == []
  finally:
    os.close(fd)


def test_apply_reads_the_ignition_again_right_before_writing(feature, advice, monkeypatch):
  real = F.get_param_values

  def switch_on_meanwhile(names, defaults):
    feature.flags["IsOnroad"] = True               # the car is started while the values are being read
    return real(names, defaults)
  monkeypatch.setattr(F, "get_param_values", switch_on_meanwhile)
  assert apply({"name": "LateralAccelLimit", "value": 40})[0] == 409
  assert advice.writes == []


def test_apply_refuses_when_the_state_cannot_be_read(feature, advice, monkeypatch):
  def broken(key):
    raise OSError("params unavailable")
  monkeypatch.setattr(feature.params, "get", broken)
  assert apply({"name": "LateralAccelLimit", "value": 40})[0] == 409
  # the native Params returns nothing (get) / False (get_bool) for a file it cannot read: not parked
  monkeypatch.setattr(feature.params, "get", lambda key: None)
  assert apply({"name": "LateralAccelLimit", "value": 40})[0] == 409
  assert advice.writes == []
