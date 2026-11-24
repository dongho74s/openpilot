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
  params = SimpleNamespace(get_bool=lambda key: flags.get(key, False))
  monkeypatch.setattr(F, "_params", lambda: params)
  job = F.Job()
  starts = []
  monkeypatch.setattr(job, "start", lambda gentle=False: starts.append("gentle" if gentle else 1) or True)
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


def test_the_button_runs_gently_while_driving(feature):
  feature.flags["IsOnroad"] = True
  status, p = call(F.api_run)
  assert status == 200 and feature.starts == ["gentle"]
  feature.flags["IsOnroad"] = False
  status, p = call(F.api_run)
  assert status == 200 and feature.starts == ["gentle", 1]


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


def test_switching_the_car_on_stops_a_parked_run_and_the_drive_gets_a_gentle_one(feature, monkeypatch):
  route(feature, "00000252--3d51466b37")
  F.tick(feature.params, 0.0)
  F.tick(feature.params, F.PARKED_BEFORE_START)
  assert feature.starts == [1]
  monkeypatch.setattr(feature.job, "running", lambda: True)
  route(feature, "00000253--121bd0b631")          # the car is switched on: a new route is recorded
  feature.flags["IsOnroad"] = True
  F.tick(feature.params, 100.0)
  assert feature.starts == [1, "stop"] and feature.job.offroad_since is None
  monkeypatch.setattr(feature.job, "running", lambda: False)
  F.tick(feature.params, 100.0 + F.DRIVING_BEFORE_START - 1)
  assert feature.starts == [1, "stop"]
  F.tick(feature.params, 100.0 + F.DRIVING_BEFORE_START + F.AUTO_CHECK_INTERVAL)
  assert feature.starts == [1, "stop", "gentle"]   # 252 again, gently, not the drive being recorded


def test_without_permanent_power_the_next_drive_analyses_the_one_before(feature):
  route(feature, "00000252--3d51466b37")
  route(feature, "00000253--121bd0b631")          # booted with the ignition: straight onroad
  feature.flags["IsOnroad"] = True
  F.tick(feature.params, 0.0)
  assert feature.starts == []
  F.tick(feature.params, F.DRIVING_BEFORE_START)
  assert feature.starts == ["gentle"]
  F.tick(feature.params, F.DRIVING_BEFORE_START + 1000)
  assert feature.starts == ["gentle"]              # once per drive


def test_a_gentle_run_is_not_killed_by_the_ignition(feature, monkeypatch):
  feature.job.gentle = True
  monkeypatch.setattr(feature.job, "running", lambda: True)
  feature.flags["IsOnroad"] = True
  F.tick(feature.params, 10.0)
  assert feature.starts == []
