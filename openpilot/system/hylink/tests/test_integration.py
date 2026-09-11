import json
import os
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, messaging
from openpilot.common.params import Params
from openpilot.system.hylink import camera, camera_lease, guard, runtime, setup, telemetry, transport
from openpilot.system.manager.process_config import managed_processes

DEVICE_ID = "0123456789abcdef"


class State(dict):
  def __init__(self):
    super().__init__()
    self.services = telemetry.SERVICES
    for name in self.services:
      event = messaging.new_message(name, size=1 if name == "pandaStates" else None, valid=True)
      self[name] = getattr(event, name)
    self["pandaStates"][0].pandaType = "tres"
    self["pandaStates"][0].voltage = 12500
    self["pandaStates"][0].current = 400
    self.seen = dict.fromkeys(self.services, True)
    self.valid = dict.fromkeys(self.services, True)
    self.recv_time = dict.fromkeys(self.services, time.monotonic())


@pytest.fixture
def configured(tmp_path, monkeypatch):
  params = Params()
  params.put("DongleId", DEVICE_ID)
  params.put_bool("IsOffroad", True)
  params.put_bool("IsOnroad", False)
  config_path = tmp_path / "config.json"
  state_path = tmp_path / "state.json"
  camera_path = tmp_path / "camera.json"
  for module, name, value in (
    (runtime, "CONFIG_PATH", config_path), (setup, "CONFIG_PATH", config_path),
    (runtime, "STATE_PATH", state_path), (runtime, "CAMERA_PATH", camera_path),
    (camera, "CAMERA_PATH", camera_path), (camera_lease, "CAMERA_LEASE_PATH", tmp_path / "lease"),
    (telemetry, "GPS_CACHE", tmp_path / "gps.json"),
  ):
    monkeypatch.setattr(module, name, value)
  config = {"enabled": True, "registered": True, "endpoint": runtime.ENDPOINT,
            "device_id": DEVICE_ID, "token": "wayon_" + "test" * 12,
            "media_enabled": True, "impact_enabled": True}
  runtime.write_json(config_path, config)
  runtime.write_json(state_path, {"pid": os.getpid(), "at": time.monotonic(), "offroad": True})
  return params, config


def test_explicit_opt_in_identity_and_private_file(configured):
  params, config = configured
  assert runtime.enabled(False, params, None)
  assert runtime.CONFIG_PATH.stat().st_mode & 0o777 == 0o600
  params.put("DongleId", "fedcba9876543210")
  assert not runtime.enabled(False, params, None)
  params.put("DongleId", DEVICE_ID)
  for invalid in ({**config, "enabled": "true"}, {**config, "endpoint": "http://localhost"},
                  {**config, "token": "bad"}, {}, {"enabled": False}):
    runtime.write_json(runtime.CONFIG_PATH, invalid)
    assert not runtime.enabled(False, params, None)


def test_corrupt_or_oversized_config_is_disabled(configured):
  params, _ = configured
  for data in ("{broken", "x" * 65537, "[]", "null"):
    runtime.CONFIG_PATH.write_text(data)
    assert not runtime.enabled(False, params, None)


def test_enrollment_preserves_pending_key_on_retry(configured):
  params, _ = configured
  runtime.CONFIG_PATH.unlink()
  keys = []
  def post(url, **kwargs):
    keys.append(kwargs["json"]["key"])
    assert kwargs["allow_redirects"] is False
    return SimpleNamespace(status_code=503 if len(keys) == 1 else 200)
  with pytest.raises(RuntimeError):
    setup.enroll(params, post)
  pending = runtime.read_json(runtime.CONFIG_PATH)
  assert pending["enabled"] is False
  result = setup.enroll(params, post)
  assert keys[0] == keys[1] == result["token"]
  assert result["enabled"] and result["registered"]
  assert not result["media_enabled"] and not result["impact_enabled"]


def test_copied_identity_refused_before_registration(configured):
  params, _ = configured
  params.put("DongleId", "fedcba9876543210")
  with pytest.raises(ValueError):
    setup.enroll(params, lambda *a, **k: pytest.fail("Must not register"))


def test_legacy_migration_preserves_only_same_device_identity(configured, tmp_path):
  params, config = configured
  legacy = tmp_path / "legacy.json"
  runtime.write_json(legacy, {**config, "ambient_command": "must not migrate"})
  with pytest.raises(ValueError, match="already enrolled"):
    setup.import_legacy(params, legacy)
  runtime.CONFIG_PATH.unlink()
  setup.import_legacy(params, legacy)
  new = runtime.read_json(runtime.CONFIG_PATH)
  assert new["token"] == config["token"] and new["device_id"] == config["device_id"]
  assert not new["enabled"] and not new["media_enabled"] and "ambient_command" not in new
  params.put("DongleId", "fedcba9876543210")
  with pytest.raises(ValueError):
    setup.import_legacy(params, legacy)


@pytest.mark.parametrize("name", ["deviceState", "pandaStates"])
def test_missing_or_stale_state_never_allows_parking(configured, name):
  params, _ = configured
  state = State()
  assert guard.observed_offroad(state, params)
  state.seen[name] = False
  assert not guard.observed_offroad(state, params)
  state.seen[name] = True
  state.valid[name] = False
  assert not guard.observed_offroad(state, params)
  state.valid[name] = True
  state.recv_time[name] -= 10
  assert not guard.observed_offroad(state, params)


@pytest.mark.parametrize("condition", ["started", "ignitionCan", "ignitionLine", "heartbeatLost", "unknown", "empty", "low_voltage", "heat", "params"])
def test_offroad_rejects_every_unsafe_transition(configured, condition):
  params, _ = configured
  state = State()
  if condition == "started":
    state["deviceState"].started = True
  elif condition in ("ignitionLine", "ignitionCan", "heartbeatLost"):
    setattr(state["pandaStates"][0], condition, True)
  elif condition == "unknown":
    state["pandaStates"][0].pandaType = "unknown"
  elif condition == "empty":
    state["pandaStates"] = []
  elif condition == "low_voltage":
    state["pandaStates"][0].voltage = 11999
  elif condition == "heat":
    state["deviceState"].thermalStatus = "red"
  elif condition == "params":
    params.put_bool("IsOnroad", True)
  assert not guard.observed_offroad(state, params)


@pytest.mark.parametrize("change", [{"at": -1}, {"at": float("nan")}, {"pid": -1}, {"pid": 99999999}, {"offroad": "true"}])
def test_dead_or_invalid_guard_never_starts_media(configured, change):
  params, _ = configured
  record = {"at": time.monotonic(), "pid": os.getpid(), "offroad": True, **change}
  runtime.STATE_PATH.write_text(json.dumps(record))
  assert not runtime.media_ready(False, params)


def test_camera_is_manager_owned_and_releases_on_ignition(configured):
  params, _ = configured
  with camera.CameraRequest(params, stream=True) as request:
    assert runtime.camera_requested(False, params)
    assert runtime.stream_requested(False, params)
    with pytest.raises(BlockingIOError):
      with camera.CameraRequest(params):
        pass
    assert not runtime.camera_requested(True, params)
    params.put_bool("IsOnroad", True)
    with pytest.raises(InterruptedError):
      request.renew()
  assert not runtime.camera_requested(False, params)
  assert not runtime.CAMERA_PATH.exists()
  assert not params.get_bool("IsTakingSnapshot")


def test_driver_view_snapshot_and_stale_request_take_priority(configured):
  params, _ = configured
  with camera.CameraRequest(params) as request:
    for flag in ("IsDriverViewEnabled", "IsTakingSnapshot"):
      params.put_bool(flag, True)
      assert not runtime.camera_requested(False, params)
      with pytest.raises(InterruptedError):
        request.renew()
      params.put_bool(flag, False)
    record = runtime.read_json(runtime.CAMERA_PATH)
    record["at"] -= 10
    runtime.write_json(runtime.CAMERA_PATH, record)
    assert not runtime.camera_requested(False, params)


def test_media_requires_both_cameras(configured):
  params, _ = configured
  assert runtime.media_ready(False, params)
  params.put_bool("UseWideCamera", False)
  assert not runtime.media_ready(False, params)
  params.put_bool("UseWideCamera", True)
  params.put_int("HardwareC3xLite", 1)
  assert not runtime.media_ready(False, params)


def test_current_cereal_payload_and_missing_fields(configured):
  params, _ = configured
  state = State()
  state["deviceState"].started = True
  state["carState"].canValid = True
  state["carState"].vEgo = 10
  state["carState"].vEgoCluster = 11
  payload = telemetry.telemetry_payload(state, params, DEVICE_ID)
  assert payload["vehicle"]["available"] and payload["openpilot"]["available"]
  assert payload["vehicleSpeedMps"] == 11
  assert payload["voltageV"] == 12.5 and payload["currentMa"] == 400
  assert payload["powerW"] == 5
  assert "controlsAllowedLateral" not in payload["panda"]
  assert "navdy" not in payload["connections"]
  assert "fuelGauge" not in payload["vehicle"]
  json.dumps(payload, allow_nan=False)
  state.recv_time["carState"] -= 10
  state.recv_time["selfdriveState"] -= 10
  payload = telemetry.telemetry_payload(state, params, DEVICE_ID)
  assert not payload["vehicle"]["available"] and not payload["openpilot"]["available"]
  assert payload["vehicleSpeedMps"] is None
  state.seen["pandaStates"] = state.seen["deviceState"] = False
  payload = telemetry.telemetry_payload(state, params, DEVICE_ID)
  assert payload["onroad"] is None and payload["ignition"] is None
  assert payload["systemState"] == "disconnected"


def test_zero_coordinate_valid_but_stale_gps_not_current(configured):
  params, _ = configured
  state = State()
  gps = state["gpsLocation"]
  gps.hasFix, gps.latitude, gps.longitude = True, 0, 0
  gps.unixTimestampMillis = int(time.time() * 1000)  # noqa: TID251 - GNSS Unix epoch.
  payload = telemetry.gps_payload(state, params)
  assert payload["fresh"] and payload["latitude"] == 0
  runtime.write_json(telemetry.GPS_CACHE, payload)
  state.recv_time["gpsLocation"] -= 10
  cached = telemetry.gps_payload(state, params)
  assert cached["fresh"] is False and cached["source"] == "lastGpsPosition"


def test_transport_blocks_redirect_and_nonfinite_values(configured, monkeypatch):
  _, config = configured
  calls = []
  def post(*args, **kwargs):
    calls.append(kwargs)
    return SimpleNamespace(status_code=302, content=b"")
  monkeypatch.setattr(transport.requests, "post", post)
  with pytest.raises(RuntimeError):
    transport.post_json(config, "/api/telemetry", {})
  assert calls[0]["allow_redirects"] is False
  with pytest.raises(ValueError):
    transport.post_json(config, "/api/telemetry", {"speed": float("nan")})
  assert len(calls) == 1
  with pytest.raises(ValueError):
    transport.post_json(config, "/api/remote/control", {})


def test_manager_gates_and_athena_pid_preserved(configured):
  params, _ = configured
  CP = car.CarParams.new_message()
  assert managed_processes["manage_athenad"].param_name == "AthenadPid"
  for name in ("hylink_worker", "hylink_live", "hylink_relay", "hylink_impact", "hylink_encoderd"):
    process = managed_processes[name]
    assert process.sigkill
    assert not process.should_run(True, params, CP)
  assert managed_processes["camerad"].should_run(True, params, CP)
  assert managed_processes["sensord"].should_run(True, params, CP)
  assert "hylink_ssh" not in managed_processes


def test_os_releases_camera_lease_on_process_death(configured, tmp_path):
  params, _ = configured
  lockpath = tmp_path / "lease"
  code = "; ".join(["import sys,time", "from pathlib import Path",
                    "from openpilot.system.hylink.camera_lease import CameraLease",
                    "lease=CameraLease('test',30,Path(sys.argv[1]))",
                    "assert lease.acquire()", "print('locked',flush=True)", "time.sleep(30)"])
  child = subprocess.Popen([sys.executable, "-c", code, str(lockpath)], stdout=subprocess.PIPE, text=True)
  try:
    assert child.stdout.readline().strip() == "locked"
    other = camera_lease.CameraLease("test-parent", 1, lockpath)
    assert not other.acquire()
    child.send_signal(signal.SIGKILL)
    child.wait(timeout=5)
    assert other.acquire()
    other.release()
  finally:
    if child.poll() is None:
      child.kill()
      child.wait(timeout=5)
    child.stdout.close()
