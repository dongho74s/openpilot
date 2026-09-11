import bz2
import json
from pathlib import Path

import pytest
import zstandard

from openpilot.cereal import messaging
from openpilot.system.hylink import routes
from openpilot.system.hylink.runtime import read_json


def synthetic_route(tmp_path):
  directory = tmp_path / "2026-09-11--12-00-00--0"
  directory.mkdir()
  events = []
  init = messaging.new_message("initData", valid=True)
  init.logMonoTime = 1
  events.append(init.to_bytes())
  for t in range(21):
    for name in ("carState", "selfdriveState", "gpsLocation"):
      msg = messaging.new_message(name, valid=True)
      msg.logMonoTime = int((100 + t) * 1e9)
      data = getattr(msg, name)
      if name == "carState":
        data.vEgo = 10
        data.canValid = True
      elif name == "selfdriveState":
        data.active = True
      else:
        data.hasFix = True
        data.latitude = 37 + t * 0.0001
        data.longitude = 127
        data.horizontalAccuracy = 3
        data.unixTimestampMillis = 1800000000000 + t * 1000
      events.append(msg.to_bytes())
  path = directory / "qlog.bz2"
  path.write_bytes(bz2.compress(b"".join(events)))
  return {"route_name": "2026-09-11--12-00-00", "segments": [(0, directory, path)]}


def test_full_cereal_qlog_not_reduced_can_schema(tmp_path):
  group = synthetic_route(tmp_path)
  result = routes.summarize_route_from_logs(group, {}, "device")
  assert result["durationS"] == 20  # initData's boot timestamp must not inflate duration.
  assert 150 < result["distanceM"] < 250
  assert result["maxSpeedMps"] == 10
  assert result["report"]["openpilot"]["activePercent"] == 100
  assert len(result["route"]) >= 2
  assert result["segmentCount"] == 1
  assert "cutInRisk" not in result["report"]
  json.dumps(result, allow_nan=False)


def test_route_cancelled_before_read_and_never_posted(tmp_path, monkeypatch):
  group = synthetic_route(tmp_path)
  group["mtime"] = 1
  monkeypatch.setattr(routes, "recent_route_groups", lambda _: [group])
  monkeypatch.setattr(routes, "ROUTE_STATE_PATH", tmp_path / "state.json")
  with pytest.raises(InterruptedError):
    routes.upload_latest({"device_id": "device"}, lambda: False, lambda *a: pytest.fail("No POST after ignition"))
  assert read_json(routes.ROUTE_STATE_PATH) == {}


def test_failed_upload_not_marked_complete_and_success_deduplicated(tmp_path, monkeypatch):
  group = synthetic_route(tmp_path)
  group["mtime"] = 1
  monkeypatch.setattr(routes, "recent_route_groups", lambda _: [group])
  monkeypatch.setattr(routes, "ROUTE_STATE_PATH", tmp_path / "state.json")
  def fail(*args):
    raise OSError("offline")
  with pytest.raises(OSError):
    routes.upload_latest({"device_id": "device"}, lambda: True, fail)
  assert not routes.ROUTE_STATE_PATH.exists()
  posts = []
  assert routes.upload_latest({"device_id": "device"}, lambda: True, lambda *args: posts.append(args))
  assert not routes.upload_latest({"device_id": "device"}, lambda: True, lambda *a: pytest.fail("Duplicate trip"))
  assert len(posts) == 1


def test_corrupt_qlog_is_not_silently_reported(tmp_path):
  path = tmp_path / "qlog"
  path.write_bytes(b"corrupt")
  with pytest.raises((RuntimeWarning, ValueError)):
    routes.read_bounded_log(path, lambda: True)


@pytest.mark.parametrize("suffix", [".zst", ".bz2", ""])
def test_decompressed_memory_limit(suffix, tmp_path, monkeypatch):
  monkeypatch.setattr(routes, "MAX_DECOMPRESSED_LOG_BYTES", 512)
  raw = b"x" * 1024
  payload = zstandard.ZstdCompressor().compress(raw) if suffix == ".zst" else bz2.compress(raw) if suffix else raw
  path = tmp_path / ("qlog" + suffix)
  path.write_bytes(payload)
  with pytest.raises(ValueError, match="memory budget"):
    routes.read_bounded_log(path, lambda: True)


def test_missing_segment_does_not_become_complete_trip(tmp_path, monkeypatch):
  group = synthetic_route(tmp_path)
  missing = tmp_path / "2026-09-11--12-00-00--1"
  missing.mkdir()
  monkeypatch.setattr(routes.Paths, "log_root", lambda: str(tmp_path))
  assert routes.recent_route_groups(3600)[0]["incomplete"] is True
  missing.rmdir()
  directory = Path(group["segments"][0][1])
  directory.rename(tmp_path / "2026-09-11--12-00-00--2")
  assert routes.recent_route_groups(3600)[0]["incomplete"] is True


def test_onroad_time_budget_uses_monotonic_not_wall_clock(tmp_path, monkeypatch):
  group = synthetic_route(tmp_path)
  calls = iter((0, 100))
  monkeypatch.setattr(routes.time, "monotonic", lambda: next(calls))
  with pytest.raises(InterruptedError):
    routes.summarize_route_from_logs(group, {}, "device")
