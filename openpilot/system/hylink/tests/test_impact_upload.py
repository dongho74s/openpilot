from datetime import datetime, UTC, timedelta

import pytest

from openpilot.system.hylink import impact_upload


@pytest.fixture
def pending(tmp_path, monkeypatch):
  path = tmp_path / "pending.json"
  monkeypatch.setattr(impact_upload, "PENDING_PATH", path)
  return path


def event(age=0):
  return {"id": "test-event", "captureRequested": True, "detectedAt": (datetime.now(UTC) - timedelta(seconds=age)).isoformat()}


def test_event_and_both_photos_link_to_same_id(pending):
  calls = []
  def post(config, path, body):
    calls.append((path, body))
  impact_upload.upload_impact({"device_id": "test"}, event(), lambda: True, lambda: True, post,
                              lambda _: {"wideJpegBase64": "wide", "driverJpegBase64": "driver"})
  assert [p for p, _ in calls] == ["/api/impact", "/api/impact-media"]
  assert calls[0][1]["id"] == calls[1][1]["id"] == "test-event"
  assert calls[1][1]["captureStatus"] == "complete" and not pending.exists()


def test_upload_retry_reuses_original_photo_and_consent_withdrawal(pending):
  def failure(*a):
    raise OSError("offline")
  with pytest.raises(OSError):
    impact_upload.upload_impact({"device_id": "test"}, event(), lambda: True, lambda: True, failure,
                                lambda _: {"wideJpegBase64": "original"})
  assert pending.exists()
  calls = []
  impact_upload.upload_impact({"device_id": "test"}, event(), lambda: True, lambda: False,
                              lambda c, p, b: calls.append(b), lambda _: pytest.fail("Do not recapture"))
  assert calls[-1]["captureStatus"] == "failed" and "wideJpegBase64" not in calls[-1]


@pytest.mark.parametrize("age,allowed", [(120, True), (0, False)])
def test_never_captures_old_event_or_onroad(pending, age, allowed):
  def capture(_):
    pytest.fail("Do not capture")
  def run():
    impact_upload.upload_impact({"device_id": "test"}, event(age), lambda: allowed, lambda: True, lambda *a: None, capture)
  if allowed:
    run()
  else:
    with pytest.raises(InterruptedError):
      run()
