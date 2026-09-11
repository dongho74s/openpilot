"""Attach post-event photos once, retry the same media after network failure."""
import json
from datetime import datetime, UTC

from openpilot.system.hylink import runtime
from openpilot.system.hylink.camera import snapshot_payload

PENDING_PATH = runtime.CONFIG_PATH.with_name("impact_pending.json")
MAX_PENDING_BYTES = 6 * 1024 * 1024
CAPTURE_WINDOW_S = 45


def upload_impact(config, event, allowed, media_allowed, post, capture=snapshot_payload):
  if not allowed():
    raise InterruptedError("Not offroad")
  pending = {}
  try:
    if PENDING_PATH.stat().st_size <= MAX_PENDING_BYTES:
      pending = json.loads(PENDING_PATH.read_text())
  except (OSError, ValueError):
    pass
  if not isinstance(pending, dict) or pending.get("id") != event["id"]:
    pending = {"id": event["id"], "deviceId": config["device_id"], "captureStatus": "failed", "captureAttempts": 0}
    if event.get("captureRequested") and media_allowed():
      try:
        age = (datetime.now(UTC) - datetime.fromisoformat(event["detectedAt"].replace("Z", "+00:00"))).total_seconds()
        if 0 <= age <= CAPTURE_WINDOW_S:
          pending["captureAttempts"] = 1
          payload = capture(config)
          if payload:
            pending.update(payload)
            pending["captureStatus"] = "complete" if all(payload.get(k) for k in ("wideJpegBase64", "driverJpegBase64")) else "partial"
        else:
          pending["reason"] = "capture_window_expired"
      except (BlockingIOError, InterruptedError, OSError, ValueError):
        pending["reason"] = "camera_unavailable"
    if len(json.dumps(pending)) > MAX_PENDING_BYTES:
      pending = {"id": event["id"], "deviceId": config["device_id"], "captureStatus": "failed", "reason": "size_limit"}
    runtime.write_json(PENDING_PATH, pending)
  if not allowed():
    raise InterruptedError("Not offroad")
  post(config, "/api/impact", {**event, "deviceId": config["device_id"]})
  if event.get("captureRequested"):
    if not allowed():
      raise InterruptedError("Not offroad")
    # Consent can be withdrawn while a captured image waits for internet access.
    if not media_allowed():
      pending = {"id": event["id"], "deviceId": config["device_id"], "captureStatus": "failed", "reason": "media_disabled"}
    post(config, "/api/impact-media", pending)
  PENDING_PATH.unlink(missing_ok=True)
