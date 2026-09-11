"""Heavy work runs only offroad, separately from the lightweight telemetry loop."""
import os
import time

from openpilot.common.params import Params
from openpilot.system.hylink import routes
from openpilot.system.hylink.camera import snapshot_payload
from openpilot.system.hylink.impact import peek_impact_event, remove_impact_event
from openpilot.system.hylink.policy import UploadBackoff
from openpilot.system.hylink.runtime import CONFIG_PATH, media_ready, offroad, read_config, read_json, write_json
from openpilot.system.hylink.transport import post_json


def main():
  if hasattr(os, "nice"):
    os.nice(10)
  params = Params()
  due = {"trip": 0.0, "snapshot": 0.0, "impact": 0.0}
  backoffs = {name: UploadBackoff() for name in due}
  while (config := read_config(params)) and offroad(False, params):
    for name in due:
      if not offroad(False, params):
        return
      now = time.monotonic()
      if now < due[name]:
        continue
      due[name] = now + {"trip": 60, "snapshot": 60, "impact": 5}[name]
      try:
        if name == "trip":
          routes.upload_latest(config, lambda: offroad(False, params), post_json)
        elif name == "snapshot" and media_ready(False, params):
          stamp_path = CONFIG_PATH.with_name("snapshot_state.json")
          last_capture = read_json(stamp_path).get("last_success", 0)
          if 0 <= time.time() - last_capture < 3600:  # noqa: TID251 - Persistent epoch across boots.
            continue
          payload = snapshot_payload(config)
          if payload and offroad(False, params):
            post_json(config, "/api/snapshot", payload)
            write_json(stamp_path, {"last_success": time.time()})  # noqa: TID251 - Persistent epoch.
        elif name == "impact":
          event = peek_impact_event()
          if event and offroad(False, params):
            # Publish the IMU event even if no camera is available. Never invent a photograph.
            post_json(config, "/api/impact", {**event, "deviceId": config["device_id"]})
            remove_impact_event(event["id"])
        backoffs[name].success()
      except Exception as exc:
        due[name] = time.monotonic() + backoffs[name].failure_delay()
        print(f"Hylink {name}: {type(exc).__name__}; retry delayed", flush=True)
    time.sleep(1)


if __name__ == "__main__":
  main()
