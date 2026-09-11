"""Fixed HTTPS destination; bounded requests; never log credentials or response bodies."""
import json
import requests

from openpilot.system.hylink.runtime import ENDPOINT


def post_json(config, path, payload):
  if config.get("endpoint") != ENDPOINT or path not in ("/api/telemetry", "/api/trips", "/api/snapshot", "/api/impact", "/api/impact-media"):
    raise ValueError("Unexpected Hylink destination")
  response = requests.post(ENDPOINT + path, data=json.dumps(payload, allow_nan=False),
                           headers={"Authorization": "Bearer " + config["token"],
                                    "Content-Type": "application/json", "User-Agent": "hylink-wip/1"},
                           timeout=(5, 10), allow_redirects=False)
  if not 200 <= response.status_code < 300:
    raise RuntimeError(f"Hylink HTTP {response.status_code}")
  return response.json() if response.content else {}
