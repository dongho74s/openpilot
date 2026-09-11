"""Small, standard-library-only manager predicates; fail closed on missing state."""
import json
import os
import re
import tempfile
import time
from pathlib import Path

CONFIG_PATH = Path(os.getenv("HYLINK_CONFIG", "/data/hylink/config.json"))
RUNTIME_ROOT = Path(os.getenv("HYLINK_RUNTIME", "/dev/shm/hylink"))
STATE_PATH = RUNTIME_ROOT / "state.json"
CAMERA_PATH = RUNTIME_ROOT / "camera.json"
ENDPOINT = "https://wayon-cloud.hyuklee.workers.dev"
STATE_TTL = 2.5


def read_json(path):
  try:
    if path.stat().st_size > 65536:
      return {}
    value = json.loads(path.read_text())
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError, RecursionError):
    return {}


def write_json(path, value):
  path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
  fd, name = tempfile.mkstemp(prefix="." + path.name, dir=path.parent)
  try:
    with os.fdopen(fd, "w") as handle:
      json.dump(value, handle, allow_nan=False, separators=(",", ":"))
    os.replace(name, path)
  finally:
    if os.path.exists(name):
      os.unlink(name)


def param_text(params, key):
  value = params.get(key)
  return value.decode().strip() if isinstance(value, bytes) else str(value or "").strip()


def read_config(params=None):
  config = read_json(CONFIG_PATH)
  # Separate from legacy /data/wayon_cloud: never inherit another car's credentials.
  if (config.get("enabled") is not True or config.get("endpoint") != ENDPOINT
      or not re.fullmatch(r"[0-9a-f]{16}", str(config.get("device_id", "")))
      or not re.fullmatch(r"wayon_[A-Za-z0-9_-]{32,128}", str(config.get("token", "")))):
    return {}
  if params is not None and config["device_id"] != param_text(params, "DongleId"):
    return {}
  return config


def enabled(started, params, CP):
  return bool(read_config(params))


def fresh_record(path, ttl=STATE_TTL):
  record = read_json(path)
  try:
    age = time.monotonic() - float(record["at"])
    if not 0 <= age <= ttl or int(record["pid"]) <= 1:
      return {}
    os.kill(int(record["pid"]), 0)
    return record
  except (OSError, KeyError, TypeError, ValueError, OverflowError):
    return {}


def offroad(started, params, CP=None):
  return bool(not started and read_config(params) and params.get_bool("IsOffroad")
              and not params.get_bool("IsOnroad") and fresh_record(STATE_PATH).get("offroad") is True)


def media_ready(started, params, CP=None):
  return (offroad(started, params, CP) and read_config(params).get("media_enabled") is True
          and bool(params.get("UseWideCamera", return_default=True)) and params.get_int("HardwareC3xLite") == 0)


def impact_ready(started, params, CP=None):
  return offroad(started, params, CP) and read_config(params).get("impact_enabled") is True


def camera_requested(started, params, CP=None):
  return bool(media_ready(started, params, CP) and not params.get_bool("IsDriverViewEnabled")
              and not params.get_bool("IsTakingSnapshot") and fresh_record(CAMERA_PATH).get("active") is True)


def stream_requested(started, params, CP=None):
  return camera_requested(started, params, CP) and fresh_record(CAMERA_PATH).get("stream") is True


def service_fresh(sm, name, now=None, ttl=2.5):
  now = time.monotonic() if now is None else now
  return bool(sm.seen.get(name, False) and sm.valid.get(name, False)
              and 0 <= now - sm.recv_time.get(name, 0.0) <= ttl)
