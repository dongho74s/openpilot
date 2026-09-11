"""Bounded, offroad-only summaries of the most recent completed route."""
import math
import bz2
import time
import warnings
from datetime import datetime, UTC
from pathlib import Path

from openpilot.system.hardware.hw import Paths
from openpilot.system.hylink.runtime import CONFIG_PATH, read_json, write_json
from openpilot.system.hylink.telemetry import car_state_speed_payload, utc_now, valid_coordinates
from openpilot.system.hylink.drive_quality import StopQualityTracker

LOG_FILE_CANDIDATES = ("qlog.zst", "qlog.bz2", "qlog")  # Do not expand huge rlogs in a parking worker.
DEFAULT_ROUTE_POINT_INTERVAL = 10.0
DEFAULT_ROUTE_POINT_MIN_DISTANCE_M = 15.0
DEFAULT_ROUTE_POINT_LIMIT = 720
ROUTE_STATE_PATH = CONFIG_PATH.with_name("route_state.json")
MAX_DECOMPRESSED_LOG_BYTES = 64 * 1024 * 1024


def read_bounded_log(path, allowed):
  from openpilot.tools.lib.logreader import LogReader
  import zstandard

  # Bound DECOMPRESSED size too. Compressed-size checks alone do not bound memory.
  with path.open("rb") as raw:
    stream = bz2.BZ2File(raw) if path.suffix == ".bz2" else zstandard.ZstdDecompressor().stream_reader(raw) if path.suffix == ".zst" else raw
    chunks = []
    size = 0
    try:
      while True:
        if not allowed():
          raise InterruptedError("Parking log read cancelled")
        chunk = stream.read(1024 * 1024)
        if not chunk:
          break
        size += len(chunk)
        if size > MAX_DECOMPRESSED_LOG_BYTES:
          raise ValueError("Decompressed qlog exceeds memory budget")
        chunks.append(chunk)
    finally:
      if stream is not raw:
        stream.close()
  with warnings.catch_warnings():
    warnings.simplefilter("error", RuntimeWarning)
    return LogReader.from_bytes(b"".join(chunks))


def haversine_m(a, b):
  lat1 = math.radians(a["latitude"])
  lon1 = math.radians(a["longitude"])
  lat2 = math.radians(b["latitude"])
  lon2 = math.radians(b["longitude"])
  d_lat = lat2 - lat1
  d_lon = lon2 - lon1
  h = math.sin(d_lat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(d_lon / 2.0) ** 2
  return 6371000.0 * 2.0 * math.atan2(math.sqrt(h), math.sqrt(max(0.0, 1.0 - h)))

def route_distance_m(route):
  return sum(haversine_m(route[i - 1], route[i]) for i in range(1, len(route)))

def parse_segment_dir_name(name):
  try:
    route_name, segment = name.rsplit("--", 1)
  except ValueError:
    return None

  if not segment.isdigit() or not route_name:
    return None
  return route_name, int(segment)

def route_started_at(route_name):
  try:
    return datetime.strptime(route_name, "%Y-%m-%d--%H-%M-%S").replace(tzinfo=UTC)
  except ValueError:
    return None

def iso_from_timestamp_ms(timestamp_ms):
  try:
    timestamp_ms = int(timestamp_ms)
  except Exception:
    return None

  if timestamp_ms <= 0:
    return None
  return datetime.fromtimestamp(timestamp_ms / 1000.0, UTC).isoformat(timespec="seconds").replace("+00:00", "Z")

def segment_log_file(segment_dir):
  for filename in LOG_FILE_CANDIDATES:
    path = segment_dir / filename
    if path.is_file() and 0 < path.stat().st_size <= 32 * 1024 * 1024:
      return path
  return None

def recent_route_groups(max_age_s):
  log_root = Path(Paths.log_root())
  if not log_root.is_dir():
    return []

  now = time.time()  # noqa: TID251 - Compare filesystem epoch timestamps, not deadlines.
  groups = {}
  for segment_dir in log_root.iterdir():
    if not segment_dir.is_dir():
      continue

    parsed = parse_segment_dir_name(segment_dir.name)
    if parsed is None:
      continue

    route_name, segment = parsed
    group = groups.setdefault(route_name, {"route_name": route_name, "segments": [], "mtime": 0.0, "incomplete": False})
    group["mtime"] = max(group["mtime"], segment_dir.stat().st_mtime)
    log_file = segment_log_file(segment_dir)
    if log_file is None:
      group["incomplete"] = True
      continue

    mtime = max(segment_dir.stat().st_mtime, log_file.stat().st_mtime)
    if now - mtime > max_age_s:
      continue

    group["segments"].append((segment, segment_dir, log_file))
    group["mtime"] = max(group["mtime"], mtime)

  route_groups = []
  for group in groups.values():
    group["segments"].sort(key=lambda item: item[0])
    numbers = [item[0] for item in group["segments"]]
    group["incomplete"] = group["incomplete"] or numbers != list(range(len(numbers)))
    route_groups.append(group)

  return sorted(route_groups, key=lambda item: item["mtime"], reverse=True)

def downsample_route(route, limit):
  if len(route) <= limit:
    return route
  if limit < 2:
    return route[:limit]

  step = (len(route) - 1) / float(limit - 1)
  return [route[round(i * step)] for i in range(limit)]

def gps_point_from_log(gps, timestamp_offset_s, route_start, speed_payload):
  if not gps.hasFix or not valid_coordinates(gps.latitude, gps.longitude) or gps.horizontalAccuracy > 100:
    return None

  point_time = iso_from_timestamp_ms(gps.unixTimestampMillis)
  if point_time is None and route_start is not None and timestamp_offset_s is not None:
    point_time = datetime.fromtimestamp(route_start.timestamp() + timestamp_offset_s, UTC).isoformat(timespec="seconds").replace("+00:00", "Z")

  point = {
    "time": point_time or utc_now(),
    "latitude": float(gps.latitude),
    "longitude": float(gps.longitude),
    "speedMps": speed_payload.get("speedMps") if speed_payload else max(0.0, float(gps.speed)),
    "speedSource": speed_payload.get("source") if speed_payload else "gps",
    "bearingDeg": float(gps.bearingDeg),
  }

  try:
    point["accuracyM"] = float(gps.horizontalAccuracy)
  except Exception:
    pass
  return point

def should_add_route_point(route, point, last_point_mono, point_mono, point_interval_s, min_distance_m):
  if not route:
    return True
  if point_mono is not None and last_point_mono is not None and point_mono - last_point_mono >= point_interval_s:
    return True
  return haversine_m(route[-1], point) >= min_distance_m

def summarize_route_from_logs(route_group, config, device_id, allowed=lambda: True):
  route_name = route_group["route_name"]
  route_start = route_started_at(route_name)
  route = []
  max_speed_sample = 0.0
  deadline = time.monotonic() + 45
  next_check = 0.0

  def check_allowed():
    nonlocal next_check
    now = time.monotonic()
    if now > deadline:
      return False
    # Avoid filesystem/Params reads for every qlog message. The manager separately
    # kills this worker on the onroad edge, even while decompression is running.
    if now >= next_check:
      if not allowed():
        return False
      next_check = now + 0.1
    return True
  first_mono = None
  last_mono = None
  last_point_mono = None
  last_vehicle_speed = None
  stop_quality = StopQualityTracker()
  active_duration_s = 0.0
  last_selfdrive_mono = None
  last_selfdrive_active = False

  point_interval_s = float(config.get("route_point_interval_s", DEFAULT_ROUTE_POINT_INTERVAL))
  min_distance_m = float(config.get("route_point_min_distance_m", DEFAULT_ROUTE_POINT_MIN_DISTANCE_M))
  point_limit = int(config.get("route_point_limit", DEFAULT_ROUTE_POINT_LIMIT))

  if len(route_group["segments"]) > 240:
    raise ValueError("Route exceeds four-hour processing budget")
  for _, _, log_file in route_group["segments"]:
    if not check_allowed():
      raise InterruptedError("Offroad route budget expired")
    reader = read_bounded_log(log_file, check_allowed)
    for msg in reader:
      if not check_allowed():
        raise InterruptedError("Offroad route budget expired")
      which = msg.which()
      if not msg.valid or which not in ("carState", "selfdriveState", "gpsLocation", "gpsLocationExternal"):
        continue
      msg_mono = float(msg.logMonoTime) / 1e9
      first_mono = msg_mono if first_mono is None else min(first_mono, msg_mono)
      last_mono = msg_mono if last_mono is None else max(last_mono, msg_mono)

      if which == "carState":
        try:
          last_vehicle_speed = car_state_speed_payload(msg.carState)
          if last_vehicle_speed.get("speedMps") is not None:
            max_speed_sample = max(max_speed_sample, float(last_vehicle_speed["speedMps"]))
            stop_quality.update(
              msg_mono,
              last_vehicle_speed["speedMps"],
              float(msg.carState.aEgo),
              bool(msg.carState.standstill),
            )
        except Exception:
          pass
        continue

      if which == "selfdriveState":
        if last_selfdrive_mono is not None and last_selfdrive_active:
          active_duration_s += min(1.0, max(0.0, msg_mono - last_selfdrive_mono))
        last_selfdrive_active = bool(msg.selfdriveState.active)
        last_selfdrive_mono = msg_mono
        continue

      if which not in ("gpsLocationExternal", "gpsLocation"):
        continue

      gps = getattr(msg, which)
      timestamp_offset_s = None if first_mono is None else msg_mono - first_mono
      point = gps_point_from_log(gps, timestamp_offset_s, route_start, last_vehicle_speed)
      if point is None:
        continue

      if should_add_route_point(route, point, last_point_mono, msg_mono, point_interval_s, min_distance_m):
        route.append(point)
        if len(route) > 10000:
          route = downsample_route(route, 5000)
        last_point_mono = msg_mono

  if len(route) < 2:
    return None

  duration_s = 0
  if first_mono is not None and last_mono is not None:
    duration_s = max(0, int(last_mono - first_mono))

  distance_m = route_distance_m(route)
  started_at = route[0]["time"]
  ended_at = route[-1]["time"]
  avg_speed_mps = distance_m / duration_s if duration_s > 0 else 0.0
  max_speed_mps = max_speed_sample if max_speed_sample > 0 else max(float(point.get("speedMps") or 0.0) for point in route)
  payload_route = downsample_route(route, point_limit)
  stop_summary = stop_quality.summary()
  openpilot_active_percent = round(active_duration_s / duration_s * 100.0, 1) if duration_s > 0 else 0.0

  return {
    "id": f"{device_id}-{route_name}",
    "deviceId": device_id,
    "startedAt": started_at,
    "endedAt": ended_at,
    "durationS": duration_s,
    "distanceM": distance_m,
    "startLat": route[0]["latitude"],
    "startLon": route[0]["longitude"],
    "endLat": route[-1]["latitude"],
    "endLon": route[-1]["longitude"],
    "source": "openpilotRoute",
    "routeName": route_name,
    "segmentCount": len(route_group["segments"]),
    "avgSpeedMps": avg_speed_mps,
    "maxSpeedMps": max_speed_mps,
    "report": {
      "schemaVersion": "wayon-trip-report-v1",
      "openpilot": {
        "activeDurationS": round(active_duration_s, 1),
        "activePercent": openpilot_active_percent,
      },
      "stopQuality": stop_summary,
    },
    "route": payload_route,
  }

def upload_latest(config, allowed, post):
  groups = recent_route_groups(24 * 3600)
  if not groups or time.time() - groups[0]["mtime"] < 45:  # noqa: TID251 - Filesystem epoch.
    return False
  group = groups[0]
  if group.get("incomplete"):
    return False
  state = read_json(ROUTE_STATE_PATH)
  completed = state.get("uploaded_routes", [])
  if not isinstance(completed, list):
    completed = []
  if group["route_name"] in completed:
    return False
  payload = summarize_route_from_logs(group, {}, config["device_id"], allowed)
  if payload is None or not allowed():
    return False
  post(config, "/api/trips", payload)
  write_json(ROUTE_STATE_PATH, {"uploaded_routes": ([group["route_name"]] + completed)[:100]})
  return True
