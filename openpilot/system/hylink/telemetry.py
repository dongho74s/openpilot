"""Read-only telemetry using the current wip cereal schema."""
import json
import math
import time
from datetime import datetime, UTC
from pathlib import Path

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.system.hylink.runtime import CONFIG_PATH, param_text, read_config, read_json, service_fresh, write_json
from openpilot.system.hylink.policy import UploadBackoff
from openpilot.system.hylink.drive_quality import telemetry_signature
from openpilot.system.hylink.transport import post_json

SERVICES = ["deviceState", "pandaStates", "gpsLocation", "gpsLocationExternal", "carState", "selfdriveState"]
GPS_CACHE = CONFIG_PATH.with_name("last_gps.json")


def utc_now():
  return datetime.now(UTC).isoformat(timespec="seconds").replace("+00:00", "Z")

def enum_name(value):
  try:
    return str(value).split(".")[-1]
  except Exception:
    return ""

def numeric_list(values):
  try:
    return [float(value) for value in values]
  except Exception:
    return []

def device_details_payload(device_state):
  temperatures = {
    "cpu": numeric_list(device_state.cpuTempC),
    "gpu": numeric_list(device_state.gpuTempC),
    "dsp": float(device_state.dspTempC),
    "memory": float(device_state.memoryTempC),
    "modem": numeric_list(device_state.modemTempC),
    "pmic": numeric_list(device_state.pmicTempC),
    "intake": float(device_state.intakeTempC),
    "exhaust": float(device_state.exhaustTempC),
    "gnss": float(device_state.gnssTempC),
    "bottomSoc": float(device_state.bottomSocTempC),
    "max": float(device_state.maxTempC),
  }
  try:
    temperatures["zones"] = [
      {"name": str(zone.name), "tempC": float(zone.temp)}
      for zone in device_state.thermalZones
    ]
  except Exception:
    temperatures["zones"] = []

  return {
    "type": enum_name(device_state.deviceType),
    "network": {
      "type": enum_name(device_state.networkType),
      "strength": enum_name(device_state.networkStrength),
      "metered": bool(device_state.networkMetered),
    },
    "usage": {
      "freeSpacePercent": float(device_state.freeSpacePercent),
      "memoryPercent": int(device_state.memoryUsagePercent),
      "gpuPercent": int(device_state.gpuUsagePercent),
      "cpuPercent": [int(value) for value in device_state.cpuUsagePercent],
    },
    "power": {
      "drawW": float(device_state.powerDrawW),
      "somDrawW": float(device_state.somPowerDrawW),
      "offroadUsageUwh": int(device_state.offroadPowerUsageUwh),
      "offroadUsageWh": float(device_state.offroadPowerUsageUwh) / 1_000_000.0,
      "carBatteryCapacityUwh": int(device_state.carBatteryCapacityUwh),
      "carBatteryCapacityWh": float(device_state.carBatteryCapacityUwh) / 1_000_000.0,
    },
    "thermal": {
      "status": enum_name(device_state.thermalStatus),
      "fanPercent": int(device_state.fanSpeedPercentDesired),
      "temperaturesC": temperatures,
    },
    "screenBrightnessPercent": int(device_state.screenBrightnessPercent),
  }

def car_state_speed_payload(car_state):
  v_ego_cluster = float(car_state.vEgoCluster)
  if v_ego_cluster != 0.0:
    return {"speedMps": max(0.0, v_ego_cluster), "source": "vEgoCluster"}
  return {"speedMps": max(0.0, float(car_state.vEgo)), "source": "vEgo"}

def vehicle_details_payload(sm, started):
  if not started:
    return {"available": False, "reason": "offroad"}
  if not service_fresh(sm, "carState"):
    return {"available": False, "reason": "stale"}

  try:
    car_state = sm["carState"]
    cruise = car_state.cruiseState
    speed = car_state_speed_payload(car_state)
    return {
      "available": True,
      "speedMps": speed["speedMps"],
      "speedKph": speed["speedMps"] * 3.6,
      "speedSource": speed["source"],
      "rawSpeedMps": float(car_state.vEgoRaw),
      "accelerationMps2": float(car_state.aEgo),
      "yawRateRadPerSec": float(car_state.yawRate),
      "standstill": bool(car_state.standstill),
      "gear": enum_name(car_state.gearShifter),
      "steeringAngleDeg": float(car_state.steeringAngleDeg),
      "steeringRateDegPerSec": float(car_state.steeringRateDeg),
      "steeringPressed": bool(car_state.steeringPressed),
      "gasPressed": bool(car_state.gasPressed),
      "brakePressed": bool(car_state.brakePressed),
      "parkingBrake": bool(car_state.parkingBrake),
      "brakeHoldActive": bool(car_state.brakeHoldActive),
      "leftBlinker": bool(car_state.leftBlinker),
      "rightBlinker": bool(car_state.rightBlinker),
      "doorOpen": bool(car_state.doorOpen),
      "seatbeltUnlatched": bool(car_state.seatbeltUnlatched),
      "can": {
        "valid": bool(car_state.canValid),
        "timeout": bool(car_state.canTimeout),
        "errorCounter": int(car_state.canErrorCounter),
      },
      "steeringFault": {
        "temporary": bool(car_state.steerFaultTemporary),
        "permanent": bool(car_state.steerFaultPermanent),
      },
      "cruise": {
        "enabled": bool(cruise.enabled),
        "available": bool(cruise.available),
        "standstill": bool(cruise.standstill),
        "nonAdaptive": bool(cruise.nonAdaptive),
        "speedMps": float(cruise.speed),
        "speedKph": float(cruise.speed) * 3.6,
        "clusterSpeedMps": float(cruise.speedCluster),
      },
    }
  except Exception:
    return {"available": False, "reason": "carState_unavailable"}

def openpilot_details_payload(sm):
  if not service_fresh(sm, "selfdriveState"):
    return {"available": False}
  try:
    state = sm["selfdriveState"]
    return {
      "available": True,
      "state": enum_name(state.state),
      "enabled": bool(state.enabled),
      "active": bool(state.active),
      "engageable": bool(state.engageable),
      "experimentalMode": bool(state.experimentalMode),
      "personality": enum_name(state.personality),
      "alert": {
        "text1": str(state.alertText1),
        "text2": str(state.alertText2),
        "type": str(state.alertType),
        "status": enum_name(state.alertStatus),
        "size": enum_name(state.alertSize),
        "sound": enum_name(state.alertSound),
        "hudVisual": enum_name(state.alertHudVisual),
      },
    }
  except Exception:
    return {"available": False}

def panda_details_payload(panda):
  if panda is None:
    return None
  # Panda publishes millivolts and milliamps in this wip schema.
  data = panda.to_dict()
  keys = ("ignitionLine", "ignitionCan", "faultStatus", "faults", "heartbeatLost", "harnessStatus",
          "controlsAllowed", "safetyModel", "safetyParam", "safetyRxInvalid", "safetyTxBlocked",
          "safetyRxChecksInvalid", "rxBufferOverflow", "txBufferOverflow", "spiErrorCount")
  return {**{key: data[key] for key in keys if key in data},
          "type": str(panda.pandaType), "uptimeS": int(panda.uptime),
          "voltageV": panda.voltage / 1000.0, "currentMa": float(panda.current)}


def valid_coordinates(latitude, longitude):
  try:
    return math.isfinite(float(latitude)) and math.isfinite(float(longitude)) and -90 <= latitude <= 90 and -180 <= longitude <= 180
  except (TypeError, ValueError):
    return False


def gps_payload(sm, params):
  candidates = []
  for name in ("gpsLocation", "gpsLocationExternal"):
    gps = sm[name]
    age_ms = time.time() * 1000 - gps.unixTimestampMillis  # noqa: TID251 - GNSS Unix epoch.
    if (service_fresh(sm, name, ttl=5) and gps.hasFix and -5000 <= age_ms <= 15000
        and valid_coordinates(gps.latitude, gps.longitude)):
      candidates.append((sm.recv_time[name], gps))
  if candidates:
    gps = max(candidates, key=lambda item: item[0])[1]
    return {"latitude": float(gps.latitude), "longitude": float(gps.longitude),
            "bearingDeg": float(gps.bearingDeg), "accuracyM": float(gps.horizontalAccuracy),
            "timestampMillis": int(gps.unixTimestampMillis), "satellites": int(gps.satelliteCount),
            "source": str(gps.source), "fresh": True}
  cached = read_json(GPS_CACHE)
  if not cached:
    try:
      cached = json.loads(param_text(params, "LastGPSPosition"))
      cached["timestampMillis"] = int(Path(params.get_param_path("LastGPSPosition")).stat().st_mtime * 1000)
      cached["bearingDeg"] = cached.get("bearing", 0.0)
    except (OSError, ValueError, TypeError):
      cached = {}
  if valid_coordinates(cached.get("latitude"), cached.get("longitude")):
    return {**cached, "source": "lastGpsPosition", "fresh": False}
  return {"source": "unavailable", "fresh": False}


def telemetry_payload(sm, params, device_id):
  fresh = {name: service_fresh(sm, name) for name in SERVICES}
  ds = sm["deviceState"]
  started = bool(ds.started) if fresh["deviceState"] else None
  pandas = list(sm["pandaStates"]) if fresh["pandaStates"] else []
  known_pandas = [p for p in pandas if str(p.pandaType) != "unknown"]
  panda = panda_details_payload(known_pandas[0] if known_pandas else None)
  ignition = any(p.ignitionLine or p.ignitionCan for p in known_pandas) if known_pandas else None
  vehicle = vehicle_details_payload(sm, started) if started is not None else {"available": False, "reason": "stale"}
  op = openpilot_details_payload(sm) if started else {"available": False, "reason": "offroad" if started is False else "stale"}
  gps = gps_payload(sm, params)
  can = vehicle.get("can") or {}
  connections = {
    "comma": {"state": "ok" if fresh["deviceState"] else "stale"},
    "panda": {"state": "ok" if panda and not panda["heartbeatLost"] and panda["faultStatus"] == "none" else "unavailable"},
    "vehicleCan": {"state": ("ok" if can.get("valid") and not can.get("timeout") else "unavailable") if started else "offroad"},
    "gps": {"state": "ok" if gps.get("fresh") else "cached" if gps.get("latitude") is not None else "unavailable"},
    "openpilot": {"state": "ok" if op.get("available") else "offroad" if started is False else "unavailable"},
  }
  voltage = panda["voltageV"] if panda else None
  current = panda["currentMa"] if panda else None
  payload = {
    "schemaVersion": "wayon-telemetry-v3", "deviceId": device_id, "dongleId": device_id,
    "updatedAt": utc_now(), "onroad": started, "ignition": ignition, "enabled": op.get("enabled"),
    "systemState": "onroad" if started else "offroad" if started is False and ignition is False else "disconnected",
    "voltageV": voltage, "currentMa": current,
    "powerW": voltage * current / 1000 if voltage is not None and current is not None else None,
    "devicePowerW": float(ds.powerDrawW) if fresh["deviceState"] else None,
    "thermalStatus": str(ds.thermalStatus) if fresh["deviceState"] else "unknown",
    "fanPercent": int(ds.fanSpeedPercentDesired) if fresh["deviceState"] else None,
    "device": device_details_payload(ds) if fresh["deviceState"] else None,
    "panda": panda, "vehicle": vehicle, "openpilot": op, "gps": gps,
    "vehicleSpeedMps": vehicle.get("speedMps"), "vehicleSpeedSource": vehicle.get("speedSource"),
    "connections": connections,
    "timestamps": {"generatedAt": utc_now(), "services": {
      name: {"seen": bool(sm.seen[name]), "alive": fresh[name],
             "ageSeconds": max(0, time.monotonic() - sm.recv_time[name]) if sm.seen[name] else None}
      for name in SERVICES}},
  }
  return payload


def main():
  params = Params()
  # Poll the low-rate device service, not 100 Hz carState, to avoid an onroad
  # filesystem/config busy loop. SubMaster still conflates the latest car data.
  sm = messaging.SubMaster(SERVICES, poll="deviceState")
  backoff = UploadBackoff()
  next_upload = next_change = 0.0
  last_signature = None
  while config := read_config(params):
    sm.update(1000)
    now = time.monotonic()
    if now < next_change:
      continue
    payload = telemetry_payload(sm, params, config["device_id"])
    signature = telemetry_signature(payload)
    if now < next_upload and signature == last_signature:
      continue
    try:
      # Reject NaN and Infinity before any network call.
      post_json(config, "/api/telemetry", payload)
      if payload["gps"].get("fresh"):
        write_json(GPS_CACHE, payload["gps"])
      backoff.success()
      last_signature = signature
      next_upload = now + (30 if payload["onroad"] else 300)
      next_change = now + (5 if payload["onroad"] else 15)
    except Exception as exc:
      next_change = now + backoff.failure_delay()
      print(f"Hylink telemetry: {type(exc).__name__}; retry delayed", flush=True)


if __name__ == "__main__":
  main()
