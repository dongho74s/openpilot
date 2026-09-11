"""Opt-in parking IMU observation; no CAN access or door-lock assumptions."""
import math
import time

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.system.hylink.impact import ImpactDetector, enqueue_impact_event
from openpilot.system.hylink.runtime import impact_ready, media_ready, service_fresh


def sensor_vector(message, field):
  try:
    vector = tuple(float(v) for v in getattr(message, field).v)
    return vector if len(vector) == 3 and all(math.isfinite(v) for v in vector) else None
  except Exception:
    return None


def main():
  params = Params()
  sm = messaging.SubMaster(["accelerometer", "gyroscope"])
  detector = ImpactDetector(arm_delay_s=30)
  last_sample = None
  while impact_ready(False, params):
    sm.update(50)
    now = time.monotonic()
    if not sm.updated["accelerometer"] or not all(service_fresh(sm, n, now, ttl=0.2) for n in sm.services):
      continue
    accel = sensor_vector(sm["accelerometer"], "acceleration")
    gyro = sensor_vector(sm["gyroscope"], "gyroUncalibrated")
    if accel is None or gyro is None:
      continue
    if last_sample is not None and now - last_sample > 0.2:
      detector = ImpactDetector(arm_delay_s=30)
    last_sample = now
    event = detector.update(accel, gyro, now)
    if event is not None and impact_ready(False, params):
      event["captureRequested"] = media_ready(False, params)
      enqueue_impact_event(event)


if __name__ == "__main__":
  main()
