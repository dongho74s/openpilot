"""Independent 2 Hz readiness heartbeat: never waits for network or log parsing."""
import os
import time

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.system.hylink.runtime import STATE_PATH, service_fresh, write_json


def observed_offroad(sm, params, now=None):
  if not params.get_bool("IsOffroad") or params.get_bool("IsOnroad"):
    return False
  if not all(service_fresh(sm, name, now) for name in ("deviceState", "pandaStates")):
    return False
  device = sm["deviceState"]
  pandas = list(sm["pandaStates"])
  if (device.started or not pandas or any(str(p.pandaType) == "unknown" for p in pandas)
      or any(p.ignitionLine or p.ignitionCan or p.heartbeatLost or str(p.faultStatus) != "none" for p in pandas)):
    return False
  # Parking features must not hold the device awake through low voltage/thermal shutdown.
  return (all(p.voltage >= 12000 for p in pandas) and str(device.thermalStatus) == "green"
          and device.maxTempC < 80)


def main():
  params = Params()
  sm = messaging.SubMaster(["deviceState", "pandaStates"])
  try:
    while True:
      sm.update(0)
      write_json(STATE_PATH, {"pid": os.getpid(), "at": time.monotonic(),
                             "offroad": observed_offroad(sm, params)})
      time.sleep(0.5)
  finally:
    STATE_PATH.unlink(missing_ok=True)


if __name__ == "__main__":
  main()
