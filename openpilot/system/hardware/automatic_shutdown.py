"""Fresh, cancellable hardware shutdown requests, separate from manual poweroff."""
import math
import time

from openpilot.cereal import log

REQUEST_KEY = "AutomaticShutdownRequest"
MAX_SAMPLE_AGE_S = 1.5
LOW_VOLTAGE_MV = 11000
# Below this a reading is not a 12 V car battery: USB power from a phone charger or laptop (5 V, or 9 V with USB PD),
# or no supply. A car battery this low at rest is already flat. Such readings never count as a low car battery, even
# when the harness status reads connected.
CAR_BATTERY_MIN_MV = 9500


def car_battery_voltage(voltage):
  return voltage is not None and CAR_BATTERY_MIN_MV <= voltage


def message_is_fresh(sm, service, now):
  return (sm.seen[service] and sm.valid[service] and sm.alive[service] and
          0 <= now - sm.recv_time[service] <= MAX_SAMPLE_AGE_S and
          0 <= now - sm.logMonoTime[service] / 1e9 <= MAX_SAMPLE_AGE_S)


def automatic_shutdown_reason(request, sm, now):
  if not isinstance(request, dict) or request.get("reason") not in ("low_voltage", "offroad_timeout"):
    return None
  requested_at = request.get("monotonicTime")
  if (type(requested_at) not in (int, float) or not math.isfinite(requested_at) or
      not 0 <= now - requested_at <= MAX_SAMPLE_AGE_S):
    return None
  if not all(message_is_fresh(sm, service, now) for service in ("deviceState", "pandaStates")):
    return None
  pandas = sm["pandaStates"]
  if (sm["deviceState"].started or not pandas or
      any(p.pandaType == log.PandaState.PandaType.unknown or p.ignitionLine or p.ignitionCan for p in pandas)):
    return None
  if request["reason"] == "low_voltage":
    if not message_is_fresh(sm, "peripheralState", now):
      return None
    peripheral = sm["peripheralState"]
    if (pandas[0].harnessStatus == log.PandaState.HarnessStatus.notConnected or
        peripheral.pandaType == log.PandaState.PandaType.unknown or
        not car_battery_voltage(peripheral.voltage) or not peripheral.voltage < LOW_VOLTAGE_MV):
      return None
  return request["reason"]


def try_automatic_shutdown(sm, params, shutdown, clock=time.monotonic, notify=None):
  reason = automatic_shutdown_reason(params.get(REQUEST_KEY), sm, clock())
  if reason is None:
    return None
  # Keep pandad/hardwared alive until this final drain and poweroff request.
  # Do not latch DoShutdown: a recovered voltage or new ignition must win.
  sm.update(0)
  if automatic_shutdown_reason(params.get(REQUEST_KEY), sm, clock()) != reason:
    return None
  if notify is not None:
    try:
      notify(reason, sm, params)
    except Exception:
      # A notification failure must never disable battery protection.
      pass
    # Network delivery can overlap ignition or voltage recovery. Drain again,
    # leaving all managed processes alive when automatic poweroff is cancelled.
    sm.update(0)
    if automatic_shutdown_reason(params.get(REQUEST_KEY), sm, clock()) != reason:
      return None
  shutdown()
  return reason
