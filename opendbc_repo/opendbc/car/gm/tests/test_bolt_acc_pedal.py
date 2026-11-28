"""Bolt ACC with a comma pedal (CHEVROLET_BOLT_ACC_PEDAL): the Bolt EUV's camera long (0x2CB, 0x315, 0x370, carrot's
LongControl and settings), with the pedal only tapping the accelerator to leave the ECM's ACC standstill (user request
2026-11-18). Real DBC, packer, parser, CarState and CarController; no device I/O."""
import hashlib
import json
from collections import defaultdict
from types import SimpleNamespace as NS

import pytest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.gm import gmcan
from opendbc.car.gm.fingerprints import FINGERPRINTS
from opendbc.car.gm.values import CAR, DBC, CanBus, CruiseButtons, GMFlags, GMSafetyFlags
from opendbc.car.gm.tests.test_display_speed import MemoryParams, runtime  # noqa: F401

PEDAL_MSG = 0x201

# ---------------------------------------------------------------------------------------------------------------------
# Message streams of the existing Bolts, frozen before the port (2026-11-17, base db68bf84 + the platform addition).
# Each case runs the real CarController (or CarInterface/CarState) through a fixed 640-frame drive and hashes every
# frame's CAN sends and actuator outputs; the port must not change a single byte for these cars.


def bolt_fingerprint(candidate, pedal, camera=True):
  source = CAR.CHEVROLET_BOLT_EUV if candidate == CAR.CHEVROLET_BOLT_ACC_PEDAL else candidate
  fingerprint = gen_empty_fingerprint()
  fingerprint[0] = dict(FINGERPRINTS[source][0])
  if pedal:
    fingerprint[0][PEDAL_MSG] = 6
  fingerprint[2] = {0x180: 4, 0x320: 6, 0x370: 6} if camera else {}  # factory camera present
  return fingerprint


def bolt_cp(runtime, candidate, pedal, alpha, camera=True):
  return runtime.interface.CarInterface.get_params(candidate, bolt_fingerprint(candidate, pedal, camera), [], alpha, True, False)


def memory_params(**values):
  class Custom(MemoryParams):
    def __init__(self, *args):
      super().__init__(*args)
      self.values.update(values)
  return Custom


ACCEL_STEPS = [1.6, 0.8, 0.3, 0.0, -0.05, -0.4, -0.9, -1.6, -2.4, -3.5, -0.2, 0.5]
STREAM_FRAMES = 640


def drive_inputs(tick):
  """One fixed drive: off, engaged cruise with every accel band and an override, a stop, a launch, off again."""
  enabled = 100 <= tick < 540
  long_active = enabled and not (200 <= tick < 210)
  if tick < 100:
    v_ego, accel, state = 8.0, 0.0, "off"
  elif tick < 400:
    v_ego, accel, state = max(0.0, 20.0 - (tick - 100) * 0.07), ACCEL_STEPS[(tick // 25) % len(ACCEL_STEPS)], "pid"
  elif tick < 480:
    v_ego, accel, state = 0.0, -0.25 - 0.01 * min(tick - 400, 30), "stopping"
  elif tick < 540:
    v_ego, accel, state = (tick - 480) * 0.03, 0.6, "starting" if tick < 495 else "pid"
  else:
    v_ego, accel, state = 6.0, 0.0, "off"
  if not long_active:
    state = "off"

  cc = structs.CarControl(enabled=enabled, longActive=long_active, latActive=enabled)
  cc.actuators.accel = accel
  cc.actuators.longControlState = state
  cc.cruiseControl.resume = 470 <= tick < 480
  cc.hudControl.setSpeed = 25.0 if tick < 300 else 80.0
  cc.hudControl.leadVisible = tick % 50 < 25
  cc.hudControl.leadDistanceBars = 2
  cc.hudControl.visualAlert = "fcw" if 150 <= tick < 160 else "none"
  cc.orientationNED = [0.0, 0.03 if 250 <= tick < 330 else -0.02, 0.0]

  out = structs.CarState(vEgo=v_ego, aEgo=accel * 0.8, standstill=v_ego < 0.05)
  out.cruiseState.available = tick < 620
  out.cruiseState.enabled = (30 <= tick < 45) or (560 <= tick < 600) or (enabled and tick % 97 < 3)
  out.cruiseState.standstill = 440 <= tick < 470
  out.cruiseState.speed = 20.0
  out.gasPressed = 220 <= tick < 230 or 60 <= tick < 65
  out.brakePressed = 395 <= tick < 398
  out.regenBraking = 600 <= tick < 610
  out.activateCruise = 1 if 80 <= tick < 90 else 0
  out.gearShifter = "drive"
  cs = NS(out=out, loopback_lka_steering_cmd_ts_nanos=0, loopback_lka_steering_cmd_updated=False,
          cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0, buttons_counter=(tick // 10) % 4,
          pcm_acc_status=1 if out.cruiseState.enabled else 0, pscm_status=defaultdict(int),
          cam_ascm_2cb_counter=0, cam_ascm_2cb_counter_updated=False, cam_ascm_2cb_counter_ts_nanos=0,
          cam_stock_long_active=None, cam_stock_long_cancel=False, cam_acc_status=None, cam_fcw_alert=0,
          stock_fcw_alert=2 if 170 <= tick < 175 else 0)
  return cc, cs


def controller_stream(runtime, monkeypatch, candidate, pedal, alpha, pitch):
  monkeypatch.setattr(runtime.controller, "Params", memory_params(LongPitch=pitch))
  controller = runtime.controller.CarController(DBC[candidate], bolt_cp(runtime, candidate, pedal, alpha))
  stream = []
  for tick in range(STREAM_FRAMES):
    cc, cs = drive_inputs(tick)
    cs.CP = controller.CP
    output, sends = controller.update(cc.as_reader(), cs, 1_000_000_000 + tick * 10_000_000)
    stream.append([sorted([addr, bytes(dat).hex(), bus] for addr, dat, bus in sends),
                   round(float(output.gas), 6), round(float(output.brake), 6), round(float(output.accel), 6)])
  return stream


def carstate_frames(packer, tick, pedal):
  v_kph = max(0.0, 40.0 - tick * 0.12)
  cruise_state = 1 if 50 <= tick < 120 else 4 if 120 <= tick < 140 else 3 if 300 <= tick < 305 else 0
  frames = [
    packer.make_can_msg("EBCMWheelSpdFront", 0, {"FLWheelSpd": v_kph, "FRWheelSpd": v_kph}),
    packer.make_can_msg("EBCMWheelSpdRear", 0, {"RLWheelSpd": v_kph, "RRWheelSpd": v_kph}),
    packer.make_can_msg("ECMEngineStatus", 0, {"CruiseMainOn": int(tick < 280), "BrakePressed": int(200 <= tick < 210)}),
    packer.make_can_msg("AcceleratorPedal2", 0, {"CruiseState": cruise_state, "AcceleratorPedal2": 30 if 230 <= tick < 240 else 0}),
    packer.make_can_msg("ECMCruiseControl", 0, {"CruiseActive": int(60 <= tick < 90), "CruiseSetSpeed": 72.0}),
    packer.make_can_msg("EBCMRegenPaddle", 0, {"RegenPaddle": 2 if 250 <= tick < 260 else 0}),
    packer.make_can_msg("ECMPRDNL2", 0, {"PRNDL2": 6 if 260 <= tick < 270 else 4}),
    packer.make_can_msg("EBCMFrictionBrakeStatus", 0, {"FrictionBrakeUnavailable": int(310 <= tick < 315)}),
    packer.make_can_msg("ASCMSteeringButton", 0, {"ACCButtons": 3 if 20 <= tick < 25 else 6 if 150 <= tick < 152 else 1}),
    packer.make_can_msg("ASCMActiveCruiseControlStatus", 2, {"ACCCmdActive": int(100 <= tick < 160), "ACCSpeedSetpoint": 90.0,
                                                             "ACCCruiseState": 2, "FCWAlert": 2 if 170 <= tick < 180 else 0}),
    packer.make_can_msg("AEBCmd", 2, {"AEBCmdActive": int(190 <= tick < 195)}),
  ]
  if pedal:
    gas = 30.0 if 240 <= tick < 250 else 21.5 if 250 <= tick < 255 else 2.0
    frames.append(packer.make_can_msg("GAS_SENSOR", 0, {"INTERCEPTOR_GAS": gas, "INTERCEPTOR_GAS2": gas}))
  return frames


CARSTATE_FIELDS = ("vEgo", "standstill", "gas", "gasPressed", "brakePressed", "regenBraking", "accFaulted", "stockFcw",
                   "stockAeb", "gearShifter")


def carstate_stream(runtime, monkeypatch, candidate, pedal, alpha, camera=True, extra=None):
  cp = bolt_cp(runtime, candidate, pedal, alpha, camera)
  ci = runtime.interface.CarInterface.__new__(runtime.interface.CarInterface)
  ci.CS = runtime.state.CarState(cp)
  ci.can_parsers = ci.CS.get_can_parsers(cp)
  ci.v_ego_cluster_seen = False
  ci.CS.update(ci.can_parsers)
  packer = runtime.CANPacker(DBC[candidate][Bus.pt])
  stream = []
  for tick in range(320):
    ret = ci.update([(1_000_000_000 + tick * 10_000_000, carstate_frames(packer, tick, pedal))])
    row = {f: (round(float(getattr(ret, f)), 5) if isinstance(getattr(ret, f), float) else str(getattr(ret, f))) for f in CARSTATE_FIELDS}
    row.update(enabled=ret.cruiseState.enabled, available=ret.cruiseState.available, speed=round(ret.cruiseState.speed, 5),
               cruise_standstill=ret.cruiseState.standstill, events=[str(e.type) + str(e.pressed) for e in ret.buttonEvents],
               single_pedal=bool(ci.CS.single_pedal_mode))
    if extra is not None:
      row.update(extra(ci.CS))
    stream.append(row)
  return stream


BASELINE_CASES = [(car, pedal, alpha, pitch) for car in (CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_EUV)
                  for pedal, alpha in ((True, False), (False, True), (False, False)) for pitch in (0, 1)]
CARSTATE_BASELINE_CASES = [(car, pedal, alpha) for car in (CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_EUV)
                           for pedal, alpha in ((True, False), (False, True))]


def plan_inputs(tick):
  """A fixed plan: launch from a stop, cruise, every decel band, a stop with a hold, and a release."""
  from openpilot.cereal import car, log
  if tick < 40:
    a_target, v_ego, stop = 0.0, 0.0, True
  elif tick < 300:
    a_target, v_ego, stop = [0.3, 0.9, 1.4, 0.4, 0.0][(tick // 52) % 5], min(15.0, (tick - 40) * 0.06), False
  elif tick < 700:
    a_target, v_ego, stop = [-0.3, -0.9, -1.4, -2.2, -3.0, -0.6, 0.2, -1.8][(tick // 50) % 8], max(0.0, 15.0 - (tick - 300) * 0.04), False
  elif tick < 800:
    a_target, v_ego, stop = -0.6, 0.0, True
  else:
    a_target, v_ego, stop = 0.5, (tick - 800) * 0.01, False
  plan = log.LongitudinalPlan.new_message(aTarget=a_target, vTargetNow=v_ego + a_target * 0.3, jTargetNow=0.0, shouldStop=stop,
                                          hasLead=tick % 300 < 150)
  cs = car.CarState.new_message(vEgo=v_ego, aEgo=a_target * 0.7 + (0.05 if tick % 7 == 0 else 0.0), standstill=v_ego < 0.05,
                                brakePressed=760 <= tick < 765)
  cs.cruiseState.standstill = 720 <= tick < 740
  active = not (500 <= tick < 520)
  return active, cs, plan, log.RadarState.new_message()


def longcontrol_stream(runtime, monkeypatch, cp, frames=900, params=MemoryParams):
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", params)
  loc = longcontrol.LongControl(cp)
  stream = []
  for tick in range(frames):
    active, cs, plan, radar = plan_inputs(tick)
    if not active:
      loc.reset()
    limits = runtime.interface.CarInterface.get_pid_accel_limits(cp, cs.vEgo, 0.0)
    accel, _, _ = loc.update(active, cs, plan, limits, 0.0, radar)
    stream.append([round(float(accel), 6), str(loc.long_control_state), round(float(loc.pid.i), 6), [round(float(x), 6) for x in limits]])
  return stream


LONGCONTROL_BASELINE_CASES = [(car, pedal, alpha) for car in (CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_EUV)
                              for pedal, alpha in ((True, False), (False, True))]


# Frozen before the port (see the comment at the top). Inputs are the deterministic drives above.
BASELINE_DIGESTS = {
  "ctrl:CHEVROLET_BOLT_CC/True/False/0": "65124a74a5d1f1f0f3f4b1582515293607ace97bd6a95040ee4a7fa3329be5b3",
  "ctrl:CHEVROLET_BOLT_CC/True/False/1": "6f2a19edbdefec0517f5f926b3f04568368d4f481b4c12ac248275d1e11615ae",
  "ctrl:CHEVROLET_BOLT_CC/False/True/0": "491e3a05178a99b6a8a0fd2c6311c7e9ee15aa7d84028f3cf2e4b80c7c65f01c",
  "ctrl:CHEVROLET_BOLT_CC/False/True/1": "cfb5432f8bb7b41560a5d944e77ef918f849a53b3dd64c036796a5392fba7d7e",
  "ctrl:CHEVROLET_BOLT_CC/False/False/0": "a30b1449f498af59fac9e218f60516184452e70851cd779b5069dae861634fe2",
  "ctrl:CHEVROLET_BOLT_CC/False/False/1": "a30b1449f498af59fac9e218f60516184452e70851cd779b5069dae861634fe2",
  "ctrl:CHEVROLET_BOLT_EUV/True/False/0": "44184aa9aeb4c8090581f8c01c0eed243e206e7ea4690c767ed9fe8133396334",
  "ctrl:CHEVROLET_BOLT_EUV/True/False/1": "e9c0b861998219cceb3c560412f911d24019c46e5d5a03ef9f3f8a71159fa070",
  "ctrl:CHEVROLET_BOLT_EUV/False/True/0": "e7819644116d594d4753d9eef771b439f0798583ca8eb69bfed400f77798554a",
  "ctrl:CHEVROLET_BOLT_EUV/False/True/1": "25090a46116f05d66731c1f99a7206c5593ab2180f73e6e81939671b45c5a9e2",
  "ctrl:CHEVROLET_BOLT_EUV/False/False/0": "676f1e59a3b9de19702a62194a5250ec91cb73942c180425993e0fb06f5d8345",
  "ctrl:CHEVROLET_BOLT_EUV/False/False/1": "676f1e59a3b9de19702a62194a5250ec91cb73942c180425993e0fb06f5d8345",
  "cs:CHEVROLET_BOLT_CC/True/False": "93f3efab84b88dad42ee3bb0767037904dfb3d92f4f5eccb8beb8839825f9e10",
  "cs:CHEVROLET_BOLT_CC/False/True": "827811b4f4a74b837ad93f808822e8eb6bb8ebdb584748700e64808b2a0155e5",
  "cs:CHEVROLET_BOLT_EUV/True/False": "7cb707c95f925d2fab05215f95fef1abfff3e93d7c4c61c2787a684a7058c288",
  "cs:CHEVROLET_BOLT_EUV/False/True": "bf9b417be6f250bbd43e0293d036ff82e16fbc7f004467cd3d14c7e57e6fe416",
  "lc:CHEVROLET_BOLT_CC/True/False": "742bbd3a1d3f80a6b776e34d1ef1f9e016ea95464b40e24ec3cdd33958813bb5",
  "lc:CHEVROLET_BOLT_CC/False/True": "82ba504ac1ff89243ade003a2c4ca57eb624235558cc9cd8bfa9fc64688eef70",
  "lc:CHEVROLET_BOLT_EUV/True/False": "db99a71254e99c6a6a1dc996ffb4df2f3384e42559fc1ac2068a8de2e21e9996",
  "lc:CHEVROLET_BOLT_EUV/False/True": "32c8f24e493893e537730282be70bf5ce91e37264d20788848f2682126405413",
}


# 2026-11-20: the gas/brake side hysteresis of the camera ACC cars (gas_brake_hysteresis.py, user request for every
# Chevrolet) changes these two streams, both with LongPitch on: at ticks 388-396 a 0.0 command on the -0.02 rad pitch
# coasts (0x2CB ZERO_GAS) instead of sending MAX_ACC_REGEN; no other frame or message changes. With the hysteresis off
# every stream is still the frozen one (test_..._without_the_gas_brake_hysteresis).
HYSTERESIS_DIGESTS = {
  "ctrl:CHEVROLET_BOLT_EUV/True/False/1": "caa24addf9e2f44b1aaf5eab8f916d044c1e814945d5b784b4adb556b97c3e89",
  "ctrl:CHEVROLET_BOLT_EUV/False/True/1": "054965a6a46d0ce267f6aecf6ae8552e706e0081ccfbb815e87b3d20fdde63c3",
}


# 2026-11-28: the Bolt EUV takes mode 1's fixes without the pedal parts (user request; values.is_bolt_camera_long):
# - every Bolt EUV case: the Bolt No-ACC's 33 Hz steering timing (0x180 only);
# - without a pedal on openpilot long (/False/True/): the stop hold kept through controlsd's resume (ticks 472-479: 0x315
#   brake 40 with mode 0xD instead of the lookup's 55) and the 0x370 gap level following the HUD only with CAN valid and
#   ACC main on (0 here: these drives leave canValid unset), and mode 1's LongControl start and stop; its CarState keeps
#   the ECM's standstill (ticks 120-139, Codex review R1), which is its only CarState change.
# With the port switched off every stream is still the frozen one (test_..._without_the_bolt_euv_port).
PORT_DIGESTS = {
  "ctrl:CHEVROLET_BOLT_EUV/True/False/0": "f810624923d5d91e09d4c1361cde1228beb9bbd4be2bb52cee85192e183b8e18",
  "ctrl:CHEVROLET_BOLT_EUV/True/False/1": "33c4389972ea90a47254d00d23848971adc26a51a6ec459789722b40f42d9e74",
  "ctrl:CHEVROLET_BOLT_EUV/False/True/0": "66e054789065faa03e95a888ed06d1acdd4453ca565ab81d6cb0fbc66b973bc7",
  "ctrl:CHEVROLET_BOLT_EUV/False/True/1": "7c7556ce7b1f5077861cad681cc0b430d2090da235db4c478fe73abd7e5a0061",
  "ctrl:CHEVROLET_BOLT_EUV/False/False/0": "a30b1449f498af59fac9e218f60516184452e70851cd779b5069dae861634fe2",
  "ctrl:CHEVROLET_BOLT_EUV/False/False/1": "a30b1449f498af59fac9e218f60516184452e70851cd779b5069dae861634fe2",
  "cs:CHEVROLET_BOLT_EUV/False/True": "920923467f7051727a5264f470827f62994d75ce5da063020766832eff8df964",
  "lc:CHEVROLET_BOLT_EUV/False/True": "3e3bbece1d29d218df8405eb2311f02840315ca1693c5538086443e18f822972",
}
PORT_DIGESTS_WITHOUT_HYSTERESIS = {
  "ctrl:CHEVROLET_BOLT_EUV/True/False/0": "f810624923d5d91e09d4c1361cde1228beb9bbd4be2bb52cee85192e183b8e18",
  "ctrl:CHEVROLET_BOLT_EUV/True/False/1": "1e6dab3f9e1447ff7dcc7fee52440d5bdc4247b8b970bcec43025a4ab87fd91c",
  "ctrl:CHEVROLET_BOLT_EUV/False/True/0": "66e054789065faa03e95a888ed06d1acdd4453ca565ab81d6cb0fbc66b973bc7",
  "ctrl:CHEVROLET_BOLT_EUV/False/True/1": "7099d857d5defc3d79ac25c34e8668567e74674ae55a052b36ce501bfb56751a",
  "ctrl:CHEVROLET_BOLT_EUV/False/False/0": "a30b1449f498af59fac9e218f60516184452e70851cd779b5069dae861634fe2",
  "ctrl:CHEVROLET_BOLT_EUV/False/False/1": "a30b1449f498af59fac9e218f60516184452e70851cd779b5069dae861634fe2",
}


def digest(stream):
  return hashlib.sha256(json.dumps(stream).encode()).hexdigest()


def without_the_bolt_euv_port(runtime, monkeypatch):
  """The code as it was before the Bolt EUV port: its 25 Hz steering, mode 1's fixes for the pedal car only."""
  from opendbc.car.gm import values
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(values, "FROGPILOT_STEER_TIMING_CAR", values.FROGPILOT_STEER_TIMING_CAR - {CAR.CHEVROLET_BOLT_EUV})
  mode_1_only = lambda CP: values.is_bolt_camera_long(CP) and bool(CP.enableGasInterceptorDEPRECATED)
  monkeypatch.setattr(runtime.controller, "is_bolt_camera_long", mode_1_only)
  monkeypatch.setattr(runtime.state, "is_bolt_camera_long", mode_1_only)
  monkeypatch.setattr(longcontrol, "is_bolt_camera_long", mode_1_only)


def pre_port_cp(runtime, *key):
  cp = bolt_cp(runtime, *key)
  if not cp.enableGasInterceptorDEPRECATED and cp.openpilotLongitudinalControl:  # the camera-ACC branch's values
    cp.minEnableSpeed, cp.startAccel, cp.vEgoStopping, cp.vEgoStarting = -1 / 3.6, 1.0, 0.5, 0.4
  return cp


@pytest.mark.parametrize("key", BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_controller_streams_unchanged(runtime, monkeypatch, key):
  name = "ctrl:" + "/".join(map(str, key))
  expected = PORT_DIGESTS.get(name, HYSTERESIS_DIGESTS.get(name, BASELINE_DIGESTS[name]))
  assert digest(controller_stream(runtime, monkeypatch, *key)) == expected


@pytest.mark.parametrize("key", BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_controller_streams_unchanged_without_the_gas_brake_hysteresis(runtime, monkeypatch, key):
  monkeypatch.setattr(runtime.controller, "has_gas_brake_step", lambda params: False)
  name = "ctrl:" + "/".join(map(str, key))
  assert digest(controller_stream(runtime, monkeypatch, *key)) == PORT_DIGESTS_WITHOUT_HYSTERESIS.get(name, BASELINE_DIGESTS[name])


@pytest.mark.parametrize("key", BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_controller_streams_unchanged_without_the_bolt_euv_port(runtime, monkeypatch, key):
  without_the_bolt_euv_port(runtime, monkeypatch)
  name = "ctrl:" + "/".join(map(str, key))
  assert digest(controller_stream(runtime, monkeypatch, *key)) == HYSTERESIS_DIGESTS.get(name, BASELINE_DIGESTS[name])


@pytest.mark.parametrize("key", CARSTATE_BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_carstate_streams_unchanged(runtime, monkeypatch, key):
  name = "cs:" + "/".join(map(str, key))
  assert digest(carstate_stream(runtime, monkeypatch, *key)) == PORT_DIGESTS.get(name, BASELINE_DIGESTS[name])


@pytest.mark.parametrize("key", CARSTATE_BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_carstate_streams_unchanged_without_the_bolt_euv_port(runtime, monkeypatch, key):
  ported = carstate_stream(runtime, monkeypatch, *key)
  without_the_bolt_euv_port(runtime, monkeypatch)
  stream = carstate_stream(runtime, monkeypatch, *key)
  assert digest(stream) == BASELINE_DIGESTS["cs:" + "/".join(map(str, key))]
  # the port changes nothing but the kept standstill
  assert [{k: v for k, v in r.items() if k != "cruise_standstill"} for r in ported] == \
         [{k: v for k, v in r.items() if k != "cruise_standstill"} for r in stream]


@pytest.mark.parametrize("key", LONGCONTROL_BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_longcontrol_streams_unchanged(runtime, monkeypatch, key):
  name = "lc:" + "/".join(map(str, key))
  stream = longcontrol_stream(runtime, monkeypatch, bolt_cp(runtime, *key))
  assert digest(stream) == PORT_DIGESTS.get(name, BASELINE_DIGESTS[name])


@pytest.mark.parametrize("key", LONGCONTROL_BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_longcontrol_streams_unchanged_without_the_bolt_euv_port(runtime, monkeypatch, key):
  without_the_bolt_euv_port(runtime, monkeypatch)
  stream = longcontrol_stream(runtime, monkeypatch, pre_port_cp(runtime, *key))
  assert digest(stream) == BASELINE_DIGESTS["lc:" + "/".join(map(str, key))]


# ---------------------------------------------------------------------------------------------------------------------
# CarParams: the Bolt EUV's camera long, minus PEDAL_LONG

BOLT = CAR.CHEVROLET_BOLT_ACC_PEDAL
HYBRID_SAFETY = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.GAS_INTERCEPTOR)
LONG_FIELDS = ("openpilotLongitudinalControl", "pcmCruise", "autoResumeSng", "startingState", "startAccel",
               "vEgoStarting", "vEgoStopping", "stopAccel", "stoppingDecelRate", "longitudinalActuatorDelay",
               "enableGasInterceptorDEPRECATED", "networkLocation", "radarUnavailable", "transmissionType")


def long_view(cp):
  lt = cp.longitudinalTuning
  return {**{f: str(getattr(cp, f)) for f in LONG_FIELDS}, "safety": cp.safetyConfigs[0].safetyParam,
          "tuning": (list(lt.kpBP), list(lt.kpV), list(lt.kiBP), list(lt.kiV), lt.kf),
          "flags": cp.flags & ~GMFlags.PEDAL_LONG.value}


@pytest.mark.parametrize("pedal,alpha", [(True, False), (True, True), (False, True), (False, False)])
def test_longitudinal_car_params_are_the_bolt_euvs(runtime, pedal, alpha):
  bolt = long_view(bolt_cp(runtime, BOLT, pedal, alpha))
  euv = long_view(bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, pedal, alpha))
  if pedal:  # mode 1 starts as StarPilot drives this car (rlog 00000028--02c4808d1b--21, 2026-11-20)
    assert (float(bolt.pop("startAccel")), float(euv.pop("startAccel"))) == (pytest.approx(0.55), 1.0)
  assert bolt == euv


@pytest.mark.parametrize("alpha", [False, True])
def test_car_params_with_pedal(runtime, alpha):
  cp = bolt_cp(runtime, BOLT, True, alpha)
  assert cp.safetyConfigs[0].safetyParam == HYBRID_SAFETY  # no NO_ACC, PEDAL_LONG or BOLT_ACC_PEDAL panda bits
  assert cp.openpilotLongitudinalControl and not cp.pcmCruise and cp.autoResumeSng
  # the ECM's own rule (above 5 km/h, or stopped with the brake): the Bolt EUV with pedal has -1 there
  assert cp.minEnableSpeed == pytest.approx(5 / 3.6)
  assert not cp.flags & GMFlags.PEDAL_LONG.value  # would cancel the ECM's ACC that camera long drives through
  assert bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True, alpha).flags & GMFlags.PEDAL_LONG.value  # the Bolt EUV keeps it
  # carrot's single-breakpoint GM tuning, so LongTuningKpV/KiV/Kf apply; the delay is the LongActuatorDelay Param's
  lt = cp.longitudinalTuning
  assert (list(lt.kpBP), list(lt.kiBP)) == ([0.0], [0.0])


def test_min_enable_speed_only_in_mode_1_with_a_pedal(runtime, monkeypatch):
  assert bolt_cp(runtime, BOLT, True, False).minEnableSpeed == pytest.approx(5 / 3.6)
  assert bolt_cp(runtime, BOLT, False, True).minEnableSpeed == bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, False, True).minEnableSpeed
  monkeypatch.setattr(runtime.interface, "Params", memory_params(BoltPedalLongMode=2))
  assert bolt_cp(runtime, BOLT, True, False).minEnableSpeed == -1  # StarPilot mode cancels the ECM's ACC


def test_car_params_without_pedal_are_the_bolt_euvs_camera_paths(runtime):
  assert bolt_cp(runtime, BOLT, False, True).safetyConfigs[0].safetyParam == int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG)
  assert bolt_cp(runtime, BOLT, False, False).safetyConfigs[0].safetyParam == int(GMSafetyFlags.HW_CAM)


STARPILOT_SAFETY = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.NO_ACC | GMSafetyFlags.PEDAL_LONG |
                       GMSafetyFlags.GAS_INTERCEPTOR | GMSafetyFlags.BOLT_ACC_PEDAL)


@pytest.mark.parametrize("mode,starpilot", [(0, False), (1, False), (2, True), (3, False), (-1, False)])
def test_bolt_pedal_long_mode_picks_the_path(runtime, monkeypatch, mode, starpilot):
  """BoltPedalLongMode 2 is StarPilot's pedal long (test_bolt_acc_pedal_starpilot.py); anything else, including a
  missing Param (0), is camera long. Read while CarParams are built."""
  monkeypatch.setattr(runtime.interface, "Params", memory_params(BoltPedalLongMode=mode))
  for alpha in (False, True):
    cp = bolt_cp(runtime, BOLT, True, alpha)
    assert cp.safetyConfigs[0].safetyParam == (STARPILOT_SAFETY if starpilot else HYBRID_SAFETY)
    assert bool(cp.flags & GMFlags.PEDAL_LONG.value) == starpilot
    assert cp.longitudinalActuatorDelay == pytest.approx(0.6 if starpilot else 0.5)  # MemoryParams LongActuatorDelay 50
    assert cp.alphaLongitudinalAvailable == (not starpilot)


@pytest.mark.parametrize("mode", [1, 2])
def test_mode_does_nothing_without_a_pedal_or_on_other_cars(runtime, monkeypatch, mode):
  monkeypatch.setattr(runtime.interface, "Params", memory_params(BoltPedalLongMode=mode))
  for alpha in (False, True):
    assert long_view(bolt_cp(runtime, BOLT, False, alpha)) == long_view(bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, False, alpha))
  for key in LONGCONTROL_BASELINE_CASES:
    monkeypatch.setattr(runtime.interface, "Params", memory_params(BoltPedalLongMode=mode))
    with_mode = bolt_cp(runtime, *key)
    monkeypatch.setattr(runtime.interface, "Params", MemoryParams)
    assert long_view(with_mode) == long_view(bolt_cp(runtime, *key)), key


def test_mode_param_is_registered_with_default_1():
  from pathlib import Path
  import json
  root = Path(__file__).resolve().parents[5]
  keys = (root / "openpilot/common/params_keys.h").read_text()
  assert '{"BoltPedalLongMode", {PERSISTENT, INT, "1"}}' in keys
  catalog = json.loads((root / "openpilot/selfdrive/carrot_settings.json").read_text())
  entry = next(p for p in catalog["params"] if p["name"] == "BoltPedalLongMode")
  assert (entry["min"], entry["max"], entry["default"], entry["control"]) == (1, 2, 1, "select")
  assert '"BoltPedalLongMode"' in json.dumps(catalog["menu"])  # listed in a settings menu


@pytest.mark.parametrize("speed", [0.0, 5.0, 15.0, 30.0])
def test_pid_accel_limits_are_carrots(runtime, speed):
  cp = bolt_cp(runtime, BOLT, True, False)
  ours = runtime.interface.CarInterface.get_pid_accel_limits(cp, speed, 0.0)
  euv = runtime.interface.CarInterface.get_pid_accel_limits(bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True, False), speed, 0.0)
  assert ours == euv


# ---------------------------------------------------------------------------------------------------------------------
# CarState


def test_carstate_is_the_bolt_euvs_but_keeps_the_ecm_standstill(runtime, monkeypatch):
  ours = carstate_stream(runtime, monkeypatch, BOLT, True, False)
  euv = carstate_stream(runtime, monkeypatch, CAR.CHEVROLET_BOLT_EUV, True, False)
  assert [t for t, r in enumerate(ours) if r["cruise_standstill"]] == list(range(120, 140))  # AcceleratorPedal2 4
  assert not any(r["cruise_standstill"] for r in euv)  # carrot clears it on the other GM cars
  for row in ours + euv:
    row.pop("cruise_standstill")
  assert ours == euv
  # cruise state is the ECM's (AcceleratorPedal2), not the camera's 0x370 or the ECM's 0x3D1
  assert [t for t, r in enumerate(ours) if r["enabled"]] == list(range(50, 140)) + list(range(300, 305))


@pytest.mark.parametrize("car", [BOLT, CAR.CHEVROLET_BOLT_EUV])
def test_carstate_without_pedal_keeps_the_standstill_only_on_camera_long(runtime, monkeypatch, car):
  """Without a pedal on camera long the ECM's standstill is kept since 2026-11-28 (Codex review R1), so LongControl waits
  on it and "Press Resume" shows; on the stock ACC carrot still clears it."""
  long_stream = carstate_stream(runtime, monkeypatch, car, False, True)
  assert [t for t, r in enumerate(long_stream) if r["cruise_standstill"]] == list(range(120, 140))  # AcceleratorPedal2 4
  assert not any(r["cruise_standstill"] for r in carstate_stream(runtime, monkeypatch, car, False, False))


def gas_pressed_for(runtime, raw1, raw2):
  cp = bolt_cp(runtime, BOLT, True, False)
  ci = runtime.interface.CarInterface.__new__(runtime.interface.CarInterface)
  ci.CS = runtime.state.CarState(cp)
  ci.can_parsers = ci.CS.get_can_parsers(cp)
  ci.v_ego_cluster_seen = False
  packer = runtime.CANPacker(DBC[BOLT][Bus.pt])
  gas1, gas2 = raw1 * 0.125677 - 75.909, raw2 * 0.251976 - 76.601
  ret = None
  for tick in range(5):
    frames = [f for f in carstate_frames(packer, 0, False)]
    frames.append(packer.make_can_msg("GAS_SENSOR", 0, {"INTERCEPTOR_GAS": gas1, "INTERCEPTOR_GAS2": gas2}))
    ret = ci.update([(1_000_000_000 + tick * 10_000_000, frames)])
  return bool(ret.gasPressed)


@pytest.mark.parametrize('raw1,raw2', [(733, 367), (734, 367), (734, 368), (735, 368), (700, 400), (700, 403), (760, 342),
                                       (1100, 0), (1102, 1), (764, 384), (765, 384)])
def test_gas_pressed_matches_the_panda_raw_average(runtime, raw1, raw2):
  """The panda's (track1 + track2) // 2 > 550 on the same frame, so host and panda see the same gas edges."""
  assert gas_pressed_for(runtime, raw1, raw2) == ((raw1 + raw2) // 2 > 550)


def gas_pressed_series(runtime, samples, disengage_on_gas=False):
  """gasPressed from one CarState fed one (raw track average, ECM AcceleratorPedal2) per 10 ms frame, track1 = track2 =
  raw. DisengageOnAccelerator off (carrot's default) sets DISABLE_DISENGAGE_ON_GAS, as card.py does."""
  from opendbc.safety import ALTERNATIVE_EXPERIENCE
  cp = bolt_cp(runtime, BOLT, True, False)
  cp.alternativeExperience = 0 if disengage_on_gas else ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS
  ci = runtime.interface.CarInterface.__new__(runtime.interface.CarInterface)
  ci.CS = runtime.state.CarState(cp)
  ci.can_parsers = ci.CS.get_can_parsers(cp)
  ci.v_ego_cluster_seen = False
  packer = runtime.CANPacker(DBC[BOLT][Bus.pt])
  pressed = []
  for tick, (raw, ecm) in enumerate(samples):
    frames = [f for f in carstate_frames(packer, 0, False) if f[0] != 0x1C4]
    frames.append(packer.make_can_msg("AcceleratorPedal2", 0, {"CruiseState": 0, "AcceleratorPedal2": ecm}))
    frames.append(packer.make_can_msg("GAS_SENSOR", 0, {"INTERCEPTOR_GAS": raw * 0.125677 - 75.909,
                                                         "INTERCEPTOR_GAS2": raw * 0.251976 - 76.601}))
    pressed.append(bool(ci.update([(1_000_000_000 + tick * 10_000_000, frames)]).gasPressed))
  return pressed


# rlog 00000023--f5c4a454cd--13, 24.6-25.6 s: a light foot sitting on 550 (the ECM's AcceleratorPedal2 read 1-13 there).
# The panda-exact threshold flipped gasPressed 10 times here, and carrot took each flip as a gas tap (+10 km/h).
LIGHT_FOOT = [(raw, 8) for raw in (544, 552, 547, 551, 542, 554, 548, 557, 549, 551, 545, 553, 550, 557, 548, 559,
                                         538, 554, 545, 564)]


def test_a_light_foot_on_the_threshold_stays_one_press(runtime):
  """A press starts above the panda's 550 and is held down to 515 while the ECM still reads the pedal pressed."""
  samples = LIGHT_FOOT + [(530, 8), (516, 8), (515, 8), (475, 0), (551, 9), (516, 9), (475, 0), (550, 0)]
  pressed = gas_pressed_series(runtime, samples)
  assert pressed == [False] + [True] * (len(LIGHT_FOOT) - 1) + [True, True, False, False, True, True, False, False]
  assert all(host or raw <= 550 for host, (raw, _) in zip(pressed, samples))  # never released while the panda sees a press


def test_the_ecm_pedal_ends_a_held_press(runtime):
  """A pedal resting inside the band (or a foot the ECM reads as off) cannot hold a press: AcceleratorPedal2 0 ends it."""
  samples = [(475, 0), (551, 9), (540, 9), (540, 0), (540, 0), (551, 9), (540, 0)]
  assert gas_pressed_series(runtime, samples) == [False, True, True, False, False, True, False]


def test_disengage_on_gas_keeps_the_panda_edges(runtime):
  """With DisengageOnAccelerator on the panda drops control on each press edge of its own, so the host must see exactly
  its edges (Codex review 2026-11-19: a held press hid the panda's second edge after a re-engage)."""
  samples = LIGHT_FOOT + [(475, 0), (565, 20), (535, 15), (535, 15), (565, 20), (475, 0)]
  assert gas_pressed_series(runtime, samples, disengage_on_gas=True) == [raw > 550 for raw, _ in samples]
  assert gas_pressed_series(runtime, samples[-6:]) == [False, True, True, True, True, False]


# ---------------------------------------------------------------------------------------------------------------------
# CarController: the camera-long frames, plus the pedal only at the ECM's standstill


def controller_stream_cp(runtime, monkeypatch, cp, pitch, resume=True):
  monkeypatch.setattr(runtime.controller, "Params", memory_params(LongPitch=pitch))
  controller = runtime.controller.CarController(DBC[cp.carFingerprint], cp)
  stream = []
  for tick in range(STREAM_FRAMES):
    cc, cs = drive_inputs(tick)
    if not resume:
      cc.cruiseControl.resume = False
    cs.CP = controller.CP
    output, sends = controller.update(cc.as_reader(), cs, 1_000_000_000 + tick * 10_000_000)
    stream.append([sorted([addr, bytes(dat).hex(), bus] for addr, dat, bus in sends if addr != 0x180),
                   round(float(output.gas), 6), round(float(output.brake), 6), round(float(output.accel), 6)])
  return stream


@pytest.mark.parametrize("resume", [False, True])
@pytest.mark.parametrize("pitch", [0, 1])
def test_longitudinal_frames_are_the_bolt_euvs_without_pedal_long(runtime, monkeypatch, pitch, resume):
  """Every frame but the steering equals the Bolt EUV with pedal once PEDAL_LONG (its stock-ACC cancel) is off and its
  stop hold is the no-pedal one (autoResumeSng off there drops FullStop and mode 0xD at a stop): that is the Bolt EUV on
  camera long, which since 2026-11-28 also keeps the hold through controlsd's resume and shows the standby gap. The
  pedal taps only to leave the ECM's standstill, which this drive never asks for."""
  euv = bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True, False)
  euv.flags &= ~GMFlags.PEDAL_LONG.value
  euv.autoResumeSng = False
  bolt_stream = controller_stream_cp(runtime, monkeypatch, bolt_cp(runtime, BOLT, True, False), pitch, resume=resume)
  euv_stream = controller_stream_cp(runtime, monkeypatch, euv, pitch, resume=resume)
  assert bolt_stream == euv_stream


def without_gap_level(stream):
  out = []
  for sends, *rest in stream:
    masked = []
    for addr, dat, bus in sends:
      if addr == 0x370:
        raw = bytearray.fromhex(dat)
        raw[2] &= ~0x30
        dat = raw.hex()
      masked.append([addr, dat, bus])
    out.append([masked, *rest])
  return out


def cluster_gaps(runtime, monkeypatch, cp, *, enabled, available, bars, can_valid=True):
  monkeypatch.setattr(runtime.controller, "Params", memory_params())
  controller = runtime.controller.CarController(DBC[cp.carFingerprint], cp)
  gaps = []
  for frame in range(12):
    cc = structs.CarControl(enabled=enabled, longActive=enabled, latActive=enabled)
    cc.actuators.longControlState = "pid" if enabled else "off"
    cc.hudControl.setSpeed = 20.0
    cc.hudControl.leadDistanceBars = bars
    out = structs.CarState(vEgo=15.0, aEgo=0.0, canValid=can_valid)
    out.cruiseState.available = available
    out.cruiseState.enabled = enabled
    out.gearShifter = "drive"
    cs = NS(out=out, CP=controller.CP, loopback_lka_steering_cmd_ts_nanos=0, loopback_lka_steering_cmd_updated=False,
            cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0, buttons_counter=0,
            pcm_acc_status=1 if enabled else 0, pscm_status=defaultdict(int),
            cam_ascm_2cb_counter=0, cam_ascm_2cb_counter_updated=False, cam_ascm_2cb_counter_ts_nanos=0,
            cam_stock_long_active=None, cam_stock_long_cancel=False, cam_acc_status=None, cam_fcw_alert=0, stock_fcw_alert=0)
    _, sends = controller.update(cc.as_reader(), cs, 1_000_000_000 + frame * 10_000_000)
    gaps += [((dat[2] >> 4) & 3, (dat[2] >> 7) & 1) for addr, dat, bus in sends if addr == 0x370 and bus == 0]
  return set(gaps)


@pytest.mark.parametrize("car,pedal", [(BOLT, True), (BOLT, False), (CAR.CHEVROLET_BOLT_EUV, False)])
@pytest.mark.parametrize("bars", [1, 2, 3, 4])
def test_cluster_gap_shows_before_engaging(runtime, monkeypatch, bars, car, pedal):
  """User report 2026-11-21: after a gap press the cluster kept an old gap (2) while the comma showed the new one (3).
  The stock camera sends its gap in 0x370 before engaging too (00 02 22 cd 00 00 in rlog 00000028--02c4808d1b--21);
  this sent 0 there. Now: up to 3 bars (2 bits) with ACC main on, engaged or not, and ACCCmdActive only when engaged.
  The same camera is in the Bolt EUV without a pedal (2026-11-28)."""
  cp = bolt_cp(runtime, car, pedal, not pedal)
  shown = min(bars, 3)
  assert cluster_gaps(runtime, monkeypatch, cp, enabled=False, available=True, bars=bars) == {(shown, 0)}
  assert cluster_gaps(runtime, monkeypatch, cp, enabled=True, available=True, bars=bars) == {(shown, 1)}
  assert cluster_gaps(runtime, monkeypatch, cp, enabled=False, available=False, bars=bars) == {(0, 0)}
  assert cluster_gaps(runtime, monkeypatch, cp, enabled=False, available=True, bars=bars, can_valid=False) == {(0, 0)}


def test_other_bolts_keep_no_gap_before_engaging(runtime, monkeypatch):
  euv = bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True, False)  # the Bolt EUV with a pedal: pedal long
  assert cluster_gaps(runtime, monkeypatch, euv, enabled=False, available=True, bars=2) == {(0, 0)}
  assert cluster_gaps(runtime, monkeypatch, euv, enabled=True, available=True, bars=2) == {(2, 1)}


def hybrid_step(controller, frame, *, accel, state, v, cruise_standstill, cruise_enabled=True, resume=False):
  cc = structs.CarControl(enabled=True, longActive=True, latActive=True)
  cc.actuators.accel = accel
  cc.actuators.longControlState = state
  cc.cruiseControl.resume = resume
  cc.hudControl.setSpeed = 20.0
  cc.orientationNED = [0.0, 0.0, 0.0]
  out = structs.CarState(vEgo=v, aEgo=0.0, standstill=v < 0.05)
  out.cruiseState.available = True
  out.cruiseState.enabled = cruise_enabled
  out.cruiseState.standstill = cruise_standstill
  out.gearShifter = "drive"
  cs = NS(out=out, CP=controller.CP, loopback_lka_steering_cmd_ts_nanos=0, loopback_lka_steering_cmd_updated=False,
          cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0, buttons_counter=0,
          pcm_acc_status=4 if cruise_standstill else (1 if cruise_enabled else 0), pscm_status=defaultdict(int),
          cam_ascm_2cb_counter=0, cam_ascm_2cb_counter_updated=False, cam_ascm_2cb_counter_ts_nanos=0,
          cam_stock_long_active=None, cam_stock_long_cancel=False, cam_acc_status=None, cam_fcw_alert=0, stock_fcw_alert=0)
  _, sends = controller.update(cc.as_reader(), cs, 1_000_000_000 + frame * 10_000_000)
  return [(addr, bytes(dat), bus) for addr, dat, bus in sends]


def pedal(sends):
  frames = [dat for addr, dat, bus in sends if addr == 0x200]
  if not frames:
    return None
  dat = frames[0]
  enabled = (dat[4] >> 7) & 1
  return round(((dat[0] << 8 | dat[1]) * 0.125677 - 75.909) / 255., 3) if enabled else 0.0


def run_hybrid(runtime, monkeypatch, frames=8, **kwargs):
  monkeypatch.setattr(runtime.controller, "Params", memory_params())
  controller = runtime.controller.CarController(DBC[BOLT], bolt_cp(runtime, BOLT, True, False))
  values = [pedal(hybrid_step(controller, f, **kwargs)) for f in range(frames)]
  return [v for v in values if v is not None]


def test_pedal_taps_only_to_start_from_the_ecm_standstill(runtime, monkeypatch):
  taps = run_hybrid(runtime, monkeypatch, accel=1.0, state="starting", v=0.0, cruise_standstill=True, resume=True)
  assert taps and all(v == pytest.approx(18. / 255., abs=0.002) for v in taps)  # SNG_INTERCEPTOR_GAS, DBC-quantized
  # stopped and holding, moving, or starting once the ECM has left its standstill: the pedal stays released
  assert set(run_hybrid(runtime, monkeypatch, accel=-0.4, state="stopping", v=0.0, cruise_standstill=True)) == {0.0}
  assert set(run_hybrid(runtime, monkeypatch, accel=1.5, state="pid", v=10.0, cruise_standstill=False)) == {0.0}
  assert set(run_hybrid(runtime, monkeypatch, accel=1.0, state="starting", v=0.0, cruise_standstill=False)) == {0.0}
  # rolling while the ECM still reports standstill, in pid or just entering starting: no tapping (Codex review 2026-11-18)
  assert set(run_hybrid(runtime, monkeypatch, accel=1.0, state="pid", v=1.0, cruise_standstill=True, resume=True)) == {0.0}
  assert set(run_hybrid(runtime, monkeypatch, accel=1.0, state="starting", v=1.0, cruise_standstill=True, resume=True)) == {0.0}
  # rolling back past vEgoStarting in drive (negative vEgo) does not tap and lift the brake either
  assert set(run_hybrid(runtime, monkeypatch, accel=1.0, state="starting", v=-0.8, cruise_standstill=True, resume=True)) == {0.0}
  taps = run_hybrid(runtime, monkeypatch, accel=1.0, state="starting", v=-0.2, cruise_standstill=True, resume=True)
  assert taps and all(v == pytest.approx(18. / 255., abs=0.002) for v in taps)


def friction_brake(dat):
  cmd = ((dat[0] & 0xF) << 8) | dat[1]
  return -(cmd - 0x1000 if cmd & 0x800 else cmd)


def full_stop_frames(runtime, monkeypatch, **kwargs):
  monkeypatch.setattr(runtime.controller, "Params", memory_params())
  controller = runtime.controller.CarController(DBC[BOLT], bolt_cp(runtime, BOLT, True, False))
  frames = []
  for f in range(24):
    sends = hybrid_step(controller, f, **kwargs)
    gas = [dat for addr, dat, bus in sends if addr == 0x2CB]
    brake = [dat for addr, dat, bus in sends if addr == 0x315]
    if gas and brake:
      frames.append(((gas[0][1] >> 5) & 1, brake[0][0] >> 4, friction_brake(brake[0])))
  return frames


def test_stop_hold_is_the_stock_cameras(runtime, monkeypatch):
  """Stopped and stopping: 0x2CB FullStop with friction brake mode 0xD, as the stock camera sends (its 0x2CB/0x315 in
  rlog 0000001d--96a597633a--10/13); without them the ECM faulted 3.2 s after a stop."""
  for standstill in (False, True):
    frames = full_stop_frames(runtime, monkeypatch, accel=-0.5, state="stopping", v=0.0, cruise_standstill=standstill)
    assert frames and set(frames) == {(1, 0xD, 40)}, standstill
  # starting from the ECM's standstill: the hold is released for the pedal tap
  frames = full_stop_frames(runtime, monkeypatch, accel=1.0, state="starting", v=0.0, cruise_standstill=True, resume=True)
  assert frames and {(fs, brake) for fs, _, brake in frames} == {(0, 0)}


def stop_and_resume_frames(runtime, monkeypatch, **kwargs):
  monkeypatch.setattr(runtime.controller, "Params", memory_params())
  controller = runtime.controller.CarController(DBC[BOLT], bolt_cp(runtime, BOLT, True, False))
  frames = []
  for f in range(24):
    sends = hybrid_step(controller, f, **kwargs)
    gas = [dat for addr, dat, bus in sends if addr == 0x2CB]
    brake = [dat for addr, dat, bus in sends if addr == 0x315]
    if gas and brake:
      frames.append((gas[0][0] & 1, (gas[0][1] >> 5) & 1, brake[0][0] >> 4, friction_brake(brake[0]), pedal(sends)))
  return frames


def test_resume_while_stopping_keeps_the_hold(runtime, monkeypatch):
  """rlog 0000001d--96a597633a--10 (2026-11-19): the ECM reached standstill, controlsd's resume came on while LongControl
  was still stopping (plan end speed 0.107 m/s), 0x2CB apply dropped to 0, the ECM's ACC went Off and then Faulted once
  apply came back. This car keeps apply=1 and the FullStop/0xD hold, with no pedal, until LongControl starts."""
  frames = stop_and_resume_frames(runtime, monkeypatch, accel=-0.5, state="stopping", v=0.0, cruise_standstill=True, resume=True)
  assert frames and set(frames) == {(1, 1, 0xD, 40, 0.0)}


def test_apply_stays_on_while_starting_from_the_ecm_standstill(runtime, monkeypatch):
  """Starting from the ECM's standstill: apply stays 1 (as the stock camera through a resume), the hold is released and
  the pedal taps. A button-less engage there therefore also gets the panda's permission from 0x2CB."""
  frames = stop_and_resume_frames(runtime, monkeypatch, accel=1.0, state="starting", v=0.0, cruise_standstill=True, resume=True)
  assert frames and {f[:4] for f in frames} == {(1, 0, 0x9, 0)}
  assert all(f[4] == pytest.approx(18. / 255., abs=0.002) for f in frames)


# ---------------------------------------------------------------------------------------------------------------------
# LongControl: carrot's, with the Carrot Web settings


LongCtrlState = structs.CarControl.Actuators.LongControlState


def bolt_longcontrol(runtime, monkeypatch, params=MemoryParams):
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", params)
  cp = bolt_cp(runtime, BOLT, True, False)
  return longcontrol.LongControl(cp), cp


def lc_step(runtime, loc, cp, a_target, v, a_ego, stop=False, lead=False, standstill=False, brake=False, soft_hold=0):
  from openpilot.cereal import car, log
  plan = log.LongitudinalPlan.new_message(aTarget=a_target, vTargetNow=v, shouldStop=stop, hasLead=lead)
  cs = car.CarState.new_message(vEgo=v, aEgo=a_ego, brakePressed=brake, softHoldActive=soft_hold)
  cs.cruiseState.standstill = standstill
  limits = runtime.interface.CarInterface.get_pid_accel_limits(cp, v, 0.0)
  return float(loc.update(True, cs, plan, limits, 0.0, log.RadarState.new_message())[0])


def test_longcontrol_takes_the_long_tuning_settings(runtime, monkeypatch):
  params = memory_params(LongTuningKpV=50, LongTuningKiV=200, LongTuningKf=80)
  loc, cp = bolt_longcontrol(runtime, monkeypatch, params)
  for _ in range(15):
    lc_step(runtime, loc, cp, 0.5, 10.0, 0.4)
  assert [list(x) for x in loc.pid._k_p] == [[0.0], [0.5]]
  assert [list(x) for x in loc.pid._k_i] == [[0.0], [0.2]]
  assert loc.pid.k_f == pytest.approx(0.8)


def test_longcontrol_starts_despite_the_ecm_standstill(runtime, monkeypatch):
  """The ECM's standstill stays set for this car, so carrot's LongControl must still request starting (the pedal tap
  needs that request)."""
  loc, cp = bolt_longcontrol(runtime, monkeypatch)
  lc_step(runtime, loc, cp, -0.5, 0.0, 0.0, stop=True, standstill=True)
  assert loc.long_control_state == LongCtrlState.stopping
  lc_step(runtime, loc, cp, 0.5, 0.0, 0.0, standstill=True)
  assert loc.long_control_state == LongCtrlState.starting


def test_other_cars_still_wait_on_cruise_standstill(runtime, monkeypatch):
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", MemoryParams)
  for key in LONGCONTROL_BASELINE_CASES + [(BOLT, False, True)]:
    loc = longcontrol.LongControl(bolt_cp(runtime, *key))
    cp = loc.CP
    lc_step(runtime, loc, cp, -0.5, 0.0, 0.0, stop=True, standstill=True)
    lc_step(runtime, loc, cp, 0.5, 0.0, 0.0, standstill=True)
    assert loc.long_control_state == LongCtrlState.stopping, key


# ---------------------------------------------------------------------------------------------------------------------
# Mode 1 starts and stops as StarPilot drives this car (rlogs 00000028--02c4808d1b--18/21, 2026-11-20)


def stop_from(runtime, loc, cp, output, frames=60, a_ego=-0.2):
  """From pid with the given last output, shouldStop at 0.3 m/s; returns the outputs while stopping."""
  loc.long_control_state = LongCtrlState.pid
  loc.last_output_accel = output
  outs = [lc_step(runtime, loc, cp, -0.2, 0.3, a_ego, stop=True) for _ in range(frames)]
  assert loc.long_control_state == LongCtrlState.stopping
  return outs


def test_mode_1_stop_ramps_only_to_a_quarter(runtime, monkeypatch):
  loc, cp = bolt_longcontrol(runtime, monkeypatch)
  assert loc.stopping_accel == pytest.approx(-0.5)  # carrot's default StoppingAccel
  outs = stop_from(runtime, loc, cp, -0.05)
  assert loc.long_control_state == LongCtrlState.stopping
  assert -0.25 - cp.stoppingDecelRate * 0.01 - 1e-9 <= outs[-1] <= -0.25
  assert all(b <= a for a, b in zip(outs, outs[1:], strict=False))  # one-way ramp
  # what the log had at the stop entry (-0.28, -0.31) stays, instead of growing to -0.5
  for entry in (-0.28, -0.31):
    assert all(x == pytest.approx(entry) for x in stop_from(runtime, loc, cp, entry)), entry
  # stronger braking already in effect is never unwound
  assert all(x == pytest.approx(-0.8) for x in stop_from(runtime, loc, cp, -0.8))


def test_mode_1_stop_takes_a_weaker_stopping_accel(runtime, monkeypatch):
  loc, cp = bolt_longcontrol(runtime, monkeypatch, memory_params(StoppingAccel=-10))
  outs = stop_from(runtime, loc, cp, 0.0, a_ego=-0.05)  # entering a stop also waits for aEgo above -0.1 then
  assert -0.1 - cp.stoppingDecelRate * 0.01 - 1e-9 <= outs[-1] <= -0.1


def test_other_cars_still_stop_at_stopping_accel(runtime, monkeypatch):
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", MemoryParams)
  for cp in (bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True, False), bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, False, False),
             runtime.interface.CarInterface.get_params(CAR.CHEVROLET_TRAVERSE, bolt_fingerprint(CAR.CHEVROLET_BOLT_EUV, False),
                                                       [], True, True, False)):
    loc = longcontrol.LongControl(cp)
    outs = stop_from(runtime, loc, cp, -0.05, frames=120)
    assert -0.5 - cp.stoppingDecelRate * 0.01 - 1e-9 <= outs[-1] <= -0.5, cp.carFingerprint


def start_from_stop(runtime, loc, cp, a_target, lead):
  lc_step(runtime, loc, cp, -0.5, 0.0, 0.0, stop=True, standstill=True)
  assert loc.long_control_state == LongCtrlState.stopping
  out = lc_step(runtime, loc, cp, a_target, 0.0, 0.0, lead=lead, standstill=True)
  assert loc.long_control_state == LongCtrlState.starting
  return out


@pytest.mark.parametrize("a_target,lead,expected", [
  (0.21, True, 0.21),   # rlog --21: plan 0.21 behind a lead; it was startAccel 1.0 (gas 673) before
  (0.25, True, 0.25), (0.1, True, 0.15), (-0.1, True, 0.15),  # at least 0.15: the stop hold is already released
  (0.26, True, 0.55),   # a faster plan, or no lead: StarPilot's startAccel for this car
  (0.1, False, 0.55),
])
def test_mode_1_start_follows_a_slow_plan_behind_a_lead(runtime, monkeypatch, a_target, lead, expected):
  loc, cp = bolt_longcontrol(runtime, monkeypatch)
  assert cp.startAccel == pytest.approx(0.55)
  assert start_from_stop(runtime, loc, cp, a_target, lead) == pytest.approx(expected)


def test_other_cars_still_start_at_start_accel(runtime, monkeypatch):
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", MemoryParams)
  cp = bolt_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True, False)  # the Bolt EUV with a pedal: pedal long
  loc = longcontrol.LongControl(cp)
  lc_step(runtime, loc, cp, -0.5, 0.0, 0.0, stop=True)
  assert lc_step(runtime, loc, cp, 0.21, 0.0, 0.0, lead=True) == pytest.approx(cp.startAccel) == pytest.approx(1.0)


# ---------------------------------------------------------------------------------------------------------------------
# The Bolt EUV without a pedal on camera long (CHEVROLET_BOLT_EUV, or this car with no pedal) takes mode 1's fixes but the
# pedal ones (user request 2026-11-28): the same ECM and camera. With no pedal to tap, leaving the ECM's standstill stays
# the driver's (RES or the accelerator, with "Press Resume"): in the pedal car's logs carrot's own RES button frames never
# took the ECM out of its standstill while the hold was on (00000023--f5c4a454cd--13: 159 frames in 12.9 s, left by the
# driver's gas; 00000028--02c4808d1b--21: 257 frames in 20.5 s, left after the pedal tap), so nothing here relies on them.

NO_PEDAL_CAMERA_LONG = [(CAR.CHEVROLET_BOLT_EUV, False, True), (BOLT, False, True)]


def test_bolt_camera_long_setups(runtime, monkeypatch):
  from openpilot.cereal import car
  from opendbc.car.gm.values import is_bolt_camera_long
  for key in NO_PEDAL_CAMERA_LONG + [(BOLT, True, False)]:  # and mode 1
    assert is_bolt_camera_long(bolt_cp(runtime, *key)), key
  for key in [(CAR.CHEVROLET_BOLT_EUV, False, False), (BOLT, False, False),  # the stock ACC
              (CAR.CHEVROLET_BOLT_EUV, True, False), (CAR.CHEVROLET_BOLT_EUV, True, True),  # pedal long
              (CAR.CHEVROLET_BOLT_CC, False, True), (CAR.CHEVROLET_BOLT_CC, True, False)]:
    assert not is_bolt_camera_long(bolt_cp(runtime, *key)), key
  monkeypatch.setattr(runtime.interface, "Params", memory_params(BoltPedalLongMode=2))
  assert not is_bolt_camera_long(bolt_cp(runtime, BOLT, True, False))  # mode 2
  # LongControl gets controlsd's capnp CarParams
  assert is_bolt_camera_long(car.CarParams.new_message(carFingerprint="CHEVROLET_BOLT_EUV", openpilotLongitudinalControl=True))
  assert not is_bolt_camera_long(car.CarParams.new_message(carFingerprint="CHEVROLET_BOLT_EUV", openpilotLongitudinalControl=True,
                                                           flags=int(GMFlags.PEDAL_LONG)))
  assert not is_bolt_camera_long(car.CarParams.new_message(carFingerprint="CHEVROLET_BOLT_EUV"))


@pytest.mark.parametrize("key", NO_PEDAL_CAMERA_LONG, ids=lambda k: "/".join(map(str, k)))
def test_no_pedal_camera_long_car_params_are_mode_1s(runtime, key):
  cp = bolt_cp(runtime, *key)
  # the ECM's own rule (above 5 km/h, or stopped with the brake), as in mode 1 and StarPilot: engaging stopped without
  # the brake faulted this ECM (rlog 00000021--282be5cb94--1)
  assert cp.minEnableSpeed == pytest.approx(5 / 3.6)
  assert (cp.startAccel, cp.vEgoStarting, cp.vEgoStopping) == (pytest.approx(0.55), pytest.approx(0.25), pytest.approx(0.25))
  assert not cp.autoResumeSng and not cp.enableGasInterceptorDEPRECATED  # "Press Resume" stays
  mode_1 = bolt_cp(runtime, BOLT, True, False)
  assert (mode_1.minEnableSpeed, mode_1.startAccel, mode_1.vEgoStarting, mode_1.vEgoStopping) == \
         (cp.minEnableSpeed, cp.startAccel, cp.vEgoStarting, cp.vEgoStopping)


@pytest.mark.parametrize("car", [CAR.CHEVROLET_BOLT_EUV, BOLT])
def test_stock_acc_bolts_keep_their_car_params(runtime, car):
  cp = bolt_cp(runtime, car, False, False)
  assert (cp.minEnableSpeed, cp.startAccel) == (pytest.approx(-1 / 3.6), pytest.approx(1.0))
  assert (cp.vEgoStarting, cp.vEgoStopping) == (pytest.approx(0.4), pytest.approx(0.5))


def no_pedal_frames(runtime, monkeypatch, cp, **kwargs):
  monkeypatch.setattr(runtime.controller, "Params", memory_params())
  controller = runtime.controller.CarController(DBC[cp.carFingerprint], cp)
  frames = []
  for f in range(24):
    sends = hybrid_step(controller, f, **kwargs)
    assert not [addr for addr, dat, bus in sends if addr == 0x200]  # never a pedal frame
    gas = [dat for addr, dat, bus in sends if addr == 0x2CB]
    brake = [dat for addr, dat, bus in sends if addr == 0x315]
    if gas and brake:
      frames.append((gas[0][0] & 1, (gas[0][1] >> 5) & 1, brake[0][0] >> 4, friction_brake(brake[0])))
  return frames


@pytest.mark.parametrize("key", NO_PEDAL_CAMERA_LONG, ids=lambda k: "/".join(map(str, k)))
def test_no_pedal_resume_at_the_ecm_standstill_keeps_apply_and_the_hold(runtime, monkeypatch, key):
  """rlog 0000001d--96a597633a--10 (the pedal car, same ECM): controlsd's resume at the ECM's standstill dropped 0x2CB
  apply, the ECM's ACC went Off and then Faulted once apply came back. The Bolt EUV without a pedal still had that rule
  (StarPilot keeps it only for its Volt SNG option): now apply stays 1 with the FullStop / 0xD hold, as in mode 1."""
  cp = bolt_cp(runtime, *key)
  kwargs = dict(accel=-0.5, state="stopping", v=0.0, cruise_standstill=True, resume=True)
  frames = no_pedal_frames(runtime, monkeypatch, cp, **kwargs)
  assert frames and set(frames) == {(1, 1, 0xD, 40)}
  without_the_bolt_euv_port(runtime, monkeypatch)  # before: apply dropped to 0 there
  frames = no_pedal_frames(runtime, monkeypatch, cp, **kwargs)
  assert frames and {f[0] for f in frames} == {0}


@pytest.mark.parametrize("key", NO_PEDAL_CAMERA_LONG, ids=lambda k: "/".join(map(str, k)))
def test_no_pedal_stop_hold_and_start(runtime, monkeypatch, key):
  cp = bolt_cp(runtime, *key)
  for standstill in (False, True):  # stopped: FullStop with 0xD, as the stock camera
    frames = no_pedal_frames(runtime, monkeypatch, cp, accel=-0.5, state="stopping", v=0.0, cruise_standstill=standstill)
    assert frames and set(frames) == {(1, 1, 0xD, 40)}, standstill
  # starting once the driver has taken the ECM out of its standstill: the hold is released, apply stays 1
  frames = no_pedal_frames(runtime, monkeypatch, cp, accel=0.55, state="starting", v=0.0, cruise_standstill=False)
  assert frames and set(frames) == {(1, 0, 0x9, 0)}


@pytest.mark.parametrize("key", NO_PEDAL_CAMERA_LONG, ids=lambda k: "/".join(map(str, k)))
def test_no_pedal_longcontrol_waits_on_the_ecm_then_starts_like_mode_1(runtime, monkeypatch, key):
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", MemoryParams)
  cp = bolt_cp(runtime, *key)
  loc = longcontrol.LongControl(cp)
  lc_step(runtime, loc, cp, -0.5, 0.0, 0.0, stop=True, standstill=True)
  lc_step(runtime, loc, cp, 0.21, 0.0, 0.0, lead=True, standstill=True)
  assert loc.long_control_state == LongCtrlState.stopping  # no pedal: the driver's RES or gas leaves the standstill
  assert lc_step(runtime, loc, cp, 0.21, 0.0, 0.0, lead=True) == pytest.approx(0.21)  # mode 1's slow plan start
  assert loc.long_control_state == LongCtrlState.starting
  loc = longcontrol.LongControl(cp)
  lc_step(runtime, loc, cp, -0.5, 0.0, 0.0, stop=True, standstill=True)
  assert lc_step(runtime, loc, cp, 0.1, 0.0, 0.0, lead=False) == pytest.approx(0.55)  # startAccel


@pytest.mark.parametrize("key", NO_PEDAL_CAMERA_LONG, ids=lambda k: "/".join(map(str, k)))
def test_no_pedal_stop_ramps_only_to_a_quarter(runtime, monkeypatch, key):
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", MemoryParams)
  cp = bolt_cp(runtime, *key)
  loc = longcontrol.LongControl(cp)
  outs = stop_from(runtime, loc, cp, -0.05)
  assert -0.25 - cp.stoppingDecelRate * 0.01 - 1e-9 <= outs[-1] <= -0.25


def ecm_standstill_carstate(runtime, cp, pedal, cruise_state=4):
  """Real DBC, packer, parser and CarState: stopped, ACC main on, the ECM's CruiseState (4: standstill), no brake or gas."""
  cs = runtime.state.CarState(cp)
  parsers = cs.get_can_parsers(cp)
  cs.update(parsers)
  packer = runtime.CANPacker(DBC[cp.carFingerprint][Bus.pt])
  override = [
    packer.make_can_msg("EBCMWheelSpdFront", 0, {"FLWheelSpd": 0, "FRWheelSpd": 0}),
    packer.make_can_msg("EBCMWheelSpdRear", 0, {"RLWheelSpd": 0, "RRWheelSpd": 0}),
    packer.make_can_msg("ECMEngineStatus", 0, {"CruiseMainOn": 1, "BrakePressed": 0}),
    packer.make_can_msg("AcceleratorPedal2", 0, {"CruiseState": cruise_state, "AcceleratorPedal2": 0}),
    packer.make_can_msg("ECMAcceleratorPos", 0, {"BrakePedalPos": 0}),
    packer.make_can_msg("ECMPRDNL2", 0, {"PRNDL2": 4}),
  ]
  replaced = {addr for addr, _, _ in override}
  frames = [f for f in carstate_frames(packer, 0, pedal) if f[0] not in replaced] + override
  for tick in range(20):
    for parser in parsers.values():
      parser.update([(1_000_000_000 + tick * 10_000_000, frames)])
    out = cs.update(parsers)
  return cs, out


@pytest.mark.parametrize("key", NO_PEDAL_CAMERA_LONG, ids=lambda k: "/".join(map(str, k)))
def test_no_pedal_waits_at_the_ecm_standstill_from_can_to_can(runtime, monkeypatch, key):
  """Codex review R1 (2026-11-28): CarState cleared the ECM's standstill on these cars, so LongControl started (hold
  released, gas asked) with the ECM still in standstill and "Press Resume" never showed. From the CAN parser on: the
  standstill is kept, LongControl waits, the hold stays with apply 1, "Press Resume" shows, and a RES there resumes
  instead of raising the set speed (it went 80 -> 81)."""
  import sys
  from openpilot.cereal import car, log
  monkeypatch.delitem(sys.modules, "openpilot.common.realtime", raising=False)  # the runtime fixture's stub lacks what
  from openpilot.selfdrive.car.car_specific import CarSpecificEvents             # car_specific's imports need
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", MemoryParams)
  cp = bolt_cp(runtime, *key)
  cs, out = ecm_standstill_carstate(runtime, cp, pedal=False)
  assert cs.pcm_acc_status == 4 and out.standstill and out.cruiseState.standstill

  cp_reader = car.CarParams.from_bytes(cp.to_bytes()).__enter__()  # as controlsd gets it
  loc = longcontrol.LongControl(cp_reader)
  limits = runtime.interface.CarInterface.get_pid_accel_limits(cp, 0.0, 0.0)
  plan = log.LongitudinalPlan.new_message(aTarget=-0.5, vTargetNow=0.0, shouldStop=True, hasLead=True)
  loc.update(True, out, plan, limits, 0.0, log.RadarState.new_message())
  plan.shouldStop, plan.aTarget = False, 0.21  # the lead moves off
  accel = float(loc.update(True, out, plan, limits, 0.0, log.RadarState.new_message())[0])
  assert loc.long_control_state == LongCtrlState.stopping and accel < 0

  cc = structs.CarControl(enabled=True, longActive=True, latActive=True)
  cc.actuators.accel, cc.actuators.longControlState = accel, loc.long_control_state
  cc.cruiseControl.resume = out.cruiseState.standstill  # controlsd: standstill and the plan's end speed above 0.1
  cc.orientationNED = [0.0, 0.0, 0.0]
  events = CarSpecificEvents(cp_reader).update(out, car.CarState.new_message(), cc)
  assert log.OnroadEvent.EventName.resumeRequired in events.names

  monkeypatch.setattr(runtime.controller, "Params", memory_params())
  controller = runtime.controller.CarController(DBC[cp.carFingerprint], cp)
  cs.out = out
  sends = []
  for tick in range(12):
    sends += controller.update(cc.as_reader(), cs, 2_000_000_000 + tick * 10_000_000)[1]
  assert {(d[0] & 1, (d[1] >> 5) & 1) for a, d, b in sends if a == 0x2CB} == {(1, 1)}
  assert {(d[0] >> 4, friction_brake(d)) for a, d, b in sends if a == 0x315} == {(0xD, 40)}
  assert not [a for a, d, b in sends if a == 0x200]

  assert res_gesture_speeds(runtime, cp, cc, [out, out]) == [80, 80]


@pytest.mark.parametrize("key", [(CAR.CHEVROLET_BOLT_EUV, True, False), (CAR.CHEVROLET_BOLT_EUV, False, False),
                                 (BOLT, False, False)], ids=lambda k: "/".join(map(str, k)))
def test_pedal_long_and_stock_acc_bolts_still_clear_the_ecm_standstill(runtime, key):
  cp = bolt_cp(runtime, *key)
  cs, out = ecm_standstill_carstate(runtime, cp, pedal=key[1])
  assert cs.pcm_acc_status == 4 and not out.cruiseState.standstill


def res_gesture_speeds(runtime, cp, cc, states, mode=0):
  """carrot's cruise buttons (VCruiseCarrot) for one RES press then release, each with its CarState; the set speeds."""
  from openpilot.cereal import car
  from openpilot.selfdrive.car.tests.test_carrot_cruise_buttons import make_cruise_helper, make_button_tracker
  helper, _, _ = make_cruise_helper(81, mode, False, True)
  helper.CP = cp
  helper._standstill_res_guard = runtime.cruise.is_bolt_camera_long(cp)  # as __init__ does
  helper._standstill_res_active = False
  tracker = make_button_tracker()
  for name in ("button_cnt", "button_prev", "button_long_time", "button_big_step", "long_pressed",
               "_cruise_speed_unit_basic", "_cruise_button_long_delay", "is_metric"):
    setattr(helper, name, getattr(tracker, name))
  helper._prepare_buttons = runtime.cruise.VCruiseCarrot._prepare_buttons.__get__(helper)
  speed, speeds = 80, []
  for pressed, out in zip((True, False), states, strict=True):
    out.buttonEvents = [car.CarState.ButtonEvent(type="accelCruise", pressed=pressed)]
    speed = helper._update_cruise_buttons(out, cc, speed)
    speeds.append(speed)
  return speeds


def test_standstill_res_guard_comes_from_car_params():
  import inspect
  from openpilot.selfdrive.car.cruise import VCruiseCarrot
  assert "self._standstill_res_guard = is_bolt_camera_long(CP)" in inspect.getsource(VCruiseCarrot.__init__)


@pytest.mark.parametrize("mode,raised", [(0, 81), (2, 90)])  # CruiseButtonMode (not BoltPedalLongMode)
@pytest.mark.parametrize("key", NO_PEDAL_CAMERA_LONG + [(BOLT, True, False)], ids=lambda k: "/".join(map(str, k)))
def test_res_pressed_in_the_ecm_standstill_only_resumes(runtime, key, mode, raised):
  """Codex review R2-2 (2026-11-28): RES pressed in the ECM's standstill, the ECM goes active before it is released: the
  release raised the set speed (80 -> 81, or 90 with CruiseButtonMode 2). Now the gesture only resumes; the next RES,
  pressed with the ECM active, still raises it."""
  cp = bolt_cp(runtime, *key)
  _, standstill = ecm_standstill_carstate(runtime, cp, pedal=key[1], cruise_state=4)
  _, active = ecm_standstill_carstate(runtime, cp, pedal=key[1], cruise_state=1)
  assert standstill.cruiseState.standstill and not active.cruiseState.standstill
  cc = structs.CarControl(enabled=True, longActive=True)
  assert res_gesture_speeds(runtime, cp, cc, [standstill, active], mode) == [80, 80]
  assert res_gesture_speeds(runtime, cp, cc, [active, active], mode) == [80, raised]


@pytest.mark.parametrize("key", [(CAR.CHEVROLET_BOLT_EUV, True, False), (CAR.CHEVROLET_BOLT_EUV, False, False)],
                         ids=lambda k: "/".join(map(str, k)))
def test_other_bolts_keep_carrots_res(runtime, key):
  cp = bolt_cp(runtime, *key)
  assert not runtime.cruise.is_bolt_camera_long(cp)
  _, active = ecm_standstill_carstate(runtime, cp, pedal=key[1], cruise_state=1)
  assert res_gesture_speeds(runtime, cp, structs.CarControl(enabled=True, longActive=True), [active, active]) == [80, 81]


def lc_from_can(runtime, monkeypatch, cp, states, plans):
  """LongControl from the CAN parser on (a serialized CarParams, as controlsd gets it): its state after each step."""
  from openpilot.cereal import car, log
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", MemoryParams)
  loc = longcontrol.LongControl(car.CarParams.from_bytes(cp.to_bytes()).__enter__())
  limits = runtime.interface.CarInterface.get_pid_accel_limits(cp, 0.0, 0.0)
  result = []
  for out, (stop, a_target) in zip(states, plans, strict=True):
    plan = log.LongitudinalPlan.new_message(aTarget=a_target, vTargetNow=0.0, shouldStop=stop, hasLead=True)
    accel = float(loc.update(True, out, plan, limits, 0.0, log.RadarState.new_message())[0])
    result.append((loc.long_control_state, accel))
  return result


@pytest.mark.parametrize("key", NO_PEDAL_CAMERA_LONG, ids=lambda k: "/".join(map(str, k)))
def test_no_pedal_holds_again_when_the_ecm_standstill_comes_after_starting(runtime, monkeypatch, key):
  """Codex review R2-1 (2026-11-28): the plan let go of the stop while the ECM was still active (1), LongControl started
  (hold released, gas asked), then the ECM's standstill (4) came in and starting went on. Now it goes back to stopping and
  the hold until the ECM leaves its standstill; the same in the other order."""
  cp = bolt_cp(runtime, *key)
  _, active = ecm_standstill_carstate(runtime, cp, pedal=False, cruise_state=1)
  cs, standstill = ecm_standstill_carstate(runtime, cp, pedal=False, cruise_state=4)
  steps = lc_from_can(runtime, monkeypatch, cp, [active, active, standstill, standstill, active],
                      [(True, -0.5), (False, 0.21), (False, 0.21), (False, 0.21), (False, 0.21)])
  assert [s for s, _ in steps] == [LongCtrlState.stopping, LongCtrlState.starting, LongCtrlState.stopping,
                                   LongCtrlState.stopping, LongCtrlState.starting]
  assert steps[2][1] <= 0. and steps[3][1] <= 0. and steps[4][1] == pytest.approx(0.21)
  # what the controller sends back in the hold
  cc = structs.CarControl(enabled=True, longActive=True, latActive=True)
  cc.actuators.accel, cc.actuators.longControlState = steps[3][1], steps[3][0]
  cc.cruiseControl.resume = True
  cc.orientationNED = [0.0, 0.0, 0.0]
  monkeypatch.setattr(runtime.controller, "Params", memory_params())
  controller = runtime.controller.CarController(DBC[cp.carFingerprint], cp)
  cs.out = standstill
  sends = []
  for tick in range(12):
    sends += controller.update(cc.as_reader(), cs, 2_000_000_000 + tick * 10_000_000)[1]
  assert {(d[0] & 1, (d[1] >> 5) & 1) for a, d, b in sends if a == 0x2CB} == {(1, 1)}
  assert {(d[0] >> 4, friction_brake(d)) for a, d, b in sends if a == 0x315} == {(0xD, 40)}


def test_mode_1_keeps_starting_into_the_ecm_standstill(runtime, monkeypatch):
  """Mode 1 taps the pedal from there instead (ignore_cruise_standstill)."""
  cp = bolt_cp(runtime, BOLT, True, False)
  _, active = ecm_standstill_carstate(runtime, cp, pedal=True, cruise_state=1)
  _, standstill = ecm_standstill_carstate(runtime, cp, pedal=True, cruise_state=4)
  steps = lc_from_can(runtime, monkeypatch, cp, [active, active, standstill], [(True, -0.5), (False, 0.21), (False, 0.21)])
  assert [s for s, _ in steps] == [LongCtrlState.stopping, LongCtrlState.starting, LongCtrlState.starting]
