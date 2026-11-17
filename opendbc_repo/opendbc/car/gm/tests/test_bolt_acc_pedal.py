"""Bolt ACC with a comma pedal (CHEVROLET_BOLT_ACC_PEDAL), ported from StarPilot's CHEVROLET_BOLT_ACC_2022_2023_PEDAL
(firestar5683/StarPilot 2589caad). Real DBC, packer, parser, CarState and CarController; no device I/O.

Expected values come from StarPilot's code at that commit (quoted in the port spec), computed by hand or by the
reference copies below, which are verbatim transcriptions of StarPilot's pure functions."""
import hashlib
import json
from collections import defaultdict
from types import SimpleNamespace as NS

import numpy as np
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


def digest(stream):
  return hashlib.sha256(json.dumps(stream).encode()).hexdigest()


@pytest.mark.parametrize("key", BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_controller_streams_unchanged(runtime, monkeypatch, key):
  assert digest(controller_stream(runtime, monkeypatch, *key)) == BASELINE_DIGESTS["ctrl:" + "/".join(map(str, key))]


@pytest.mark.parametrize("key", CARSTATE_BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_carstate_streams_unchanged(runtime, monkeypatch, key):
  assert digest(carstate_stream(runtime, monkeypatch, *key)) == BASELINE_DIGESTS["cs:" + "/".join(map(str, key))]


@pytest.mark.parametrize("key", LONGCONTROL_BASELINE_CASES, ids=lambda k: "/".join(map(str, k)))
def test_other_bolt_longcontrol_streams_unchanged(runtime, monkeypatch, key):
  stream = longcontrol_stream(runtime, monkeypatch, bolt_cp(runtime, *key))
  assert digest(stream) == BASELINE_DIGESTS["lc:" + "/".join(map(str, key))]


# ---------------------------------------------------------------------------------------------------------------------
# CarParams

BOLT = CAR.CHEVROLET_BOLT_ACC_PEDAL
STARPILOT_SAFETY = GMSafetyFlags.HW_CAM | GMSafetyFlags.NO_ACC | GMSafetyFlags.PEDAL_LONG | GMSafetyFlags.GAS_INTERCEPTOR | \
                   GMSafetyFlags.BOLT_ACC_PEDAL


@pytest.mark.parametrize("alpha", [False, True])
def test_car_params_with_pedal_match_starpilot(runtime, alpha):
  cp = bolt_cp(runtime, BOLT, True, alpha)
  assert cp.safetyConfigs[0].safetyParam == STARPILOT_SAFETY == 2273
  assert cp.enableGasInterceptorDEPRECATED and cp.flags & GMFlags.PEDAL_LONG and not cp.flags & GMFlags.CC_LONG
  assert cp.openpilotLongitudinalControl and not cp.pcmCruise and not cp.alphaLongitudinalAvailable
  assert cp.networkLocation == structs.CarParams.NetworkLocation.fwdCamera and cp.radarUnavailable
  assert cp.transmissionType == structs.CarParams.TransmissionType.direct
  assert cp.autoResumeSng and cp.minEnableSpeed == -1
  assert cp.mass == pytest.approx(1669 + 136) and cp.wheelbase == pytest.approx(2.63779)
  # carrot's single-breakpoint GM tuning at the LongTuning* defaults, and the LongActuatorDelay Param like the other GM cars
  lt = cp.longitudinalTuning
  assert (list(lt.kpBP), list(lt.kpV), list(lt.kiBP), list(lt.kiV), lt.kf) == ([0.0], [1.0], [0.0], [0.0], 1.0)
  assert cp.longitudinalActuatorDelay == pytest.approx(bolt_cp(runtime, CAR.CHEVROLET_BOLT_CC, True, alpha).longitudinalActuatorDelay)
  assert cp.startingState
  assert (cp.startAccel, cp.vEgoStarting, cp.vEgoStopping, cp.stopAccel, cp.stoppingDecelRate) == \
         pytest.approx((0.55, 0.35, 0.25, -0.25, 0.8))


def test_car_params_without_camera_keep_carrot_no_camera_bit(runtime):
  cp = bolt_cp(runtime, BOLT, True, False, camera=False)
  assert cp.safetyConfigs[0].safetyParam == STARPILOT_SAFETY | GMSafetyFlags.NO_CAMERA


@pytest.mark.parametrize("alpha", [False, True])
def test_car_params_without_pedal_have_no_pedal_bits(runtime, alpha):
  cp = bolt_cp(runtime, BOLT, False, alpha)
  param = cp.safetyConfigs[0].safetyParam
  assert not param & (GMSafetyFlags.BOLT_ACC_PEDAL | GMSafetyFlags.GAS_INTERCEPTOR | GMSafetyFlags.PEDAL_LONG)
  assert not cp.enableGasInterceptorDEPRECATED and not cp.flags & GMFlags.PEDAL_LONG
  assert runtime.interface.CarInterface.get_pid_accel_limits(cp, 3.0, 0.0) == (-4.0, 2.0)
  assert not runtime.controller.supports_bolt_acc_pedal_friction_experiment(cp)


@pytest.mark.parametrize("speed,accel_max", [(0.0, 0.54), (1.5, 0.74), (2.75, 0.885), (6.0, 1.245), (8.0, 1.46), (15.0, 2.0), (30.0, 2.0)])
def test_pid_accel_limits(runtime, speed, accel_max):
  cp = bolt_cp(runtime, BOLT, True, False)
  accel_min, upper = runtime.interface.CarInterface.get_pid_accel_limits(cp, speed, 0.0)
  assert accel_min == -4.0 and upper == pytest.approx(accel_max)


# ---------------------------------------------------------------------------------------------------------------------
# CarState


def test_carstate_follows_the_camera_acc_and_starpilot_fields(runtime, monkeypatch):
  rows = carstate_stream(runtime, monkeypatch, BOLT, True, False, extra=lambda cs: {"fcw_level": cs.stock_fcw_alert})
  # ACCCmdActive on the camera's 0x370 (ticks 100-159), never ECMCruiseControl.CruiseActive (ticks 60-89)
  assert [t for t, r in enumerate(rows) if r["enabled"]] == list(range(100, 160))
  assert all(r["speed"] == pytest.approx(72 / 3.6) for r in rows[1:])  # 0x3D1 set speed
  # the stock standstill passes on as in StarPilot (controlsd's resume launches through the pedal)
  assert [t for t, r in enumerate(rows) if r["cruise_standstill"]] == list(range(120, 140))  # AcceleratorPedal2 4
  # camera FCWAlert kept for the 0x370 replay; stockFcw stays off (carrot would sound an FCW, StarPilot does not)
  assert [t for t, r in enumerate(rows) if r["fcw_level"] == 2] == list(range(170, 180))
  assert not any(r["stockFcw"] == "True" for r in rows)
  assert [t for t, r in enumerate(rows) if r["stockAeb"] == "True"] == list(range(190, 195))
  assert not any(r["accFaulted"] == "True" for r in rows)  # CC-only: even FrictionBrakeUnavailable (310-314)
  assert rows[245]["gasPressed"] == "True" and rows[252]["gasPressed"] == "False"  # raw averages 633 / 582 against the panda's 595
  assert [t for t, r in enumerate(rows) if r["regenBraking"] == "True"] == list(range(250, 260))
  assert all(r["single_pedal"] for r in rows)
  assert [t for t, r in enumerate(rows) if not r["available"]] == list(range(280, 320))


def test_carstate_without_camera_uses_ecm_cruise(runtime, monkeypatch):
  rows = carstate_stream(runtime, monkeypatch, BOLT, True, False, camera=False)
  assert [t for t, r in enumerate(rows) if r["enabled"]] == list(range(60, 90))
  assert not any(r["stockFcw"] == "True" or r["stockAeb"] == "True" for r in rows)


# ---------------------------------------------------------------------------------------------------------------------
# CarController


def make_inputs(enabled=True, long_active=None, accel=0.0, state="pid", v=10.0, a_ego=0.0, available=True,
                cruise_enabled=False, gas=False, brake=False, set_speed=25.0, lead=False, bars=2, alert="none",
                stock_fcw=0, stock_aeb=False, counter=0, resume=False, pitch=0.0, activate=0):
  cc = structs.CarControl(enabled=enabled, longActive=enabled if long_active is None else long_active, latActive=enabled)
  cc.actuators.accel = accel
  cc.actuators.longControlState = state if cc.longActive else "off"
  cc.cruiseControl.resume = resume
  cc.hudControl.setSpeed = set_speed
  cc.hudControl.leadVisible = lead
  cc.hudControl.leadDistanceBars = bars
  cc.hudControl.visualAlert = alert
  cc.orientationNED = [0.0, pitch, 0.0]
  out = structs.CarState(vEgo=v, aEgo=a_ego, standstill=v < 0.05, gasPressed=gas, brakePressed=brake, stockAeb=stock_aeb,
                         activateCruise=activate)
  out.cruiseState.available = available
  out.cruiseState.enabled = cruise_enabled
  out.gearShifter = "drive"
  cs = NS(out=out, loopback_lka_steering_cmd_ts_nanos=0, loopback_lka_steering_cmd_updated=False,
          cam_lka_steering_cmd_counter=0, pt_lka_steering_cmd_counter=0, buttons_counter=counter, pcm_acc_status=0,
          pscm_status=defaultdict(int), stock_fcw_alert=stock_fcw)
  return cc, cs


class Drive:
  def __init__(self, runtime, monkeypatch, pitch=0, camera=True):
    monkeypatch.setattr(runtime.controller, "Params", memory_params(LongPitch=pitch))
    cp = bolt_cp(runtime, BOLT, True, False, camera)
    self.controller = runtime.controller.CarController(DBC[BOLT], cp)
    self.packer = runtime.CANPacker(DBC[BOLT][Bus.pt])
    self.frame = 0

  def step(self, **kwargs):
    cc, cs = make_inputs(**kwargs)
    cs.CP = self.controller.CP
    output, sends = self.controller.update(cc.as_reader(), cs, 1_000_000_000 + self.frame * 10_000_000)
    self.frame += 1
    return output, [(addr, bytes(dat), bus) for addr, dat, bus in sends]

  def step25(self, **kwargs):
    """Run frames with the same inputs through the next 25 Hz (frame % 4 == 0) frame and return that frame's."""
    while True:
      due = self.frame % 4 == 0
      output, sends = self.step(**kwargs)
      if due:
        return output, sends


def by_addr(sends, addr):
  return [(dat, bus) for a, dat, bus in sends if a == addr]


def friction(dat):
  cmd = ((dat[0] & 0xF) << 8) | dat[1]
  return {"mode": dat[0] >> 4, "brake": -(cmd - 0x1000 if cmd & 0x800 else cmd), "checksum": (dat[2] << 8) | dat[3],
          "counter": dat[4] & 0x3}


def test_cancel_goes_to_the_camera_at_20hz_only_while_its_acc_is_active(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch)
  cancels = []
  for tick in range(300):
    cruise = 20 <= tick < 120 or 200 <= tick < 260
    _, sends = drive.step(enabled=100 <= tick < 220, cruise_enabled=cruise, counter=(tick // 7) % 4,
                          activate=1, state="stopping" if 150 <= tick < 180 else "pid", accel=0.3)
    for dat, bus in by_addr(sends, 0x1E1):
      assert bus == CanBus.CAMERA
      assert (dat[5] >> 4) & 0x7 == CruiseButtons.CANCEL
      assert dat[4] & 0x3 == ((tick // 7) % 4 + 1) % 4
      cancels.append(tick)
  assert cancels == list(range(20, 120, 5)) + list(range(200, 260, 5))


def test_no_stock_acc_commands_or_other_buttons_over_a_whole_drive(runtime, monkeypatch):
  stream = controller_stream(runtime, monkeypatch, BOLT, True, False, 1)
  addrs = {m[0] for row in stream for m in row[0]}
  assert addrs == {0x180, 0x184, 0x1E1, 0xBD, 0x1F5, 0x200, 0x315, 0x370}
  assert not addrs & {0x2CB, 0x2CD, 0x3D1, 0x409, 0x40A}
  buttons = [(bytes.fromhex(m[1]), m[2]) for row in stream for m in row[0] if m[0] == 0x1E1]
  assert buttons and all(bus == CanBus.CAMERA and (dat[5] >> 4) & 0x7 == CruiseButtons.CANCEL for dat, bus in buttons)


def test_message_rates_buses_and_paddle_handoff(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch)
  seen = defaultdict(list)
  for tick in range(400):
    enabled = 20 <= tick < 200 or 240 <= tick < 301
    _, sends = drive.step(enabled=enabled, available=tick < 340, accel=-2.0 if 100 <= tick < 120 else 0.5)
    for addr, dat, bus in sends:
      seen[addr].append((tick, bus, len(dat)))
  quarter = list(range(0, 400, 4))
  assert seen[0x200] == [(t, 0, 6) for t in quarter]  # always, ENABLE 0 when not engaged
  assert seen[0x370] == [(t, 0, 6) for t in quarter]
  # main on, then 5 release frames of zero brake after main off (the last brake was nonzero at 100-119)
  assert [t for t, *_ in seen[0x315]] == [t for t in quarter if t < 340]
  assert {(bus, n) for _, bus, n in seen[0x315]} == {(0, 5)}
  # 0xBD/0x1F5 while enabled and 2 frames after; tick 200 is a 4th frame inside that handoff, 301-302 are not
  feed = [t for t in quarter if 20 <= t < 200 or 240 <= t < 301] + [200]
  assert [t for t, *_ in seen[0xBD]] == sorted(feed) and {(b, n) for _, b, n in seen[0xBD]} == {(0, 7)}
  assert [t for t, *_ in seen[0x1F5]] == sorted(feed) and {(b, n) for _, b, n in seen[0x1F5]} == {(0, 8)}


def test_friction_release_frames_after_main_off(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch)
  sent = []
  for tick in range(80):
    _, sends = drive.step(accel=-2.0, available=tick < 40)
    sent += [(tick, friction(dat)) for dat, _ in by_addr(sends, 0x315)]
  # release_frames 5 counts down before the send check, so 4 zero frames follow (StarPilot)
  assert [t for t, _ in sent] == list(range(0, 40, 4)) + [40, 44, 48, 52]
  assert all(f["brake"] == 183 and f["mode"] == 0xA for t, f in sent if t < 40)
  assert all(f["brake"] == 0 and f["mode"] == 0x1 for t, f in sent if t >= 40)


def test_pedal_values_follow_starpilot_calc_pedal_command(runtime, monkeypatch):
  from opendbc.car import create_gas_interceptor_command
  drive = Drive(runtime, monkeypatch)
  _, sends = drive.step(accel=1.0, v=10.0)  # first activation: target directly
  assert by_addr(sends, 0x200) == [(bytes(create_gas_interceptor_command(drive.packer, 0.7232126984126984, 0)[1]), 0)]
  for _ in range(3):
    drive.step(accel=2.0, v=10.0)
  _, sends = drive.step(accel=2.0, v=10.0)  # rate-limited toward 0.89 by 0.0388333 per 25 Hz frame
  assert by_addr(sends, 0x200) == [(bytes(create_gas_interceptor_command(drive.packer, 0.7620460317460318, 1)[1]), 0)]


def test_hard_decel_presses_the_paddle_and_brakes_like_starpilot(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch)
  output, sends = drive.step(accel=-2.0, v=10.0)
  assert by_addr(sends, 0xBD) == [(bytes.fromhex("20000000000000"), 0)]
  assert by_addr(sends, 0x1F5) == [(bytes.fromhex("0c0c000500020100"), 0)]
  assert by_addr(sends, 0x200)[0][0][4] & 0x80 == 0  # pedal released
  brake = friction(by_addr(sends, 0x315)[0][0])
  # switch 5500, mapped 92, remapped x1.369 x1.114 x1.304 = 183 (spec section 8 values for 10 m/s)
  assert brake == {"mode": 0xA, "brake": 183, "checksum": (0x10000 - (0xA << 12) - ((0x1000 - 183) & 0xFFF)) & 0xFFFF,
                   "counter": 0}
  assert (output.gas, output.brake) == (5500, 183)


def test_paddle_hysteresis_holds_22_frames_and_is_not_sent_below_2_68(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch)
  pressed = []
  for tick in range(200):
    _, sends = drive.step(accel=-2.0 if tick < 4 else 0.0, v=10.0)
    pressed += [tick for dat, _ in by_addr(sends, 0xBD) if dat[0] == 0x20]
  assert pressed == list(range(0, 88, 4))  # min_on 22 at 10 m/s (25 Hz frames)

  slow = Drive(runtime, monkeypatch)
  _, sends = slow.step(accel=-2.0, v=2.0)
  assert slow.controller.bolt_acc_pedal.regen_paddle_pressed  # pressed internally ...
  assert by_addr(sends, 0xBD) == [(bytes(7), 0)]  # ... but not spoofed at or below 2.68 m/s
  assert by_addr(sends, 0x1F5) == [(bytes.fromhex("0c0c000600000100"), 0)]


def pedal_enabled(sends):
  (dat, bus), = by_addr(sends, 0x200)
  return bool(dat[4] & 0x80)


def test_stop_hold_zeroes_pedal_and_fades_friction_to_zero(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch)
  _, sends = drive.step25(accel=-0.6, v=3.0)
  assert friction(by_addr(sends, 0x315)[0][0])["brake"] == 56  # low-speed friction engaged (>= 12 at 3 m/s)
  _, sends = drive.step25(accel=-0.25, v=0.0, state="stopping")
  # 25 -> 40 remapped, then faded to 0 below 0.6 m/s while stopping; mode 0x9 (main on, longActive, no brake)
  assert friction(by_addr(sends, 0x315)[0][0]) | {"checksum": 0} == {"mode": 0x9, "brake": 0, "checksum": 0, "counter": 1}
  assert drive.controller.bolt_acc_pedal.bolt_acc_pedal_stop_hold and not pedal_enabled(sends)
  # still stopping after creeping past the standstill threshold: the hold stays latched
  _, sends = drive.step25(accel=-0.25, v=0.2, state="stopping")
  assert drive.controller.bolt_acc_pedal.bolt_acc_pedal_stop_hold and not pedal_enabled(sends)
  # starting releases it; the launch floor (18/255) lifts the rate-limited pedal off zero
  _, sends = drive.step25(accel=0.55, v=0.2, state="starting")
  assert not drive.controller.bolt_acc_pedal.bolt_acc_pedal_stop_hold and pedal_enabled(sends)
  from opendbc.car import create_gas_interceptor_command
  assert by_addr(sends, 0x200) == [(bytes(create_gas_interceptor_command(drive.packer, 18. / 255., 3)[1]), 0)]


def test_sng_launch_floor_on_a_weak_request_near_a_stop(runtime, monkeypatch):
  from opendbc.car import create_gas_interceptor_command
  drive = Drive(runtime, monkeypatch)
  output, sends = drive.step(accel=-0.2, v=0.2)  # pedal 0.0429 < 18/255; brake 33 < engage 37.3, gas torque 6026
  assert by_addr(sends, 0x200) == [(bytes(create_gas_interceptor_command(drive.packer, 18. / 255., 0)[1]), 0)]
  assert (output.gas, output.brake) == (5500, 0)
  assert friction(by_addr(sends, 0x315)[0][0])["brake"] == 0


@pytest.mark.parametrize("kwargs,expected", [
  (dict(), dict(ACCCmdActive=1, ACCGapLevel=2, ACCLeadCar=0, FCWAlert=0, ACCSpeedSetpoint=90.0)),
  (dict(lead=True, bars=3, set_speed=80.0), dict(ACCCmdActive=1, ACCGapLevel=3, ACCLeadCar=1, ACCSpeedSetpoint=0.0)),
  (dict(enabled=False), dict(ACCCmdActive=0, ACCGapLevel=0)),
  (dict(alert="fcw"), dict(FCWAlert=3)),
  (dict(stock_fcw=2), dict(FCWAlert=2)),
  (dict(stock_aeb=True), dict(FCWAlert=3)),
])
def test_dashboard_0x370_is_starpilots(runtime, monkeypatch, kwargs, expected):
  drive = Drive(runtime, monkeypatch)
  _, sends = drive.step(**kwargs)
  parser = runtime.CANParser(DBC[BOLT][Bus.pt], [("ASCMActiveCruiseControlStatus", float("nan"))], 0)
  parser.update([(1_000_000_000, [(a, d, b) for a, d, b in sends if a == 0x370])])
  values = parser.vl["ASCMActiveCruiseControlStatus"]
  assert (values["ACCCruiseState"], values["ACCAlwaysOne"], values["ACCAlwaysOne2"], values["ACCResumeButton"]) == (2, 1, 1, 0)
  assert {k: values[k] for k in expected} == pytest.approx(expected)


def test_keepalive_only_without_the_camera(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch, camera=False)
  ticks = []
  for tick in range(250):
    _, sends = drive.step()
    ticks += [(tick, a) for a, _, _ in sends if a in (0x409, 0x40A)]
  assert ticks == [(t, a) for t in (0, 100, 200) for a in (0x409, 0x40A)]


# ---------------------------------------------------------------------------------------------------------------------
# Message builders against StarPilot's


def starpilot_pedal_checksum(d):
  # StarPilot opendbc/can/dbc.py pedal_checksum (CRC8 over the payload minus its last byte, reversed)
  crc = 0xFF
  poly = 0xD5
  for i in range(len(d) - 2, -1, -1):
    crc ^= d[i]
    for _ in range(8):
      if (crc & 0x80) != 0:
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc = (crc << 1) & 0xFF
  return crc


@pytest.mark.parametrize("gas", [0.0, 0.0005, 18. / 255., 0.2, 0.7232126984126984, 1.0])
@pytest.mark.parametrize("idx", [0, 1, 2, 3])
def test_pedal_frame_is_starpilots(runtime, gas, idx):
  from opendbc.car import create_gas_interceptor_command
  packer = runtime.CANPacker(DBC[BOLT][Bus.pt])
  values = {"ENABLE": gas > 0.001, "COUNTER_PEDAL": idx & 0xF}
  if gas > 0.001:
    values.update(GAS_COMMAND=gas * 255., GAS_COMMAND2=gas * 255.)
  dat = bytearray(packer.make_can_msg("GAS_COMMAND", 0, values)[1])
  dat[5] = starpilot_pedal_checksum(dat)
  assert create_gas_interceptor_command(packer, gas, idx) == (0x200, bytes(dat), 0)


@pytest.mark.parametrize("press,prndl2,paddle", [(True, "0c0c000500020100", "20000000000000"),
                                                 (False, "0c0c000600000100", "00000000000000")])
def test_prndl2_and_paddle_bytes_are_starpilots(runtime, press, prndl2, paddle):
  packer = runtime.CANPacker(DBC[BOLT][Bus.pt])
  cp = NS(carFingerprint=BOLT)
  # StarPilot: Byte0=Byte1=0x0C, PRNDL2 5/6 (27|4), ManualMode 1/0 (41|1), TransmissionState 1 (48|4 Intel)
  assert gmcan.create_prndl2_command(packer, 0, press, cp) == (0x1F5, bytes.fromhex(prndl2), 0)
  addr, dat, bus = gmcan.create_regen_paddle_press_command(packer, 0, press)
  assert (addr, bytes(dat), bus) == (0xBD, bytes.fromhex(paddle), 0)


# ---------------------------------------------------------------------------------------------------------------------
# LongControl: carrot's, with the Carrot Web settings (user request 2026-11-17)


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


@pytest.mark.parametrize('stopping_accel,floor', [(-50, -0.5), (-100, -1.0)])
def test_longcontrol_stop_ramps_to_the_stopping_accel_setting(runtime, monkeypatch, stopping_accel, floor):
  loc, cp = bolt_longcontrol(runtime, monkeypatch, memory_params(StoppingAccel=stopping_accel))
  out = [lc_step(runtime, loc, cp, -0.4, 0.0, 0.0, stop=True) for _ in range(250)]
  assert loc.long_control_state == LongCtrlState.stopping
  # stoppingDecelRate 0.8 m/s^3 until just past the setting, then held
  assert out[30] == pytest.approx(-0.008 * 31)
  assert floor - 0.008 < out[-1] <= floor and out[-1] == out[-2]


def test_pitch_compensation_follows_long_pitch(runtime, monkeypatch):
  """15 m/s, accel -1.0, pitch -0.05 rad: StarPilot (LongPitch on by default there) sends 0x315 af9f506100 (Codex review
  2026-11-17); carrot's LongPitch setting decides it here, off by default."""
  sent = {}
  for long_pitch in (0, 1):
    drive = Drive(runtime, monkeypatch, pitch=long_pitch)
    _, sends = drive.step(accel=-1.0, v=15.0, pitch=-0.05)
    sent[long_pitch] = bytes(by_addr(sends, 0x315)[0][0])
  assert sent[1].hex() == "af9f506100"
  assert friction(sent[0])["brake"] == 0


def test_longcontrol_starts_despite_the_stock_standstill(runtime, monkeypatch):
  """The stock standstill stays set for this car, so carrot's LongControl must still leave stopping to start."""
  loc, cp = bolt_longcontrol(runtime, monkeypatch)
  lc_step(runtime, loc, cp, -0.5, 0.0, 0.0, stop=True, standstill=True)
  assert loc.long_control_state == LongCtrlState.stopping
  assert lc_step(runtime, loc, cp, 0.5, 0.0, 0.0, standstill=True) == pytest.approx(0.54)  # startAccel 0.55 under the 0.54 ceiling at 0 m/s
  assert loc.long_control_state == LongCtrlState.starting


def test_other_cars_still_wait_on_cruise_standstill(runtime, monkeypatch):
  from openpilot.selfdrive.controls.lib import longcontrol
  monkeypatch.setattr(longcontrol, "Params", MemoryParams)
  for key in LONGCONTROL_BASELINE_CASES:
    loc = longcontrol.LongControl(bolt_cp(runtime, *key))
    cp = loc.CP
    lc_step(runtime, loc, cp, -0.5, 0.0, 0.0, stop=True, standstill=True)
    lc_step(runtime, loc, cp, 0.5, 0.0, 0.0, standstill=True)
    assert loc.long_control_state == LongCtrlState.stopping, key


# ---------------------------------------------------------------------------------------------------------------------
# Codex review 2026-11-18 fixes


def step_25hz(drive, **kwargs):
  while True:
    output, sends = drive.step(**kwargs)
    if by_addr(sends, 0x200):
      return output, sends


def test_reengage_does_not_reuse_the_old_pedal(runtime, monkeypatch):
  """Hard acceleration, a long disengage while slowing down, then SET at 0.3 m/s: the first pedal is the fresh one
  (Codex reproduced 0.987 against 0.2105 before the fix)."""
  drive = Drive(runtime, monkeypatch)
  for _ in range(100):
    drive.step(accel=1.5, v=20.0)
  assert drive.controller.bolt_acc_pedal.pedal_steady > 0.9
  for i in range(400):
    drive.step(enabled=False, v=max(0.3, 20.0 - i * 0.05))
  step_25hz(drive, accel=0.55, state="starting", v=0.3)
  fresh = Drive(runtime, monkeypatch)
  step_25hz(fresh, accel=0.55, state="starting", v=0.3)
  assert drive.controller.bolt_acc_pedal.pedal_steady == pytest.approx(fresh.controller.bolt_acc_pedal.pedal_steady)
  assert drive.controller.bolt_acc_pedal.pedal_steady <= 0.235  # 0.3 m/s pedal limit


def test_inactive_frame_between_25hz_sends_resets_the_pedal(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch)
  for _ in range(40):
    drive.step(accel=1.0, v=10.0)
  while drive.frame % 4 != 1:
    drive.step(accel=1.0, v=10.0)
  drive.step(long_active=False, v=10.0)  # one inactive 100 Hz frame, not a 25 Hz one
  assert not drive.controller.bolt_acc_pedal.pedal_active_last
  assert drive.controller.bolt_acc_pedal.pedal_steady == 0.0


def test_pedal_never_exceeds_the_speed_limit(runtime, monkeypatch):
  drive = Drive(runtime, monkeypatch)
  limits = []
  for i in range(600):
    v = max(0.0, 15.0 - i * 0.04)
    _, sends = drive.step(accel=1.5 if i < 100 else 0.2, v=v)
    if by_addr(sends, 0x200):
      cap = float(np.interp(v, [0.0, 1.0, 2.5, 4.5, 6.0, 8.0, 12.0], [0.20, 0.235, 0.29, 0.365, 0.52, 0.78, 1.0]))
      limits.append(drive.controller.bolt_acc_pedal.pedal_steady <= cap + 1e-6)  # vEgo is float32 in CarState
  assert limits and all(limits)


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


@pytest.mark.parametrize('raw1,raw2', [(793, 397), (794, 397), (795, 397), (796, 397), (790, 401), (792, 400),
                                       (800, 392), (1190, 1), (1192, 0), (700, 492), (700, 490)])
def test_gas_pressed_matches_the_panda_raw_average(runtime, raw1, raw2):
  """The host decides gasPressed with the panda's own (track1 + track2) // 2 > 595 on the same frame (Codex review
  2026-11-18: a scaled 23 left a band where only the host saw a press, so their disengage edges differed)."""
  assert gas_pressed_for(runtime, raw1, raw2) == ((raw1 + raw2) // 2 > 595)
