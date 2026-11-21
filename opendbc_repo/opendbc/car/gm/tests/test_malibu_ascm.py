"""Malibu with the comma between the ASCM and the powertrain bus, plus a SASCM (CHEVROLET_MALIBU_ASCM), ported from
StarPilot's ASCM_INT path (StarPilotHQ/StarPilot stable 8589b73f, CHEVROLET_MALIBU_ASCM). The ASCM keeps the stock ACC and
radar; openpilot steers. Real DBC, packer, parser, CarState and CarController; no device I/O."""
import time
from types import SimpleNamespace as NS

import pytest

from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm.fingerprints import FINGERPRINTS
from opendbc.car.gm.radar_interface import RADAR_HEADER_MSG, LAST_RADAR_MSG
from opendbc.car.gm.values import CAR, DBC, ASCM_INT_CAR, CanBus, CarControllerParams, CruiseButtons, GMFlags, GMSafetyFlags
from opendbc.car.gm.tests.test_display_speed import runtime  # noqa: F401

MALIBU_ASCM = CAR.CHEVROLET_MALIBU_ASCM
SASCM = 0x2FF
# The ASCM's own frames in the 2017 Premier fingerprint: behind this harness they are on bus 2 once the relay closes
ASCM_OUTPUTS = {0x180: 4, 0x2CB: 8, 0x2CD: 5, 0x365: 4, 0x370: 6, 0x409: 7, 0x40A: 7, 0x510: 4, 0x78A: 7}


def malibu_fingerprint(radar=False, sascm=True, accel_pos=True, pedal=False):
  """Fingerprinting runs with the relay open, so both sides of the harness see the same bus."""
  fingerprint = gen_empty_fingerprint()
  pt = dict(FINGERPRINTS[CAR.CHEVROLET_MALIBU][0])
  if sascm:
    pt[SASCM] = 8
  if not accel_pos:
    del pt[0xBE]
  if pedal:
    pt[0x201] = 6
  fingerprint[0] = dict(pt)
  fingerprint[2] = dict(pt)
  if radar:
    fingerprint[1] = {msg: 8 for msg in range(RADAR_HEADER_MSG, LAST_RADAR_MSG + 1)}
  return fingerprint


def malibu_cp(runtime, alpha=False, **kwargs):
  return runtime.interface.CarInterface.get_params(MALIBU_ASCM, malibu_fingerprint(**kwargs), [], alpha, True, False)


def test_only_the_malibu_ascm_is_an_ascm_intercept_car():
  assert ASCM_INT_CAR == {MALIBU_ASCM}


def test_chosen_from_the_car_list_and_never_fingerprinted(runtime):
  from opendbc.car.selected_car import get_selected_car_platform
  assert get_selected_car_platform("Chevrolet Malibu ASCM Harness 2017-19") == MALIBU_ASCM
  # Same CAN as the Premier: a fingerprint would make the Premier ambiguous, so the platform has none
  assert MALIBU_ASCM not in FINGERPRINTS
  assert CAR.CHEVROLET_MALIBU in FINGERPRINTS
  assert MALIBU_ASCM.config.specs == CAR.CHEVROLET_MALIBU.config.specs


@pytest.mark.parametrize("alpha,sascm", [(False, False), (True, False), (False, True)])
def test_car_params_keep_the_stock_acc_and_forward_like_a_camera(runtime, alpha, sascm):
  # The stock ACC unless a SASCM is fitted and alpha longitudinal is turned on
  cp = malibu_cp(runtime, alpha, sascm=sascm)
  assert cp.networkLocation == structs.CarParams.NetworkLocation.fwdCamera
  assert cp.safetyConfigs[0].safetyParam == int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_ASCM_INT)
  assert cp.pcmCruise
  assert not cp.openpilotLongitudinalControl
  # offered (StarPilot ASCM_INT) only with the SASCM's 0x2FF on the powertrain bus
  assert cp.alphaLongitudinalAvailable == sascm
  # The ASCM sends no AEBCmd; it still sends the LKAS frame openpilot syncs to, so this is no camera-less car
  assert not cp.flags & GMFlags.NO_CAMERA.value
  assert not cp.flags & GMFlags.NO_ACCELERATOR_POS_MSG.value
  assert cp.radarUnavailable
  assert cp.minEnableSpeed == pytest.approx(5 * CV.KPH_TO_MS)
  assert cp.minSteerSpeed == pytest.approx(7 * CV.MPH_TO_MS)
  assert cp.steerActuatorDelay == pytest.approx(0.2)
  assert cp.lateralTuning.which() == "torque"


def test_a_pedal_frame_does_not_turn_on_openpilot_longitudinal(runtime):
  # The common GM pedal branch would make it a pedal-long car; this one never takes the pedal path
  for alpha in (False, True):
    cp = malibu_cp(runtime, alpha, sascm=False, pedal=True)
    assert cp.pcmCruise and not cp.openpilotLongitudinalControl and not cp.enableGasInterceptorDEPRECATED
    assert cp.safetyConfigs[0].safetyParam == int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_ASCM_INT)
    assert not cp.flags & GMFlags.PEDAL_LONG.value
  # with a SASCM and alpha long it is the SASCM path, still without the pedal interceptor
  cp = malibu_cp(runtime, True, pedal=True)
  assert cp.openpilotLongitudinalControl and not cp.enableGasInterceptorDEPRECATED
  assert not cp.safetyConfigs[0].safetyParam & (GMSafetyFlags.GAS_INTERCEPTOR | GMSafetyFlags.HW_ASCM_LONG)
  assert not cp.flags & GMFlags.PEDAL_LONG.value


ASCM_INT_LONG = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_ASCM_INT | GMSafetyFlags.HW_CAM_LONG)


@pytest.mark.parametrize("accel_pos", [True, False])
def test_sascm_with_alpha_long_runs_openpilot_longitudinal(runtime, accel_pos):
  """User request 2026-11-21 (the friend's Malibu has a SASCM): StarPilot's ASCM_INT alpha long."""
  cp = malibu_cp(runtime, True, accel_pos=accel_pos)
  assert cp.alphaLongitudinalAvailable and cp.openpilotLongitudinalControl and not cp.pcmCruise
  c9 = 0 if accel_pos else int(GMSafetyFlags.ASCM_INT_C9_BRAKE)
  assert cp.safetyConfigs[0].safetyParam == ASCM_INT_LONG | c9
  assert cp.networkLocation == structs.CarParams.NetworkLocation.fwdCamera
  assert not cp.flags & GMFlags.NO_CAMERA.value
  # the tuning of the same car with openpilot in place of the ASCM (CHEVROLET_MALIBU)
  premier = runtime.interface.CarInterface.get_params(CAR.CHEVROLET_MALIBU, malibu_fingerprint(), [], True, True, False)
  assert list(cp.longitudinalTuning.kpV) == list(premier.longitudinalTuning.kpV) == [1.0]
  assert list(cp.longitudinalTuning.kiV) == pytest.approx(list(premier.longitudinalTuning.kiV)) and \
         list(cp.longitudinalTuning.kiV) == pytest.approx([0.3])
  assert cp.minEnableSpeed == pytest.approx(5 * CV.KPH_TO_MS)  # first engagement from 5 km/h, as StarPilot
  # steering stays exactly as in the stock-ACC mode
  stock = malibu_cp(runtime, False, accel_pos=accel_pos)
  assert cp.lateralTuning.to_dict() == stock.lateralTuning.to_dict()
  assert (cp.steerActuatorDelay, cp.minSteerSpeed) == (stock.steerActuatorDelay, stock.minSteerSpeed)


def test_gas_regen_limits_are_the_ascm_ones_the_panda_checks(runtime):
  # The ASCM sends -500 Nm idle in the same 0x2CB layout; the host and panda use the ASCM limits (-650 idle, 1018 max)
  params = CarControllerParams(malibu_cp(runtime, True))
  assert (params.MAX_GAS, params.MAX_ACC_REGEN, params.INACTIVE_REGEN, params.MAX_BRAKE) == (1018.0, -650.0, -650.0, 400)


@pytest.mark.parametrize("alpha", [True, False])
def test_the_ascms_fcw_reaches_the_car(runtime, monkeypatch, alpha):
  """The panda blocks the ASCM's 0x370 under openpilot longitudinal, so its FCWAlert is replayed on openpilot's 0x370
  (StarPilot does the same; Codex review 2026-11-21). No new openpilot alert: stockFcw stays False."""
  cp = malibu_cp(runtime, alpha, radar=True)
  ci = malibu_interface(runtime, cp)
  controller = runtime.controller.CarController(DBC[MALIBU_ASCM], ci.CP)
  packer = runtime.CANPacker(DBC[MALIBU_ASCM][Bus.pt])
  t0 = time.monotonic_ns()
  dashboard = []
  for tick in range(30):
    now = t0 + tick * 10_000_000
    slice_ = frames(packer, tick, ascm_counter=2)
    fcw = packer.make_can_msg("ASCMActiveCruiseControlStatus", 2, {"ACCSpeedSetpoint": 80.0, "FCWAlert": 3})
    slice_ = [m for m in slice_ if not (m[0] == 0x370 and m[2] == 2)] + [tuple(fcw)]
    monkeypatch.setattr("opendbc.car.gm.stock_fcw.boottime_ns", lambda now=now: now)
    ret = ci.update([(now, slice_)])
    cc = structs.CarControl(enabled=True, latActive=True, longActive=alpha)
    _, sends = controller.update(cc.as_reader(), ci.CS, now)
    dashboard += [dat for addr, dat, bus in sends if addr == 0x370 and bus == 0]
  assert not ret.stockFcw
  if alpha:
    assert ci.CS.cam_fcw_alert == 3
    # from the first frame sent after the ASCM's alert was parsed
    assert len(dashboard) > 2 and all((dat[5] & 0x3) == 3 for dat in dashboard[1:])
  else:
    assert ci.CS.stock_fcw is None and not dashboard   # stock ACC: the ASCM's own 0x370 is forwarded


def test_frames_match_the_ascms_own_layout(runtime):
  """0x2CB and 0x370 exactly as the friend's ASCM sent them (rlog 0000001d--c1e04a331d--15, 2026-11-21)."""
  from opendbc.car.gm import gmcan
  packer = runtime.CANPacker(DBC[MALIBU_ASCM][Bus.pt])
  # counter 2, ACC active, -500 Nm (the ASCM's idle) / inactive
  assert gmcan.create_gas_regen_command(packer, 0, -500, 2, True, False, MALIBU_ASCM)[1] == bytes.fromhex("8142b09000bd4f6e")
  assert gmcan.create_gas_regen_command(packer, 0, -500, 2, False, False, MALIBU_ASCM)[1] == bytes.fromhex("8042b09001bd4f6e")
  # ACC active at 77 km/h, three gap bars, without and with a lead car
  for lead, expected in ((False, "0100b4d00100"), (True, "0100b4d00110")):
    hud = NS(leadDistanceBars=3, leadVisible=lead)
    assert gmcan.create_acc_dashboard_command(packer, 0, True, 77.0, hud, False)[1] == bytes.fromhex(expected)


def test_without_0xbe_the_brake_switch_in_0xc9_is_used(runtime):
  cp = malibu_cp(runtime, accel_pos=False)
  assert cp.safetyConfigs[0].safetyParam == int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_ASCM_INT | GMSafetyFlags.ASCM_INT_C9_BRAKE)
  assert cp.flags & GMFlags.NO_ACCELERATOR_POS_MSG.value


def test_radar_read_only_when_the_whole_stock_stream_is_on_bus_1(runtime):
  assert not malibu_cp(runtime, radar=True).radarUnavailable
  partial = malibu_fingerprint(radar=True)
  del partial[1][LAST_RADAR_MSG]
  assert runtime.interface.CarInterface.get_params(MALIBU_ASCM, partial, [], False, True, False).radarUnavailable


STEER_FIELDS = ("STEER_MAX", "STEER_MAX_EPS", "STEER_STEP", "INACTIVE_STEER_STEP", "STEER_DELTA_UP", "STEER_DELTA_DOWN",
                "STEER_DRIVER_ALLOWANCE", "STEER_DRIVER_MULTIPLIER", "STEER_DRIVER_FACTOR", "PANDA_MAX_RATE_UP",
                "PANDA_MAX_RATE_DOWN", "PANDA_MAX_RT_DELTA", "PANDA_RT_INTERVAL_MS")


def steer_view(params):
  return {**{f: getattr(params, f) for f in STEER_FIELDS}, "custom_up_ceiling": params.max_custom_steer_delta_up()}


def test_steers_like_the_traverse(runtime):
  # user request 2026-11-19: same steering as the Traverse (33 Hz, +10/-15) and the other GM cars' limits
  traverse = runtime.interface.CarInterface.get_params(CAR.CHEVROLET_TRAVERSE, gen_empty_fingerprint(), [], False, True, False)
  params = CarControllerParams(malibu_cp(runtime))
  assert steer_view(params) == steer_view(CarControllerParams(traverse))
  assert (params.STEER_STEP, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN) == (3, 10, 15)


@pytest.mark.parametrize("custom", [{}, {"CustomSteerDeltaUp": 99, "CustomSteerDeltaDown": 99},
                                    {"CustomSteerDeltaUp": 3, "CustomSteerDeltaDown": 5}, {"CustomSteerMax": 250}])
def test_custom_steering_settings_act_as_on_the_traverse(runtime, monkeypatch, custom):
  from opendbc.car.gm.tests.test_display_speed import MemoryParams

  class Custom(MemoryParams):
    def __init__(self, *args):
      super().__init__(*args)
      self.values.update(custom)
  monkeypatch.setattr(runtime.controller, "Params", Custom)
  ci, _ = run(runtime, ticks=1)
  views = []
  for cp in (malibu_cp(runtime), runtime.interface.CarInterface.get_params(CAR.CHEVROLET_TRAVERSE, gen_empty_fingerprint(), [],
                                                                           False, True, False)):
    controller = runtime.controller.CarController(DBC[cp.carFingerprint], cp)
    controller.update(structs.CarControl().as_reader(), ci.CS, time.monotonic_ns())
    views.append(steer_view(controller.params))
  assert views[0] == views[1]


def test_the_other_malibus_are_unchanged(runtime):
  premier = runtime.interface.CarInterface.get_params(CAR.CHEVROLET_MALIBU, malibu_fingerprint(sascm=False), [], False, True, False)
  assert premier.networkLocation == structs.CarParams.NetworkLocation.gateway
  assert premier.safetyConfigs[0].safetyParam == 0
  assert premier.openpilotLongitudinalControl and not premier.pcmCruise and not premier.radarUnavailable
  # their layouts are unchanged; since 2026-11-19 (user request) they steer at the Traverse's timing like this car
  for params in (CarControllerParams(premier), CarControllerParams(NS(carFingerprint=CAR.CHEVROLET_MALIBU_CC))):
    assert (params.STEER_STEP, params.STEER_DELTA_UP, params.STEER_DELTA_DOWN) == (3, 10, 15)

  fingerprint = gen_empty_fingerprint()
  fingerprint[0] = dict(FINGERPRINTS[CAR.CHEVROLET_MALIBU_CC][0])
  fingerprint[2] = {0x180: 4, 0x320: 6}
  no_acc = runtime.interface.CarInterface.get_params(CAR.CHEVROLET_MALIBU_CC, fingerprint, [], False, True, False)
  assert no_acc.safetyConfigs[0].safetyParam == int(GMSafetyFlags.HW_CAM | GMSafetyFlags.CC_LONG | GMSafetyFlags.NO_ACC)
  assert not no_acc.safetyConfigs[0].safetyParam & GMSafetyFlags.HW_ASCM_INT


def frames(packer, tick, *, brake_pos=0, ecm_brake=0, cruise_state=1, ascm_lkas=True, ascm_counter=None, accel_pos=True,
           driver_torque=0.0, pscm=True):
  """One 10 ms slice of the car side (bus 0) and the ASCM side (bus 2) with the relay closed."""
  sends = {(addr, 0): (addr, bytes(length), 0) for addr, length in FINGERPRINTS[CAR.CHEVROLET_MALIBU][0].items()
           if addr not in ASCM_OUTPUTS}
  msg = packer.make_can_msg("PSCMStatus", 0, {"LKADriverAppldTrq": driver_torque})
  sends[(msg[0], msg[2])] = tuple(msg)
  if not pscm:
    del sends[(msg[0], msg[2])]
  sends[(SASCM, 0)] = (SASCM, bytes(8), 0)
  if accel_pos:
    msg = packer.make_can_msg("ECMAcceleratorPos", 0, {"BrakePedalPos": brake_pos})
    sends[(msg[0], msg[2])] = tuple(msg)
  else:
    del sends[(0xBE, 0)]
    msg = packer.make_can_msg("EBCMBrakePedalPosition", 0, {"BrakePedalPosition": brake_pos})
    sends[(msg[0], msg[2])] = tuple(msg)
  for msg in (
    packer.make_can_msg("ECMEngineStatus", 0, {"BrakePressed": ecm_brake, "CruiseMainOn": 1}),
    packer.make_can_msg("AcceleratorPedal2", 0, {"CruiseState": cruise_state}),
    packer.make_can_msg("EBCMWheelSpdFront", 0, {"FLWheelSpd": 60.0, "FRWheelSpd": 60.0}),
    packer.make_can_msg("EBCMWheelSpdRear", 0, {"RLWheelSpd": 60.0, "RRWheelSpd": 60.0}),
    packer.make_can_msg("ASCMSteeringButton", 0, {"ACCButtons": CruiseButtons.UNPRESS, "RollingCounter": tick % 4}),
    packer.make_can_msg("ASCMActiveCruiseControlStatus", 2, {"ACCSpeedSetpoint": 80.0, "ACCCruiseState": 0}),
  ):
    sends[(msg[0], msg[2])] = tuple(msg)
  if ascm_lkas:
    counter = tick % 4 if ascm_counter is None else ascm_counter
    msg = packer.make_can_msg("ASCMLKASteeringCmd", 2, {"RollingCounter": counter})
    sends[(msg[0], msg[2])] = tuple(msg)
  return list(sends.values())


def malibu_interface(runtime, cp=None):
  cp = cp or malibu_cp(runtime)
  ci = runtime.interface.CarInterface.__new__(runtime.interface.CarInterface)
  ci.CP = cp
  ci.CS = runtime.state.CarState(cp)
  ci.can_parsers = ci.CS.get_can_parsers(cp)
  ci.v_ego_cluster_seen = False
  return ci


def run(runtime, ticks=300, cp=None, **kwargs):
  ci = malibu_interface(runtime, cp)
  packer = runtime.CANPacker(DBC[MALIBU_ASCM][Bus.pt])
  ret = None
  # The parser stamps a message's registration with the real monotonic clock, so CAN times start from it
  t0 = time.monotonic_ns()
  for tick in range(ticks):
    ret = ci.update([(t0 + tick * 10_000_000, frames(packer, tick, **kwargs))])
  return ci, ret


def test_car_state_is_valid_with_the_ascm_and_sascm_on_the_bus(runtime):
  _, ret = run(runtime)
  assert ret.canValid and not ret.canTimeout
  assert ret.cruiseState.enabled
  assert ret.cruiseState.speed == pytest.approx(80.0 * CV.KPH_TO_MS)
  # The ASCM's cruise state is not the camera's: a non-2/3 value is not read as non-adaptive cruise
  assert not ret.cruiseState.nonAdaptive


@pytest.mark.parametrize("brake_pos,ecm_brake,expected", [(10, 0, True), (9, 1, False), (0, 1, False), (40, 0, True)])
def test_brake_comes_from_the_pedal_position_like_the_panda(runtime, brake_pos, ecm_brake, expected):
  _, ret = run(runtime, ticks=50, brake_pos=brake_pos, ecm_brake=ecm_brake)
  assert ret.brakePressed == expected


@pytest.mark.parametrize("brake_pos,ecm_brake,expected", [(0, 1, True), (100, 0, False), (0, 0, False)])
def test_without_0xbe_car_state_reads_the_brake_switch(runtime, brake_pos, ecm_brake, expected):
  _, ret = run(runtime, ticks=300, cp=malibu_cp(runtime, accel_pos=False), accel_pos=False, brake_pos=brake_pos,
               ecm_brake=ecm_brake)
  assert ret.canValid
  assert ret.brakePressed == expected


def test_the_ascm_lkas_frame_is_required_on_bus_2(runtime):
  _, ret = run(runtime, ticks=500, ascm_lkas=False)
  assert not ret.canValid


def controller_frames(runtime, *, lat_active=True, cancel=False, ticks=60, interface=None):
  ci = interface or malibu_interface(runtime)
  controller = runtime.controller.CarController(DBC[MALIBU_ASCM], ci.CP)
  packer = runtime.CANPacker(DBC[MALIBU_ASCM][Bus.pt])
  sent = []
  t0 = time.monotonic_ns()
  for tick in range(ticks):
    now = t0 + tick * 10_000_000
    ci.update([(now, frames(packer, tick, ascm_counter=2))])
    cc = structs.CarControl(enabled=lat_active, latActive=lat_active)
    cc.actuators.torque = 0.5 if lat_active else 0.0
    cc.cruiseControl.cancel = cancel
    _, sends = controller.update(cc.as_reader(), ci.CS, now)
    sent.extend((tick, addr, bytes(dat), bus) for addr, dat, bus in sends)
  return sent


def test_controller_only_steers_and_leaves_the_acc_to_the_ascm(runtime):
  ci = malibu_interface(runtime)
  sent = controller_frames(runtime, interface=ci)
  addresses = {(addr, bus) for _, addr, _, bus in sent}
  assert (0x180, CanBus.POWERTRAIN) in addresses
  assert (0x184, CanBus.CAMERA) in addresses  # PSCMStatus toward the ASCM, as for a camera harness
  # No ASCM replacement: no gas/regen, friction brake, ACC dashboard, keepalives or radar feed
  assert not addresses & {(0x2CB, 0), (0x315, 2), (0x315, 0), (0x370, 0), (0x409, 0), (0x40A, 0), (0xA1, 1), (0x306, 1),
                          (0x308, 1), (0x310, 1)}
  # The ASCM's LKAS frame on bus 2 is what the counter sync follows, as with a camera
  assert ci.CS.cam_lka_steering_cmd_counter == 2


def test_cancel_goes_to_the_ascm_on_bus_2(runtime):
  sent = controller_frames(runtime, lat_active=False, cancel=True)
  cancels = [(addr, bus, (dat[5] >> 4) & 0x7) for _, addr, dat, bus in sent if addr == 0x1E1]
  assert cancels and all(bus == CanBus.CAMERA and button == CruiseButtons.CANCEL for _, bus, button in cancels)
