"""Panda path of the Malibu with the comma between the ASCM and the powertrain bus (CHEVROLET_MALIBU_ASCM, HW_CAM +
HW_ASCM_INT, StarPilot's ASCM_INT). Forwarded and transmitted like a camera harness with the stock ACC; only the brake
source differs (ECMAcceleratorPos 0xBE, as on the ASCM path). Compiles the production C hooks; no USB, CAN device or
vehicle I/O."""
import ctypes
import os
from pathlib import Path
import subprocess
import time

import pytest

from opendbc.car import structs
from opendbc.car.gm.values import GMSafetyFlags
from opendbc.car.gm.tests import test_malibu_ascm as car
from opendbc.car.gm.tests.test_display_speed import runtime  # noqa: F401

GM = structs.CarParams.SafetyModel.gm
ASCM_INT = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_ASCM_INT)
# the same car without ECMAcceleratorPos (0xBE)
ASCM_INT_C9 = ASCM_INT | int(GMSafetyFlags.ASCM_INT_C9_BRAKE)
CAMERA = int(GMSafetyFlags.HW_CAM)
ASCM = 0
CANCEL = 6
UNPRESS = 1


@pytest.fixture(scope='module')
def native(tmp_path_factory):
  root = Path(__file__).resolve().parents[1]
  output = tmp_path_factory.mktemp('gm-malibu-ascm') / 'libsafety.so'
  subprocess.run([os.environ.get('CC', 'cc'), '-shared', '-fPIC', '-O2', '-Wall', '-Werror',
                  '-Wno-pointer-to-int-cast', '-fno-builtin', '-std=gnu11',
                  '-I', str(root), '-I', str(root / 'board'),
                  str(root / 'tests/libsafety/gm_sdgm.c'), '-o', str(output)], check=True, capture_output=True)
  lib = ctypes.CDLL(str(output))
  lib.set_safety_hooks.argtypes = [ctypes.c_uint16, ctypes.c_uint16]
  lib.set_safety_hooks.restype = ctypes.c_int
  lib.set_controls_allowed.argtypes = [ctypes.c_bool]
  lib.get_controls_allowed.restype = ctypes.c_bool
  lib.get_brake_pressed_prev.restype = ctypes.c_bool
  lib.safety_config_valid.restype = ctypes.c_bool
  lib.get_cruise_engaged_prev.restype = ctypes.c_bool
  lib.set_cruise_engaged_prev.argtypes = [ctypes.c_bool]
  lib.set_timer.argtypes = [ctypes.c_uint32]
  lib.set_torque_driver.argtypes = [ctypes.c_int, ctypes.c_int]
  lib.gm_test_frame.argtypes = [ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint]
  lib.gm_test_frame.restype = ctypes.c_int
  return lib


def use(lib, param):
  assert lib.set_safety_hooks(GM, param) == 0
  lib.init_tests()
  lib.set_timer(1_000_000)
  return lib


def rx(lib, address, data, bus=0):
  return lib.gm_test_frame(0, address, bus, data, len(data))


def tx(lib, address, data, bus=0):
  return bool(lib.gm_test_frame(1, address, bus, data, len(data)))


def fwd(lib, address, bus, length=8):
  return lib.gm_test_frame(2, address, bus, bytes(length), length)


def accelerator_pos(brake_pos):  # ECMAcceleratorPos 0xBE (6 bytes on the Malibu): BrakePedalPos is byte 1
  return bytes([0, brake_pos, 0, 0, 0, 0])


def engine_status(brake_bit):  # ECMEngineStatus 0xC9: BrakePressed is bit 40 (byte 5, bit 0)
  return bytes([0, 0, 0, 0, 0, 1 if brake_bit else 0, 0, 0])


def pedal2(cruise_state):  # AcceleratorPedal2 0x1C4: CruiseState in the top 3 bits of byte 1
  return bytes([0, cruise_state << 5, 0, 0, 0, 0, 0, 0])


def button(value):
  return bytes([0, 0, 0, 1, 0, value << 4, 0])


def brake_state(lib, brake_pos, brake_bit):
  """Engaged, then one 0xBE and one 0xC9 frame in each order; returns (brake seen, controls kept) for both orders."""
  results = []
  for first, second in (((0xBE, accelerator_pos(brake_pos)), (0xC9, engine_status(brake_bit))),
                        ((0xC9, engine_status(brake_bit)), (0xBE, accelerator_pos(brake_pos)))):
    lib.init_tests()
    # init_tests leaves the brake globals alone: settle both sources released first
    rx(lib, 0xBE, accelerator_pos(0))
    rx(lib, 0xC9, engine_status(0))
    rx(lib, 0xBE, accelerator_pos(0))
    assert not lib.get_brake_pressed_prev()
    lib.set_controls_allowed(True)
    rx(lib, *first)
    rx(lib, *second)
    results.append((bool(lib.get_brake_pressed_prev()), bool(lib.get_controls_allowed())))
  return results


@pytest.mark.parametrize('brake_pos,brake_bit,pressed', [(10, 0, True), (9, 1, False), (0, 1, False), (0, 0, False),
                                                        (255, 0, True)])
def test_ascm_intercept_reads_the_brake_from_0xbe_only(native, brake_pos, brake_bit, pressed):
  use(native, ASCM_INT)
  assert brake_state(native, brake_pos, brake_bit) == [(pressed, not pressed)] * 2


@pytest.mark.parametrize('param', [CAMERA, ASCM_INT_C9, int(GMSafetyFlags.HW_CAM | GMSafetyFlags.ASCM_INT_C9_BRAKE)])
@pytest.mark.parametrize('brake_pos,brake_bit,pressed', [(10, 0, False), (0, 1, True), (255, 1, True)])
def test_camera_harness_and_the_no_0xbe_variant_read_the_brake_from_0xc9(native, param, brake_pos, brake_bit, pressed):
  use(native, param)
  assert brake_state(native, brake_pos, brake_bit) == [(pressed, not pressed)] * 2


RX_FRAMES = {0x184: 8, 0x34A: 5, 0x1E1: 7, 0x1C4: 8, 0xC9: 8, 0xBE: 6, 0xF1: 6}


@pytest.mark.parametrize('param,missing,valid', [
  (ASCM_INT, None, True),
  (ASCM_INT, 0xBE, False),     # the brake source is checked for freshness
  (ASCM_INT, 0xF1, True),
  (ASCM_INT_C9, 0xBE, True),   # no 0xBE on this car: 0xF1 takes its place
  (ASCM_INT_C9, 0xF1, False),
  (ASCM_INT_C9, 0xC9, False),
  (CAMERA, 0xBE, False),       # the camera harness keeps its RX checks
  (CAMERA, 0xF1, True),
])
def test_rx_checks_follow_the_brake_source(native, param, missing, valid):
  use(native, param)
  for address, length in RX_FRAMES.items():
    if address != missing:
      rx(native, address, bytes(length))
  assert native.safety_config_valid() == valid


@pytest.mark.parametrize('param', [ASCM, int(GMSafetyFlags.HW_ASCM_INT)])
def test_ascm_layout_is_unchanged_with_or_without_the_bit(native, param):
  use(native, param)
  assert brake_state(native, 10, 0) == [(True, False)] * 2
  assert brake_state(native, 9, 1) == [(False, True)] * 2
  # no forwarding on the ASCM layout
  assert fwd(native, 0x370, 2) == -1 and fwd(native, 0x1C4, 0) == -1


@pytest.mark.parametrize('param', [ASCM_INT, CAMERA])
@pytest.mark.parametrize('address,bus,expected', [
  (0x180, 2, -1),   # the ASCM's LKAS command is replaced by openpilot's
  (0x370, 2, 0),    # the ASCM's ACC dashboard, gas/regen, friction brake and keepalive reach the car
  (0x2CB, 2, 0),
  (0x315, 2, 0),
  (0x409, 2, 0),
  (0x184, 0, -1),   # PSCMStatus toward the ASCM comes from openpilot
  (0x1E1, 0, 2),
  (0x1C4, 0, 2),
  (0x2FF, 0, 2),    # the SASCM's frame passes through
])
def test_forwarding_is_the_camera_harnesss(native, param, address, bus, expected):
  use(native, param)
  assert fwd(native, address, bus) == expected


def test_engages_on_the_stock_acc_and_drops_on_its_release(native):
  use(native, ASCM_INT)
  rx(native, 0x1C4, pedal2(0))
  assert not native.get_controls_allowed()
  rx(native, 0x1C4, pedal2(1))
  assert native.get_controls_allowed()
  rx(native, 0x1C4, pedal2(0))
  assert not native.get_controls_allowed()


@pytest.mark.parametrize('param', [ASCM_INT, ASCM_INT_C9])
def test_transmits_only_steering_pscm_and_cancel(native, param):
  use(native, param)
  native.set_controls_allowed(True)
  native.set_cruise_engaged_prev(True)
  assert tx(native, 0x180, bytes(4))
  assert tx(native, 0x184, bytes(8), bus=2)
  assert tx(native, 0x1E1, button(CANCEL), bus=2)
  # stock longitudinal: no resume or set spam, no buttons on the car side, no pedal, no ASCM replacement frames
  assert not tx(native, 0x1E1, button(UNPRESS), bus=2)
  assert not tx(native, 0x1E1, button(CANCEL), bus=0)
  assert not tx(native, 0x200, bytes([0x10, 0x10, 0, 0, 0, 0]))
  for address, length, bus in ((0x2CB, 8, 0), (0x315, 5, 0), (0x315, 5, 2), (0x370, 6, 0), (0x409, 7, 0), (0xA1, 7, 1)):
    assert not tx(native, address, bytes(length), bus=bus)


@pytest.mark.parametrize('param', [ASCM_INT, ASCM_INT_C9])
def test_a_rejected_gas_regen_frame_does_not_grant_controls(native, param):
  use(native, param)
  native.set_controls_allowed(False)
  assert not tx(native, 0x2CB, bytes([1, 0, 0, 0, 0, 0, 0, 0]))
  assert not native.get_controls_allowed()


def test_the_camera_harness_keeps_its_transmit_list(native):
  use(native, CAMERA)
  native.set_controls_allowed(True)
  native.set_cruise_engaged_prev(True)
  assert tx(native, 0x1E1, button(CANCEL), bus=0)
  assert tx(native, 0x200, bytes(6))


def test_stock_lkas_on_the_car_side_is_a_relay_malfunction(native):
  use(native, ASCM_INT)
  native.set_controls_allowed(True)
  rx(native, 0x180, bytes(4))
  assert not tx(native, 0x180, bytes(4))


def lkas(torque, request=True):
  raw = torque & 0x7FF
  return bytes([(raw >> 8) | (0x08 if request else 0), raw & 0xFF, 0, 0])


def steer(lib, frames, start_us=1_000_000, step_us=30_000):
  """Sends (torque, request) frames 30 ms apart from a fresh state; returns which were accepted."""
  lib.set_timer(start_us)
  lib.set_torque_driver(0, 0)
  accepted = []
  for i, (torque, request) in enumerate(frames):
    lib.set_timer(start_us + i * step_us)
    accepted.append(tx(lib, 0x180, lkas(torque, request)))
  return accepted


VIOLATIONS = {
  "over the 300 limit": [(0, True)] + [(min(10 * i, 300), True) for i in range(1, 31)] + [(301, True)],
  "a jump from 0 to 100": [(0, True), (100, True)],
  "a rise of 11": [(0, True), (10, True), (21, True)],
  "torque without the request bit": [(0, True), (10, False)],
}


# What the Traverse runs with openpilot longitudinal (interface.py); any other GM layout gives the same steering checks
TRAVERSE = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.HW_SDGM | GMSafetyFlags.MANUAL_AUTO_HOLD)


@pytest.mark.parametrize('param', [ASCM_INT, ASCM_INT_C9])
@pytest.mark.parametrize('case', sorted(VIOLATIONS))
def test_steering_checks_are_the_traverses_and_other_gm_cars(native, param, case):
  """Steering stays carrot's GM policy (user request 2026-11-19): the panda does not drop LKA frames that fail the
  torque checks, exactly as on the camera harness and the Traverse."""
  results = {}
  for p in (param, CAMERA, TRAVERSE):
    use(native, p)
    native.set_controls_allowed(True)
    results[p] = steer(native, VIOLATIONS[case])
  assert results[param] == results[CAMERA] == results[TRAVERSE]
  assert all(results[param])


def test_ascm_intercept_keeps_always_on_lateral(native):
  use(native, ASCM_INT)
  native.set_controls_allowed(False)
  assert all(steer(native, [(0, True), (10, True), (20, True), (10, True), (0, True)]))


MODE_BITS = [int(GMSafetyFlags.HW_CAM_LONG), int(GMSafetyFlags.CC_LONG), int(GMSafetyFlags.NO_CAMERA),
             int(GMSafetyFlags.HW_ASCM_LONG), int(GMSafetyFlags.NO_ACC), int(GMSafetyFlags.PEDAL_LONG),
             int(GMSafetyFlags.GAS_INTERCEPTOR), int(GMSafetyFlags.EV), int(GMSafetyFlags.HW_SDGM),
             int(GMSafetyFlags.MANUAL_AUTO_HOLD), int(GMSafetyFlags.BOLT_ACC_PEDAL)]
NEW_BITS = int(GMSafetyFlags.HW_ASCM_INT | GMSafetyFlags.ASCM_INT_C9_BRAKE)


def behavior(lib, param):
  """What a param does: brake source, RX checks, a TX sample, forwarding sample and steering-limit enforcement."""
  use(lib, param)
  out = [tuple(brake_state(lib, pos, bit)) for pos, bit in ((10, 0), (0, 1))]
  for missing in (None, 0xBE, 0xF1, 0x3D1, 0xBD, 0x201):
    use(lib, param)
    for address, length in {**RX_FRAMES, 0x3D1: 8, 0xBD: 7, 0x201: 6}.items():
      if address != missing:
        rx(lib, address, bytes(length))
    out.append(lib.safety_config_valid())
  for address, length, bus, data in ((0x200, 6, 0, None), (0x1E1, 7, 0, button(CANCEL)), (0x2CB, 8, 0, None), (0x370, 6, 0, None),
                                     (0x315, 5, 2, None), (0x184, 8, 2, None), (0x1E1, 7, 2, button(CANCEL))):
    use(lib, param)
    lib.set_controls_allowed(True)
    lib.set_cruise_engaged_prev(True)
    out.append(tx(lib, address, data or bytes(length), bus=bus))
  use(lib, param)
  out.append(tuple(fwd(lib, address, bus) for address, bus in ((0x180, 2), (0x370, 2), (0x315, 2), (0x2CB, 2), (0x184, 0), (0x3D1, 0))))
  use(lib, param)
  lib.set_controls_allowed(True)
  out.append(tuple(steer(lib, VIOLATIONS["a jump from 0 to 100"])))
  return out


@pytest.mark.parametrize('extra', MODE_BITS)
@pytest.mark.parametrize('camera', [True, False])
def test_the_new_bits_are_ignored_next_to_any_other_mode_bit(native, extra, camera):
  base = extra | (CAMERA if camera else 0)
  assert behavior(native, base | NEW_BITS) == behavior(native, base)
  assert behavior(native, base | int(GMSafetyFlags.HW_ASCM_INT)) == behavior(native, base)


def lateral_drive(scenario):
  """(torque request, driver torque in Nm) for every 10 ms tick."""
  sign = -1.0 if scenario.endswith("negative") else 1.0
  ticks = []
  for tick in range(700):
    request = 0.0 if tick < 20 else 1.0 if tick < 420 else 0.3 if tick < 520 else 0.0
    driver = 0.0
    if scenario == "steady" and 300 <= tick < 420:
      request = -1.0
    elif scenario.startswith("driver opposes") and 150 <= tick < 260:
      driver = -3.0
    elif scenario.startswith("driver pulses") and 150 <= tick < 400 and (tick // 15) % 2 == 0:
      driver = -3.0
    elif scenario == "driver helps" and 150 <= tick < 260:
      driver = 3.0
    ticks.append((sign * request, sign * driver))
  return ticks


SCENARIOS = ["steady", "driver opposes", "driver pulses negative"]
CUSTOMS = [{}, {"CustomSteerMax": 999, "CustomSteerDeltaUp": 99, "CustomSteerDeltaDown": 99}]


@pytest.mark.parametrize('param', [ASCM_INT, ASCM_INT_C9])
@pytest.mark.parametrize('pscm_period', [1])
@pytest.mark.parametrize('custom', range(len(CUSTOMS)))
@pytest.mark.parametrize('scenario', SCENARIOS)
def test_the_hosts_steering_frames_reach_the_car(native, runtime, monkeypatch, param, pscm_period, custom, scenario):
  """The real CarController against the C hooks, with the driver's torque reaching both the host (PSCMStatus) and the
  panda's driver samples: every LKA frame is transmitted, as on the other GM cars."""
  from opendbc.car.gm.tests.test_display_speed import MemoryParams

  class Custom(MemoryParams):
    def __init__(self, *args):
      super().__init__(*args)
      self.values.update(CUSTOMS[custom])
  monkeypatch.setattr(runtime.controller, "Params", Custom)

  use(native, param)
  native.set_torque_driver(0, 0)
  ci = car.malibu_interface(runtime, car.malibu_cp(runtime, accel_pos=not param & int(GMSafetyFlags.ASCM_INT_C9_BRAKE)))
  assert ci.CP.safetyConfigs[0].safetyParam == param
  controller = runtime.controller.CarController(car.DBC[car.MALIBU_ASCM], ci.CP)
  packer = runtime.CANPacker(car.DBC[car.MALIBU_ASCM][car.Bus.pt])
  t0 = time.monotonic_ns()
  sent, rejected, peak = 0, [], 0
  for tick, (request, driver) in enumerate(lateral_drive(scenario)):
    now = t0 + tick * 10_000_000
    native.set_timer(1_000_000 + tick * 10_000)
    slice_ = car.frames(packer, tick, ascm_counter=2, driver_torque=driver, pscm=tick % pscm_period == 0)
    for address, dat, bus in slice_:
      if address == 0x184 and bus == 0:
        rx(native, address, bytes(dat))
    ci.update([(now, slice_)])
    cc = car.structs.CarControl(enabled=True, latActive=True)
    cc.actuators.torque = request
    output, sends = controller.update(cc.as_reader(), ci.CS, now)
    peak = max(peak, abs(controller.apply_torque_last))
    for address, dat, bus in sends:
      if address == 0x180:
        sent += 1
        if not tx(native, address, bytes(dat), bus=bus):
          rejected.append((tick, controller.apply_torque_last))
  assert sent > 200 and peak >= 100  # it really steered, also at a custom up step of 3
  assert rejected == []
