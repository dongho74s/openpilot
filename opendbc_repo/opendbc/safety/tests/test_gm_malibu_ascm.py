"""Panda path of the Malibu with the comma between the ASCM and the powertrain bus (CHEVROLET_MALIBU_ASCM, HW_CAM +
HW_ASCM_INT, StarPilot's ASCM_INT). Forwarded and transmitted like a camera harness with the stock ACC; only the brake
source differs (ECMAcceleratorPos 0xBE, as on the ASCM path). Compiles the production C hooks; no USB, CAN device or
vehicle I/O."""
import ctypes
import os
from pathlib import Path
import subprocess

import pytest

from opendbc.car import structs
from opendbc.car.gm.values import GMSafetyFlags

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
