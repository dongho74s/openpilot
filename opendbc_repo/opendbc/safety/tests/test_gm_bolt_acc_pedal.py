"""Panda path of the Bolt with factory ACC + comma pedal (CHEVROLET_BOLT_ACC_PEDAL), ported from StarPilot's
BOLT_2022_PEDAL + PANDA_PADDLE_SCHED (firestar5683/StarPilot 2589caad gm.h). Compiles the production C hooks; no USB,
CAN device or vehicle I/O."""
import ctypes
import os
from pathlib import Path
import subprocess

import pytest

from opendbc.car import structs
from opendbc.car.gm.values import GMSafetyFlags

GM = structs.CarParams.SafetyModel.gm
# What carrot gives the Bolt No-ACC with a pedal today, and the same plus the new bit
PEDAL_CC = int(GMSafetyFlags.HW_CAM | GMSafetyFlags.NO_ACC | GMSafetyFlags.PEDAL_LONG | GMSafetyFlags.GAS_INTERCEPTOR)
BOLT_ACC_PEDAL = PEDAL_CC | int(GMSafetyFlags.BOLT_ACC_PEDAL)
CANCEL = 6


@pytest.fixture(scope='module')
def native(tmp_path_factory):
  root = Path(__file__).resolve().parents[1]
  output = tmp_path_factory.mktemp('gm-bolt-acc-pedal') / 'libsafety.so'
  subprocess.run([os.environ.get('CC', 'cc'), '-shared', '-fPIC', '-O2', '-Wall', '-Werror',
                  '-Wno-pointer-to-int-cast', '-fno-builtin', '-std=gnu11',
                  '-I', str(root), '-I', str(root / 'board'),
                  str(root / 'tests/libsafety/gm_sdgm.c'), '-o', str(output)], check=True, capture_output=True)
  lib = ctypes.CDLL(str(output))
  lib.set_safety_hooks.argtypes = [ctypes.c_uint16, ctypes.c_uint16]
  lib.set_safety_hooks.restype = ctypes.c_int
  lib.set_controls_allowed.argtypes = [ctypes.c_bool]
  lib.get_controls_allowed.restype = ctypes.c_bool
  lib.set_cruise_engaged_prev.argtypes = [ctypes.c_bool]
  lib.set_timer.argtypes = [ctypes.c_uint32]
  lib.gm_test_frame.argtypes = [ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint]
  lib.gm_test_frame.restype = ctypes.c_int
  lib.gm_test_sent_rejected_count.restype = ctypes.c_int
  lib.gm_test_sent_frame.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint), ctypes.c_char_p]
  lib.gm_test_sent_frame.restype = ctypes.c_int
  return lib


def use(lib, param):
  assert lib.set_safety_hooks(GM, param) == 0
  lib.init_tests()
  lib.set_timer(1_000_000)
  lib.gm_test_sent_clear()
  return lib


@pytest.fixture
def safety(native):
  return use(native, BOLT_ACC_PEDAL)


def rx(lib, address, data, bus=0):
  return lib.gm_test_frame(0, address, bus, data, len(data))


def tx(lib, address, data, bus=0):
  return bool(lib.gm_test_frame(1, address, bus, data, len(data)))


def fwd(lib, address, bus, length=8):
  return lib.gm_test_frame(2, address, bus, bytes(length), length)


def sent(lib):
  frames = []
  address, bus, data = ctypes.c_uint(), ctypes.c_uint(), ctypes.create_string_buffer(8)
  while (length := lib.gm_test_sent_frame(len(frames), ctypes.byref(address), ctypes.byref(bus), data)) >= 0:
    frames.append((address.value, bus.value, data.raw[:length]))
  return frames


def button(value):
  return bytes([0, 0, 0, 1, 0, value << 4, 0])


def brake(amount):
  raw = (0x1000 - amount) & 0xfff
  return bytes([0xa0 | (raw >> 8), raw & 0xff, 0, 0, 0])


PADDLE_PRESS = bytes([0x20, 0, 0, 0, 0, 0, 0])
PADDLE_RELEASE = bytes(7)
STOCK_PADDLE = bytes(7)  # the driver is not touching the paddle


def prndl2(value):
  return bytes([0, 0, 0, value, 0, 0, 0, 0])


def path_active(lib):
  lib.set_controls_allowed(True)
  return tx(lib, 0x315, brake(100))


@pytest.mark.parametrize('param,active', [
  (BOLT_ACC_PEDAL, True),
  (PEDAL_CC, False),                                                     # the Bolt No-ACC with pedal keeps its path
  (BOLT_ACC_PEDAL & ~int(GMSafetyFlags.PEDAL_LONG), False),
  (BOLT_ACC_PEDAL & ~int(GMSafetyFlags.GAS_INTERCEPTOR), False),
  (BOLT_ACC_PEDAL & ~int(GMSafetyFlags.NO_ACC), False),                  # an ACC car
  (BOLT_ACC_PEDAL & ~int(GMSafetyFlags.HW_CAM), False),
  (BOLT_ACC_PEDAL | int(GMSafetyFlags.CC_LONG), False),
  (BOLT_ACC_PEDAL | int(GMSafetyFlags.HW_SDGM), False),
  (BOLT_ACC_PEDAL | int(GMSafetyFlags.HW_ASCM_LONG), False),
])
def test_path_needs_every_bit(native, param, active):
  assert path_active(use(native, param)) == active


def test_allowlist(safety):
  safety.set_controls_allowed(True)
  allowed = [(0x180, 0, bytes(4)), (0x200, 0, bytes(6)), (0x1E1, 0, button(CANCEL)), (0x1E1, 2, button(CANCEL)),
             (0x370, 0, bytes(6)), (0x315, 0, brake(100)), (0x184, 2, bytes(8))]
  allowed_pairs = {(address, bus) for address, bus, _ in allowed}
  for address, bus, data in allowed:
    assert tx(safety, address, data, bus), hex(address)
    for other in {0, 1, 2, 3} - {b for a, b in allowed_pairs if a == address}:
      assert not tx(safety, address, data, other), (hex(address), other)
  # StarPilot sends none of these for this car
  for address, length in ((0x2CB, 8), (0x2CD, 5), (0x3D1, 8), (0x409, 7), (0x40A, 7), (0x315, 5)):
    bus = 2 if address == 0x315 else 0
    assert not tx(safety, address, bytes(length), bus), hex(address)


@pytest.mark.parametrize('controls', [False, True])
@pytest.mark.parametrize('cruise', [False, True])
@pytest.mark.parametrize('bus', [0, 2])
def test_only_cancel_is_sent_and_always(safety, controls, cruise, bus):
  safety.set_controls_allowed(controls)
  safety.set_cruise_engaged_prev(cruise)
  for value in range(8):
    assert tx(safety, 0x1E1, button(value), bus) == (value == CANCEL), value


def test_gas_regen_cannot_grant_controls(safety):
  safety.set_controls_allowed(False)
  assert not tx(safety, 0x2CB, bytes([1, 0, 0, 0, 0, 0, 0, 0]))
  assert not safety.get_controls_allowed()


@pytest.mark.parametrize('controls', [False, True])
def test_friction_brake_limits(safety, controls):
  safety.set_controls_allowed(controls)
  for amount in range(4096):
    assert tx(safety, 0x315, brake(amount)) == (amount == 0 or (controls and amount <= 400)), amount


def test_host_paddle_frames_are_never_sent_directly(safety):
  for controls in (False, True):
    safety.set_controls_allowed(controls)
    for data in (PADDLE_PRESS, PADDLE_RELEASE):
      assert not tx(safety, 0xBD, data)
    for value in range(16):
      assert not tx(safety, 0x1F5, prndl2(value))


def test_paddle_follows_each_stock_frame(safety):
  safety.set_controls_allowed(True)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert sent(safety) == []                      # nothing fed yet
  tx(safety, 0xBD, PADDLE_PRESS)
  assert sent(safety) == []                      # buffered, not sent on the host's timing
  for count in range(1, 4):
    rx(safety, 0xBD, STOCK_PADDLE)
    assert sent(safety) == [(0xBD, 0, PADDLE_PRESS)] * count
  tx(safety, 0xBD, PADDLE_RELEASE)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert sent(safety)[-1] == (0xBD, 0, PADDLE_RELEASE)


def test_stale_apply_is_rejected_once_controls_are_off(safety):
  safety.set_controls_allowed(True)
  tx(safety, 0xBD, PADDLE_PRESS)
  safety.set_controls_allowed(False)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert sent(safety) == []
  assert safety.gm_test_sent_rejected_count() == 1


def test_apply_is_not_buffered_without_controls_but_release_is(safety):
  safety.set_controls_allowed(False)
  tx(safety, 0xBD, PADDLE_PRESS)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert sent(safety) == []
  tx(safety, 0xBD, PADDLE_RELEASE)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert sent(safety) == [(0xBD, 0, PADDLE_RELEASE)]


def test_feed_older_than_100ms_is_not_sent(safety):
  safety.set_controls_allowed(True)
  safety.set_timer(1_000_000)
  tx(safety, 0xBD, PADDLE_PRESS)
  safety.set_timer(1_100_000)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert len(sent(safety)) == 1
  safety.set_timer(1_100_001)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert len(sent(safety)) == 1


@pytest.mark.parametrize('bus,length', [(1, 7), (2, 7), (0, 6), (0, 8)])
def test_feed_on_wrong_bus_or_length_is_ignored(safety, bus, length):
  safety.set_controls_allowed(True)
  assert not tx(safety, 0xBD, bytes([0x20]) + bytes(length - 1), bus)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert sent(safety) == []


@pytest.mark.parametrize('bus,length', [(2, 7), (0, 6), (0, 8)])
def test_only_a_stock_powertrain_frame_triggers(safety, bus, length):
  safety.set_controls_allowed(True)
  tx(safety, 0xBD, PADDLE_PRESS)
  rx(safety, 0xBD, bytes(length), bus)
  assert sent(safety) == []


@pytest.mark.parametrize('value', range(16))
def test_prndl2_follows_stock_and_needs_controls_to_apply(safety, value):
  apply = value in (5, 7)
  for controls in (False, True):
    use(safety, BOLT_ACC_PEDAL)
    safety.set_controls_allowed(controls)
    tx(safety, 0x1F5, prndl2(value))
    rx(safety, 0x1F5, prndl2(0))
    assert sent(safety) == ([] if apply and not controls else [(0x1F5, 0, prndl2(value))]), (value, controls)


def test_driver_paddle_still_disengages(safety):
  safety.set_controls_allowed(True)
  rx(safety, 0xBD, PADDLE_PRESS)
  assert not safety.get_controls_allowed()


@pytest.mark.parametrize('address,bus,destination', [
  (0x3D1, 0, -1), (0x184, 0, -1), (0x1C4, 0, 2), (0xBD, 0, 2), (0x1E1, 0, 2),
  (0x180, 2, -1), (0x370, 2, -1), (0x315, 2, -1), (0x2CB, 2, 0), (0x2CD, 2, 0), (0x320, 2, 0), (0x1E1, 2, 0),
])
def test_forwarding(safety, address, bus, destination):
  assert fwd(safety, address, bus) == destination


@pytest.mark.parametrize('address,bus,destination', [(0x3D1, 0, 2), (0x370, 2, 0), (0x315, 2, 0)])
def test_bolt_no_acc_pedal_forwarding_unchanged(native, address, bus, destination):
  assert fwd(use(native, PEDAL_CC), address, bus) == destination


def test_bolt_no_acc_pedal_tx_unchanged(native):
  lib = use(native, PEDAL_CC)
  lib.set_controls_allowed(True)
  assert not tx(lib, 0xBD, PADDLE_PRESS)
  assert not tx(lib, 0x315, brake(0))
  assert not tx(lib, 0x370, bytes(6))
  tx(lib, 0xBD, PADDLE_PRESS)
  rx(lib, 0xBD, STOCK_PADDLE)
  assert sent(lib) == []
  lib.set_cruise_engaged_prev(False)
  assert tx(lib, 0x1E1, button(3))           # carrot lets SET through on this path
  assert not tx(lib, 0x1E1, button(CANCEL))  # and CANCEL only with the 0x3D1 cruise bit
  lib.set_cruise_engaged_prev(True)
  assert tx(lib, 0x1E1, button(CANCEL))


def interceptor(average):
  # both tracks equal, so the panda's (track1 + track2) / 2 is `average`
  return bytes([average >> 8, average & 0xff, average >> 8, average & 0xff, 0, 0])


@pytest.mark.parametrize('param,threshold', [(BOLT_ACC_PEDAL, 595), (PEDAL_CC, 550)])
def test_gas_interceptor_threshold(native, param, threshold):
  """StarPilot's 595 for this car (its carstate threshold 23 sits just below); the Bolt No-ACC keeps carrot's 550."""
  lib = use(native, param)
  lib.get_gas_pressed_prev.restype = ctypes.c_bool
  for average, pressed in ((threshold, False), (threshold + 1, True)):
    rx(lib, 0x201, interceptor(average))
    assert lib.get_gas_pressed_prev() == pressed, average


def build(tmp_path, release):
  root = Path(__file__).resolve().parents[1]
  include = []
  if release:
    # Release panda builds have no ALLOW_DEBUG (panda/SConscript); the test board header defines it, so shadow it.
    shadow = tmp_path / 'shadow'
    shadow.mkdir()
    header = (root / 'board/fake_stm.h').read_text()
    assert '#define ALLOW_DEBUG\n' in header
    (shadow / 'fake_stm.h').write_text(header.replace('#define ALLOW_DEBUG\n', ''))
    include = ['-I', str(shadow)]
  output = tmp_path / ('release.so' if release else 'debug.so')
  subprocess.run([os.environ.get('CC', 'cc'), '-shared', '-fPIC', '-O2', '-Wall', '-Werror',
                  '-Wno-pointer-to-int-cast', '-fno-builtin', '-std=gnu11', *include,
                  '-I', str(root), '-I', str(root / 'board'),
                  str(root / 'tests/libsafety/gm_sdgm.c'), '-o', str(output)], check=True, capture_output=True)
  lib = ctypes.CDLL(str(output))
  lib.set_safety_hooks.argtypes = [ctypes.c_uint16, ctypes.c_uint16]
  lib.set_safety_hooks.restype = ctypes.c_int
  lib.gm_test_frame.argtypes = [ctypes.c_int, ctypes.c_uint, ctypes.c_uint, ctypes.c_char_p, ctypes.c_uint]
  lib.gm_test_frame.restype = ctypes.c_int
  return lib


@pytest.mark.parametrize('release', [False, True])
def test_path_isolated_over_every_param_in_debug_and_release(tmp_path, release):
  """Only HW_CAM + NO_ACC + PEDAL_LONG + GAS_INTERCEPTOR + BOLT_ACC_PEDAL, without CAM_LONG (even in release builds,
  where the bit is otherwise ignored), CC_LONG, ASCM_LONG or SDGM, selects the path (seen as 0x3D1 not reaching the
  camera while the camera-harness forwarding is on)."""
  lib = build(tmp_path, release)
  forbidden = int(GMSafetyFlags.HW_CAM_LONG | GMSafetyFlags.CC_LONG | GMSafetyFlags.HW_ASCM_LONG | GMSafetyFlags.HW_SDGM)
  mismatches = []
  for param in range(1 << 16):
    assert lib.set_safety_hooks(GM, param) == 0
    camera_harness = fwd(lib, 0x1C4, 0) == 2
    active = camera_harness and fwd(lib, 0x3D1, 0) == -1
    expected = (param & BOLT_ACC_PEDAL) == BOLT_ACC_PEDAL and not (param & forbidden)
    if active != expected:
      mismatches.append(param)
  assert mismatches == []


def test_driver_paddle_is_never_covered(safety):
  """A buffered release (allowed even without controls) must not follow the driver's own paddle press (Codex review
  2026-11-18 sent 0xBD 00.. after each stock press for 100 ms before the fix), nor the PRNDL2 spoof while it is held."""
  safety.set_controls_allowed(True)
  tx(safety, 0xBD, PADDLE_RELEASE)
  tx(safety, 0x1F5, prndl2(6))
  for t in (1_000_001, 1_025_000, 1_050_000, 1_075_000, 1_100_000):
    safety.set_timer(t)
    rx(safety, 0xBD, PADDLE_PRESS)
    rx(safety, 0x1F5, prndl2(4))
    tx(safety, 0xBD, PADDLE_RELEASE)  # the host may still feed for a couple of frames
  assert sent(safety) == []
  assert not safety.get_controls_allowed()
  # released: the stock frame is followed again by a fresh feed (here a release, the same as stock)
  safety.set_timer(1_120_000)
  tx(safety, 0xBD, PADDLE_RELEASE)
  rx(safety, 0xBD, STOCK_PADDLE)
  assert sent(safety) == [(0xBD, 0, PADDLE_RELEASE)]
