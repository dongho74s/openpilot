"""Joystick (DJI RC 2 remote control) mode steers with the fastest ramps the panda accepts; normal driving is unchanged."""
from types import SimpleNamespace as NS

import pytest

from opendbc.car import DT_CTRL, structs
from opendbc.car.gm.values import CAR, DBC, CarControllerParams
from opendbc.car.gm.tests.test_display_speed import MemoryParams, runtime, build_cp, controller_state  # noqa: F401
from opendbc.car.gm.tests.test_traverse_steering_timing import ramp_commands, violations_at_any_phase


def test_traverse_joystick_ramps_are_the_panda_maximum(runtime):
  # the controller module is loaded through the runtime fixture, which stands in for openpilot's Params on a computer
  p = CarControllerParams(NS(carFingerprint=CAR.CHEVROLET_TRAVERSE))
  fast = runtime.controller.JoystickSteerRamp(p)
  assert (fast.STEER_DELTA_UP, fast.STEER_DELTA_DOWN) == (12, 25)
  assert (fast.STEER_MAX, fast.STEER_STEP) == (p.STEER_MAX, p.STEER_STEP)
  assert (p.STEER_DELTA_UP, p.STEER_DELTA_DOWN) == (10, 15)  # the normal params are left alone
  assert ramp_commands(fast, 300) * p.STEER_STEP * DT_CTRL == pytest.approx(0.75)
  assert ramp_commands(fast, 0, start=300) * p.STEER_STEP * DT_CTRL == pytest.approx(0.36)


def trace(runtime, monkeypatch, joystick, candidate=CAR.CHEVROLET_TRAVERSE):
  monkeypatch.setattr(runtime.controller, "Params", MemoryParams)
  controller = runtime.controller.CarController(DBC[candidate], build_cp(runtime, candidate, alpha=False))
  assert controller.joystick_mode is False
  controller.joystick_mode = joystick
  cc = structs.CarControl(enabled=True, latActive=True)
  state = controller_state()
  messages, now = [], 1_000_000_000
  for tick in range(500):
    now += 10_000_000
    cc.actuators.torque = 0. if tick < 20 or tick >= 400 else (1. if tick < 200 else -1.)
    _, sends = controller.update(cc.as_reader(), state, now)
    state.loopback_lka_steering_cmd_updated = False
    for addr, data, _bus in sends:
      if addr == 0x180:
        torque = ((data[0] & 0x7) << 8) | data[1]
        messages.append(((now - 1_000_000_000) // 1000, torque - 2048 if torque & 0x400 else torque))
        state.loopback_lka_steering_cmd_ts_nanos = now
        state.loopback_lka_steering_cmd_updated = True
  return messages


def frames_to_full(messages):
  t0 = next(ts for ts, t in messages if t != 0)
  t1 = next(ts for ts, t in messages if t == 300)
  return (t1 - t0) / 1e6


def test_controller_ramps_faster_only_in_joystick_mode(runtime, monkeypatch):
  normal = trace(runtime, monkeypatch, joystick=False)
  joystick = trace(runtime, monkeypatch, joystick=True)
  assert frames_to_full(joystick) < frames_to_full(normal)

  def biggest_rise(msgs):  # while building up to +300 the first time
    up = msgs[:next(i for i, (_, t) in enumerate(msgs) if t == 300) + 1]
    return max(b[1] - a[1] for a, b in zip(up, up[1:], strict=False))

  def biggest_release(msgs):  # from -300 back toward zero at the end
    i = max(i for i, (_, t) in enumerate(msgs) if t == -300)
    return max(b[1] - a[1] for a, b in zip(msgs[i:], msgs[i + 1:], strict=False))

  assert (biggest_rise(normal), biggest_release(normal)) == (10, 15)
  assert (biggest_rise(joystick), biggest_release(joystick)) == (12, 25)


@pytest.mark.parametrize("candidate", [CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_BOLT_EUV])
def test_joystick_ramps_stay_inside_panda_limits(runtime, monkeypatch, candidate):
  messages = trace(runtime, monkeypatch, joystick=True, candidate=candidate)
  assert max(t for _, t in messages) == 300 and min(t for _, t in messages) == -300
  assert violations_at_any_phase(messages) == {}
