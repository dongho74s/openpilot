"""Traverse LKA command timing and torque ceiling. Real GM controller and limiter; no device I/O."""
from types import SimpleNamespace as NS

import pytest

from opendbc.car import DT_CTRL, apply_driver_steer_torque_limits, structs
from opendbc.car.gm.values import CAR, DBC, FROGPILOT_STEER_TIMING_CAR, CarControllerParams
from opendbc.car.gm.tests.test_display_speed import MemoryParams, runtime, build_cp, controller_state  # noqa: F401


def ramp_commands(params, target, start=0, limit=500):
  last, commands = start, 0
  while last != target and commands < limit:
    last = apply_driver_steer_torque_limits(target, last, 0.0, params)
    commands += 1
  return commands


def test_frogpilot_timing_cars_are_traverse_trailblazer_and_trax():
  assert FROGPILOT_STEER_TIMING_CAR == {CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_TRAILBLAZER_CC, CAR.CHEVROLET_TRAX}


@pytest.mark.parametrize("candidate", sorted(FROGPILOT_STEER_TIMING_CAR))
def test_traverse_trailblazer_trax_use_upstream_frogpilot_command_timing(candidate):
  p = CarControllerParams(NS(carFingerprint=candidate))
  assert (p.STEER_STEP, p.STEER_DELTA_UP, p.STEER_DELTA_DOWN, p.STEER_MAX) == (3, 10, 15, 300)


@pytest.mark.parametrize("candidate", [c for c in CAR if c not in FROGPILOT_STEER_TIMING_CAR])
def test_other_gm_cars_keep_existing_timing(candidate):
  p = CarControllerParams(NS(carFingerprint=candidate))
  assert (p.STEER_STEP, p.STEER_DELTA_UP, p.STEER_DELTA_DOWN, p.STEER_MAX) == (4, 5, 7, 300)


def test_traverse_full_torque_ramp_time():
  p = CarControllerParams(NS(carFingerprint=CAR.CHEVROLET_TRAVERSE))
  assert ramp_commands(p, 300) * p.STEER_STEP * DT_CTRL == pytest.approx(0.9)
  assert ramp_commands(p, 0, start=300) * p.STEER_STEP * DT_CTRL == pytest.approx(0.6)
  assert ramp_commands(p, -300, start=300) * p.STEER_STEP * DT_CTRL == pytest.approx(1.5)


@pytest.mark.parametrize("custom,expected", [(0, 300), (250, 250), (300, 300), (400, 300), (1000, 300)])
def test_custom_steer_max_is_capped_at_eps_limit(runtime, monkeypatch, custom, expected):
  class CustomParams(MemoryParams):
    def __init__(self, *args):
      super().__init__(*args)
      self.values.update({"CustomSteerMax": custom, "CustomSteerDeltaUp": 10, "CustomSteerDeltaDown": 15})

  monkeypatch.setattr(runtime.controller, "Params", CustomParams)
  controller = runtime.controller.CarController(DBC[CAR.CHEVROLET_TRAVERSE], build_cp(runtime))
  cc = structs.CarControl(enabled=True, latActive=True)
  cc.actuators.torque = 1.0
  state = controller_state()
  sent = []
  for tick in range(200):
    output, _ = controller.update(cc.as_reader(), state, 1_000_000_000 + tick * 10_000_000)
    sent.append(output.torqueOutputCan)
  assert controller.params.STEER_MAX == expected
  assert max(abs(x) for x in sent) == expected
  assert controller.params.STEER_STEP == 3



def test_custom_delta_ceilings_follow_the_command_rate():
  # Traverse sends every 3 frames (FrogPilot timing), Bolt EUV every 4 frames; the Bolt No-ACC is still 4 frames here,
  # so the 3-frame example is the Traverse (the WIP-FR test used the Bolt No-ACC, which moved to 3 frames there).
  traverse = CarControllerParams(NS(carFingerprint=CAR.CHEVROLET_TRAVERSE))
  bolt_euv = CarControllerParams(NS(carFingerprint=CAR.CHEVROLET_BOLT_EUV))
  assert (traverse.STEER_STEP, traverse.max_custom_steer_delta_up()) == (3, 12)
  assert (bolt_euv.STEER_STEP, bolt_euv.max_custom_steer_delta_up()) == (4, 16)
  # the validated defaults are below the ceilings
  for p in (traverse, bolt_euv):
    assert p.STEER_DELTA_UP <= p.max_custom_steer_delta_up() and p.STEER_DELTA_DOWN <= p.PANDA_MAX_RATE_DOWN


def panda_gm_torque_violations(messages, limits=CarControllerParams):
  """safety.h driver_limit_check (no driver torque) + rt_rate_limit_check with GM_STEERING_LIMITS, checked before the
  250 ms reference reset as the C code does."""
  violations, last, rt_last, rt_ts = [], 0, 0, messages[0][0] if messages else 0
  for ts_us, torque in messages:
    rate_ok = min(last, 0) - limits.PANDA_MAX_RATE_UP <= torque <= max(last, 0) + limits.PANDA_MAX_RATE_UP
    rt_ok = min(rt_last, 0) - limits.PANDA_MAX_RT_DELTA <= torque <= max(rt_last, 0) + limits.PANDA_MAX_RT_DELTA
    if not (rate_ok and rt_ok and abs(torque) <= 300):
      violations.append((ts_us, torque))
    last = torque
    if ts_us - rt_ts > limits.PANDA_RT_INTERVAL_MS * 1000:
      rt_last, rt_ts = torque, ts_us
  return violations


def gm_command_trace(runtime, monkeypatch, up, down, candidate=CAR.CHEVROLET_TRAVERSE):
  class CustomParams(MemoryParams):
    def __init__(self, *args):
      super().__init__(*args)
      self.values.update({"CustomSteerDeltaUp": up, "CustomSteerDeltaDown": down})

  monkeypatch.setattr(runtime.controller, "Params", CustomParams)
  controller = runtime.controller.CarController(DBC[candidate], build_cp(runtime, candidate, alpha=False))
  cc = structs.CarControl(enabled=True, latActive=True)
  state = controller_state()
  messages = []
  for tick in range(800):
    cc.actuators.torque = 0. if tick < 50 or tick >= 700 else (1. if tick < 300 else -1.)
    now = 1_000_000_000 + tick * 10_000_000
    _, sends = controller.update(cc.as_reader(), state, now)
    state.loopback_lka_steering_cmd_updated = False
    for addr, data, _bus in sends:
      if addr == 0x180:
        torque = ((data[0] & 0x7) << 8) | data[1]
        messages.append((tick * 10_000, torque - 2048 if torque & 0x400 else torque))
        state.loopback_lka_steering_cmd_ts_nanos = now   # the bus echoes our command
        state.loopback_lka_steering_cmd_updated = True
  return controller, messages


@pytest.mark.parametrize("up,down", [(0, 0), (10, 15), (12, 25), (15, 15), (50, 50)])
def test_traverse_custom_deltas_stay_inside_panda_limits(runtime, monkeypatch, up, down):
  controller, messages = gm_command_trace(runtime, monkeypatch, up, down)
  p = controller.params
  assert p.STEER_STEP == 3
  assert p.STEER_DELTA_UP == (min(up, 12) if up else 10)
  assert p.STEER_DELTA_DOWN == (min(down, 25) if down else 15)
  assert max(t for _, t in messages) == 300 and min(t for _, t in messages) == -300
  assert panda_gm_torque_violations(messages) == []


def test_panda_check_emulation_catches_the_uncapped_rise():
  # 15 per command every 30 ms: nine rises (135) on one reference before the 250 ms reset
  messages = [(i * 30_000, min(15 * i, 300)) for i in range(30)]
  assert panda_gm_torque_violations(messages) != []
  messages = [(i * 30_000, min(12 * i, 300)) for i in range(30)]
  assert panda_gm_torque_violations(messages) == []
