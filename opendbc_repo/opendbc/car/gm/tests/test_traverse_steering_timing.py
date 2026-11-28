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


def test_frogpilot_timing_cars_are_traverse_trailblazer_trax_and_bolt_cc():
  # the Bolt ACC with pedal and the Bolt EUV share the Bolt No-ACC's steering; the Malibu ASCM harness takes StarPilot's GM
  # timing (values.py)
  assert FROGPILOT_STEER_TIMING_CAR == {CAR.CHEVROLET_TRAVERSE, CAR.CHEVROLET_TRAILBLAZER, CAR.CHEVROLET_TRAILBLAZER_CC, CAR.CHEVROLET_TRAX,
                                        CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_ACC_PEDAL, CAR.CHEVROLET_BOLT_EUV,
                                        CAR.CHEVROLET_MALIBU_ASCM, CAR.CHEVROLET_MALIBU, CAR.CHEVROLET_MALIBU_CC,
                                        CAR.CHEVROLET_VOLT_2019, CAR.CHEVROLET_VOLT_CC}


def test_bolt_euv_with_acc_takes_the_bolt_timing():
  # the Bolt EUV with the factory ACC is the Bolt ACC with pedal's car (user request 2026-11-28); it was 25 Hz +5/-7
  p = CarControllerParams(NS(carFingerprint=CAR.CHEVROLET_BOLT_EUV))
  assert (p.STEER_STEP, p.STEER_DELTA_UP, p.STEER_DELTA_DOWN) == (3, 10, 15)


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


def test_bolt_cc_controller_sends_at_33hz_with_frogpilot_steps(runtime, monkeypatch):
  """Real GM CarController on the Bolt No-ACC: a full request ramps +10 every 3 frames and stays in panda's rate limits."""
  monkeypatch.setattr(runtime.controller, "Params", MemoryParams)
  controller = runtime.controller.CarController(DBC[CAR.CHEVROLET_BOLT_CC], build_cp(runtime, CAR.CHEVROLET_BOLT_CC, alpha=False))
  assert (controller.params.STEER_STEP, controller.params.STEER_DELTA_UP, controller.params.STEER_DELTA_DOWN) == (3, 10, 15)
  cc = structs.CarControl(enabled=True, latActive=True)
  state = controller_state()
  sent = []
  for tick in range(150):
    cc.actuators.torque = 1.0 if tick < 100 else -1.0
    output, _ = controller.update(cc.as_reader(), state, 1_000_000_000 + tick * 10_000_000)
    sent.append(output.torqueOutputCan)
  changes = [b - a for a, b in zip(sent, sent[1:]) if b != a]
  assert max(sent) == 300
  assert max(changes) <= 10 and min(changes) >= -15   # within panda's +20/-25 per message
  # 0 -> 300 in 30 commands of 3 frames = 0.9 s (25 Hz +5 needed 2.4 s)
  assert sent.index(300) <= 92


def test_custom_delta_ceilings_follow_the_command_rate():
  bolt_cc = CarControllerParams(NS(carFingerprint=CAR.CHEVROLET_BOLT_CC))
  equinox = CarControllerParams(NS(carFingerprint=CAR.CHEVROLET_EQUINOX))  # a 25 Hz car (the Bolt EUV was, until 2026-11-28)
  assert (bolt_cc.STEER_STEP, bolt_cc.max_custom_steer_delta_up()) == (3, 12)
  assert (equinox.STEER_STEP, equinox.max_custom_steer_delta_up()) == (4, 16)
  # the validated defaults are below the ceilings
  for p in (bolt_cc, equinox):
    assert p.STEER_DELTA_UP <= p.max_custom_steer_delta_up() and p.STEER_DELTA_DOWN <= p.PANDA_MAX_RATE_DOWN


def panda_gm_torque_violations(messages, rt_ts_start=0, limits=CarControllerParams):
  """safety.h steer_torque_cmd_checks for GM with no driver torque: driver_limit_check, rt_rate_limit_check checked
  before the 250 ms reference reset, and a violation resetting the last torque, the reference and its time as the C
  code does. The panda's reference time starts at rt_ts_start (set_safety_hooks starts it at 0)."""
  violations, last, rt_last, rt_ts = [], 0, 0, rt_ts_start
  for ts_us, torque in messages:
    violation = not (min(last, 0) - limits.PANDA_MAX_RATE_UP <= torque <= max(last, 0) + limits.PANDA_MAX_RATE_UP)
    violation |= abs(torque) > 300
    last = torque
    violation |= not (min(rt_last, 0) - limits.PANDA_MAX_RT_DELTA <= torque <= max(rt_last, 0) + limits.PANDA_MAX_RT_DELTA)
    if ts_us - rt_ts > limits.PANDA_RT_INTERVAL_MS * 1000:
      rt_last, rt_ts = torque, ts_us
    if violation:
      violations.append((ts_us, torque))
      last, rt_last, rt_ts = 0, 0, ts_us
  return violations


def violations_at_any_phase(messages):
  # the panda timer origin is not ours: try every reference phase within one 250 ms interval
  return {phase: v for phase in range(0, 260_000, 10_000) if (v := panda_gm_torque_violations(messages, -phase))}


def command_trace(runtime, monkeypatch, up, down, candidate=CAR.CHEVROLET_BOLT_CC, stall_at=None, guard=True,
                  lead_zero=True, dt_ms=10.):
  class CustomParams(MemoryParams):
    def __init__(self, *args):
      super().__init__(*args)
      self.values.update({"CustomSteerDeltaUp": up, "CustomSteerDeltaDown": down})

  monkeypatch.setattr(runtime.controller, "Params", CustomParams)
  controller = runtime.controller.CarController(DBC[candidate], build_cp(runtime, candidate, alpha=False))
  if not guard:
    monkeypatch.setattr(controller.panda_rt_guard, "clip", lambda torque, now: torque)
  cc = structs.CarControl(enabled=True, latActive=True)
  state = controller_state()
  messages, now = [], 1_000_000_000
  for tick in range(800):
    dt = int(dt_ms * 1e6)
    if stall_at is not None:   # the loop runs 80 ms late once, then catches up with 7.5 ms calls (Codex review 2)
      dt = 90_000_000 if tick == stall_at else 7_500_000 if stall_at < tick <= stall_at + 32 else dt
    now += dt
    cc.actuators.torque = 0. if (lead_zero and tick < 50) or tick >= 700 else (1. if tick < 300 else -1.)
    _, sends = controller.update(cc.as_reader(), state, now)
    state.loopback_lka_steering_cmd_updated = False
    for addr, data, _bus in sends:
      if addr == 0x180:
        torque = ((data[0] & 0x7) << 8) | data[1]
        messages.append(((now - 1_000_000_000) // 1000, torque - 2048 if torque & 0x400 else torque))
        state.loopback_lka_steering_cmd_ts_nanos = now   # the bus echoes our command
        state.loopback_lka_steering_cmd_updated = True
  return controller, messages


@pytest.mark.parametrize("up,down", [(0, 0), (10, 15), (12, 25), (15, 15), (50, 50)])
def test_bolt_cc_custom_deltas_stay_inside_panda_limits(runtime, monkeypatch, up, down):
  controller, messages = command_trace(runtime, monkeypatch, up, down)
  p = controller.params
  assert p.STEER_STEP == 3
  assert p.STEER_DELTA_UP == (min(up, 12) if up else 10)
  assert p.STEER_DELTA_DOWN == (min(down, 25) if down else 15)
  assert max(t for _, t in messages) == 300 and min(t for _, t in messages) == -300
  assert violations_at_any_phase(messages) == {}


@pytest.mark.parametrize("candidate", [CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_TRAVERSE])
@pytest.mark.parametrize("up,down", [(0, 0), (50, 50)])
@pytest.mark.parametrize("stall_at", [40, 44, 62, 75])
def test_late_loop_catching_up_stays_inside_panda_limits(runtime, monkeypatch, candidate, up, down, stall_at):
  _, messages = command_trace(runtime, monkeypatch, up, down, candidate, stall_at=stall_at)
  assert violations_at_any_phase(messages) == {}


def test_late_loop_breaks_the_panda_limit_without_the_guard(runtime, monkeypatch):
  # the scenario above is a real counterexample for the per-command limit alone
  _, messages = command_trace(runtime, monkeypatch, 50, 50, CAR.CHEVROLET_BOLT_CC, stall_at=40, guard=False)
  assert violations_at_any_phase(messages) != {}


def test_guard_never_slows_the_nominal_ramps(runtime, monkeypatch):
  # at the regular command rate the per-command ceilings alone keep inside the band, so the guard never clips
  for candidate, up in ((CAR.CHEVROLET_BOLT_CC, 50), (CAR.CHEVROLET_BOLT_EUV, 50), (CAR.CHEVROLET_TRAVERSE, 0)):
    _, guarded = command_trace(runtime, monkeypatch, up, up, candidate)
    _, unguarded = command_trace(runtime, monkeypatch, up, up, candidate, guard=False)
    assert guarded == unguarded


def test_guard_band():
  from opendbc.car.gm.torque_rt_guard import PandaRtGuard
  guard = PandaRtGuard(128, 250)
  ms = 1_000_000
  assert guard.clip(100, 0) == 100
  assert guard.clip(300, 10 * ms) == 128          # the panda's initial reference 0 is still in the window
  assert guard.clip(-300, 20 * ms) == -128        # unwinding through zero is allowed down to -128
  # the window is anchored at the previous command: at 400 ms everything sent by 20 ms still counts
  assert guard.clip(300, 400 * ms) == 128
  # at 700 ms only the 128 sent at 400 ms is left
  assert guard.clip(300, 700 * ms) == 256


def test_panda_check_emulation_matches_the_c_code_on_known_cases():
  # 15 per command every 30 ms: nine rises (135) on one reference before the 250 ms reset
  assert panda_gm_torque_violations([(i * 30_000, min(15 * i, 300)) for i in range(30)]) != []
  assert violations_at_any_phase([(i * 30_000, min(12 * i, 300)) for i in range(30)]) == {}
  # a violation resets the last torque to 0 as in C: (0,0),(30000,21),(60000,22) both 21 and 22 are violations
  assert panda_gm_torque_violations([(0, 0), (30_000, 21), (60_000, 22)]) == [(30_000, 21), (60_000, 22)]


@pytest.mark.parametrize("candidate", [CAR.CHEVROLET_BOLT_CC, CAR.CHEVROLET_BOLT_EUV, CAR.CHEVROLET_TRAVERSE])
@pytest.mark.parametrize("up,down", [(0, 0), (50, 50)])
@pytest.mark.parametrize("dt_ms", [6., 10.])
def test_start_without_leading_zero_respects_the_initial_panda_reference(runtime, monkeypatch, candidate, up, down, dt_ms):
  # Codex review 3: torque from the first command, fast calls, panda reference still its initial 0
  _, messages = command_trace(runtime, monkeypatch, up, down, candidate, lead_zero=False, dt_ms=dt_ms)
  assert messages[0][1] != 0 or messages[1][1] != 0
  assert violations_at_any_phase(messages) == {}


def boundary_commands(guard):
  # Codex review 3: the 0 reference reaches the panda 40 ms late, the following commands on time, each rising 16
  ms = 1_000_000
  host = [1000 * ms] + [1290 * ms + 500 - 30 * ms * j for j in range(8, -1, -1)] + [1320 * ms + 500]
  sent, last = [guard.clip(0, host[0])], 0
  for t in host[1:]:
    last = guard.clip(min(last + 16, 300), t)
    sent.append(last)
  panda_us = [1_040_000] + [t // 1000 for t in host[1:]]   # whole microseconds: 1290.0005 ms -> 1_290_000
  return list(zip(panda_us, sent))


def test_guard_keeps_a_reference_the_panda_still_holds_at_exactly_250_ms():
  from opendbc.car.gm.torque_rt_guard import PandaRtGuard
  messages = boundary_commands(PandaRtGuard(128, 250))
  assert messages[-2] == (1_290_000, 128) and messages[-1] == (1_320_000, 128)
  assert panda_gm_torque_violations(messages, rt_ts_start=1_040_000 - 300_000) == []
  # a window of exactly 250 + 40 ms drops the 0 at 1290.0005 ms and lets the next command break the C check
  late = boundary_commands(PandaRtGuard(128, 250, margin_ms=40))
  assert late[-1][1] > 128
  assert panda_gm_torque_violations(late, rt_ts_start=1_040_000 - 300_000) != []


def test_guard_starts_from_the_panda_initial_zero():
  from opendbc.car.gm.torque_rt_guard import PandaRtGuard
  guard = PandaRtGuard(128, 250)
  assert guard.clip(200, 0) == 128
