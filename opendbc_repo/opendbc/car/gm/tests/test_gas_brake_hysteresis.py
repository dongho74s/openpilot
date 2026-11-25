"""Gas/brake side hysteresis for the GM camera ACC / SDGM openpilot long cars (gas_brake_hysteresis.py, user request
2026-11-20 for every Chevrolet). Real DBC, packer and CarController; no device I/O."""
import pytest

from opendbc.car import structs
from opendbc.car.gm import gas_brake_hysteresis as gbh
from opendbc.car.gm.gas_brake_hysteresis import GasBrakeHysteresis
from opendbc.car.gm.values import CAR, DBC, EV_CAR
from opendbc.car.gm.tests.test_display_speed import build_cp, controller_state, runtime  # noqa: F401
from opendbc.car.gm.tests.test_bolt_acc_pedal import bolt_cp, friction_brake, memory_params, pedal

ZERO_GAS, REGEN = 0.0, -540.0


# ---------------------------------------------------------------------------------------------------------------------
# The side logic alone (one call per 25 Hz command)


def feed(h, accel, n=1, brake_accel=None):
  """The camera lookups for accel (gas >= 0 or MAX_ACC_REGEN, brake 100 per m/s^2), through the hysteresis."""
  brake_accel = accel if brake_accel is None else brake_accel
  out = None
  for _ in range(n):
    g = REGEN if accel < 0 else round(accel * 673)
    b = max(0, round(-brake_accel * 100))
    out = h.update(accel, brake_accel, g, b, ZERO_GAS, REGEN)
  return out


def test_first_command_takes_its_own_side():
  assert feed(GasBrakeHysteresis(), -0.05) == (-540, 5)
  assert feed(GasBrakeHysteresis(), 0.05) == (34, 0)


def test_gas_side_coasts_until_brake_enter():
  h = GasBrakeHysteresis()
  feed(h, 0.2, n=20)
  for accel in (-0.01, -0.1, -0.19, -0.2):
    assert feed(h, accel) == (0, 0), accel  # ZERO_GAS, no friction brake, not MAX_ACC_REGEN
  assert feed(h, -0.21) == (-540, 21)  # held long enough already: switches


def test_brake_side_keeps_regen_until_gas_enter():
  h = GasBrakeHysteresis()
  feed(h, -0.5, n=20)
  for accel in (0.0, 0.05, 0.1):
    assert feed(h, accel) == (-540, 0), accel
  assert feed(h, 0.11) == (74, 0)


def test_a_new_side_is_held_unless_the_command_is_strong():
  h = GasBrakeHysteresis()
  feed(h, -0.5, n=20)
  assert feed(h, 0.35) == (236, 0)  # above GAS_NOW: at once
  # just switched to gas: a moderate decel coasts for HOLD_FRAMES - 1 more commands, then brakes
  for i in range(gbh.HOLD_FRAMES - 1):
    assert feed(h, -0.3) == (0, 0), i
  assert feed(h, -0.3) == (-540, 30)
  # just switched to brake: a moderate accel keeps regen, a strong one switches at once
  assert feed(h, 0.2) == (-540, 0)
  assert feed(h, 0.31) == (209, 0)
  # just switched to gas: a strong decel brakes at once
  assert feed(h, -0.41) == (-540, 41)


def test_a_strong_raw_brake_request_switches_at_once():
  """LongPitch on, uphill below 10 m/s: the gas input is near 0 but the brake input keeps the raw -0.5 (Codex R1)."""
  h = GasBrakeHysteresis()
  feed(h, 0.2, n=20)
  assert feed(h, -0.11, brake_accel=-0.5) == (-540, 50)
  h = GasBrakeHysteresis()
  feed(h, 0.2, n=20)
  assert feed(h, -0.05, brake_accel=-0.3) == (-540, 30)  # held long enough already: a moderate one switches too


def test_reset_forgets_the_side():
  h = GasBrakeHysteresis()
  feed(h, 0.2, n=20)
  assert feed(h, -0.1) == (0, 0)
  h.reset()
  assert feed(h, -0.1) == (-540, 10)


# ---------------------------------------------------------------------------------------------------------------------
# Blend (user request 2026-11-25, every car with the hysteresis): the coast stays, the brake side fades in after it


@pytest.mark.parametrize("ev", [True, False], ids=["ev_regen_fades", "engine_regen_at_once"])
def test_blend_fades_the_brake_side_in_after_the_coast(ev):
  h = GasBrakeHysteresis(blend=True, blend_regen=ev)
  feed(h, 0.2, n=20)
  for i in range(30):
    assert feed(h, -0.15) == (0, 0), i  # the coast is unchanged
  fade = [feed(h, -0.3) for _ in range(gbh.BLEND_FRAMES)]  # below BRAKE_ENTER, long after the last switch
  assert fade == [(round(-54 * k) if ev else -540, round(3 * k)) for k in range(1, gbh.BLEND_FRAMES + 1)]
  assert feed(h, -0.3, n=5) == (-540, 30)  # then the plain brake side
  # the old controller jumped from (0, 0) to (-540, 30) in one command
  assert max(b2 - b1 for (_, b1), (_, b2) in zip([(0, 0)] + fade, fade)) <= 3


def test_blend_is_shorter_for_a_deeper_request_and_off_when_urgent():
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  fade = [feed(h, -0.7) for _ in range(6)]  # below BRAKE_NOW: at once, over 5 commands
  assert [b for _, b in fade] == [14, 28, 42, 56, 70, 70]
  assert fade[4] == (-540, 70)
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  assert feed(h, -1.0) == (-540, 100)  # BLEND_URGENT: the full brake side in the first command
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  assert feed(h, -1.5) == (-540, 150)


def test_a_request_turning_urgent_ends_the_fade_at_once():
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  feed(h, -0.5)  # the fade has begun
  assert feed(h, -0.5)[1] < 50
  assert feed(h, -1.1) == (-540, 110)


def test_every_switch_from_the_gas_side_fades_and_reset_ends_it():
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  feed(h, -0.5, n=20)
  feed(h, 0.5, n=gbh.BLEND_FRAMES + gbh.QUICK_RETURN_FRAMES)  # a real gas phase (above GAS_NOW: at once)
  assert feed(h, -0.5)[1] < 50  # fades again
  h.reset()
  assert feed(h, -0.5) == (-540, 50)  # a fresh controller takes the brake side as it is (stops, re-engage)


def test_a_quick_return_to_the_brake_side_resumes_instead_of_fading_from_0():
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  feed(h, -0.6, n=20)  # fully braking
  feed(h, 0.5)  # one command of gas (above GAS_NOW)
  assert feed(h, -0.6)[1] >= 50  # back within 0.4 s: resumes from 90 %, not from 0
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  feed(h, -0.6, n=20)
  feed(h, 0.5, n=gbh.BLEND_FRAMES + gbh.QUICK_RETURN_FRAMES)  # a real gas phase
  assert feed(h, -0.6)[1] < 15  # fades in from 0 again
  # a fast back-and-forth cannot keep the brake weak: every return resumes where it was let off
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  brakes = []
  for _ in range(10):
    brakes += [feed(h, -0.6)[1] for _ in range(3)]
    feed(h, 0.5)
  assert brakes[-1] >= 40
  # command by command (+0.35 / -0.45, independent review 2026-11-25): the brake still reaches the request
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  brakes = []
  for _ in range(12):
    brakes.append(feed(h, -0.45)[1])
    feed(h, 0.35)
  assert brakes[-1] == 45 and brakes == sorted(brakes)


def test_the_fade_rises_from_the_brake_already_sent():
  """Independent review 2026-11-25: with LongPitch on a slow uphill the gas side sends a brake (gas input >= 0, brake
  input < 0). When the request deepens past the switch the brake must not restart from 0 (it fell 60 -> 11)."""
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  assert feed(h, 0.05, n=15, brake_accel=-0.6) == (34, 60)  # gas side, brake already on
  deeper = [feed(h, -0.1, brake_accel=-0.8) for _ in range(5)]
  assert all(b >= 60 for _, b in deeper) and [b for _, b in deeper] == sorted(b for _, b in deeper)
  assert deeper[-1] == (-540, 80)
  # a lower brake than the one sent is not held up
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  feed(h, 0.05, n=15, brake_accel=-0.6)
  assert feed(h, -0.45, brake_accel=-0.45) == (-540, 45)


def test_brake_side_regen_through_a_small_accel_also_fades_after_a_switch():
  h = GasBrakeHysteresis(blend=True, blend_regen=True)
  feed(h, 0.2, n=20)
  feed(h, -0.5)  # switched, fading
  gas, brake = feed(h, 0.05)  # back above 0 but held on the brake side: regen faded too (EV), no brake
  assert -540 < gas < 0 and brake == 0
  h = GasBrakeHysteresis(blend=True)
  feed(h, 0.2, n=20)
  feed(h, -0.5)
  assert feed(h, 0.05) == (-540, 0)  # engine car: 0x2CB takes only its two ends


def test_without_blend_nothing_changes():
  a, b = GasBrakeHysteresis(), GasBrakeHysteresis(blend=False)
  seq = [0.2] * 20 + [-0.3] * 30 + [0.5] * 3 + [-0.7] * 10 + [0.05] * 5 + [-1.2] * 3 + [0.35] * 4 + [-0.45] * 12
  assert [feed(a, x) for x in seq] == [feed(b, x) for x in seq]


# ---------------------------------------------------------------------------------------------------------------------
# Which cars


STEP_CARS = [
  ("traverse", lambda rt: build_cp(rt, CAR.CHEVROLET_TRAVERSE, True)),
  ("bolt_euv", lambda rt: build_cp(rt, CAR.CHEVROLET_BOLT_EUV, True)),
  ("bolt_acc_pedal_mode_1", lambda rt: bolt_cp(rt, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False)),
  ("trailblazer", lambda rt: build_cp(rt, CAR.CHEVROLET_TRAILBLAZER, True)),
  ("equinox", lambda rt: build_cp(rt, CAR.CHEVROLET_EQUINOX, True)),
  ("silverado", lambda rt: build_cp(rt, CAR.CHEVROLET_SILVERADO, True)),
]


def controller(runtime, monkeypatch, cp, **params):
  monkeypatch.setattr(runtime.controller, "Params", memory_params(**params))
  return runtime.controller.CarController(DBC[cp.carFingerprint], cp)


@pytest.mark.parametrize("name,make_cp", STEP_CARS, ids=[n for n, _ in STEP_CARS])
def test_camera_long_chevrolets_have_it(runtime, monkeypatch, name, make_cp):
  cp = make_cp(runtime)
  assert cp.openpilotLongitudinalControl, name
  assert controller(runtime, monkeypatch, cp).gas_brake_hysteresis is not None


@pytest.mark.parametrize("name,make_cp", STEP_CARS, ids=[n for n, _ in STEP_CARS])
def test_every_camera_long_chevrolet_blends(runtime, monkeypatch, name, make_cp):
  cp = make_cp(runtime)
  h = controller(runtime, monkeypatch, cp).gas_brake_hysteresis
  assert h.blend_enabled and h.blend_regen == (cp.carFingerprint in EV_CAR)  # 0x2CB in-between values: EVs only


def test_cars_without_the_step_or_camera_long_do_not(runtime, monkeypatch):
  # stock ACC (no alpha long), the Volt's ASCM regen band, a CC-only car, and the Bolt pedal car in StarPilot mode
  cps = [build_cp(runtime, CAR.CHEVROLET_TRAVERSE, False), build_cp(runtime, CAR.CHEVROLET_VOLT, True),
         build_cp(runtime, CAR.CHEVROLET_BOLT_CC, True)]
  monkeypatch.setattr(runtime.interface, "Params", memory_params(BoltPedalLongMode=2))
  cps.append(bolt_cp(runtime, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False))
  for cp in cps:
    assert controller(runtime, monkeypatch, cp).gas_brake_hysteresis is None, cp.carFingerprint


# ---------------------------------------------------------------------------------------------------------------------
# Through the real CarController (100 Hz updates, 25 Hz 0x2CB/0x315)


class Drive:
  def __init__(self, runtime, monkeypatch, cp, **params):
    self.c = controller(runtime, monkeypatch, cp, **params)
    self.c.fcw_warning_test = None
    self.frame = 0

  def run(self, accel, seconds=0.0, state="pid", v=12.0, long_active=True, pitch=0.0, frames=None,
          cruise_standstill=False, stock_active=True, soft_hold=0, raw_state=False):
    """Returns (gas, brake) of every 0x2CB/0x315 pair sent, decoded from the CAN frames; self.taps gets the pedal."""
    pairs, self.taps = [], []
    for _ in range(int(round(seconds * 100)) if frames is None else frames):
      cc = structs.CarControl(enabled=True, longActive=long_active, latActive=True)
      cc.actuators.accel = accel
      cc.actuators.longControlState = state if long_active or raw_state else "off"
      cc.hudControl.setSpeed = 20.0
      cc.orientationNED = [0.0, pitch, 0.0]
      cs = controller_state()
      cs.out = structs.CarState(vEgo=v, aEgo=0.0, standstill=v < 0.05, softHoldActive=soft_hold)
      cs.out.cruiseState.available = True
      cs.out.cruiseState.enabled = True
      cs.out.gearShifter = "drive"
      cs.out.cruiseState.standstill = cruise_standstill
      cs.pcm_acc_status = 4 if cruise_standstill else 1
      cs.stock_fcw_alert = 0
      cs.CP = self.c.CP
      if self.c.CP.carFingerprint == CAR.CHEVROLET_TRAILBLAZER:
        # its 0x2CB follows the stock camera's command counter (gmcan.get_longitudinal_command_timing)
        cs.cam_ascm_2cb_counter_ts_nanos = 1
        cs.cam_ascm_2cb_counter_updated = self.frame % 4 == 0
        cs.cam_ascm_2cb_counter = (self.frame // 4) % 4
        cs.cam_stock_long_active = stock_active
        cs.cam_acc_status = {"ACCCmdActive": 1}
      _, sends = self.c.update(cc.as_reader(), cs, 1_000_000_000 + self.frame * 10_000_000)
      self.frame += 1
      gas = [dat for addr, dat, bus in sends if addr == 0x2CB and bus == 0]
      brake = [dat for addr, dat, bus in sends if addr == 0x315]
      if gas and brake:
        raw = ((gas[0][1] & 7) << 16) | (gas[0][2] << 8) | gas[0][3]
        pairs.append((round((raw - 180272) / 8), friction_brake(brake[0])))
        self.taps.append(pedal(sends))
        self.brake_modes = getattr(self, "brake_modes", []) + [brake[0][0] >> 4]
    return pairs


@pytest.mark.parametrize("name,make_cp", STEP_CARS, ids=[n for n, _ in STEP_CARS])
def test_small_decel_coasts_and_strong_decel_brakes(runtime, monkeypatch, name, make_cp):
  d = Drive(runtime, monkeypatch, make_cp(runtime))
  assert all(g > 0 and b == 0 for g, b in d.run(0.2, 1.0)[2:])
  coast = d.run(-0.15, 2.0)
  assert coast and set(coast) == {(0, 0)}  # the old lookups sent MAX_ACC_REGEN with friction brake 15 here
  # blend: the brake side fades in over BLEND_FRAMES commands (the old controller jumped to (-540, 30) at once)
  braking = d.run(-0.3, 1.0)
  fade = braking[:gbh.BLEND_FRAMES]
  assert all(b1 <= b2 and g1 >= g2 for (g1, b1), (g2, b2) in zip(fade, fade[1:])) and fade[0][1] < 30
  assert set(braking[gbh.BLEND_FRAMES:]) == {(-540, 30)}
  d.run(0.5, 0.5)
  strong = d.run(-0.6, 0.6)
  assert strong[0][1] < 60 and set(strong[-5:]) == {(-540, 60)}  # strong: switches at once, fades over ~7


def test_brake_side_keeps_regen_through_a_small_accel(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, bolt_cp(runtime, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False))
  d.run(-0.5, 1.0)
  regen = d.run(0.08, 2.0)[1:]
  assert regen and set(regen) == {(-540, 0)}
  assert all(g > 0 and b == 0 for g, b in d.run(0.2, 0.5)[1:])


def test_stopping_and_disengaging_bypass_it(runtime, monkeypatch):
  d = Drive(runtime, monkeypatch, bolt_cp(runtime, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False))
  d.run(0.2, 1.0)
  stopping = d.run(-0.15, 0.5, state="stopping", v=2.0)[1:]
  assert stopping and set(stopping) == {(-500, 15)}  # INACTIVE_REGEN while stopping, the lookup's brake
  d.run(0.2, 1.0)
  d.run(0.0, 0.2, long_active=False)
  assert set(d.run(-0.15, 0.3)[1:]) == {(-540, 15)}  # re-engaged into a decel: starts on the brake side


def test_ev_table_path_is_unchanged(runtime, monkeypatch):
  """EVTable already blends regen below 0, so the hysteresis stays out of it."""
  cp = build_cp(runtime, CAR.CHEVROLET_BOLT_EUV, True)
  with_it = Drive(runtime, monkeypatch, cp, EVTable=True)
  monkeypatch.setattr(runtime.controller, "has_gas_brake_step", lambda params: False)
  without = Drive(runtime, monkeypatch, cp, EVTable=True)
  for accel in (0.2, -0.15, -0.3, 0.08, -0.6):
    assert with_it.run(accel, 0.6) == without.run(accel, 0.6), accel


def test_pitch_compensated_accel_picks_the_side(runtime, monkeypatch):
  """With LongPitch on the gas lookup's input (accel plus gravity) is what the hysteresis sees."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAVERSE, True), LongPitch=True)
  d.run(0.3, 1.0, pitch=0.0)
  # 0.0 commanded on a 2% downhill: the compensated accel settles near -0.09 m/s^2 (0.02 rad less the 0.01 rad
  # deadzone), which used to send MAX_ACC_REGEN with friction brake 9; now it coasts
  coast = d.run(0.0, 1.0, pitch=-0.02)[5:]
  assert coast and all(g == 0 and b == 0 for g, b in coast)


@pytest.mark.parametrize("name,make_cp", STEP_CARS[:3], ids=[n for n, _ in STEP_CARS[:3]])
def test_strong_raw_brake_on_a_slow_uphill_is_kept(runtime, monkeypatch, name, make_cp):
  """Codex review 2026-11-20 R1: 4 m/s, pitch +0.05 rad, -0.5 after gas. The gas input is about -0.11 (gravity added),
  the brake input stays -0.5 (no brake pitch below 5 m/s); the old lookups sent -540/50, and so must this."""
  d = Drive(runtime, monkeypatch, make_cp(runtime), LongPitch=True)
  d.run(0.2, 2.0, v=4.0, pitch=0.05)
  braking = d.run(-0.5, 1.0, v=4.0, pitch=0.05)[1:]
  # blend: switched at once, the brake only rises (never restarts below what was sent), then the old lookups
  assert [b for _, b in braking] == sorted(b for _, b in braking)
  braking = braking[gbh.BLEND_FRAMES:]
  assert braking and set(braking) == {(-540, 50)}


@pytest.mark.parametrize("plan_start", [0.15, 0.21])
def test_bolt_mode_1_taps_to_start_on_a_downhill(runtime, monkeypatch, plan_start):
  """Codex review 2026-11-20 R2: the lower mode 1 start plus a -0.04 rad downhill made the gas input negative, and the
  tap (which needed gas above INACTIVE_REGEN) never came, so the ECM would stay in its standstill."""
  d = Drive(runtime, monkeypatch, bolt_cp(runtime, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False), LongPitch=True)
  d.run(-0.4, 2.0, v=0.0, state="stopping", pitch=-0.04, cruise_standstill=True)
  d.run(plan_start, 1.0, v=0.0, state="starting", pitch=-0.04, cruise_standstill=True)
  assert d.taps and all(t == pytest.approx(18. / 255., abs=0.002) for t in d.taps)


def test_an_inactive_pulse_between_two_commands_resets_it(runtime, monkeypatch):
  """Codex review 2026-11-20 R3: frames 101-103 inactive, between two 25 Hz commands."""
  d = Drive(runtime, monkeypatch, bolt_cp(runtime, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False))
  d.run(0.2, frames=101)
  assert d.run(0.0, frames=3, long_active=False) == []
  assert d.run(-0.15, frames=4)[0] == (-540, 15)


def test_a_command_the_stock_gate_blocked_is_not_remembered(runtime, monkeypatch):
  """Codex review 2026-11-20 R4: the Trailblazer's stock gate closed for one command while -0.5 was asked; nothing
  was braked, so +0.2 afterwards accelerates at once instead of holding regen for 0.48 s."""
  d = Drive(runtime, monkeypatch, build_cp(runtime, CAR.CHEVROLET_TRAILBLAZER, True))
  d.run(0.2, frames=100)
  gated = d.run(-0.5, frames=4, stock_active=False)
  assert gated == [(-500, 0)]
  assert d.run(0.2, frames=4)[0] == (135, 0)


@pytest.mark.parametrize("soft_hold", [1, 2])
def test_bolt_mode_1_soft_hold_keeps_the_hold_over_a_start_request(runtime, monkeypatch, soft_hold):
  """Codex review 2026-11-20 N1: a starting request with soft hold active must not tap the pedal or lift the hold."""
  d = Drive(runtime, monkeypatch, bolt_cp(runtime, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False))
  d.run(-0.4, 1.0, v=0.0, state="stopping", cruise_standstill=True)
  d.brake_modes = []
  held = d.run(0.15, 0.5, v=0.0, state="starting", cruise_standstill=True, soft_hold=soft_hold)
  assert held and set(held) == {(-500, 40)} and set(d.brake_modes) == {0xD}
  assert set(d.taps) == {0.0}


def test_bolt_mode_1_does_not_tap_while_long_is_inactive(runtime, monkeypatch):
  """Codex review 2026-11-20 N1: longActive off with a starting state and a positive accel left in the actuators."""
  d = Drive(runtime, monkeypatch, bolt_cp(runtime, CAR.CHEVROLET_BOLT_ACC_PEDAL, True, False))
  d.run(0.15, 0.5, v=0.0, state="starting", cruise_standstill=True, long_active=False, raw_state=True)
  assert d.taps and set(d.taps) == {0.0}
