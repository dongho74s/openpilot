"""Gas/brake side hysteresis for the GM camera ACC / SDGM openpilot long cars (gas_brake_hysteresis.py, user request
2026-11-20 for every Chevrolet). Real DBC, packer and CarController; no device I/O."""
import pytest

from opendbc.car import structs
from opendbc.car.gm import gas_brake_hysteresis as gbh
from opendbc.car.gm.gas_brake_hysteresis import GasBrakeHysteresis
from opendbc.car.gm.values import CAR, DBC
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
          cruise_standstill=False, stock_active=True):
    """Returns (gas, brake) of every 0x2CB/0x315 pair sent, decoded from the CAN frames; self.taps gets the pedal."""
    pairs, self.taps = [], []
    for _ in range(int(round(seconds * 100)) if frames is None else frames):
      cc = structs.CarControl(enabled=True, longActive=long_active, latActive=True)
      cc.actuators.accel = accel
      cc.actuators.longControlState = state if long_active else "off"
      cc.hudControl.setSpeed = 20.0
      cc.orientationNED = [0.0, pitch, 0.0]
      cs = controller_state()
      cs.out = structs.CarState(vEgo=v, aEgo=0.0, standstill=v < 0.05)
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
    return pairs


@pytest.mark.parametrize("name,make_cp", STEP_CARS, ids=[n for n, _ in STEP_CARS])
def test_small_decel_coasts_and_strong_decel_brakes(runtime, monkeypatch, name, make_cp):
  d = Drive(runtime, monkeypatch, make_cp(runtime))
  assert all(g > 0 and b == 0 for g, b in d.run(0.2, 1.0)[2:])
  coast = d.run(-0.15, 2.0)
  assert coast and set(coast) == {(0, 0)}  # the old lookups sent MAX_ACC_REGEN with friction brake 15 here
  braking = d.run(-0.3, 0.5)[1:]
  assert braking and set(braking) == {(-540, 30)}
  d.run(0.5, 0.5)
  assert set(d.run(-0.6, 0.2)[1:]) == {(-540, 60)}  # strong: at once, right after switching to gas


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
